#include "spiffs.h"
#include "spiffs_nucleus.h"

#if !SPIFFS_READ_ONLY

// Erases a logical block and updates the erase counter.
// If cache is enabled, all pages that might be cached in this block
// is dropped.
static s32_t spiffs_gc_erase_block(
    spiffs *fs,
    spiffs_block_ix bix)
{
    s32_t res;

    SPIFFS_GC_DBG("gc: erase block "_SPIPRIbl"\n", bix);
    res = spiffs_erase_block(fs, bix);
    SPIFFS_CHECK_RES(res);

#if SPIFFS_CACHE
    {
        u32_t i;
        for (i = 0; i < SPIFFS_PAGES_PER_BLOCK(fs); i++)
        {
            spiffs_cache_drop_page(fs, SPIFFS_PAGE_FOR_BLOCK(fs, bix) + i);
        }
    }
#endif
    return res;
}

// Searches for blocks where all entries are deleted - if one is found,
// the block is erased. Compared to the non-quick gc, the quick one ensures
// that no updates are needed on existing objects on pages that are erased.
s32_t spiffs_gc_quick(
    spiffs *fs, u16_t max_free_pages)
{
    s32_t res = SPIFFS_OK;
    u32_t blocks = fs->block_count;
    spiffs_block_ix cur_block = 0;
    u32_t cur_block_addr = 0;
    int cur_entry = 0;
    spiffs_obj_id *obj_lu_buf = (spiffs_obj_id *)fs->lu_work;

    SPIFFS_GC_DBG("gc_quick: running\n");
#if SPIFFS_GC_STATS
    fs->stats_gc_runs++;
#endif

    int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(spiffs_obj_id));

    // find fully deleted blocks
    // check each block
    while (res == SPIFFS_OK && blocks--)
    {
        u16_t deleted_pages_in_block = 0;
        u16_t free_pages_in_block = 0;

        int obj_lookup_page = 0;
        // check each object lookup page
        while (res == SPIFFS_OK && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
            int entry_offset = obj_lookup_page * entries_per_page;
            res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                             0, cur_block_addr + SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page), SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
            // check each entry
            while (res == SPIFFS_OK &&
                    cur_entry - entry_offset < entries_per_page &&
                    cur_entry < (int)(SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
                spiffs_obj_id obj_id = obj_lu_buf[cur_entry - entry_offset];
                if (obj_id == SPIFFS_OBJ_ID_DELETED)
                {
                    deleted_pages_in_block++;
                }
                else if (obj_id == SPIFFS_OBJ_ID_FREE)
                {
                    // kill scan, go for next block
                    free_pages_in_block++;
                    if (free_pages_in_block > max_free_pages)
                    {
                        obj_lookup_page = SPIFFS_OBJ_LOOKUP_PAGES(fs);
                        res = 1; // kill object lu loop
                        break;
                    }
                }
                else
                {
                    // kill scan, go for next block
                    obj_lookup_page = SPIFFS_OBJ_LOOKUP_PAGES(fs);
                    res = 1; // kill object lu loop
                    break;
                }
                cur_entry++;
            } // per entry
            obj_lookup_page++;
        } // per object lookup page
        if (res == 1) res = SPIFFS_OK;

        if (res == SPIFFS_OK &&
                deleted_pages_in_block + free_pages_in_block == SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs) &&
                free_pages_in_block <= max_free_pages)
        {
            // found a fully deleted block
            fs->stats_p_deleted -= deleted_pages_in_block;
            res = spiffs_gc_erase_block(fs, cur_block);
            return res;
        }

        cur_entry = 0;
        cur_block++;
        cur_block_addr += SPIFFS_CFG_LOG_BLOCK_SZ(fs);
    } // per block

    if (res == SPIFFS_OK)
    {
        res = SPIFFS_ERR_NO_DELETED_BLOCKS;
    }
    return res;
}

// Checks if garbage collecting is necessary. If so a candidate block is found,
// cleansed and erased
s32_t spiffs_gc_check(
    spiffs *fs,
    u32_t len)
{
    s32_t res;
    s32_t free_pages =
        (SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)) * (fs->block_count - 2)
        - fs->stats_p_allocated - fs->stats_p_deleted;
    int tries = 0;

    if (fs->free_blocks > 3 &&
            (s32_t)len < free_pages * (s32_t)SPIFFS_DATA_PAGE_SIZE(fs))
    {
        return SPIFFS_OK;
    }

    u32_t needed_pages = (len + SPIFFS_DATA_PAGE_SIZE(fs) - 1) / SPIFFS_DATA_PAGE_SIZE(fs);
    //  if (fs->free_blocks <= 2 && (s32_t)needed_pages > free_pages) {
    //    SPIFFS_GC_DBG("gc: full freeblk:"_SPIPRIi" needed:"_SPIPRIi" free:"_SPIPRIi" dele:"_SPIPRIi"\n", fs->free_blocks, needed_pages, free_pages, fs->stats_p_deleted);
    //    return SPIFFS_ERR_FULL;
    //  }
    if ((s32_t)needed_pages > (s32_t)(free_pages + fs->stats_p_deleted))
    {
        SPIFFS_GC_DBG("gc_check: full freeblk:"_SPIPRIi" needed:"_SPIPRIi" free:"_SPIPRIi" dele:"_SPIPRIi"\n", fs->free_blocks, needed_pages, free_pages, fs->stats_p_deleted);
        return SPIFFS_ERR_FULL;
    }

    do
    {
        SPIFFS_GC_DBG("\ngc_check #"_SPIPRIi": run gc free_blocks:"_SPIPRIi" pfree:"_SPIPRIi" pallo:"_SPIPRIi" pdele:"_SPIPRIi" ["_SPIPRIi"] len:"_SPIPRIi" of "_SPIPRIi"\n",
                      tries,
                      fs->free_blocks, free_pages, fs->stats_p_allocated, fs->stats_p_deleted, (free_pages + fs->stats_p_allocated + fs->stats_p_deleted),
                      len, (u32_t)(free_pages * SPIFFS_DATA_PAGE_SIZE(fs)));

        spiffs_block_ix *cands;
        int count;
        spiffs_block_ix cand;
        s32_t prev_free_pages = free_pages;
        // if the fs is crammed, ignore block age when selecting candidate - kind of a bad state
        res = spiffs_gc_find_candidate(fs, &cands, &count, free_pages <= 0);
        SPIFFS_CHECK_RES(res);
        if (count == 0)
        {
            SPIFFS_GC_DBG("gc_check: no candidates, return\n");
            return (s32_t)needed_pages < free_pages ? SPIFFS_OK : SPIFFS_ERR_FULL;
        }
#if SPIFFS_GC_STATS
        fs->stats_gc_runs++;
#endif
        cand = cands[0];
        fs->cleaning = 1;
        //SPIFFS_GC_DBG("gcing: cleaning block "_SPIPRIi"\n", cand);
        res = spiffs_gc_clean(fs, cand);
        fs->cleaning = 0;
        if (res < 0)
        {
            SPIFFS_GC_DBG("gc_check: cleaning block "_SPIPRIi", result "_SPIPRIi"\n", cand, res);
        }
        else
        {
            SPIFFS_GC_DBG("gc_check: cleaning block "_SPIPRIi", result "_SPIPRIi"\n", cand, res);
        }
        SPIFFS_CHECK_RES(res);

        res = spiffs_gc_erase_page_stats(fs, cand);
        SPIFFS_CHECK_RES(res);

        res = spiffs_gc_erase_block(fs, cand);
        SPIFFS_CHECK_RES(res);

        free_pages =
            (SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)) * (fs->block_count - 2)
            - fs->stats_p_allocated - fs->stats_p_deleted;

        if (prev_free_pages <= 0 && prev_free_pages == free_pages)
        {
            // abort early to reduce wear, at least tried once
            SPIFFS_GC_DBG("gc_check: early abort, no result on gc when fs crammed\n");
            break;
        }

    }
    while (++tries < SPIFFS_GC_MAX_RUNS && (fs->free_blocks <= 2 ||
                                            (s32_t)len > free_pages * (s32_t)SPIFFS_DATA_PAGE_SIZE(fs)));

    free_pages =
        (SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)) * (fs->block_count - 2)
        - fs->stats_p_allocated - fs->stats_p_deleted;
    if ((s32_t)len > free_pages * (s32_t)SPIFFS_DATA_PAGE_SIZE(fs))
    {
        res = SPIFFS_ERR_FULL;
    }

    SPIFFS_GC_DBG("gc_check: finished, "_SPIPRIi" dirty, blocks "_SPIPRIi" free, "_SPIPRIi" pages free, "_SPIPRIi" tries, res "_SPIPRIi"\n",
                  fs->stats_p_allocated + fs->stats_p_deleted,
                  fs->free_blocks, free_pages, tries, res);

    return res;
}

// Updates page statistics for a block that is about to be erased
s32_t spiffs_gc_erase_page_stats(
    spiffs *fs,
    spiffs_block_ix bix)
{
    s32_t res = SPIFFS_OK;
    int obj_lookup_page = 0;
    int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(spiffs_obj_id));
    spiffs_obj_id *obj_lu_buf = (spiffs_obj_id *)fs->lu_work;
    int cur_entry = 0;
    u32_t dele = 0;
    u32_t allo = 0;

    // check each object lookup page
    while (res == SPIFFS_OK && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
        int entry_offset = obj_lookup_page * entries_per_page;
        res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                         0, bix * SPIFFS_CFG_LOG_BLOCK_SZ(fs) + SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page), SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
        // check each entry
        while (res == SPIFFS_OK &&
                cur_entry - entry_offset < entries_per_page && cur_entry < (int)(SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)))
        {
            spiffs_obj_id obj_id = obj_lu_buf[cur_entry - entry_offset];
            if (obj_id == SPIFFS_OBJ_ID_FREE)
            {
            }
            else if (obj_id == SPIFFS_OBJ_ID_DELETED)
            {
                dele++;
            }
            else
            {
                allo++;
            }
            cur_entry++;
        } // per entry
        obj_lookup_page++;
    } // per object lookup page
    SPIFFS_GC_DBG("gc_check: wipe pallo:"_SPIPRIi" pdele:"_SPIPRIi"\n", allo, dele);
    fs->stats_p_allocated -= allo;
    fs->stats_p_deleted -= dele;
    return res;
}

// Finds block candidates to erase
s32_t spiffs_gc_find_candidate(
    spiffs *fs,
    spiffs_block_ix **block_candidates,
    int *candidate_count,
    char fs_crammed)
{
    s32_t res = SPIFFS_OK;
    u32_t blocks = fs->block_count;
    spiffs_block_ix cur_block = 0;
    u32_t cur_block_addr = 0;
    spiffs_obj_id *obj_lu_buf = (spiffs_obj_id *)fs->lu_work;
    int cur_entry = 0;

    // using fs->work area as sorted candidate memory, (spiffs_block_ix)cand_bix/(s32_t)score
    int max_candidates = MIN(fs->block_count, (SPIFFS_CFG_LOG_PAGE_SZ(fs) - 8) / (sizeof(spiffs_block_ix) + sizeof(s32_t)));
    *candidate_count = 0;
    memset(fs->work, 0xff, SPIFFS_CFG_LOG_PAGE_SZ(fs));

    // divide up work area into block indices and scores
    spiffs_block_ix *cand_blocks = (spiffs_block_ix *)fs->work;
    s32_t *cand_scores = (s32_t *)(fs->work + max_candidates * sizeof(spiffs_block_ix));

    // align cand_scores on s32_t boundary
    cand_scores = (s32_t *)(((intptr_t)cand_scores + sizeof(intptr_t) - 1) & ~(sizeof(intptr_t) - 1));

    *block_candidates = cand_blocks;

    int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(spiffs_obj_id));

    // check each block
    while (res == SPIFFS_OK && blocks--)
    {
        u16_t deleted_pages_in_block = 0;
        u16_t used_pages_in_block = 0;

        int obj_lookup_page = 0;
        // check each object lookup page
        while (res == SPIFFS_OK && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
            int entry_offset = obj_lookup_page * entries_per_page;
            res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                             0, cur_block_addr + SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page), SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
            // check each entry
            while (res == SPIFFS_OK &&
                    cur_entry - entry_offset < entries_per_page &&
                    cur_entry < (int)(SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
                spiffs_obj_id obj_id = obj_lu_buf[cur_entry - entry_offset];
                if (obj_id == SPIFFS_OBJ_ID_FREE)
                {
                    // when a free entry is encountered, scan logic ensures that all following entries are free also
                    res = 1; // kill object lu loop
                    break;
                }
                else  if (obj_id == SPIFFS_OBJ_ID_DELETED)
                {
                    deleted_pages_in_block++;
                }
                else
                {
                    used_pages_in_block++;
                }
                cur_entry++;
            } // per entry
            obj_lookup_page++;
        } // per object lookup page
        if (res == 1) res = SPIFFS_OK;

        // calculate score and insert into candidate table
        // stoneage sort, but probably not so many blocks
        if (res == SPIFFS_OK /*&& deleted_pages_in_block > 0*/)
        {
            // read erase count
            spiffs_obj_id erase_count;
            res = _spiffs_rd(fs, SPIFFS_OP_C_READ | SPIFFS_OP_T_OBJ_LU2, 0,
                             SPIFFS_ERASE_COUNT_PADDR(fs, cur_block),
                             sizeof(spiffs_obj_id), (u8_t *)&erase_count);
            SPIFFS_CHECK_RES(res);

            spiffs_obj_id erase_age;
            if (fs->max_erase_count > erase_count)
            {
                erase_age = fs->max_erase_count - erase_count;
            }
            else
            {
                erase_age = SPIFFS_OBJ_ID_FREE - (erase_count - fs->max_erase_count);
            }

            s32_t score =
                deleted_pages_in_block * SPIFFS_GC_HEUR_W_DELET +
                used_pages_in_block * SPIFFS_GC_HEUR_W_USED +
                erase_age * (fs_crammed ? 0 : SPIFFS_GC_HEUR_W_ERASE_AGE);
            int cand_ix = 0;
            SPIFFS_GC_DBG("gc_check: bix:"_SPIPRIbl" del:"_SPIPRIi" use:"_SPIPRIi" score:"_SPIPRIi"\n", cur_block, deleted_pages_in_block, used_pages_in_block, score);
            while (cand_ix < max_candidates)
            {
                if (cand_blocks[cand_ix] == (spiffs_block_ix) - 1)
                {
                    cand_blocks[cand_ix] = cur_block;
                    cand_scores[cand_ix] = score;
                    break;
                }
                else if (cand_scores[cand_ix] < score)
                {
                    int reorder_cand_ix = max_candidates - 2;
                    while (reorder_cand_ix >= cand_ix)
                    {
                        cand_blocks[reorder_cand_ix + 1] = cand_blocks[reorder_cand_ix];
                        cand_scores[reorder_cand_ix + 1] = cand_scores[reorder_cand_ix];
                        reorder_cand_ix--;
                    }
                    cand_blocks[cand_ix] = cur_block;
                    cand_scores[cand_ix] = score;
                    break;
                }
                cand_ix++;
            }
            (*candidate_count)++;
        }

        cur_entry = 0;
        cur_block++;
        cur_block_addr += SPIFFS_CFG_LOG_BLOCK_SZ(fs);
    } // per block

    return res;
}

typedef enum
{
    FIND_OBJ_DATA,
    MOVE_OBJ_DATA,
    MOVE_OBJ_IX,
    FINISHED
} spiffs_gc_clean_state;

typedef struct
{
    spiffs_gc_clean_state state;
    spiffs_obj_id cur_obj_id;
    spiffs_span_ix cur_objix_spix;
    spiffs_page_ix cur_objix_pix;
    spiffs_page_ix cur_data_pix;
    int stored_scan_entry_index;
    u8_t obj_id_found;
} spiffs_gc;

// Empties given block by moving all data into free pages of another block
// Strategy:
//   loop:
//   scan object lookup for object data pages
//   for first found id, check spix and load corresponding object index page to memory
//   push object scan lookup entry index
//     rescan object lookup, find data pages with same id and referenced by same object index
//     move data page, update object index in memory
//     when reached end of lookup, store updated object index
//   pop object scan lookup entry index
//   repeat loop until end of object lookup
//   scan object lookup again for remaining object index pages, move to new page in other block
//
s32_t spiffs_gc_clean(spiffs *fs, spiffs_block_ix bix)
{
    s32_t res = SPIFFS_OK;
    const int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(spiffs_obj_id));
    // this is the global localizer being pushed and popped
    int cur_entry = 0;
    spiffs_obj_id *obj_lu_buf = (spiffs_obj_id *)fs->lu_work;
    spiffs_gc gc; // our stack frame/state
    spiffs_page_ix cur_pix = 0;
    spiffs_page_object_ix_header *objix_hdr = (spiffs_page_object_ix_header *)fs->work;
    spiffs_page_object_ix *objix = (spiffs_page_object_ix *)fs->work;

    SPIFFS_GC_DBG("gc_clean: cleaning block "_SPIPRIbl"\n", bix);

    memset(&gc, 0, sizeof(spiffs_gc));
    gc.state = FIND_OBJ_DATA;

    if (fs->free_cursor_block_ix == bix)
    {
        // move free cursor to next block, cannot use free pages from the block we want to clean
        fs->free_cursor_block_ix = (bix + 1) % fs->block_count;
        fs->free_cursor_obj_lu_entry = 0;
        SPIFFS_GC_DBG("gc_clean: move free cursor to block "_SPIPRIbl"\n", fs->free_cursor_block_ix);
    }

    while (res == SPIFFS_OK && gc.state != FINISHED)
    {
        SPIFFS_GC_DBG("gc_clean: state = "_SPIPRIi" entry:"_SPIPRIi"\n", gc.state, cur_entry);
        gc.obj_id_found = 0; // reset (to no found data page)

        // scan through lookup pages
        int obj_lookup_page = cur_entry / entries_per_page;
        u8_t scan = 1;
        // check each object lookup page
        while (scan && res == SPIFFS_OK && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
            int entry_offset = obj_lookup_page * entries_per_page;
            res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                             0, bix * SPIFFS_CFG_LOG_BLOCK_SZ(fs) + SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                             SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
            // check each object lookup entry
            while (scan && res == SPIFFS_OK &&
                    cur_entry - entry_offset < entries_per_page && cur_entry < (int)(SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
                spiffs_obj_id obj_id = obj_lu_buf[cur_entry - entry_offset];
                cur_pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, cur_entry);

                // act upon object id depending on gc state
                switch (gc.state)
                {
                case FIND_OBJ_DATA:
                    // find a data page
                    if (obj_id != SPIFFS_OBJ_ID_DELETED && obj_id != SPIFFS_OBJ_ID_FREE &&
                            ((obj_id & SPIFFS_OBJ_ID_IX_FLAG) == 0))
                    {
                        // found a data page, stop scanning and handle in switch case below
                        SPIFFS_GC_DBG("gc_clean: FIND_DATA state:"_SPIPRIi" - found obj id "_SPIPRIid"\n", gc.state, obj_id);
                        gc.obj_id_found = 1;
                        gc.cur_obj_id = obj_id;
                        gc.cur_data_pix = cur_pix;
                        scan = 0;
                    }
                    break;
                case MOVE_OBJ_DATA:
                    // evacuate found data pages for corresponding object index we have in memory,
                    // update memory representation
                    if (obj_id == gc.cur_obj_id)
                    {
                        spiffs_page_header p_hdr;
                        res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                         0, SPIFFS_PAGE_TO_PADDR(fs, cur_pix), sizeof(spiffs_page_header), (u8_t *)&p_hdr);
                        SPIFFS_CHECK_RES(res);
                        SPIFFS_GC_DBG("gc_clean: MOVE_DATA found data page "_SPIPRIid":"_SPIPRIsp" @ "_SPIPRIpg"\n", gc.cur_obj_id, p_hdr.span_ix, cur_pix);
                        if (SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, p_hdr.span_ix) != gc.cur_objix_spix)
                        {
                            SPIFFS_GC_DBG("gc_clean: MOVE_DATA no objix spix match, take in another run\n");
                        }
                        else
                        {
                            spiffs_page_ix new_data_pix;
                            if (p_hdr.flags & SPIFFS_PH_FLAG_DELET)
                            {
                                // move page
                                res = spiffs_page_move(fs, 0, 0, obj_id, &p_hdr, cur_pix, &new_data_pix);
                                SPIFFS_GC_DBG("gc_clean: MOVE_DATA move objix "_SPIPRIid":"_SPIPRIsp" page "_SPIPRIpg" to "_SPIPRIpg"\n", gc.cur_obj_id, p_hdr.span_ix, cur_pix, new_data_pix);
                                SPIFFS_CHECK_RES(res);
                                // move wipes obj_lu, reload it
                                res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                                 0, bix * SPIFFS_CFG_LOG_BLOCK_SZ(fs) + SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                                 SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
                                SPIFFS_CHECK_RES(res);
                            }
                            else
                            {
                                // page is deleted but not deleted in lookup, scrap it -
                                // might seem unnecessary as we will erase this block, but
                                // we might get aborted
                                SPIFFS_GC_DBG("gc_clean: MOVE_DATA wipe objix "_SPIPRIid":"_SPIPRIsp" page "_SPIPRIpg"\n", obj_id, p_hdr.span_ix, cur_pix);
                                res = spiffs_page_delete(fs, cur_pix);
                                SPIFFS_CHECK_RES(res);
                                new_data_pix = SPIFFS_OBJ_ID_FREE;
                            }
                            // update memory representation of object index page with new data page
                            if (gc.cur_objix_spix == 0)
                            {
                                // update object index header page
                                ((spiffs_page_ix *)((u8_t *)objix_hdr + sizeof(spiffs_page_object_ix_header)))[p_hdr.span_ix] = new_data_pix;
                                SPIFFS_GC_DBG("gc_clean: MOVE_DATA wrote page "_SPIPRIpg" to objix_hdr entry "_SPIPRIsp" in mem\n", new_data_pix, (spiffs_span_ix)SPIFFS_OBJ_IX_ENTRY(fs, p_hdr.span_ix));
                            }
                            else
                            {
                                // update object index page
                                ((spiffs_page_ix *)((u8_t *)objix + sizeof(spiffs_page_object_ix)))[SPIFFS_OBJ_IX_ENTRY(fs, p_hdr.span_ix)] = new_data_pix;
                                SPIFFS_GC_DBG("gc_clean: MOVE_DATA wrote page "_SPIPRIpg" to objix entry "_SPIPRIsp" in mem\n", new_data_pix, (spiffs_span_ix)SPIFFS_OBJ_IX_ENTRY(fs, p_hdr.span_ix));
                            }
                        }
                    }
                    break;
                case MOVE_OBJ_IX:
                    // find and evacuate object index pages
                    if (obj_id != SPIFFS_OBJ_ID_DELETED && obj_id != SPIFFS_OBJ_ID_FREE &&
                            (obj_id & SPIFFS_OBJ_ID_IX_FLAG))
                    {
                        // found an index object id
                        spiffs_page_header p_hdr;
                        spiffs_page_ix new_pix;
                        // load header
                        res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                         0, SPIFFS_PAGE_TO_PADDR(fs, cur_pix), sizeof(spiffs_page_header), (u8_t *)&p_hdr);
                        SPIFFS_CHECK_RES(res);
                        if (p_hdr.flags & SPIFFS_PH_FLAG_DELET)
                        {
                            // move page
                            res = spiffs_page_move(fs, 0, 0, obj_id, &p_hdr, cur_pix, &new_pix);
                            SPIFFS_GC_DBG("gc_clean: MOVE_OBJIX move objix "_SPIPRIid":"_SPIPRIsp" page "_SPIPRIpg" to "_SPIPRIpg"\n", obj_id, p_hdr.span_ix, cur_pix, new_pix);
                            SPIFFS_CHECK_RES(res);
                            spiffs_cb_object_event(fs, (spiffs_page_object_ix *)&p_hdr,
                                                   SPIFFS_EV_IX_MOV, obj_id, p_hdr.span_ix, new_pix, 0);
                            // move wipes obj_lu, reload it
                            res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                             0, bix * SPIFFS_CFG_LOG_BLOCK_SZ(fs) + SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                             SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
                            SPIFFS_CHECK_RES(res);
                        }
                        else
                        {
                            // page is deleted but not deleted in lookup, scrap it -
                            // might seem unnecessary as we will erase this block, but
                            // we might get aborted
                            SPIFFS_GC_DBG("gc_clean: MOVE_OBJIX wipe objix "_SPIPRIid":"_SPIPRIsp" page "_SPIPRIpg"\n", obj_id, p_hdr.span_ix, cur_pix);
                            res = spiffs_page_delete(fs, cur_pix);
                            if (res == SPIFFS_OK)
                            {
                                spiffs_cb_object_event(fs, (spiffs_page_object_ix *)0,
                                                       SPIFFS_EV_IX_DEL, obj_id, p_hdr.span_ix, cur_pix, 0);
                            }
                        }
                        SPIFFS_CHECK_RES(res);
                    }
                    break;
                default:
                    scan = 0;
                    break;
                } // switch gc state
                cur_entry++;
            } // per entry
            obj_lookup_page++; // no need to check scan variable here, obj_lookup_page is set in start of loop
        } // per object lookup page
        if (res != SPIFFS_OK) break;

        // state finalization and switch
        switch (gc.state)
        {
        case FIND_OBJ_DATA:
            if (gc.obj_id_found)
            {
                // handle found data page -
                // find out corresponding obj ix page and load it to memory
                spiffs_page_header p_hdr;
                spiffs_page_ix objix_pix;
                gc.stored_scan_entry_index = cur_entry; // push cursor
                cur_entry = 0; // restart scan from start
                gc.state = MOVE_OBJ_DATA;
                res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                 0, SPIFFS_PAGE_TO_PADDR(fs, cur_pix), sizeof(spiffs_page_header), (u8_t *)&p_hdr);
                SPIFFS_CHECK_RES(res);
                gc.cur_objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, p_hdr.span_ix);
                SPIFFS_GC_DBG("gc_clean: FIND_DATA find objix span_ix:"_SPIPRIsp"\n", gc.cur_objix_spix);
                res = spiffs_obj_lu_find_id_and_span(fs, gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG, gc.cur_objix_spix, 0, &objix_pix);
                if (res == SPIFFS_ERR_NOT_FOUND)
                {
                    // on borked systems we might get an ERR_NOT_FOUND here -
                    // this is handled by simply deleting the page as it is not referenced
                    // from anywhere
                    SPIFFS_GC_DBG("gc_clean: FIND_OBJ_DATA objix not found! Wipe page "_SPIPRIpg"\n", gc.cur_data_pix);
                    res = spiffs_page_delete(fs, gc.cur_data_pix);
                    SPIFFS_CHECK_RES(res);
                    // then we restore states and continue scanning for data pages
                    cur_entry = gc.stored_scan_entry_index; // pop cursor
                    gc.state = FIND_OBJ_DATA;
                    break; // done
                }
                SPIFFS_CHECK_RES(res);
                SPIFFS_GC_DBG("gc_clean: FIND_DATA found object index at page "_SPIPRIpg"\n", objix_pix);
                res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                 0, SPIFFS_PAGE_TO_PADDR(fs, objix_pix), SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
                SPIFFS_CHECK_RES(res);
                // cannot allow a gc if the presumed index in fact is no index, a
                // check must run or lot of data may be lost
                SPIFFS_VALIDATE_OBJIX(objix->p_hdr, gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG, gc.cur_objix_spix);
                gc.cur_objix_pix = objix_pix;
            }
            else
            {
                // no more data pages found, passed thru all block, start evacuating object indices
                gc.state = MOVE_OBJ_IX;
                cur_entry = 0; // restart entry scan index
            }
            break;
        case MOVE_OBJ_DATA:
        {
            // store modified objix (hdr) page residing in memory now that all
            // data pages belonging to this object index and residing in the block
            // we want to evacuate
            spiffs_page_ix new_objix_pix;
            gc.state = FIND_OBJ_DATA;
            cur_entry = gc.stored_scan_entry_index; // pop cursor
            if (gc.cur_objix_spix == 0)
            {
                // store object index header page
                res = spiffs_object_update_index_hdr(fs, 0, gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG, gc.cur_objix_pix, fs->work, 0, 0, 0, &new_objix_pix);
                SPIFFS_GC_DBG("gc_clean: MOVE_DATA store modified objix_hdr page, "_SPIPRIpg":"_SPIPRIsp"\n", new_objix_pix, 0);
                SPIFFS_CHECK_RES(res);
            }
            else
            {
                // store object index page
                res = spiffs_page_move(fs, 0, fs->work, gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG, 0, gc.cur_objix_pix, &new_objix_pix);
                SPIFFS_GC_DBG("gc_clean: MOVE_DATA store modified objix page, "_SPIPRIpg":"_SPIPRIsp"\n", new_objix_pix, objix->p_hdr.span_ix);
                SPIFFS_CHECK_RES(res);
                spiffs_cb_object_event(fs, (spiffs_page_object_ix *)fs->work,
                                       SPIFFS_EV_IX_UPD, gc.cur_obj_id, objix->p_hdr.span_ix, new_objix_pix, 0);
            }
        }
        break;
        case MOVE_OBJ_IX:
            // scanned thru all block, no more object indices found - our work here is done
            gc.state = FINISHED;
            break;
        default:
            cur_entry = 0;
            break;
        } // switch gc.state
        SPIFFS_GC_DBG("gc_clean: state-> "_SPIPRIi"\n", gc.state);
    } // while state != FINISHED


    return res;
}

#endif // !SPIFFS_READ_ONLY
