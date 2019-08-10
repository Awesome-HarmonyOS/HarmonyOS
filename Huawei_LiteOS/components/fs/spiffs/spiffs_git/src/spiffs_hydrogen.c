/*
 * spiffs_hydrogen.c
 *
 *  Created on: Jun 16, 2013
 *      Author: petera
 */

#include "spiffs.h"
#include "spiffs_nucleus.h"

#if SPIFFS_CACHE == 1
static s32_t spiffs_fflush_cache(spiffs *fs, spiffs_file fh);
#endif

#if SPIFFS_BUFFER_HELP
u32_t SPIFFS_buffer_bytes_for_filedescs(spiffs *fs, u32_t num_descs)
{
    return num_descs * sizeof(spiffs_fd);
}
#if SPIFFS_CACHE
u32_t SPIFFS_buffer_bytes_for_cache(spiffs *fs, u32_t num_pages)
{
    return sizeof(spiffs_cache) + num_pages * (sizeof(spiffs_cache_page) + SPIFFS_CFG_LOG_PAGE_SZ(fs));
}
#endif
#endif

u8_t SPIFFS_mounted(spiffs *fs)
{
    return SPIFFS_CHECK_MOUNT(fs);
}

s32_t SPIFFS_format(spiffs *fs)
{
#if SPIFFS_READ_ONLY
    (void)fs;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    if (SPIFFS_CHECK_MOUNT(fs))
    {
        fs->err_code = SPIFFS_ERR_MOUNTED;
        return -1;
    }

    s32_t res;
    SPIFFS_LOCK(fs);

    spiffs_block_ix bix = 0;
    while (bix < fs->block_count)
    {
        fs->max_erase_count = 0;
        res = spiffs_erase_block(fs, bix);
        if (res != SPIFFS_OK)
        {
            res = SPIFFS_ERR_ERASE_FAIL;
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        bix++;
    }

    SPIFFS_UNLOCK(fs);

    return 0;
#endif // SPIFFS_READ_ONLY
}

#if SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH && SPIFFS_SINGLETON==0

s32_t SPIFFS_probe_fs(spiffs_config *config)
{
    SPIFFS_API_DBG("%s\n", __func__);
    s32_t res = spiffs_probe(config);
    return res;
}

#endif // SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH && SPIFFS_SINGLETON==0

s32_t SPIFFS_mount(spiffs *fs, spiffs_config *config, u8_t *work,
                   u8_t *fd_space, u32_t fd_space_size,
                   void *cache, u32_t cache_size,
                   spiffs_check_callback check_cb_f)
{
    SPIFFS_API_DBG("%s "
                   " sz:"_SPIPRIi " logpgsz:"_SPIPRIi " logblksz:"_SPIPRIi " perasz:"_SPIPRIi
                   " addr:"_SPIPRIad
                   " fdsz:"_SPIPRIi " cachesz:"_SPIPRIi
                   "\n",
                   __func__,
                   SPIFFS_CFG_PHYS_SZ(fs),
                   SPIFFS_CFG_LOG_PAGE_SZ(fs),
                   SPIFFS_CFG_LOG_BLOCK_SZ(fs),
                   SPIFFS_CFG_PHYS_ERASE_SZ(fs),
                   SPIFFS_CFG_PHYS_ADDR(fs),
                   fd_space_size, cache_size);
    void *user_data;
    SPIFFS_LOCK(fs);
    user_data = fs->user_data;
    memset(fs, 0, sizeof(spiffs));
    _SPIFFS_MEMCPY(&fs->cfg, config, sizeof(spiffs_config));
    fs->user_data = user_data;
    fs->block_count = SPIFFS_CFG_PHYS_SZ(fs) / SPIFFS_CFG_LOG_BLOCK_SZ(fs);
    fs->work = &work[0];
    fs->lu_work = &work[SPIFFS_CFG_LOG_PAGE_SZ(fs)];
    memset(fd_space, 0, fd_space_size);
    // align fd_space pointer to pointer size byte boundary
    u8_t ptr_size = sizeof(void *);
    u8_t addr_lsb = ((u8_t)(intptr_t)fd_space) & (ptr_size - 1);
    if (addr_lsb)
    {
        fd_space += (ptr_size - addr_lsb);
        fd_space_size -= (ptr_size - addr_lsb);
    }
    fs->fd_space = fd_space;
    fs->fd_count = (fd_space_size / sizeof(spiffs_fd));

    // align cache pointer to 4 byte boundary
    addr_lsb = ((u8_t)(intptr_t)cache) & (ptr_size - 1);
    if (addr_lsb)
    {
        u8_t *cache_8 = (u8_t *)cache;
        cache_8 += (ptr_size - addr_lsb);
        cache = cache_8;
        cache_size -= (ptr_size - addr_lsb);
    }
    if (cache_size & (ptr_size - 1))
    {
        cache_size -= (cache_size & (ptr_size - 1));
    }

#if SPIFFS_CACHE
    fs->cache = cache;
    fs->cache_size = (cache_size > (SPIFFS_CFG_LOG_PAGE_SZ(fs) * 32)) ? SPIFFS_CFG_LOG_PAGE_SZ(fs) * 32 : cache_size;
    spiffs_cache_init(fs);
#endif

    s32_t res;

#if SPIFFS_USE_MAGIC
    res = SPIFFS_CHECK_MAGIC_POSSIBLE(fs) ? SPIFFS_OK : SPIFFS_ERR_MAGIC_NOT_POSSIBLE;
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
#endif

    fs->config_magic = SPIFFS_CONFIG_MAGIC;

    res = spiffs_obj_lu_scan(fs);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_DBG("page index byte len:         "_SPIPRIi"\n", (u32_t)SPIFFS_CFG_LOG_PAGE_SZ(fs));
    SPIFFS_DBG("object lookup pages:         "_SPIPRIi"\n", (u32_t)SPIFFS_OBJ_LOOKUP_PAGES(fs));
    SPIFFS_DBG("page pages per block:        "_SPIPRIi"\n", (u32_t)SPIFFS_PAGES_PER_BLOCK(fs));
    SPIFFS_DBG("page header length:          "_SPIPRIi"\n", (u32_t)sizeof(spiffs_page_header));
    SPIFFS_DBG("object header index entries: "_SPIPRIi"\n", (u32_t)SPIFFS_OBJ_HDR_IX_LEN(fs));
    SPIFFS_DBG("object index entries:        "_SPIPRIi"\n", (u32_t)SPIFFS_OBJ_IX_LEN(fs));
    SPIFFS_DBG("available file descriptors:  "_SPIPRIi"\n", (u32_t)fs->fd_count);
    SPIFFS_DBG("free blocks:                 "_SPIPRIi"\n", (u32_t)fs->free_blocks);

    fs->check_cb_f = check_cb_f;

    fs->mounted = 1;

    SPIFFS_UNLOCK(fs);

    return 0;
}

void SPIFFS_unmount(spiffs *fs)
{
    SPIFFS_API_DBG("%s\n", __func__);
    if (!SPIFFS_CHECK_CFG(fs) || !SPIFFS_CHECK_MOUNT(fs)) return;
    SPIFFS_LOCK(fs);
    u32_t i;
    spiffs_fd *fds = (spiffs_fd *)fs->fd_space;
    for (i = 0; i < fs->fd_count; i++)
    {
        spiffs_fd *cur_fd = &fds[i];
        if (cur_fd->file_nbr != 0)
        {
#if SPIFFS_CACHE
            (void)spiffs_fflush_cache(fs, cur_fd->file_nbr);
#endif
            spiffs_fd_return(fs, cur_fd->file_nbr);
        }
    }
    fs->mounted = 0;

    SPIFFS_UNLOCK(fs);
}

s32_t SPIFFS_errno(spiffs *fs)
{
    return fs->err_code;
}

void SPIFFS_clearerr(spiffs *fs)
{
    SPIFFS_API_DBG("%s\n", __func__);
    fs->err_code = SPIFFS_OK;
}

s32_t SPIFFS_creat(spiffs *fs, const char *path, spiffs_mode mode)
{
    SPIFFS_API_DBG("%s '%s'\n", __func__, path);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)path;
    (void)mode;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    (void)mode;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    if (strlen(path) > SPIFFS_OBJ_NAME_LEN - 1)
    {
        SPIFFS_API_CHECK_RES(fs, SPIFFS_ERR_NAME_TOO_LONG);
    }
    SPIFFS_LOCK(fs);
    spiffs_obj_id obj_id;
    s32_t res;

    res = spiffs_obj_lu_find_free_obj_id(fs, &obj_id, (const u8_t *)path);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    res = spiffs_object_create(fs, obj_id, (const u8_t *)path, 0, SPIFFS_TYPE_FILE, 0);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    SPIFFS_UNLOCK(fs);
    return 0;
#endif // SPIFFS_READ_ONLY
}

spiffs_file SPIFFS_open(spiffs *fs, const char *path, spiffs_flags flags, spiffs_mode mode)
{
    SPIFFS_API_DBG("%s '%s' "_SPIPRIfl "\n", __func__, path, flags);
    (void)mode;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    if (strlen(path) > SPIFFS_OBJ_NAME_LEN - 1)
    {
        SPIFFS_API_CHECK_RES(fs, SPIFFS_ERR_NAME_TOO_LONG);
    }
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    spiffs_page_ix pix;

#if SPIFFS_READ_ONLY
    // not valid flags in read only mode
    flags &= ~(SPIFFS_WRONLY | SPIFFS_CREAT | SPIFFS_TRUNC);
#endif // SPIFFS_READ_ONLY

    s32_t res = spiffs_fd_find_new(fs, &fd, path);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_find_object_index_header_by_name(fs, (const u8_t *)path, &pix);
    if ((flags & SPIFFS_O_CREAT) == 0)
    {
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

    if (res == SPIFFS_OK &&
            (flags & (SPIFFS_O_CREAT | SPIFFS_O_EXCL)) == (SPIFFS_O_CREAT | SPIFFS_O_EXCL))
    {
        // creat and excl and file exists - fail
        res = SPIFFS_ERR_FILE_EXISTS;
        spiffs_fd_return(fs, fd->file_nbr);
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

    if ((flags & SPIFFS_O_CREAT) && res == SPIFFS_ERR_NOT_FOUND)
    {
#if !SPIFFS_READ_ONLY
        spiffs_obj_id obj_id;
        // no need to enter conflicting name here, already looked for it above
        res = spiffs_obj_lu_find_free_obj_id(fs, &obj_id, 0);
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        res = spiffs_object_create(fs, obj_id, (const u8_t *)path, 0, SPIFFS_TYPE_FILE, &pix);
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        flags &= ~SPIFFS_O_TRUNC;
#endif // !SPIFFS_READ_ONLY
    }
    else
    {
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }
    res = spiffs_object_open_by_page(fs, pix, fd, flags, mode);
    if (res < SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
#if !SPIFFS_READ_ONLY
    if (flags & SPIFFS_O_TRUNC)
    {
        res = spiffs_object_truncate(fd, 0, 0);
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }
#endif // !SPIFFS_READ_ONLY

    fd->fdoffset = 0;

    SPIFFS_UNLOCK(fs);

    return SPIFFS_FH_OFFS(fs, fd->file_nbr);
}

spiffs_file SPIFFS_open_by_dirent(spiffs *fs, struct spiffs_dirent *e, spiffs_flags flags, spiffs_mode mode)
{
    SPIFFS_API_DBG("%s '%s':"_SPIPRIid " "_SPIPRIfl "\n", __func__, e->name, e->obj_id, flags);
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;

    s32_t res = spiffs_fd_find_new(fs, &fd, 0);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_open_by_page(fs, e->pix, fd, flags, mode);
    if (res < SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
#if !SPIFFS_READ_ONLY
    if (flags & SPIFFS_O_TRUNC)
    {
        res = spiffs_object_truncate(fd, 0, 0);
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }
#endif // !SPIFFS_READ_ONLY

    fd->fdoffset = 0;

    SPIFFS_UNLOCK(fs);

    return SPIFFS_FH_OFFS(fs, fd->file_nbr);
}

spiffs_file SPIFFS_open_by_page(spiffs *fs, spiffs_page_ix page_ix, spiffs_flags flags, spiffs_mode mode)
{
    SPIFFS_API_DBG("%s "_SPIPRIpg " "_SPIPRIfl "\n", __func__, page_ix, flags);
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;

    s32_t res = spiffs_fd_find_new(fs, &fd, 0);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if (SPIFFS_IS_LOOKUP_PAGE(fs, page_ix))
    {
        res = SPIFFS_ERR_NOT_A_FILE;
        spiffs_fd_return(fs, fd->file_nbr);
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

    res = spiffs_object_open_by_page(fs, page_ix, fd, flags, mode);
    if (res == SPIFFS_ERR_IS_FREE ||
            res == SPIFFS_ERR_DELETED ||
            res == SPIFFS_ERR_NOT_FINALIZED ||
            res == SPIFFS_ERR_NOT_INDEX ||
            res == SPIFFS_ERR_INDEX_SPAN_MISMATCH)
    {
        res = SPIFFS_ERR_NOT_A_FILE;
    }
    if (res < SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

#if !SPIFFS_READ_ONLY
    if (flags & SPIFFS_O_TRUNC)
    {
        res = spiffs_object_truncate(fd, 0, 0);
        if (res < SPIFFS_OK)
        {
            spiffs_fd_return(fs, fd->file_nbr);
        }
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }
#endif // !SPIFFS_READ_ONLY

    fd->fdoffset = 0;

    SPIFFS_UNLOCK(fs);

    return SPIFFS_FH_OFFS(fs, fd->file_nbr);
}

static s32_t spiffs_hydro_read(spiffs *fs, spiffs_file fh, void *buf, s32_t len)
{
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    s32_t res;

    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if ((fd->flags & SPIFFS_O_RDONLY) == 0)
    {
        res = SPIFFS_ERR_NOT_READABLE;
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

    if (fd->size == SPIFFS_UNDEFINED_LEN && len > 0)
    {
        // special case for zero sized files
        res = SPIFFS_ERR_END_OF_OBJECT;
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

#if SPIFFS_CACHE_WR
    spiffs_fflush_cache(fs, fh);
#endif

    if (fd->fdoffset + len >= fd->size)
    {
        // reading beyond file size
        s32_t avail = fd->size - fd->fdoffset;
        if (avail <= 0)
        {
            SPIFFS_API_CHECK_RES_UNLOCK(fs, SPIFFS_ERR_END_OF_OBJECT);
        }
        res = spiffs_object_read(fd, fd->fdoffset, avail, (u8_t *)buf);
        if (res == SPIFFS_ERR_END_OF_OBJECT)
        {
            fd->fdoffset += avail;
            SPIFFS_UNLOCK(fs);
            return avail;
        }
        else
        {
            SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
            len = avail;
        }
    }
    else
    {
        // reading within file size
        res = spiffs_object_read(fd, fd->fdoffset, len, (u8_t *)buf);
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }
    fd->fdoffset += len;

    SPIFFS_UNLOCK(fs);

    return len;
}

s32_t SPIFFS_read(spiffs *fs, spiffs_file fh, void *buf, s32_t len)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd " "_SPIPRIi "\n", __func__, fh, len);
    s32_t res = spiffs_hydro_read(fs, fh, buf, len);
    if (res == SPIFFS_ERR_END_OF_OBJECT)
    {
        res = 0;
    }
    return res;
}


#if !SPIFFS_READ_ONLY
static s32_t spiffs_hydro_write(spiffs *fs, spiffs_fd *fd, void *buf, u32_t offset, s32_t len)
{
    (void)fs;
    s32_t res = SPIFFS_OK;
    s32_t remaining = len;
    if (fd->size != SPIFFS_UNDEFINED_LEN && offset < fd->size)
    {
        s32_t m_len = MIN((s32_t)(fd->size - offset), len);
        res = spiffs_object_modify(fd, offset, (u8_t *)buf, m_len);
        SPIFFS_CHECK_RES(res);
        remaining -= m_len;
        u8_t *buf_8 = (u8_t *)buf;
        buf_8 += m_len;
        buf = buf_8;
        offset += m_len;
    }
    if (remaining > 0)
    {
        res = spiffs_object_append(fd, offset, (u8_t *)buf, remaining);
        SPIFFS_CHECK_RES(res);
    }
    return len;

}
#endif // !SPIFFS_READ_ONLY

s32_t SPIFFS_write(spiffs *fs, spiffs_file fh, void *buf, s32_t len)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd " "_SPIPRIi "\n", __func__, fh, len);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)fh;
    (void)buf;
    (void)len;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    s32_t res;
    u32_t offset;

    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if ((fd->flags & SPIFFS_O_WRONLY) == 0)
    {
        res = SPIFFS_ERR_NOT_WRITABLE;
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

    if ((fd->flags & SPIFFS_O_APPEND))
    {
        fd->fdoffset = fd->size == SPIFFS_UNDEFINED_LEN ? 0 : fd->size;
    }
    offset = fd->fdoffset;

#if SPIFFS_CACHE_WR
    if (fd->cache_page == 0)
    {
        // see if object id is associated with cache already
        fd->cache_page = spiffs_cache_page_get_by_fd(fs, fd);
    }
#endif
    if (fd->flags & SPIFFS_O_APPEND)
    {
        if (fd->size == SPIFFS_UNDEFINED_LEN)
        {
            offset = 0;
        }
        else
        {
            offset = fd->size;
        }
#if SPIFFS_CACHE_WR
        if (fd->cache_page)
        {
            offset = MAX(offset, fd->cache_page->offset + fd->cache_page->size);
        }
#endif
    }

#if SPIFFS_CACHE_WR
    if ((fd->flags & SPIFFS_O_DIRECT) == 0)
    {
        if (len < (s32_t)SPIFFS_CFG_LOG_PAGE_SZ(fs))
        {
            // small write, try to cache it
            u8_t alloc_cpage = 1;
            if (fd->cache_page)
            {
                // have a cached page for this fd already, check cache page boundaries
                if (offset < fd->cache_page->offset || // writing before cache
                        offset > fd->cache_page->offset + fd->cache_page->size || // writing after cache
                        offset + len > fd->cache_page->offset + SPIFFS_CFG_LOG_PAGE_SZ(fs)) // writing beyond cache page
                {
                    // boundary violation, write back cache first and allocate new
                    SPIFFS_CACHE_DBG("CACHE_WR_DUMP: dumping cache page "_SPIPRIi" for fd "_SPIPRIfd":"_SPIPRIid", boundary viol, offs:"_SPIPRIi" size:"_SPIPRIi"\n",
                                     fd->cache_page->ix, fd->file_nbr, fd->obj_id, fd->cache_page->offset, fd->cache_page->size);
                    res = spiffs_hydro_write(fs, fd,
                                             spiffs_get_cache_page(fs, spiffs_get_cache(fs), fd->cache_page->ix),
                                             fd->cache_page->offset, fd->cache_page->size);
                    spiffs_cache_fd_release(fs, fd->cache_page);
                    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
                }
                else
                {
                    // writing within cache
                    alloc_cpage = 0;
                }
            }

            if (alloc_cpage)
            {
                fd->cache_page = spiffs_cache_page_allocate_by_fd(fs, fd);
                if (fd->cache_page)
                {
                    fd->cache_page->offset = offset;
                    fd->cache_page->size = 0;
                    SPIFFS_CACHE_DBG("CACHE_WR_ALLO: allocating cache page "_SPIPRIi" for fd "_SPIPRIfd":"_SPIPRIid"\n",
                                     fd->cache_page->ix, fd->file_nbr, fd->obj_id);
                }
            }

            if (fd->cache_page)
            {
                u32_t offset_in_cpage = offset - fd->cache_page->offset;
                SPIFFS_CACHE_DBG("CACHE_WR_WRITE: storing to cache page "_SPIPRIi" for fd "_SPIPRIfd":"_SPIPRIid", offs "_SPIPRIi":"_SPIPRIi" len "_SPIPRIi"\n",
                                 fd->cache_page->ix, fd->file_nbr, fd->obj_id,
                                 offset, offset_in_cpage, len);
                spiffs_cache *cache = spiffs_get_cache(fs);
                u8_t *cpage_data = spiffs_get_cache_page(fs, cache, fd->cache_page->ix);
#ifdef _SPIFFS_TEST
                {
                    intptr_t __a1 = (u8_t *)&cpage_data[offset_in_cpage] - (u8_t *)cache;
                    intptr_t __a2 = (u8_t *)&cpage_data[offset_in_cpage] + len - (u8_t *)cache;
                    intptr_t __b = sizeof(spiffs_cache) + cache->cpage_count * (sizeof(spiffs_cache_page) + SPIFFS_CFG_LOG_PAGE_SZ(fs));
                    if (__a1 > __b || __a2 > __b)
                    {
                        printf("FATAL OOB: CACHE_WR: memcpy to cache buffer ixs:%4ld..%4ld of %4ld\n", __a1, __a2, __b);
                        ERREXIT();
                    }
                }
#endif
                _SPIFFS_MEMCPY(&cpage_data[offset_in_cpage], buf, len);
                fd->cache_page->size = MAX(fd->cache_page->size, offset_in_cpage + len);
                fd->fdoffset += len;
                SPIFFS_UNLOCK(fs);
                return len;
            }
            else
            {
                res = spiffs_hydro_write(fs, fd, buf, offset, len);
                SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
                fd->fdoffset += len;
                SPIFFS_UNLOCK(fs);
                return res;
            }
        }
        else
        {
            // big write, no need to cache it - but first check if there is a cached write already
            if (fd->cache_page)
            {
                // write back cache first
                SPIFFS_CACHE_DBG("CACHE_WR_DUMP: dumping cache page "_SPIPRIi" for fd "_SPIPRIfd":"_SPIPRIid", big write, offs:"_SPIPRIi" size:"_SPIPRIi"\n",
                                 fd->cache_page->ix, fd->file_nbr, fd->obj_id, fd->cache_page->offset, fd->cache_page->size);
                res = spiffs_hydro_write(fs, fd,
                                         spiffs_get_cache_page(fs, spiffs_get_cache(fs), fd->cache_page->ix),
                                         fd->cache_page->offset, fd->cache_page->size);
                spiffs_cache_fd_release(fs, fd->cache_page);
                SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
                // data written below
            }
        }
    }
#endif

    res = spiffs_hydro_write(fs, fd, buf, offset, len);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    fd->fdoffset += len;

    SPIFFS_UNLOCK(fs);

    return res;
#endif // SPIFFS_READ_ONLY
}

s32_t SPIFFS_lseek(spiffs *fs, spiffs_file fh, s32_t offs, int whence)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd " "_SPIPRIi " %s\n", __func__, fh, offs, (const char *[])
    {"SET", "CUR", "END", "???"
    }[MIN(whence, 3)]);
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    s32_t res;
    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

#if SPIFFS_CACHE_WR
    spiffs_fflush_cache(fs, fh);
#endif

    s32_t file_size = fd->size == SPIFFS_UNDEFINED_LEN ? 0 : fd->size;

    switch (whence)
    {
    case SPIFFS_SEEK_SET:
        break;
    case SPIFFS_SEEK_CUR:
        offs = fd->fdoffset + offs;
        break;
    case SPIFFS_SEEK_END:
        offs = file_size + offs;
        break;
    default:
        res = SPIFFS_ERR_INVALID_PARA;
        break;
    }
    if (offs < 0)
    {
        SPIFFS_API_CHECK_RES_UNLOCK(fs, SPIFFS_ERR_INVALID_PARA);
    }
    if (offs > file_size)
    {
        fd->fdoffset = file_size;
        res = SPIFFS_ERR_SEEK_BOUNDS;
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    spiffs_span_ix data_spix = (offs > 0 ? (offs - 1) : 0) / SPIFFS_DATA_PAGE_SIZE(fs);
    spiffs_span_ix objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);
    if (fd->cursor_objix_spix != objix_spix)
    {
        spiffs_page_ix pix;
        res = spiffs_obj_lu_find_id_and_span(
                  fs, fd->obj_id | SPIFFS_OBJ_ID_IX_FLAG, objix_spix, 0, &pix);
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        fd->cursor_objix_spix = objix_spix;
        fd->cursor_objix_pix = pix;
    }
    fd->fdoffset = offs;

    SPIFFS_UNLOCK(fs);

    return offs;
}

s32_t SPIFFS_remove(spiffs *fs, const char *path)
{
    SPIFFS_API_DBG("%s '%s'\n", __func__, path);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)path;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    if (strlen(path) > SPIFFS_OBJ_NAME_LEN - 1)
    {
        SPIFFS_API_CHECK_RES(fs, SPIFFS_ERR_NAME_TOO_LONG);
    }
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    spiffs_page_ix pix;
    s32_t res;

    res = spiffs_fd_find_new(fs, &fd, 0);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_find_object_index_header_by_name(fs, (const u8_t *)path, &pix);
    if (res != SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_open_by_page(fs, pix, fd, 0, 0);
    if (res != SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_truncate(fd, 0, 1);
    if (res != SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);
    return 0;
#endif // SPIFFS_READ_ONLY
}

s32_t SPIFFS_fremove(spiffs *fs, spiffs_file fh)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)fh;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    s32_t res;
    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if ((fd->flags & SPIFFS_O_WRONLY) == 0)
    {
        res = SPIFFS_ERR_NOT_WRITABLE;
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

#if SPIFFS_CACHE_WR
    spiffs_cache_fd_release(fs, fd->cache_page);
#endif

    res = spiffs_object_truncate(fd, 0, 1);

    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);

    return 0;
#endif // SPIFFS_READ_ONLY
}

static s32_t spiffs_stat_pix(spiffs *fs, spiffs_page_ix pix, spiffs_file fh, spiffs_stat *s)
{
    (void)fh;
    spiffs_page_object_ix_header objix_hdr;
    spiffs_obj_id obj_id;
    s32_t res = _spiffs_rd(fs,  SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ, fh,
                           SPIFFS_PAGE_TO_PADDR(fs, pix), sizeof(spiffs_page_object_ix_header), (u8_t *)&objix_hdr);
    SPIFFS_API_CHECK_RES(fs, res);

    u32_t obj_id_addr = SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs , pix)) +
                        SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pix) * sizeof(spiffs_obj_id);
    res = _spiffs_rd(fs,  SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, fh,
                     obj_id_addr, sizeof(spiffs_obj_id), (u8_t *)&obj_id);
    SPIFFS_API_CHECK_RES(fs, res);

    s->obj_id = obj_id & ~SPIFFS_OBJ_ID_IX_FLAG;
    s->type = objix_hdr.type;
    s->size = objix_hdr.size == SPIFFS_UNDEFINED_LEN ? 0 : objix_hdr.size;
    s->pix = pix;
    strncpy((char *)s->name, (char *)objix_hdr.name, SPIFFS_OBJ_NAME_LEN);
#if SPIFFS_OBJ_META_LEN
    _SPIFFS_MEMCPY(s->meta, objix_hdr.meta, SPIFFS_OBJ_META_LEN);
#endif

    return res;
}

s32_t SPIFFS_stat(spiffs *fs, const char *path, spiffs_stat *s)
{
    SPIFFS_API_DBG("%s '%s'\n", __func__, path);
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    if (strlen(path) > SPIFFS_OBJ_NAME_LEN - 1)
    {
        SPIFFS_API_CHECK_RES(fs, SPIFFS_ERR_NAME_TOO_LONG);
    }
    SPIFFS_LOCK(fs);

    s32_t res;
    spiffs_page_ix pix;

    res = spiffs_object_find_object_index_header_by_name(fs, (const u8_t *)path, &pix);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_stat_pix(fs, pix, 0, s);

    SPIFFS_UNLOCK(fs);

    return res;
}

s32_t SPIFFS_fstat(spiffs *fs, spiffs_file fh, spiffs_stat *s)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_fd *fd;
    s32_t res;

    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

#if SPIFFS_CACHE_WR
    spiffs_fflush_cache(fs, fh);
#endif

    res = spiffs_stat_pix(fs, fd->objix_hdr_pix, fh, s);

    SPIFFS_UNLOCK(fs);

    return res;
}

// Checks if there are any cached writes for the object id associated with
// given filehandle. If so, these writes are flushed.
#if SPIFFS_CACHE == 1
static s32_t spiffs_fflush_cache(spiffs *fs, spiffs_file fh)
{
    (void)fs;
    (void)fh;
    s32_t res = SPIFFS_OK;
#if !SPIFFS_READ_ONLY && SPIFFS_CACHE_WR

    spiffs_fd *fd;
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES(fs, res);

    if ((fd->flags & SPIFFS_O_DIRECT) == 0)
    {
        if (fd->cache_page == 0)
        {
            // see if object id is associated with cache already
            fd->cache_page = spiffs_cache_page_get_by_fd(fs, fd);
        }
        if (fd->cache_page)
        {
            SPIFFS_CACHE_DBG("CACHE_WR_DUMP: dumping cache page "_SPIPRIi" for fd "_SPIPRIfd":"_SPIPRIid", flush, offs:"_SPIPRIi" size:"_SPIPRIi"\n",
                             fd->cache_page->ix, fd->file_nbr,  fd->obj_id, fd->cache_page->offset, fd->cache_page->size);
            res = spiffs_hydro_write(fs, fd,
                                     spiffs_get_cache_page(fs, spiffs_get_cache(fs), fd->cache_page->ix),
                                     fd->cache_page->offset, fd->cache_page->size);
            if (res < SPIFFS_OK)
            {
                fs->err_code = res;
            }
            spiffs_cache_fd_release(fs, fd->cache_page);
        }
    }
#endif

    return res;
}
#endif

s32_t SPIFFS_fflush(spiffs *fs, spiffs_file fh)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
    (void)fh;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    s32_t res = SPIFFS_OK;
#if !SPIFFS_READ_ONLY && SPIFFS_CACHE_WR
    SPIFFS_LOCK(fs);
    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fflush_cache(fs, fh);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    SPIFFS_UNLOCK(fs);
#endif

    return res;
}

s32_t SPIFFS_close(spiffs *fs, spiffs_file fh)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);

    s32_t res = SPIFFS_OK;
    SPIFFS_LOCK(fs);

    fh = SPIFFS_FH_UNOFFS(fs, fh);
#if SPIFFS_CACHE
    res = spiffs_fflush_cache(fs, fh);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
#endif
    res = spiffs_fd_return(fs, fh);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);

    return res;
}

s32_t SPIFFS_rename(spiffs *fs, const char *old_path, const char *new_path)
{
    SPIFFS_API_DBG("%s %s %s\n", __func__, old_path, new_path);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)old_path;
    (void)new_path;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    if (strlen(new_path) > SPIFFS_OBJ_NAME_LEN - 1 ||
            strlen(old_path) > SPIFFS_OBJ_NAME_LEN - 1)
    {
        SPIFFS_API_CHECK_RES(fs, SPIFFS_ERR_NAME_TOO_LONG);
    }
    SPIFFS_LOCK(fs);

    spiffs_page_ix pix_old, pix_dummy;
    spiffs_fd *fd;

    s32_t res = spiffs_object_find_object_index_header_by_name(fs, (const u8_t *)old_path, &pix_old);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_find_object_index_header_by_name(fs, (const u8_t *)new_path, &pix_dummy);
    if (res == SPIFFS_ERR_NOT_FOUND)
    {
        res = SPIFFS_OK;
    }
    else if (res == SPIFFS_OK)
    {
        res = SPIFFS_ERR_CONFLICTING_NAME;
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_fd_find_new(fs, &fd, 0);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_open_by_page(fs, pix_old, fd, 0, 0);
    if (res != SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id, fd->objix_hdr_pix, 0, (const u8_t *)new_path,
                                         0, 0, &pix_dummy);
#if SPIFFS_TEMPORAL_FD_CACHE
    if (res == SPIFFS_OK)
    {
        spiffs_fd_temporal_cache_rehash(fs, old_path, new_path);
    }
#endif

    spiffs_fd_return(fs, fd->file_nbr);

    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);

    return res;
#endif // SPIFFS_READ_ONLY
}

#if SPIFFS_OBJ_META_LEN
s32_t SPIFFS_update_meta(spiffs *fs, const char *name, const void *meta)
{
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)name;
    (void)meta;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    spiffs_page_ix pix, pix_dummy;
    spiffs_fd *fd;

    s32_t res = spiffs_object_find_object_index_header_by_name(fs, (const u8_t *)name, &pix);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_fd_find_new(fs, &fd, 0);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_open_by_page(fs, pix, fd, 0, 0);
    if (res != SPIFFS_OK)
    {
        spiffs_fd_return(fs, fd->file_nbr);
    }
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id, fd->objix_hdr_pix, 0, 0, meta,
                                         0, &pix_dummy);

    spiffs_fd_return(fs, fd->file_nbr);

    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);

    return res;
#endif // SPIFFS_READ_ONLY
}

s32_t SPIFFS_fupdate_meta(spiffs *fs, spiffs_file fh, const void *meta)
{
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)fh;
    (void)meta;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    s32_t res;
    spiffs_fd *fd;
    spiffs_page_ix pix_dummy;

    fh = SPIFFS_FH_UNOFFS(fs, fh);
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if ((fd->flags & SPIFFS_O_WRONLY) == 0)
    {
        res = SPIFFS_ERR_NOT_WRITABLE;
        SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    }

    res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id, fd->objix_hdr_pix, 0, 0, meta,
                                         0, &pix_dummy);

    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);

    return res;
#endif // SPIFFS_READ_ONLY
}
#endif // SPIFFS_OBJ_META_LEN

spiffs_DIR *SPIFFS_opendir(spiffs *fs, const char *name, spiffs_DIR *d)
{
    SPIFFS_API_DBG("%s\n", __func__);
    (void)name;

    if (!SPIFFS_CHECK_CFG((fs)))
    {
        (fs)->err_code = SPIFFS_ERR_NOT_CONFIGURED;
        return 0;
    }

    if (!SPIFFS_CHECK_MOUNT(fs))
    {
        fs->err_code = SPIFFS_ERR_NOT_MOUNTED;
        return 0;
    }

    d->fs = fs;
    d->block = 0;
    d->entry = 0;
    return d;
}

static s32_t spiffs_read_dir_v(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_block_ix bix,
    int ix_entry,
    const void *user_const_p,
    void *user_var_p)
{
    (void)user_const_p;
    s32_t res;
    spiffs_page_object_ix_header objix_hdr;
    if (obj_id == SPIFFS_OBJ_ID_FREE || obj_id == SPIFFS_OBJ_ID_DELETED ||
            (obj_id & SPIFFS_OBJ_ID_IX_FLAG) == 0)
    {
        return SPIFFS_VIS_COUNTINUE;
    }

    spiffs_page_ix pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, ix_entry);
    res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                     0, SPIFFS_PAGE_TO_PADDR(fs, pix), sizeof(spiffs_page_object_ix_header), (u8_t *)&objix_hdr);
    if (res != SPIFFS_OK) return res;
    if ((obj_id & SPIFFS_OBJ_ID_IX_FLAG) &&
            objix_hdr.p_hdr.span_ix == 0 &&
            (objix_hdr.p_hdr.flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_IXDELE)) ==
            (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE))
    {
        struct spiffs_dirent *e = (struct spiffs_dirent *)user_var_p;
        e->obj_id = obj_id;
        strcpy((char *)e->name, (char *)objix_hdr.name);
        e->type = objix_hdr.type;
        e->size = objix_hdr.size == SPIFFS_UNDEFINED_LEN ? 0 : objix_hdr.size;
        e->pix = pix;
#if SPIFFS_OBJ_META_LEN
        _SPIFFS_MEMCPY(e->meta, objix_hdr.meta, SPIFFS_OBJ_META_LEN);
#endif
        return SPIFFS_OK;
    }
    return SPIFFS_VIS_COUNTINUE;
}

struct spiffs_dirent *SPIFFS_readdir(spiffs_DIR *d, struct spiffs_dirent *e)
{
    SPIFFS_API_DBG("%s\n", __func__);
    if (!SPIFFS_CHECK_MOUNT(d->fs))
    {
        d->fs->err_code = SPIFFS_ERR_NOT_MOUNTED;
        return 0;
    }
    SPIFFS_LOCK(d->fs);

    spiffs_block_ix bix;
    int entry;
    s32_t res;
    struct spiffs_dirent *ret = 0;

    res = spiffs_obj_lu_find_entry_visitor(d->fs,
                                           d->block,
                                           d->entry,
                                           SPIFFS_VIS_NO_WRAP,
                                           0,
                                           spiffs_read_dir_v,
                                           0,
                                           e,
                                           &bix,
                                           &entry);
    if (res == SPIFFS_OK)
    {
        d->block = bix;
        d->entry = entry + 1;
        e->obj_id &= ~SPIFFS_OBJ_ID_IX_FLAG;
        ret = e;
    }
    else
    {
        d->fs->err_code = res;
    }
    SPIFFS_UNLOCK(d->fs);
    return ret;
}

s32_t SPIFFS_closedir(spiffs_DIR *d)
{
    SPIFFS_API_DBG("%s\n", __func__);
    SPIFFS_API_CHECK_CFG(d->fs);
    SPIFFS_API_CHECK_MOUNT(d->fs);
    return 0;
}

s32_t SPIFFS_check(spiffs *fs)
{
    SPIFFS_API_DBG("%s\n", __func__);
#if SPIFFS_READ_ONLY
    (void)fs;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    res = spiffs_lookup_consistency_check(fs, 0);

    res = spiffs_object_index_consistency_check(fs);

    res = spiffs_page_consistency_check(fs);

    res = spiffs_obj_lu_scan(fs);

    SPIFFS_UNLOCK(fs);
    return res;
#endif // SPIFFS_READ_ONLY
}

s32_t SPIFFS_info(spiffs *fs, u32_t *total, u32_t *used)
{
    SPIFFS_API_DBG("%s\n", __func__);
    s32_t res = SPIFFS_OK;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    u32_t pages_per_block = SPIFFS_PAGES_PER_BLOCK(fs);
    u32_t blocks = fs->block_count;
    u32_t obj_lu_pages = SPIFFS_OBJ_LOOKUP_PAGES(fs);
    u32_t data_page_size = SPIFFS_DATA_PAGE_SIZE(fs);
    u32_t total_data_pages = (blocks - 2) * (pages_per_block - obj_lu_pages) + 1; // -2 for spare blocks, +1 for emergency page

    if (total)
    {
        *total = total_data_pages * data_page_size;
    }

    if (used)
    {
        *used = fs->stats_p_allocated * data_page_size;
    }

    SPIFFS_UNLOCK(fs);
    return res;
}

s32_t SPIFFS_gc_quick(spiffs *fs, u16_t max_free_pages)
{
    SPIFFS_API_DBG("%s "_SPIPRIi "\n", __func__, max_free_pages);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)max_free_pages;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    res = spiffs_gc_quick(fs, max_free_pages);

    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    SPIFFS_UNLOCK(fs);
    return 0;
#endif // SPIFFS_READ_ONLY
}


s32_t SPIFFS_gc(spiffs *fs, u32_t size)
{
    SPIFFS_API_DBG("%s "_SPIPRIi "\n", __func__, size);
#if SPIFFS_READ_ONLY
    (void)fs;
    (void)size;
    return SPIFFS_ERR_RO_NOT_IMPL;
#else
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    res = spiffs_gc_check(fs, size);

    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
    SPIFFS_UNLOCK(fs);
    return 0;
#endif // SPIFFS_READ_ONLY
}

s32_t SPIFFS_eof(spiffs *fs, spiffs_file fh)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    fh = SPIFFS_FH_UNOFFS(fs, fh);

    spiffs_fd *fd;
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

#if SPIFFS_CACHE_WR
    res = spiffs_fflush_cache(fs, fh);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
#endif

    res = (fd->fdoffset >= (fd->size == SPIFFS_UNDEFINED_LEN ? 0 : fd->size));

    SPIFFS_UNLOCK(fs);
    return res;
}

s32_t SPIFFS_tell(spiffs *fs, spiffs_file fh)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    fh = SPIFFS_FH_UNOFFS(fs, fh);

    spiffs_fd *fd;
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

#if SPIFFS_CACHE_WR
    res = spiffs_fflush_cache(fs, fh);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
#endif

    res = fd->fdoffset;

    SPIFFS_UNLOCK(fs);
    return res;
}

s32_t SPIFFS_set_file_callback_func(spiffs *fs, spiffs_file_callback cb_func)
{
    SPIFFS_API_DBG("%s\n", __func__);
    SPIFFS_LOCK(fs);
    fs->file_cb_f = cb_func;
    SPIFFS_UNLOCK(fs);
    return 0;
}

#if SPIFFS_IX_MAP

s32_t SPIFFS_ix_map(spiffs *fs,  spiffs_file fh, spiffs_ix_map *map,
                    u32_t offset, u32_t len, spiffs_page_ix *map_buf)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd " "_SPIPRIi " "_SPIPRIi "\n", __func__, fh, offset, len);
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    fh = SPIFFS_FH_UNOFFS(fs, fh);

    spiffs_fd *fd;
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if (fd->ix_map)
    {
        SPIFFS_API_CHECK_RES_UNLOCK(fs, SPIFFS_ERR_IX_MAP_MAPPED);
    }

    map->map_buf = map_buf;
    map->offset = offset;
    // nb: spix range includes last
    map->start_spix = offset / SPIFFS_DATA_PAGE_SIZE(fs);
    map->end_spix = (offset + len) / SPIFFS_DATA_PAGE_SIZE(fs);
    memset(map_buf, 0, sizeof(spiffs_page_ix) * (map->end_spix - map->start_spix + 1));
    fd->ix_map = map;

    // scan for pixes
    res = spiffs_populate_ix_map(fs, fd, 0, map->end_spix - map->start_spix + 1);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    SPIFFS_UNLOCK(fs);
    return res;
}

s32_t SPIFFS_ix_unmap(spiffs *fs,  spiffs_file fh)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd "\n", __func__, fh);
    s32_t res;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    fh = SPIFFS_FH_UNOFFS(fs, fh);

    spiffs_fd *fd;
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if (fd->ix_map == 0)
    {
        SPIFFS_API_CHECK_RES_UNLOCK(fs, SPIFFS_ERR_IX_MAP_UNMAPPED);
    }

    fd->ix_map = 0;

    SPIFFS_UNLOCK(fs);
    return res;
}

s32_t SPIFFS_ix_remap(spiffs *fs, spiffs_file fh, u32_t offset)
{
    SPIFFS_API_DBG("%s "_SPIPRIfd " "_SPIPRIi "\n", __func__, fh, offset);
    s32_t res = SPIFFS_OK;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    fh = SPIFFS_FH_UNOFFS(fs, fh);

    spiffs_fd *fd;
    res = spiffs_fd_get(fs, fh, &fd);
    SPIFFS_API_CHECK_RES_UNLOCK(fs, res);

    if (fd->ix_map == 0)
    {
        SPIFFS_API_CHECK_RES_UNLOCK(fs, SPIFFS_ERR_IX_MAP_UNMAPPED);
    }

    spiffs_ix_map *map = fd->ix_map;

    s32_t spix_diff = offset / SPIFFS_DATA_PAGE_SIZE(fs) - map->start_spix;
    map->offset = offset;

    // move existing pixes if within map offs
    if (spix_diff != 0)
    {
        // move vector
        int i;
        const s32_t vec_len = map->end_spix - map->start_spix + 1; // spix range includes last
        map->start_spix += spix_diff;
        map->end_spix += spix_diff;
        if (spix_diff >= vec_len)
        {
            // moving beyond range
            memset(&map->map_buf, 0, vec_len * sizeof(spiffs_page_ix));
            // populate_ix_map is inclusive
            res = spiffs_populate_ix_map(fs, fd, 0, vec_len - 1);
            SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        }
        else if (spix_diff > 0)
        {
            // diff positive
            for (i = 0; i < vec_len - spix_diff; i++)
            {
                map->map_buf[i] = map->map_buf[i + spix_diff];
            }
            // memset is non-inclusive
            memset(&map->map_buf[vec_len - spix_diff], 0, spix_diff * sizeof(spiffs_page_ix));
            // populate_ix_map is inclusive
            res = spiffs_populate_ix_map(fs, fd, vec_len - spix_diff, vec_len - 1);
            SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        }
        else
        {
            // diff negative
            for (i = vec_len - 1; i >= -spix_diff; i--)
            {
                map->map_buf[i] = map->map_buf[i + spix_diff];
            }
            // memset is non-inclusive
            memset(&map->map_buf[0], 0, -spix_diff * sizeof(spiffs_page_ix));
            // populate_ix_map is inclusive
            res = spiffs_populate_ix_map(fs, fd, 0, -spix_diff - 1);
            SPIFFS_API_CHECK_RES_UNLOCK(fs, res);
        }

    }

    SPIFFS_UNLOCK(fs);
    return res;
}

s32_t SPIFFS_bytes_to_ix_map_entries(spiffs *fs, u32_t bytes)
{
    SPIFFS_API_CHECK_CFG(fs);
    // always add one extra page, the offset might change to the middle of a page
    return (bytes + SPIFFS_DATA_PAGE_SIZE(fs) ) / SPIFFS_DATA_PAGE_SIZE(fs);
}

s32_t SPIFFS_ix_map_entries_to_bytes(spiffs *fs, u32_t map_page_ix_entries)
{
    SPIFFS_API_CHECK_CFG(fs);
    return map_page_ix_entries * SPIFFS_DATA_PAGE_SIZE(fs);
}

#endif // SPIFFS_IX_MAP

#if SPIFFS_TEST_VISUALISATION
s32_t SPIFFS_vis(spiffs *fs)
{
    s32_t res = SPIFFS_OK;
    SPIFFS_API_CHECK_CFG(fs);
    SPIFFS_API_CHECK_MOUNT(fs);
    SPIFFS_LOCK(fs);

    int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(spiffs_obj_id));
    spiffs_obj_id *obj_lu_buf = (spiffs_obj_id *)fs->lu_work;
    spiffs_block_ix bix = 0;

    while (bix < fs->block_count)
    {
        // check each object lookup page
        int obj_lookup_page = 0;
        int cur_entry = 0;

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
                if (cur_entry == 0)
                {
                    spiffs_printf(_SPIPRIbl" ", bix);
                }
                else if ((cur_entry & 0x3f) == 0)
                {
                    spiffs_printf("     ");
                }
                if (obj_id == SPIFFS_OBJ_ID_FREE)
                {
                    spiffs_printf(SPIFFS_TEST_VIS_FREE_STR);
                }
                else if (obj_id == SPIFFS_OBJ_ID_DELETED)
                {
                    spiffs_printf(SPIFFS_TEST_VIS_DELE_STR);
                }
                else if (obj_id & SPIFFS_OBJ_ID_IX_FLAG)
                {
                    spiffs_printf(SPIFFS_TEST_VIS_INDX_STR(obj_id));
                }
                else
                {
                    spiffs_printf(SPIFFS_TEST_VIS_DATA_STR(obj_id));
                }
                cur_entry++;
                if ((cur_entry & 0x3f) == 0)
                {
                    spiffs_printf("\n");
                }
            } // per entry
            obj_lookup_page++;
        } // per object lookup page

        spiffs_obj_id erase_count;
        res = _spiffs_rd(fs, SPIFFS_OP_C_READ | SPIFFS_OP_T_OBJ_LU2, 0,
                         SPIFFS_ERASE_COUNT_PADDR(fs, bix),
                         sizeof(spiffs_obj_id), (u8_t *)&erase_count);
        SPIFFS_CHECK_RES(res);

        if (erase_count != (spiffs_obj_id) - 1)
        {
            spiffs_printf("\tera_cnt: %d\n", erase_count);
        }
        else
        {
            spiffs_printf("\tera_cnt: N/A\n");
        }

        bix++;
    } // per block

    spiffs_printf("era_cnt_max: %d\n", fs->max_erase_count);
    spiffs_printf("last_errno:  "_SPIPRIi"\n", fs->err_code);
    spiffs_printf("blocks:      "_SPIPRIi"\n", fs->block_count);
    spiffs_printf("free_blocks: "_SPIPRIi"\n", fs->free_blocks);
    spiffs_printf("page_alloc:  "_SPIPRIi"\n", fs->stats_p_allocated);
    spiffs_printf("page_delet:  "_SPIPRIi"\n", fs->stats_p_deleted);
    SPIFFS_UNLOCK(fs);
    u32_t total, used;
    SPIFFS_info(fs, &total, &used);
    spiffs_printf("used:        "_SPIPRIi" of "_SPIPRIi"\n", used, total);
    return res;
}
#endif
