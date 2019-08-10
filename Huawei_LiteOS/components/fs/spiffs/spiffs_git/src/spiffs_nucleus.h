/*
 * spiffs_nucleus.h
 *
 *  Created on: Jun 15, 2013
 *      Author: petera
 */

/* SPIFFS layout
 *
 * spiffs is designed for following spi flash characteristics:
 *   - only big areas of data (blocks) can be erased
 *   - erasing resets all bits in a block to ones
 *   - writing pulls ones to zeroes
 *   - zeroes cannot be pulled to ones, without erase
 *   - wear leveling
 *
 * spiffs is also meant to be run on embedded, memory constraint devices.
 *
 * Entire area is divided in blocks. Entire area is also divided in pages.
 * Each block contains same number of pages. A page cannot be erased, but a
 * block can be erased.
 *
 * Entire area must be block_size * x
 * page_size must be block_size / (2^y) where y > 2
 *
 * ex: area = 1024*1024 bytes, block size = 65536 bytes, page size = 256 bytes
 *
 * BLOCK 0  PAGE 0       object lookup 1
 *          PAGE 1       object lookup 2
 *          ...
 *          PAGE n-1     object lookup n
 *          PAGE n       object data 1
 *          PAGE n+1     object data 2
 *          ...
 *          PAGE n+m-1   object data m
 *
 * BLOCK 1  PAGE n+m     object lookup 1
 *          PAGE n+m+1   object lookup 2
 *          ...
 *          PAGE 2n+m-1  object lookup n
 *          PAGE 2n+m    object data 1
 *          PAGE 2n+m    object data 2
 *          ...
 *          PAGE 2n+2m-1 object data m
 * ...
 *
 * n is number of object lookup pages, which is number of pages needed to index all pages
 * in a block by object id
 *   : block_size / page_size * sizeof(obj_id) / page_size
 * m is number data pages, which is number of pages in block minus number of lookup pages
 *   : block_size / page_size - block_size / page_size * sizeof(obj_id) / page_size
 * thus, n+m is total number of pages in a block
 *   : block_size / page_size
 *
 * ex: n = 65536/256*2/256 = 2, m = 65536/256 - 2 = 254 => n+m = 65536/256 = 256
 *
 * Object lookup pages contain object id entries. Each entry represent the corresponding
 * data page.
 * Assuming a 16 bit object id, an object id being 0xffff represents a free page.
 * An object id being 0x0000 represents a deleted page.
 *
 * ex: page 0 : lookup : 0008 0001 0aaa ffff ffff ffff ffff ffff ..
 *     page 1 : lookup : ffff ffff ffff ffff ffff ffff ffff ffff ..
 *     page 2 : data   : data for object id 0008
 *     page 3 : data   : data for object id 0001
 *     page 4 : data   : data for object id 0aaa
 *     ...
 *
 *
 * Object data pages can be either object index pages or object content.
 * All object data pages contains a data page header, containing object id and span index.
 * The span index denotes the object page ordering amongst data pages with same object id.
 * This applies to both object index pages (when index spans more than one page of entries),
 * and object data pages.
 * An object index page contains page entries pointing to object content page. The entry index
 * in a object index page correlates to the span index in the actual object data page.
 * The first object index page (span index 0) is called object index header page, and also
 * contains object flags (directory/file), size, object name etc.
 *
 * ex:
 *  BLOCK 1
 *    PAGE 256: objectl lookup page 1
 *      [*123] [ 123] [ 123] [ 123]
 *      [ 123] [*123] [ 123] [ 123]
 *      [free] [free] [free] [free] ...
 *    PAGE 257: objectl lookup page 2
 *      [free] [free] [free] [free] ...
 *    PAGE 258: object index page (header)
 *      obj.id:0123 span.ix:0000 flags:INDEX
 *      size:1600 name:ex.txt type:file
 *      [259] [260] [261] [262]
 *    PAGE 259: object data page
 *      obj.id:0123 span.ix:0000 flags:DATA
 *    PAGE 260: object data page
 *      obj.id:0123 span.ix:0001 flags:DATA
 *    PAGE 261: object data page
 *      obj.id:0123 span.ix:0002 flags:DATA
 *    PAGE 262: object data page
 *      obj.id:0123 span.ix:0003 flags:DATA
 *    PAGE 263: object index page
 *      obj.id:0123 span.ix:0001 flags:INDEX
 *      [264] [265] [fre] [fre]
 *      [fre] [fre] [fre] [fre]
 *    PAGE 264: object data page
 *      obj.id:0123 span.ix:0004 flags:DATA
 *    PAGE 265: object data page
 *      obj.id:0123 span.ix:0005 flags:DATA
 *
 */
#ifndef SPIFFS_NUCLEUS_H_
#define SPIFFS_NUCLEUS_H_

#define _SPIFFS_ERR_CHECK_FIRST         (SPIFFS_ERR_INTERNAL - 1)
#define SPIFFS_ERR_CHECK_OBJ_ID_MISM    (SPIFFS_ERR_INTERNAL - 1)
#define SPIFFS_ERR_CHECK_SPIX_MISM      (SPIFFS_ERR_INTERNAL - 2)
#define SPIFFS_ERR_CHECK_FLAGS_BAD      (SPIFFS_ERR_INTERNAL - 3)
#define _SPIFFS_ERR_CHECK_LAST          (SPIFFS_ERR_INTERNAL - 4)

// visitor result, continue searching
#define SPIFFS_VIS_COUNTINUE            (SPIFFS_ERR_INTERNAL - 20)
// visitor result, continue searching after reloading lu buffer
#define SPIFFS_VIS_COUNTINUE_RELOAD     (SPIFFS_ERR_INTERNAL - 21)
// visitor result, stop searching
#define SPIFFS_VIS_END                  (SPIFFS_ERR_INTERNAL - 22)

// updating an object index contents
#define SPIFFS_EV_IX_UPD                (0)
// creating a new object index
#define SPIFFS_EV_IX_NEW                (1)
// deleting an object index
#define SPIFFS_EV_IX_DEL                (2)
// moving an object index without updating contents
#define SPIFFS_EV_IX_MOV                (3)
// updating an object index header data only, not the table itself
#define SPIFFS_EV_IX_UPD_HDR            (4)

#define SPIFFS_OBJ_ID_IX_FLAG           ((spiffs_obj_id)(1<<(8*sizeof(spiffs_obj_id)-1)))

#define SPIFFS_UNDEFINED_LEN            (u32_t)(-1)

#define SPIFFS_OBJ_ID_DELETED           ((spiffs_obj_id)0)
#define SPIFFS_OBJ_ID_FREE              ((spiffs_obj_id)-1)



#if defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__)
    /* For GCC, clang and TI compilers */
#define SPIFFS_PACKED __attribute__((packed))
#elif defined(__ICCARM__) || defined(__CC_ARM)
    /* For IAR ARM and Keil MDK-ARM compilers */
#define SPIFFS_PACKED 

#else
    /* Unknown compiler */
#define SPIFFS_PACKED 
#endif



#if SPIFFS_USE_MAGIC
#if !SPIFFS_USE_MAGIC_LENGTH
#define SPIFFS_MAGIC(fs, bix)           \
  ((spiffs_obj_id)(0x20140529 ^ SPIFFS_CFG_LOG_PAGE_SZ(fs)))
#else // SPIFFS_USE_MAGIC_LENGTH
#define SPIFFS_MAGIC(fs, bix)           \
  ((spiffs_obj_id)(0x20140529 ^ SPIFFS_CFG_LOG_PAGE_SZ(fs) ^ ((fs)->block_count - (bix))))
#endif // SPIFFS_USE_MAGIC_LENGTH
#endif // SPIFFS_USE_MAGIC

#define SPIFFS_CONFIG_MAGIC             (0x20090315)

#if SPIFFS_SINGLETON == 0
#define SPIFFS_CFG_LOG_PAGE_SZ(fs) \
  ((fs)->cfg.log_page_size)
#define SPIFFS_CFG_LOG_BLOCK_SZ(fs) \
  ((fs)->cfg.log_block_size)
#define SPIFFS_CFG_PHYS_SZ(fs) \
  ((fs)->cfg.phys_size)
#define SPIFFS_CFG_PHYS_ERASE_SZ(fs) \
  ((fs)->cfg.phys_erase_block)
#define SPIFFS_CFG_PHYS_ADDR(fs) \
  ((fs)->cfg.phys_addr)
#endif

// total number of pages
#define SPIFFS_MAX_PAGES(fs) \
  ( SPIFFS_CFG_PHYS_SZ(fs)/SPIFFS_CFG_LOG_PAGE_SZ(fs) )
// total number of pages per block, including object lookup pages
#define SPIFFS_PAGES_PER_BLOCK(fs) \
  ( SPIFFS_CFG_LOG_BLOCK_SZ(fs)/SPIFFS_CFG_LOG_PAGE_SZ(fs) )
// number of object lookup pages per block
#define SPIFFS_OBJ_LOOKUP_PAGES(fs)     \
  (MAX(1, (SPIFFS_PAGES_PER_BLOCK(fs) * sizeof(spiffs_obj_id)) / SPIFFS_CFG_LOG_PAGE_SZ(fs)) )
// checks if page index belongs to object lookup
#define SPIFFS_IS_LOOKUP_PAGE(fs,pix)     \
  (((pix) % SPIFFS_PAGES_PER_BLOCK(fs)) < SPIFFS_OBJ_LOOKUP_PAGES(fs))
// number of object lookup entries in all object lookup pages
#define SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) \
  (SPIFFS_PAGES_PER_BLOCK(fs)-SPIFFS_OBJ_LOOKUP_PAGES(fs))
// converts a block to physical address
#define SPIFFS_BLOCK_TO_PADDR(fs, block) \
  ( SPIFFS_CFG_PHYS_ADDR(fs) + (block)* SPIFFS_CFG_LOG_BLOCK_SZ(fs) )
// converts a object lookup entry to page index
#define SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, block, entry) \
  ((block)*SPIFFS_PAGES_PER_BLOCK(fs) + (SPIFFS_OBJ_LOOKUP_PAGES(fs) + entry))
// converts a object lookup entry to physical address of corresponding page
#define SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, block, entry) \
  (SPIFFS_BLOCK_TO_PADDR(fs, block) + (SPIFFS_OBJ_LOOKUP_PAGES(fs) + entry) * SPIFFS_CFG_LOG_PAGE_SZ(fs) )
// converts a page to physical address
#define SPIFFS_PAGE_TO_PADDR(fs, page) \
  ( SPIFFS_CFG_PHYS_ADDR(fs) + (page) * SPIFFS_CFG_LOG_PAGE_SZ(fs) )
// converts a physical address to page
#define SPIFFS_PADDR_TO_PAGE(fs, addr) \
  ( ((addr) -  SPIFFS_CFG_PHYS_ADDR(fs)) / SPIFFS_CFG_LOG_PAGE_SZ(fs) )
// gives index in page for a physical address
#define SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr) \
  ( ((addr) - SPIFFS_CFG_PHYS_ADDR(fs)) % SPIFFS_CFG_LOG_PAGE_SZ(fs) )
// returns containing block for given page
#define SPIFFS_BLOCK_FOR_PAGE(fs, page) \
  ( (page) / SPIFFS_PAGES_PER_BLOCK(fs) )
// returns starting page for block
#define SPIFFS_PAGE_FOR_BLOCK(fs, block) \
  ( (block) * SPIFFS_PAGES_PER_BLOCK(fs) )
// converts page to entry in object lookup page
#define SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, page) \
  ( (page) % SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs) )
// returns data size in a data page
#define SPIFFS_DATA_PAGE_SIZE(fs) \
    ( SPIFFS_CFG_LOG_PAGE_SZ(fs) - sizeof(spiffs_page_header) )
// returns physical address for block's erase count,
// always in the physical last entry of the last object lookup page
#define SPIFFS_ERASE_COUNT_PADDR(fs, bix) \
  ( SPIFFS_BLOCK_TO_PADDR(fs, bix) + SPIFFS_OBJ_LOOKUP_PAGES(fs) * SPIFFS_CFG_LOG_PAGE_SZ(fs) - sizeof(spiffs_obj_id) )
// returns physical address for block's magic,
// always in the physical second last entry of the last object lookup page
#define SPIFFS_MAGIC_PADDR(fs, bix) \
  ( SPIFFS_BLOCK_TO_PADDR(fs, bix) + SPIFFS_OBJ_LOOKUP_PAGES(fs) * SPIFFS_CFG_LOG_PAGE_SZ(fs) - sizeof(spiffs_obj_id)*2 )
// checks if there is any room for magic in the object luts
#define SPIFFS_CHECK_MAGIC_POSSIBLE(fs) \
  ( (SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) % (SPIFFS_CFG_LOG_PAGE_SZ(fs)/sizeof(spiffs_obj_id))) * sizeof(spiffs_obj_id) \
    <= (SPIFFS_CFG_LOG_PAGE_SZ(fs)-sizeof(spiffs_obj_id)*2) )

// define helpers object

// entries in an object header page index
#define SPIFFS_OBJ_HDR_IX_LEN(fs) \
  ((SPIFFS_CFG_LOG_PAGE_SZ(fs) - sizeof(spiffs_page_object_ix_header))/sizeof(spiffs_page_ix))
// entries in an object page index
#define SPIFFS_OBJ_IX_LEN(fs) \
  ((SPIFFS_CFG_LOG_PAGE_SZ(fs) - sizeof(spiffs_page_object_ix))/sizeof(spiffs_page_ix))
// object index entry for given data span index
#define SPIFFS_OBJ_IX_ENTRY(fs, spix) \
  ((spix) < SPIFFS_OBJ_HDR_IX_LEN(fs) ? (spix) : (((spix)-SPIFFS_OBJ_HDR_IX_LEN(fs))%SPIFFS_OBJ_IX_LEN(fs)))
// object index span index number for given data span index or entry
#define SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, spix) \
  ((spix) < SPIFFS_OBJ_HDR_IX_LEN(fs) ? 0 : (1+((spix)-SPIFFS_OBJ_HDR_IX_LEN(fs))/SPIFFS_OBJ_IX_LEN(fs)))
// get data span index for object index span index
#define SPIFFS_DATA_SPAN_IX_FOR_OBJ_IX_SPAN_IX(fs, spix) \
  ( (spix) == 0 ? 0 : (SPIFFS_OBJ_HDR_IX_LEN(fs) + (((spix)-1) * SPIFFS_OBJ_IX_LEN(fs))) )

#if SPIFFS_FILEHDL_OFFSET
#define SPIFFS_FH_OFFS(fs, fh)   ((fh) != 0 ? ((fh) + (fs)->cfg.fh_ix_offset) : 0)
#define SPIFFS_FH_UNOFFS(fs, fh) ((fh) != 0 ? ((fh) - (fs)->cfg.fh_ix_offset) : 0)
#else
#define SPIFFS_FH_OFFS(fs, fh)   (fh)
#define SPIFFS_FH_UNOFFS(fs, fh) (fh)
#endif


#define SPIFFS_OP_T_OBJ_LU    (0<<0)
#define SPIFFS_OP_T_OBJ_LU2   (1<<0)
#define SPIFFS_OP_T_OBJ_IX    (2<<0)
#define SPIFFS_OP_T_OBJ_DA    (3<<0)
#define SPIFFS_OP_C_DELE      (0<<2)
#define SPIFFS_OP_C_UPDT      (1<<2)
#define SPIFFS_OP_C_MOVS      (2<<2)
#define SPIFFS_OP_C_MOVD      (3<<2)
#define SPIFFS_OP_C_FLSH      (4<<2)
#define SPIFFS_OP_C_READ      (5<<2)
#define SPIFFS_OP_C_WRTHRU    (6<<2)

#define SPIFFS_OP_TYPE_MASK (3<<0)
#define SPIFFS_OP_COM_MASK  (7<<2)


// if 0, this page is written to, else clean
#define SPIFFS_PH_FLAG_USED   (1<<0)
// if 0, writing is finalized, else under modification
#define SPIFFS_PH_FLAG_FINAL  (1<<1)
// if 0, this is an index page, else a data page
#define SPIFFS_PH_FLAG_INDEX  (1<<2)
// if 0, page is deleted, else valid
#define SPIFFS_PH_FLAG_DELET  (1<<7)
// if 0, this index header is being deleted
#define SPIFFS_PH_FLAG_IXDELE (1<<6)


#define SPIFFS_CHECK_MOUNT(fs) \
  ((fs)->mounted != 0)

#define SPIFFS_CHECK_CFG(fs) \
  ((fs)->config_magic == SPIFFS_CONFIG_MAGIC)

#define SPIFFS_CHECK_RES(res) \
  do { \
    if ((res) < SPIFFS_OK) return (res); \
  } while (0);

#define SPIFFS_API_CHECK_MOUNT(fs) \
  if (!SPIFFS_CHECK_MOUNT((fs))) { \
    (fs)->err_code = SPIFFS_ERR_NOT_MOUNTED; \
    return SPIFFS_ERR_NOT_MOUNTED; \
  }

#define SPIFFS_API_CHECK_CFG(fs) \
  if (!SPIFFS_CHECK_CFG((fs))) { \
    (fs)->err_code = SPIFFS_ERR_NOT_CONFIGURED; \
    return SPIFFS_ERR_NOT_CONFIGURED; \
  }

#define SPIFFS_API_CHECK_RES(fs, res) \
  if ((res) < SPIFFS_OK) { \
    (fs)->err_code = (res); \
    return (res); \
  }

#define SPIFFS_API_CHECK_RES_UNLOCK(fs, res) \
  if ((res) < SPIFFS_OK) { \
    (fs)->err_code = (res); \
    SPIFFS_UNLOCK(fs); \
    return (res); \
  }

#define SPIFFS_VALIDATE_OBJIX(ph, objid, spix) \
    if (((ph).flags & SPIFFS_PH_FLAG_USED) != 0) return SPIFFS_ERR_IS_FREE; \
    if (((ph).flags & SPIFFS_PH_FLAG_DELET) == 0) return SPIFFS_ERR_DELETED; \
    if (((ph).flags & SPIFFS_PH_FLAG_FINAL) != 0) return SPIFFS_ERR_NOT_FINALIZED; \
    if (((ph).flags & SPIFFS_PH_FLAG_INDEX) != 0) return SPIFFS_ERR_NOT_INDEX; \
    if (((objid) & SPIFFS_OBJ_ID_IX_FLAG) == 0) return SPIFFS_ERR_NOT_INDEX; \
    if ((ph).span_ix != (spix)) return SPIFFS_ERR_INDEX_SPAN_MISMATCH;
    //if ((spix) == 0 && ((ph).flags & SPIFFS_PH_FLAG_IXDELE) == 0) return SPIFFS_ERR_DELETED;

#define SPIFFS_VALIDATE_DATA(ph, objid, spix) \
    if (((ph).flags & SPIFFS_PH_FLAG_USED) != 0) return SPIFFS_ERR_IS_FREE; \
    if (((ph).flags & SPIFFS_PH_FLAG_DELET) == 0) return SPIFFS_ERR_DELETED; \
    if (((ph).flags & SPIFFS_PH_FLAG_FINAL) != 0) return SPIFFS_ERR_NOT_FINALIZED; \
    if (((ph).flags & SPIFFS_PH_FLAG_INDEX) == 0) return SPIFFS_ERR_IS_INDEX; \
    if ((objid) & SPIFFS_OBJ_ID_IX_FLAG) return SPIFFS_ERR_IS_INDEX; \
    if ((ph).span_ix != (spix)) return SPIFFS_ERR_DATA_SPAN_MISMATCH;


// check id, only visit matching objec ids
#define SPIFFS_VIS_CHECK_ID     (1<<0)
// report argument object id to visitor - else object lookup id is reported
#define SPIFFS_VIS_CHECK_PH     (1<<1)
// stop searching at end of all look up pages
#define SPIFFS_VIS_NO_WRAP      (1<<2)

#if SPIFFS_HAL_CALLBACK_EXTRA

#define SPIFFS_HAL_WRITE(_fs, _paddr, _len, _src) \
  (_fs)->cfg.hal_write_f((_fs), (_paddr), (_len), (_src))
#define SPIFFS_HAL_READ(_fs, _paddr, _len, _dst) \
  (_fs)->cfg.hal_read_f((_fs), (_paddr), (_len), (_dst))
#define SPIFFS_HAL_ERASE(_fs, _paddr, _len) \
  (_fs)->cfg.hal_erase_f((_fs), (_paddr), (_len))

#else // SPIFFS_HAL_CALLBACK_EXTRA

#define SPIFFS_HAL_WRITE(_fs, _paddr, _len, _src) \
  (_fs)->cfg.hal_write_f((_paddr), (_len), (_src))
#define SPIFFS_HAL_READ(_fs, _paddr, _len, _dst) \
  (_fs)->cfg.hal_read_f((_paddr), (_len), (_dst))
#define SPIFFS_HAL_ERASE(_fs, _paddr, _len) \
  (_fs)->cfg.hal_erase_f((_paddr), (_len))

#endif // SPIFFS_HAL_CALLBACK_EXTRA

#if SPIFFS_CACHE

#define SPIFFS_CACHE_FLAG_DIRTY       (1<<0)
#define SPIFFS_CACHE_FLAG_WRTHRU      (1<<1)
#define SPIFFS_CACHE_FLAG_OBJLU       (1<<2)
#define SPIFFS_CACHE_FLAG_OBJIX       (1<<3)
#define SPIFFS_CACHE_FLAG_DATA        (1<<4)
#define SPIFFS_CACHE_FLAG_TYPE_WR     (1<<7)

#define SPIFFS_CACHE_PAGE_SIZE(fs) \
  (sizeof(spiffs_cache_page) + SPIFFS_CFG_LOG_PAGE_SZ(fs))

#define spiffs_get_cache(fs) \
  ((spiffs_cache *)((fs)->cache))

#define spiffs_get_cache_page_hdr(fs, c, ix) \
  ((spiffs_cache_page *)(&((c)->cpages[(ix) * SPIFFS_CACHE_PAGE_SIZE(fs)])))

#define spiffs_get_cache_page(fs, c, ix) \
  ((u8_t *)(&((c)->cpages[(ix) * SPIFFS_CACHE_PAGE_SIZE(fs)])) + sizeof(spiffs_cache_page))

// cache page struct
typedef struct {
  // cache flags
  u8_t flags;
  // cache page index
  u8_t ix;
  // last access of this cache page
  u32_t last_access;
  union {
    // type read cache
    struct {
      // read cache page index
      spiffs_page_ix pix;
    };
#if SPIFFS_CACHE_WR
    // type write cache
    struct {
      // write cache
      spiffs_obj_id obj_id;
      // offset in cache page
      u32_t offset;
      // size of cache page
      u16_t size;
    };
#endif
  };
} spiffs_cache_page;

// cache struct
typedef struct {
  u8_t cpage_count;
  u32_t last_access;
  u32_t cpage_use_map;
  u32_t cpage_use_mask;
  u8_t *cpages;
} spiffs_cache;

#endif


// spiffs nucleus file descriptor
typedef struct {
  // the filesystem of this descriptor
  spiffs *fs;
  // number of file descriptor - if 0, the file descriptor is closed
  spiffs_file file_nbr;
  // object id - if SPIFFS_OBJ_ID_ERASED, the file was deleted
  spiffs_obj_id obj_id;
  // size of the file
  u32_t size;
  // cached object index header page index
  spiffs_page_ix objix_hdr_pix;
  // cached offset object index page index
  spiffs_page_ix cursor_objix_pix;
  // cached offset object index span index
  spiffs_span_ix cursor_objix_spix;
  // current absolute offset
  u32_t offset;
  // current file descriptor offset (cached)
  u32_t fdoffset;
  // fd flags
  spiffs_flags flags;
#if SPIFFS_CACHE_WR
  spiffs_cache_page *cache_page;
#endif
#if SPIFFS_TEMPORAL_FD_CACHE
  // djb2 hash of filename
  u32_t name_hash;
  // hit score (score == 0 indicates never used fd)
  u16_t score;
#endif
#if SPIFFS_IX_MAP
  // spiffs index map, if 0 it means unmapped
  spiffs_ix_map *ix_map;
#endif
} spiffs_fd;


// object structs

// page header, part of each page except object lookup pages
// NB: this is always aligned when the data page is an object index,
// as in this case struct spiffs_page_object_ix is used
typedef struct SPIFFS_PACKED {
  // object id
  spiffs_obj_id obj_id;
  // object span index
  spiffs_span_ix span_ix;
  // flags
  u8_t flags;
} spiffs_page_header;

// object index header page header
typedef struct SPIFFS_PACKED
#if SPIFFS_ALIGNED_OBJECT_INDEX_TABLES
                __attribute(( aligned(sizeof(spiffs_page_ix)) ))
#endif
{
  // common page header
  spiffs_page_header p_hdr;
  // alignment
  u8_t _align0[4 - ((sizeof(spiffs_page_header)&3)==0 ? 4 : (sizeof(spiffs_page_header)&3))];
  // size of object
  u32_t size;
  // type of object
  spiffs_obj_type type;
  u8_t _align1[4 - ((sizeof(spiffs_obj_type)&3)==0 ? 4 : (sizeof(spiffs_obj_type)&3))];
  // name of object
  u8_t name[SPIFFS_OBJ_NAME_LEN];
#if SPIFFS_OBJ_META_LEN
  // metadata. not interpreted by SPIFFS in any way.
  u8_t meta[SPIFFS_OBJ_META_LEN];
#endif
} spiffs_page_object_ix_header;

// object index page header
typedef struct SPIFFS_PACKED {
 spiffs_page_header p_hdr;
 u8_t _align[4 - ((sizeof(spiffs_page_header)&3)==0 ? 4 : (sizeof(spiffs_page_header)&3))];
} spiffs_page_object_ix;

// callback func for object lookup visitor
typedef s32_t (*spiffs_visitor_f)(spiffs *fs, spiffs_obj_id id, spiffs_block_ix bix, int ix_entry,
    const void *user_const_p, void *user_var_p);


#if SPIFFS_CACHE
#define _spiffs_rd(fs, op, fh, addr, len, dst) \
    spiffs_phys_rd((fs), (op), (fh), (addr), (len), (dst))
#define _spiffs_wr(fs, op, fh, addr, len, src) \
    spiffs_phys_wr((fs), (op), (fh), (addr), (len), (src))
#else
#define _spiffs_rd(fs, op, fh, addr, len, dst) \
    spiffs_phys_rd((fs), (addr), (len), (dst))
#define _spiffs_wr(fs, op, fh, addr, len, src) \
    spiffs_phys_wr((fs), (addr), (len), (src))
#endif

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

// ---------------

s32_t spiffs_phys_rd(
    spiffs *fs,
#if SPIFFS_CACHE
    u8_t op,
    spiffs_file fh,
#endif
    u32_t addr,
    u32_t len,
    u8_t *dst);

s32_t spiffs_phys_wr(
    spiffs *fs,
#if SPIFFS_CACHE
    u8_t op,
    spiffs_file fh,
#endif
    u32_t addr,
    u32_t len,
    u8_t *src);

s32_t spiffs_phys_cpy(
    spiffs *fs,
    spiffs_file fh,
    u32_t dst,
    u32_t src,
    u32_t len);

s32_t spiffs_phys_count_free_blocks(
    spiffs *fs);

s32_t spiffs_obj_lu_find_entry_visitor(
    spiffs *fs,
    spiffs_block_ix starting_block,
    int starting_lu_entry,
    u8_t flags,
    spiffs_obj_id obj_id,
    spiffs_visitor_f v,
    const void *user_const_p,
    void *user_var_p,
    spiffs_block_ix *block_ix,
    int *lu_entry);

s32_t spiffs_erase_block(
    spiffs *fs,
    spiffs_block_ix bix);

#if SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH
s32_t spiffs_probe(
    spiffs_config *cfg);
#endif // SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH

// ---------------

s32_t spiffs_obj_lu_scan(
    spiffs *fs);

s32_t spiffs_obj_lu_find_free_obj_id(
    spiffs *fs,
    spiffs_obj_id *obj_id,
    const u8_t *conflicting_name);

s32_t spiffs_obj_lu_find_free(
    spiffs *fs,
    spiffs_block_ix starting_block,
    int starting_lu_entry,
    spiffs_block_ix *block_ix,
    int *lu_entry);

s32_t spiffs_obj_lu_find_id(
    spiffs *fs,
    spiffs_block_ix starting_block,
    int starting_lu_entry,
    spiffs_obj_id obj_id,
    spiffs_block_ix *block_ix,
    int *lu_entry);

s32_t spiffs_obj_lu_find_id_and_span(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_span_ix spix,
    spiffs_page_ix exclusion_pix,
    spiffs_page_ix *pix);

s32_t spiffs_obj_lu_find_id_and_span_by_phdr(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_span_ix spix,
    spiffs_page_ix exclusion_pix,
    spiffs_page_ix *pix);

// ---------------

s32_t spiffs_page_allocate_data(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_page_header *ph,
    u8_t *data,
    u32_t len,
    u32_t page_offs,
    u8_t finalize,
    spiffs_page_ix *pix);

s32_t spiffs_page_move(
    spiffs *fs,
    spiffs_file fh,
    u8_t *page_data,
    spiffs_obj_id obj_id,
    spiffs_page_header *page_hdr,
    spiffs_page_ix src_pix,
    spiffs_page_ix *dst_pix);

s32_t spiffs_page_delete(
    spiffs *fs,
    spiffs_page_ix pix);

// ---------------

s32_t spiffs_object_create(
    spiffs *fs,
    spiffs_obj_id obj_id,
    const u8_t name[],
    const u8_t meta[],
    spiffs_obj_type type,
    spiffs_page_ix *objix_hdr_pix);

s32_t spiffs_object_update_index_hdr(
    spiffs *fs,
    spiffs_fd *fd,
    spiffs_obj_id obj_id,
    spiffs_page_ix objix_hdr_pix,
    u8_t *new_objix_hdr_data,
    const u8_t name[],
    const u8_t meta[],
    u32_t size,
    spiffs_page_ix *new_pix);

#if SPIFFS_IX_MAP

s32_t spiffs_populate_ix_map(
    spiffs *fs,
    spiffs_fd *fd,
    u32_t vec_entry_start,
    u32_t vec_entry_end);

#endif

void spiffs_cb_object_event(
    spiffs *fs,
    spiffs_page_object_ix *objix,
    int ev,
    spiffs_obj_id obj_id,
    spiffs_span_ix spix,
    spiffs_page_ix new_pix,
    u32_t new_size);

s32_t spiffs_object_open_by_id(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_fd *f,
    spiffs_flags flags,
    spiffs_mode mode);

s32_t spiffs_object_open_by_page(
    spiffs *fs,
    spiffs_page_ix pix,
    spiffs_fd *f,
    spiffs_flags flags,
    spiffs_mode mode);

s32_t spiffs_object_append(
    spiffs_fd *fd,
    u32_t offset,
    u8_t *data,
    u32_t len);

s32_t spiffs_object_modify(
    spiffs_fd *fd,
    u32_t offset,
    u8_t *data,
    u32_t len);

s32_t spiffs_object_read(
    spiffs_fd *fd,
    u32_t offset,
    u32_t len,
    u8_t *dst);

s32_t spiffs_object_truncate(
    spiffs_fd *fd,
    u32_t new_len,
    u8_t remove_object);

s32_t spiffs_object_find_object_index_header_by_name(
    spiffs *fs,
    const u8_t name[SPIFFS_OBJ_NAME_LEN],
    spiffs_page_ix *pix);

// ---------------

s32_t spiffs_gc_check(
    spiffs *fs,
    u32_t len);

s32_t spiffs_gc_erase_page_stats(
    spiffs *fs,
    spiffs_block_ix bix);

s32_t spiffs_gc_find_candidate(
    spiffs *fs,
    spiffs_block_ix **block_candidate,
    int *candidate_count,
    char fs_crammed);

s32_t spiffs_gc_clean(
    spiffs *fs,
    spiffs_block_ix bix);

s32_t spiffs_gc_quick(
    spiffs *fs, u16_t max_free_pages);

// ---------------

s32_t spiffs_fd_find_new(
    spiffs *fs,
    spiffs_fd **fd,
    const char *name);

s32_t spiffs_fd_return(
    spiffs *fs,
    spiffs_file f);

s32_t spiffs_fd_get(
    spiffs *fs,
    spiffs_file f,
    spiffs_fd **fd);

#if SPIFFS_TEMPORAL_FD_CACHE
void spiffs_fd_temporal_cache_rehash(
    spiffs *fs,
    const char *old_path,
    const char *new_path);
#endif

#if SPIFFS_CACHE
void spiffs_cache_init(
    spiffs *fs);

void spiffs_cache_drop_page(
    spiffs *fs,
    spiffs_page_ix pix);

#if SPIFFS_CACHE_WR
spiffs_cache_page *spiffs_cache_page_allocate_by_fd(
    spiffs *fs,
    spiffs_fd *fd);

void spiffs_cache_fd_release(
    spiffs *fs,
    spiffs_cache_page *cp);

spiffs_cache_page *spiffs_cache_page_get_by_fd(
    spiffs *fs,
    spiffs_fd *fd);
#endif
#endif

s32_t spiffs_lookup_consistency_check(
    spiffs *fs,
    u8_t check_all_objects);

s32_t spiffs_page_consistency_check(
    spiffs *fs);

s32_t spiffs_object_index_consistency_check(
    spiffs *fs);

// memcpy macro,
// checked in test builds, otherwise plain memcpy (unless already defined)
#ifdef _SPIFFS_TEST
#define _SPIFFS_MEMCPY(__d, __s, __l) do { \
    intptr_t __a1 = (intptr_t)((u8_t*)(__s)); \
    intptr_t __a2 = (intptr_t)((u8_t*)(__s)+(__l)); \
    intptr_t __b1 = (intptr_t)((u8_t*)(__d)); \
    intptr_t __b2 = (intptr_t)((u8_t*)(__d)+(__l)); \
    if (__a1 <= __b2 && __b1 <= __a2) { \
      printf("FATAL OVERLAP: memcpy from %lx..%lx to %lx..%lx\n", __a1, __a2, __b1, __b2); \
      ERREXIT(); \
    } \
    memcpy((__d),(__s),(__l)); \
} while (0)
#else
#ifndef _SPIFFS_MEMCPY
#define _SPIFFS_MEMCPY(__d, __s, __l) do{memcpy((__d),(__s),(__l));}while(0)
#endif
#endif //_SPIFFS_TEST

#endif /* SPIFFS_NUCLEUS_H_ */
