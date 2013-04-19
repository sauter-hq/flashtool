/*
 ============================================================================
 Name        : bchtool.c
 Author      : Robert Bouwens / Ivan Djelic
 Version     :
 Copyright   : Your copyright notice
 Description : NAND flash update tool - derived from the linux 3.2 kernel 
               sources
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bchtool.h"

#include <arpa/inet.h>

#include "bch.c"


/*
 * This looks more complex than it should be. But we need to
 * get the type for the ~ right in round_down (it needs to be
 * as wide as the result!), and we want to evaluate the macro
 * arguments just once each.
 */
#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)
#define round_down(x, y) ((x) & ~__round_mask(x, y))

#define FIELD_SIZEOF(t, f) (sizeof(((t*)0)->f))
#define DIV_ROUND_UP_ULL(ll,d) \
        ({ unsigned long long _tmp = (ll)+(d)-1; do_div(_tmp, d); _tmp; })


#if defined(CONFIG_BCH_CONST_PARAMS)
#define GF_M(_p)               (CONFIG_BCH_CONST_M)
#define GF_T(_p)               (CONFIG_BCH_CONST_T)
#define GF_N(_p)               ((1 << (CONFIG_BCH_CONST_M))-1)
#else
#define GF_M(_p)               ((_p)->m)
#define GF_T(_p)               ((_p)->t)
#define GF_N(_p)               ((_p)->n)
#endif

#define BCH_ECC_WORDS(_p)      DIV_ROUND_UP(GF_M(_p)*GF_T(_p), 32)
#define BCH_ECC_BYTES(_p)      DIV_ROUND_UP(GF_M(_p)*GF_T(_p), 8)


enum omap_ecc {
                /* 1-bit ecc: stored at end of spare area */
        OMAP_ECC_HAMMING_CODE_DEFAULT = 0, /* Default, s/w method */
        OMAP_ECC_HAMMING_CODE_HW, /* gpmc to detect the error */
                /* 1-bit ecc: stored at beginning of spare area as romcode */
        OMAP_ECC_HAMMING_CODE_HW_ROMCODE, /* gpmc method & romcode layout */
        OMAP_ECC_BCH4_CODE_HW,
        OMAP_ECC_BCH8_CODE_HW,
        OMAP_ECC_BCH8_CODE_HW_ROMCODE,
};

/*
 * Constants for ECC_MODES
 */
typedef enum
{
        NAND_ECC_NONE,
        NAND_ECC_SOFT,
        NAND_ECC_HW,
        NAND_ECC_HW_SYNDROME,
        NAND_ECC_HW_OOB_FIRST,
        NAND_ECC_SOFT_BCH,
} nand_ecc_modes_t;



/*
 * Obsolete legacy interface. Keep it in order not to break userspace
 * interfaces
 */
struct nand_oobinfo
{
        uint32_t useecc;
        uint32_t eccbytes;
        uint32_t oobfree[8][2];
        uint32_t eccpos[32];
};

struct nand_oobfree
{
	uint32_t offset;
	uint32_t length;
};



/**
 * struct mtd_oob_ops - oob operation operands
 * @mode:       operation mode
 *
 * @len:        number of data bytes to write/read
 *
 * @retlen:     number of data bytes written/read
 *
 * @ooblen:     number of oob bytes to write/read
 * @oobretlen:  number of oob bytes written/read
 * @ooboffs:    offset of oob data in the oob area (only relevant when
 *              mode = MTD_OPS_PLACE_OOB or MTD_OPS_RAW)
 * @datbuf:     data buffer - if NULL only oob data are read/written
 * @oobbuf:     oob data buffer
 *
 * Note, it is allowed to read more than one OOB area at one go, but not write.
 * The interface assumes that the OOB write requests program only one page's
 * OOB area.
 */
struct mtd_oob_ops
{
        unsigned int    mode;
        size_t          len;
        size_t          retlen;
        size_t          ooblen;
        size_t          oobretlen;
        uint32_t        ooboffs;
        uint8_t         *datbuf;
        uint8_t         *oobbuf;
};

#define MTD_MAX_OOBFREE_ENTRIES_LARGE   32
#define MTD_MAX_ECCPOS_ENTRIES_LARGE    448
/*
 * Internal ECC layout control structure. For historical reasons, there is a
 * similar, smaller struct nand_ecclayout_user (in mtd-abi.h) that is retained
 * for export to user-space via the ECCGETLAYOUT ioctl.
 * nand_ecclayout should be expandable in the future simply by the above macros.
 */
struct nand_ecclayout {
        uint32_t eccbytes;
        uint32_t eccpos[MTD_MAX_ECCPOS_ENTRIES_LARGE];
        uint32_t oobavail;
        struct nand_oobfree oobfree[MTD_MAX_OOBFREE_ENTRIES_LARGE];
};

struct module;  /* only needed for owner field in mtd_info */


/**
 * struct nand_ecc_ctrl - Control structure for ECC
 * @mode:       ECC mode
 * @steps:      number of ECC steps per page
 * @size:       data bytes per ECC step
 * @bytes:      ECC bytes per step
 * @total:      total number of ECC bytes per page
 * @prepad:     padding information for syndrome based ECC generators
 * @postpad:    padding information for syndrome based ECC generators
 * @layout:     ECC layout control struct pointer
 * @priv:       pointer to private ECC control data
 * @hwctl:      function to control hardware ECC generator. Must only
 *              be provided if an hardware ECC is available
 * @calculate:  function for ECC calculation or readback from ECC hardware
 * @correct:    function for ECC correction, matching to ECC generator (sw/hw)
 * @read_page_raw:      function to read a raw page without ECC
 * @write_page_raw:     function to write a raw page without ECC
 * @read_page:  function to read a page according to the ECC generator
 *              requirements.
 * @read_subpage:       function to read parts of the page covered by ECC.
 * @write_page: function to write a page according to the ECC generator
 *              requirements.
 * @write_oob_raw:      function to write chip OOB data without ECC
 * @read_oob_raw:       function to read chip OOB data without ECC
 * @read_oob:   function to read chip OOB data
 * @write_oob:  function to write chip OOB data
 */
struct nand_ecc_ctrl {
        nand_ecc_modes_t mode;
        int steps;
        int size;
        int bytes;
        int total;
        int prepad;
        int postpad;
        struct nand_ecclayout   *layout;
};


/**
 * struct nand_chip - NAND Private Flash Chip Data
 * @IO_ADDR_R:          [BOARDSPECIFIC] address to read the 8 I/O lines of the
 *                      flash device
 * @IO_ADDR_W:          [BOARDSPECIFIC] address to write the 8 I/O lines of the
 *                      flash device.
 * @read_byte:          [REPLACEABLE] read one byte from the chip
 * @read_word:          [REPLACEABLE] read one word from the chip
 * @write_buf:          [REPLACEABLE] write data from the buffer to the chip
 * @read_buf:           [REPLACEABLE] read data from the chip into the buffer
 * @verify_buf:         [REPLACEABLE] verify buffer contents against the chip
 *                      data.
 * @select_chip:        [REPLACEABLE] select chip nr
 * @block_bad:          [REPLACEABLE] check, if the block is bad
 * @block_markbad:      [REPLACEABLE] mark the block bad
 * @cmd_ctrl:           [BOARDSPECIFIC] hardwarespecific function for controlling
 *                      ALE/CLE/nCE. Also used to write command and address
 * @init_size:          [BOARDSPECIFIC] hardwarespecific function for setting
 *                      mtd->oobsize, mtd->writesize and so on.
 *                      @id_data contains the 8 bytes values of NAND_CMD_READID.
 *                      Return with the bus width.
 * @dev_ready:          [BOARDSPECIFIC] hardwarespecific function for accessing
 *                      device ready/busy line. If set to NULL no access to
 *                      ready/busy is available and the ready/busy information
 *                      is read from the chip status register.
 * @cmdfunc:            [REPLACEABLE] hardwarespecific function for writing
 *                      commands to the chip.
 * @waitfunc:           [REPLACEABLE] hardwarespecific function for wait on
 *                      ready.
 * @ecc:                [BOARDSPECIFIC] ECC control structure
 * @buffers:            buffer structure for read/write
 * @hwcontrol:          platform-specific hardware control structure
 * @erase_cmd:          [INTERN] erase command write function, selectable due
 *                      to AND support.
 * @scan_bbt:           [REPLACEABLE] function to scan bad block table
 * @chip_delay:         [BOARDSPECIFIC] chip dependent delay for transferring
 *                      data from array to read regs (tR).
 * @state:              [INTERN] the current state of the NAND device
 * @oob_poi:            "poison value buffer," used for laying out OOB data
 *                      before writing
 * @page_shift:         [INTERN] number of address bits in a page (column
 *                      address bits).
 * @phys_erase_shift:   [INTERN] number of address bits in a physical eraseblock
 * @bbt_erase_shift:    [INTERN] number of address bits in a bbt entry
 * @chip_shift:         [INTERN] number of address bits in one chip
 * @options:            [BOARDSPECIFIC] various chip options. They can partly
 *                      be set to inform nand_scan about special functionality.
 *                      See the defines for further explanation.
 * @bbt_options:        [INTERN] bad block specific options. All options used
 *                      here must come from bbm.h. By default, these options
 *                      will be copied to the appropriate nand_bbt_descr's.
 * @badblockpos:        [INTERN] position of the bad block marker in the oob
 *                      area.
 * @badblockbits:       [INTERN] number of bits to left-shift the bad block
 *                      number
 * @cellinfo:           [INTERN] MLC/multichip data from chip ident
 * @numchips:           [INTERN] number of physical chips
 * @chipsize:           [INTERN] the size of one chip for multichip arrays
 * @pagemask:           [INTERN] page number mask = number of (pages / chip) - 1
 * @pagebuf:            [INTERN] holds the pagenumber which is currently in
 *                      data_buf.
 * @subpagesize:        [INTERN] holds the subpagesize
 * @onfi_version:       [INTERN] holds the chip ONFI version (BCD encoded),
 *                      non 0 if ONFI supported.
 * @onfi_params:        [INTERN] holds the ONFI page parameter when ONFI is
 *                      supported, 0 otherwise.
 * @ecclayout:          [REPLACEABLE] the default ECC placement scheme
 * @bbt:                [INTERN] bad block table pointer
 * @bbt_td:             [REPLACEABLE] bad block table descriptor for flash
 *                      lookup.
 * @bbt_md:             [REPLACEABLE] bad block table mirror descriptor
 * @badblock_pattern:   [REPLACEABLE] bad block scan pattern used for initial
 *                      bad block scan.
 * @controller:         [REPLACEABLE] a pointer to a hardware controller
 *                      structure which is shared among multiple independent
 *                      devices.
 * @priv:               [OPTIONAL] pointer to private chip data
 * @errstat:            [OPTIONAL] hardware specific function to perform
 *                      additional error status checks (determine if errors are
 *                      correctable).
 * @write_page:         [REPLACEABLE] High-level page write function
 */

struct nand_chip
{
        struct nand_ecc_ctrl ecc;
};

struct omap_nand_info
{
        struct nand_chip                nand;
        struct bch_control              *bch;
        struct nand_ecclayout           ecc;
        unsigned char                   *eccmask;

        int                             ecc_opt;
        uint32_t                        oobsize;   // Amount of OOB data per block (e.g. 16)   
};

// the working structure for an update
struct omap_nand_info info;

void omap_hwecc_fill_eccinfo(struct omap_nand_info *info,
                                    enum omap_ecc ecc_mode)
{
        int             i;
        unsigned int    tmp;
        unsigned int    oobsize = info->oobsize;

        // phycard-l has a 16 bis wide bus for the nand flash
        tmp = 2;
//        if (info->nand.options & NAND_BUSWIDTH_16)
//                tmp = 2;
//        else {
//                tmp = 1;
//                info->nand.badblock_pattern = &bb_descrip_flashbased;
//        }

        switch (ecc_mode) {
        case OMAP_ECC_HAMMING_CODE_HW:
        case OMAP_ECC_HAMMING_CODE_HW_ROMCODE:
                info->ecc.eccbytes = 3 * (oobsize/16);

                for(i = 0; i < info->ecc.eccbytes; ++i)
                        info->ecc.eccpos[i] = i + tmp;

                info->ecc.oobfree[0].offset = i + tmp;
                info->ecc.oobfree[0].length = (oobsize -
                                               tmp - /* bad block markers */
                                               info->ecc.eccbytes);
                break;

        case OMAP_ECC_BCH4_CODE_HW:
        case OMAP_ECC_BCH8_CODE_HW:
                info->ecc.eccbytes = info->nand.ecc.bytes;

                info->ecc.oobfree[0].offset = tmp;
                info->ecc.oobfree[0].length = (oobsize -
                                               tmp - /* bad block markers */
                                               info->ecc.eccbytes);

                tmp += info->ecc.oobfree[0].length;

                for(i = 0; i < info->ecc.eccbytes; ++i)
                        info->ecc.eccpos[i] = i + tmp;

                break;

        case OMAP_ECC_BCH8_CODE_HW_ROMCODE:
                info->ecc.eccbytes = info->nand.ecc.bytes;

                for (i = 0; i < 13; ++i) {
                        info->ecc.eccpos[i]      = i + 2;
                        info->ecc.eccpos[i + 13] = i + 16;
                        info->ecc.eccpos[i + 26] = i + 30;
                        info->ecc.eccpos[i + 39] = i + 44;
                }

                for (i = 0; i < 3; ++i) {
                        info->ecc.oobfree[i].offset = 14*i + 15;
                        info->ecc.oobfree[i].length = 1;
                }

                info->ecc.oobfree[i].offset = 56;
                info->ecc.oobfree[i].length =  8;
                break;

        default:
                printf("undefined ecc mode");
                exit(1);
        }

        info->nand.ecc.layout = &info->ecc;
        printf("ecc=[%u, [", info->ecc.eccbytes);
        for (i = 0; i < info->ecc.eccbytes; ++i) {
                printf("%s%u",
                       i == 0 ? "" :
                       i % 4 ? "," : ", ",
                       info->ecc.eccpos[i]);
        }
        printf("], %u, [", info->ecc.oobavail);
        for (i = 0; i < ARRAY_SIZE(info->ecc.oobfree); ++i) {
                printf("%s%u+%u", i == 0 ? "" : ",",
                       info->ecc.oobfree[i].offset,
                       info->ecc.oobfree[i].length);
        }
        printf("]\n");
}

/**
 * nand_bch_calculate_ecc - [NAND Interface] Calculate ECC for data block
 * @mtd:        MTD block structure
 * @buf:        input buffer with raw data
 * @code:       output buffer with ECC
 */
int nand_bch_calculate_ecc(struct omap_nand_info *info, const unsigned char *buf,
                           unsigned char *code)
{
        const struct nand_chip *chip = &info->nand;
        int i, blocks = 0;
        unsigned char *ecc_ptr = code;
        unsigned char *data = buf;
        int ecc_bytes = 0;

        if (chip->ecc.size  == 2048)
            blocks = 4;
        else
            blocks = 1;

        ecc_bytes = chip->ecc.bytes / blocks;
            
        for (i = 0; i < blocks; i++)
        {
            data = buf + i * 512;
            ecc_ptr = code + i * ecc_bytes;
            memset(ecc_ptr, 0, ecc_bytes);
            encode_bch(info->bch, data, 512, ecc_ptr);
        }

        return 0;
}

/**
 ** compute and store the inverted ecc of an erased ecc block
 **/
void prepare_ecc_mask(struct omap_nand_info *info, const unsigned int eccsize, const unsigned int eccbytes)
{
   int i;
   unsigned char erased_page[eccsize];

   memset(erased_page, 0xff, eccsize);
   info->eccmask = malloc(eccbytes);
   memset(info->eccmask, 0, eccbytes);

   encode_bch(info->bch, erased_page, eccsize, info->eccmask);
   for (i = 0; i < eccbytes; i++)
   {
      info->eccmask[i] ^= 0xff;
   }
}

int omap_hwecc_init(struct omap_nand_info *info,
                           enum omap_ecc ecc_mode)
{
        int             rc, i;
        bool            have_bch = true;

        switch (ecc_mode) {
        case OMAP_ECC_HAMMING_CODE_HW:
        case OMAP_ECC_HAMMING_CODE_HW_ROMCODE:
                info->nand.ecc.bytes  = 3;
                info->nand.ecc.size   = 512;
                have_bch = false;
                break;

        case OMAP_ECC_BCH4_CODE_HW:
                info->nand.ecc.bytes  = 4*7;
                info->nand.ecc.size   = 4*512;
                info->bch = init_bch(7, 4, 0x201B);
                break;

        case OMAP_ECC_BCH8_CODE_HW:
        case OMAP_ECC_BCH8_CODE_HW_ROMCODE:
                info->nand.ecc.bytes  = 4*13;
                info->nand.ecc.size   = 4*512;
                info->bch = init_bch(13, 8, 0x201B);
                printf("bch control allocated - not zero %d\n",NULL != info->bch);
                break;

        default:
                printf("undefined ecc_mode!\n");
                exit(1);
        }

        if (have_bch && info->bch == NULL) {
                rc = -1;
                goto out;
        }

        rc = 0;

out:
        return rc;
}

int bch_calculate_ecc(const unsigned char *buf, unsigned char *code)
{
    static int _init = 0;
    int res;

    if (!_init)
    {
        _init = 1;
        // prepare bch structure and ecc layout
        info.nand.ecc.mode = NAND_ECC_HW;
        info.oobsize = 64;
        omap_hwecc_init(&info, OMAP_ECC_BCH8_CODE_HW);
        omap_hwecc_fill_eccinfo(&info, OMAP_ECC_BCH8_CODE_HW);
        // the inverted ecc mask
        prepare_ecc_mask(&info, info.nand.ecc.size, info.nand.ecc.bytes);
        printf("init bch calculate finished\n");
    }
    res = nand_bch_calculate_ecc(&info, buf, code);
    return res;
}
