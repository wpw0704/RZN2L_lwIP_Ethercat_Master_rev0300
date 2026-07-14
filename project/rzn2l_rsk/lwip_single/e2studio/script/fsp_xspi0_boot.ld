/*
                  Linker File for Renesas RZ/N2 FSP
*/

INCLUDE memory_regions.ld

INCLUDE redefinition_memory_regions_gcc.h

/*============================================================================*/
/* A D D R E S S   S Y M B O L                                                */
/*============================================================================*/

/* Determine whether it is a primary project or not. */
_RZN_ORDINAL = (0 == SECONDARY_PRV) ? 1 : 2;

/* The start address of the xSPI0_CS0_SPACE in the secondary project is located 0x20000 from the end address of the primary project. */
xSPI0_CS0_SPACE_ALIGN_START        = ALIGN(xSPI0_CS0_SPACE_PRV_START, 0x00020000);
xSPI0_CS0_SPACE_ALIGN_LENGTH       = xSPI0_CS0_SPACE_PRV_LENGTH - (xSPI0_CS0_SPACE_PRV_LENGTH % 0x00020000);

/* The start address of the xSPI1_CS0_SPACE in the secondary project is located 0x20000 from the end address of the primary project. */
xSPI1_CS0_SPACE_ALIGN_START        = ALIGN(xSPI1_CS0_SPACE_PRV_START, 0x00020000);
xSPI1_CS0_SPACE_ALIGN_LENGTH       = xSPI1_CS0_SPACE_PRV_LENGTH - (xSPI1_CS0_SPACE_PRV_LENGTH % 0x00020000);

/* The start address of the CS0_SPACE in the secondary project is located 0x20000 from the end address of the primary project. */
CS0_SPACE_ALIGN_START              = ALIGN(CS0_SPACE_PRV_START, 0x00020000);
CS0_SPACE_ALIGN_LENGTH             = CS0_SPACE_PRV_LENGTH - (CS0_SPACE_PRV_LENGTH % 0x00020000);

/* The start address of the SYSTEM_RAM in the secondary project is located 0x20000 from the end address of the primary project. */
SYSTEM_RAM_ALIGN_START             = ALIGN(SYSTEM_RAM_PRV_START, 0x00020000);
SYSTEM_RAM_ALIGN_LENGTH            = SYSTEM_RAM_PRV_LENGTH - (SYSTEM_RAM_PRV_LENGTH % 0x00020000);

/* This is the offset of the mirror region of SYSTEM_RAM. It varies depending on the device. */
SYSTEM_RAM_MIRROR_OFFSET           = DEFINED(SYSTEM_RAM_START) ?
                                         DEFINED(SYSTEM_RAM_MIRROR_START) ?
                                             0x20000000 :
                                             (DEFINED(CR52_0) || DEFINED(CR52_1)) ?
                                                 0x00000000 :
                                                 0x00200000 :
                                         0;

/* Specifies the region where the user program is placed. */
/****************************************************************
RAM_START                          = (DEFINED(ATCM_CR52_0_START) || DEFINED(ATCM_CR52_1_START)) ? ATCM_PRV_START : SYSTEM_RAM_ALIGN_START;
RAM_LENGTH                         = (DEFINED(ATCM_CR52_0_LENGTH) || DEFINED(ATCM_CR52_1_LENGTH)) ? ATCM_PRV_LENGTH : SYSTEM_RAM_ALIGN_LENGTH;
*****************************************************************/
RAM_START                         = SYSTEM_RAM_PRV_START;
RAM_LENGTH                        = SYSTEM_RAM_PRV_START+0x100000;

TEXT_OFFSET                        = (DEFINED(ATCM_CR52_0_START) || DEFINED(ATCM_CR52_1_START)) ? 0x00000100 : 0x00010000;
TEXT_ADDRESS                       = RAM_START + TEXT_OFFSET;

/* Specifies the region where the loader program (SSBL) is placed. */
LOADER_START                       = (DEFINED(BTCM_CR52_0_START) || DEFINED(BTCM_CR52_1_START)) ? BTCM_PRV_START : SYSTEM_RAM_ALIGN_START;
LOADER_LENGTH                      = (DEFINED(BTCM_CR52_0_LENGTH) || DEFINED(BTCM_CR52_1_LENGTH)) ? BTCM_PRV_LENGTH : SYSTEM_RAM_ALIGN_LENGTH;
LOADER_TEXT_OFFSET                 = (DEFINED(BTCM_CR52_0_START) || DEFINED(BTCM_CR52_1_START)) ? 0x00002000 : 0x00001000;
LOADER_TEXT_ADDRESS                = LOADER_START + LOADER_TEXT_OFFSET;

/* Specifies the region where the readonly program is placed. */
ROM_START                          = xSPI0_CS0_SPACE_ALIGN_START;
ROM_LENGTH                         = xSPI0_CS0_SPACE_ALIGN_LENGTH;
LOADER_PARAM_ADDRESS               = ROM_START;
FLASH_CONTENTS_OFFSET              = 0x00000050;
FLASH_CONTENTS_ADDRESS             = LOADER_PARAM_ADDRESS + FLASH_CONTENTS_OFFSET;

/* Specifies the region where the vector table is placed. */
INTVEC_ADDRESS                     = RAM_START;

/* Specifies the region where the g_bsp_image_info_cpu is placed. */
IMAGE_INFO_OFFSET                  = (DEFINED(ATCM_CR52_0_START) || DEFINED(ATCM_CR52_1_START)) ? 0x00000080 : 0x00000800;
IMAGE_INFO_ADDRESS                 = INTVEC_ADDRESS + IMAGE_INFO_OFFSET;

/* Calculate the offset address of the noncache region. */
NONCACHE_OFFSET                    = DEFINED(NONCACHE_START) ? 0x00080000 : 0;
SHARED_NONCACHE_BUFFER_OFFSET      = DEFINED(NONCACHE_START) ? 0x00020000 : 0;
LCDC_FRAME_BUFFER_OFFSET           = ((0 != DDR_MIRROR0_PRV_START) || (0 != DDR_MIRROR1_PRV_START)) ? 0x04000000 : 0;

/* Calculate the usage of the noncache region. For multi-core projects, the calculation also takes into account the usage on the primary core. */
NONCACHE_USAGE                     = (DEFINED(NONCACHE_START) && (NONCACHE_OFFSET >= NONCACHE_PRV_LENGTH)) ? ALIGN((NONCACHE_OFFSET - NONCACHE_PRV_LENGTH), 0x20) : 0;

/* Calculate the start address of the noncache region. */
NONCACHE_AREA_INIT_START           = NONCACHE_PRV_START + NONCACHE_PRV_LENGTH - NONCACHE_OFFSET + NONCACHE_USAGE;
NONCACHE_AREA_INIT_LENGTH          = NONCACHE_OFFSET - SHARED_NONCACHE_BUFFER_OFFSET - NONCACHE_USAGE;

NONCACHE_AREA_START                = NONCACHE_AREA_INIT_START + SYSTEM_RAM_MIRROR_OFFSET;
NONCACHE_AREA_LENGTH               = NONCACHE_AREA_INIT_LENGTH;

/* Calculate the start address of the core common noncache region. */
SHARED_NONCACHE_BUFFER_INIT_START  = NONCACHE_PRV_START + NONCACHE_PRV_LENGTH - SHARED_NONCACHE_BUFFER_OFFSET;
SHARED_NONCACHE_BUFFER_INIT_LENGTH = SHARED_NONCACHE_BUFFER_OFFSET;

SHARED_NONCACHE_BUFFER_START       = SHARED_NONCACHE_BUFFER_INIT_START + SYSTEM_RAM_MIRROR_OFFSET;
SHARED_NONCACHE_BUFFER_LENGTH      = SHARED_NONCACHE_BUFFER_INIT_LENGTH;

/* Calculate the start address of the noncache region used by the LCDC. */
LCDC_FRAME_BUFFER_START            = (DEFINED(CR52_0) || DEFINED(CR52_1)) ?
                                        DDR_MIRROR1_PRV_START + DDR_MIRROR1_PRV_LENGTH - LCDC_FRAME_BUFFER_OFFSET :
                                        DDR_MIRROR0_PRV_START + DDR_MIRROR0_PRV_LENGTH - LCDC_FRAME_BUFFER_OFFSET;
LCDC_FRAME_BUFFER_LENGTH           = LCDC_FRAME_BUFFER_OFFSET;

/*============================================================================*/
/* M E M O R Y   R E G I O N                                                  */
/*============================================================================*/

MEMORY
{
    ATCM                   : ORIGIN = ATCM_PRV_START,                   LENGTH = ATCM_PRV_LENGTH
    BTCM                   : ORIGIN = BTCM_PRV_START,                   LENGTH = BTCM_PRV_LENGTH
    SYSTEM_RAM             : ORIGIN = SYSTEM_RAM_ALIGN_START,           LENGTH = SYSTEM_RAM_ALIGN_LENGTH
    SYSTEM_RAM_MIRROR      : ORIGIN = SYSTEM_RAM_MIRROR_PRV_START,      LENGTH = SYSTEM_RAM_MIRROR_PRV_LENGTH
    xSPI0_CS0_SPACE_MIRROR : ORIGIN = xSPI0_CS0_SPACE_MIRROR_PRV_START, LENGTH = xSPI0_CS0_SPACE_MIRROR_PRV_LENGTH
    xSPI0_CS1_SPACE_MIRROR : ORIGIN = xSPI0_CS1_SPACE_MIRROR_PRV_START, LENGTH = xSPI0_CS1_SPACE_MIRROR_PRV_LENGTH
    xSPI1_CS0_SPACE_MIRROR : ORIGIN = xSPI1_CS0_SPACE_MIRROR_PRV_START, LENGTH = xSPI1_CS0_SPACE_MIRROR_PRV_LENGTH
    xSPI1_CS1_SPACE_MIRROR : ORIGIN = xSPI1_CS1_SPACE_MIRROR_PRV_START, LENGTH = xSPI1_CS1_SPACE_MIRROR_PRV_LENGTH
    CS0_SPACE_MIRROR       : ORIGIN = CS0_SPACE_MIRROR_PRV_START,       LENGTH = CS0_SPACE_MIRROR_PRV_LENGTH
    CS2_SPACE_MIRROR       : ORIGIN = CS2_SPACE_MIRROR_PRV_START,       LENGTH = CS2_SPACE_MIRROR_PRV_LENGTH
    CS3_SPACE_MIRROR       : ORIGIN = CS3_SPACE_MIRROR_PRV_START,       LENGTH = CS3_SPACE_MIRROR_PRV_LENGTH
    CS5_SPACE_MIRROR       : ORIGIN = CS5_SPACE_MIRROR_PRV_START,       LENGTH = CS5_SPACE_MIRROR_PRV_LENGTH
    xSPI0_CS0_SPACE        : ORIGIN = xSPI0_CS0_SPACE_ALIGN_START,      LENGTH = xSPI0_CS0_SPACE_ALIGN_LENGTH
    xSPI0_CS1_SPACE        : ORIGIN = xSPI0_CS1_SPACE_PRV_START,        LENGTH = xSPI0_CS1_SPACE_PRV_LENGTH
    xSPI1_CS0_SPACE        : ORIGIN = xSPI1_CS0_SPACE_ALIGN_START,      LENGTH = xSPI1_CS0_SPACE_ALIGN_START
    xSPI1_CS1_SPACE        : ORIGIN = xSPI1_CS1_SPACE_PRV_START,        LENGTH = xSPI1_CS1_SPACE_PRV_LENGTH
    CS0_SPACE              : ORIGIN = CS0_SPACE_ALIGN_START,            LENGTH = CS0_SPACE_ALIGN_LENGTH
    CS2_SPACE              : ORIGIN = CS2_SPACE_PRV_START,              LENGTH = CS2_SPACE_PRV_LENGTH
    CS3_SPACE              : ORIGIN = CS3_SPACE_PRV_START,              LENGTH = CS3_SPACE_PRV_LENGTH
    CS5_SPACE              : ORIGIN = CS5_SPACE_PRV_START,              LENGTH = CS5_SPACE_PRV_LENGTH
    PCIE0_MIRROR           : ORIGIN = PCIE0_MIRROR_PRV_START,           LENGTH = PCIE0_MIRROR_PRV_LENGTH
    PCIE1_MIRROR           : ORIGIN = PCIE1_MIRROR_PRV_START,           LENGTH = PCIE1_MIRROR_PRV_LENGTH
    DDR_MIRROR0            : ORIGIN = DDR_MIRROR0_PRV_START,            LENGTH = DDR_MIRROR0_PRV_LENGTH
    DDR_MIRROR1            : ORIGIN = DDR_MIRROR1_PRV_START,            LENGTH = DDR_MIRROR1_PRV_LENGTH
    DDR                    : ORIGIN = DDR_PRV_START,                    LENGTH = DDR_PRV_LENGTH
    PCIE0                  : ORIGIN = PCIE0_PRV_START,                  LENGTH = PCIE0_PRV_LENGTH
    PCIE1                  : ORIGIN = PCIE1_PRV_START,                  LENGTH = PCIE1_PRV_LENGTH

    RAM_TEXT               : ORIGIN = RAM_START,                        LENGTH = RAM_LENGTH
    RAM_DATA               : ORIGIN = RAM_START,                        LENGTH = RAM_LENGTH
    LOADER_TEXT_STACK      : ORIGIN = LOADER_START,                     LENGTH = LOADER_LENGTH
    LOADER_DATA_STACK      : ORIGIN = LOADER_START,                     LENGTH = LOADER_LENGTH
    FLASH_CONTENTS         : ORIGIN = ROM_START,                        LENGTH = ROM_LENGTH
    NONCACHE               : ORIGIN = NONCACHE_AREA_START,              LENGTH = NONCACHE_AREA_LENGTH
    SHARED_NONCACHE_BUFFER : ORIGIN = SHARED_NONCACHE_BUFFER_START,     LENGTH = SHARED_NONCACHE_BUFFER_LENGTH
    LCDC_FRAME_BUFFER      : ORIGIN = LCDC_FRAME_BUFFER_START,          LENGTH = LCDC_FRAME_BUFFER_LENGTH
}

/*============================================================================*/
/* S E C T I O N   L A Y O U T                                                */
/*============================================================================*/

SECTIONS
{
/*-----------------------------------------------*/
/*  FLASH CONTENTS                               */
/*-----------------------------------------------*/
    /* Place the loader parameters to be used for booting. */
    .loader_param LOADER_PARAM_ADDRESS :
    {
        KEEP(*(.loader_param))
    } > FLASH_CONTENTS

    /* This section is for determining the LMA for each section. Nothing is actually placed here. */
    .flash_contents FLASH_CONTENTS_ADDRESS :
    {
        _mloader_text = .;
        . = (1 == _RZN_ORDINAL) ? . + (_loader_text_end - _loader_text_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _mloader_data = .;
        . = (1 == _RZN_ORDINAL) ? . + (_loader_data_end - _loader_data_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _mfvector = .;
        . = (1 == _RZN_ORDINAL) ? . + (_fvector_end - _fvector_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _mtext = .;
        . = (1 == _RZN_ORDINAL) ? . + (_text_end - _text_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _mextab = .;
        . = (1 == _RZN_ORDINAL) ? . + (__extab_end - __extab_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _mexidx = .;
        . = (1 == _RZN_ORDINAL) ? . + (__exidx_end - __exidx_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _mdata = .;
        . = (1 == _RZN_ORDINAL) ? . + (_data_end - _data_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(32) : .;

        _mnoncache = .;
        . = (1 == _RZN_ORDINAL) ? . + (_noncache_end  - _noncache_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(32) : .;
        _msecondary_noncache = .;
        . = (1 == _RZN_ORDINAL) ? . + (_secondary_noncache_end - _secondary_noncache_start) : .;
        . = (1 == _RZN_ORDINAL) ? ALIGN(32) : .;
        _msncbuffer = .;
        . = (1 == _RZN_ORDINAL) ? . + (_sncbuffer_end  - _sncbuffer_start) : .;
        flash_contents_end = .;
    } > FLASH_CONTENTS

    /* Place the secondary TCM binaries etc. */
    .flash_contents_secondary :
    {
        . = (1 == _RZN_ORDINAL) ? ALIGN(0x20000) : .;
        /* Place g_bsp_image_info_array. This array will store the address of the placed binary. */
        KEEP(*(.image_info_secondary))

        /* Places secondary binaries that will be copied to ATCM CR52_0. */
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _msecondary_atcm_cr520_start = .;
        KEEP(*(.secondary_atcm_cr520))
        _msecondary_atcm_cr520_end = .;

        /* Places secondary binaries that will be copied to BTCM CR52_0. */
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _msecondary_btcm_cr520_start = .;
        KEEP(*(.secondary_btcm_cr520))
        _msecondary_btcm_cr520_end = .;

        /* Places secondary binaries that will be copied to ATCM CR52_1. */
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _msecondary_atcm_cr521_start = .;
        KEEP(*(.secondary_atcm_cr521))
        _msecondary_atcm_cr521_end = .;

        /* Places secondary binaries that will be copied to BTCM CR52_1. */
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _msecondary_btcm_cr521_start = .;
        KEEP(*(.secondary_btcm_cr521))
        _msecondary_btcm_cr521_end = .;

        /*  Immediately after this place the secondary SYSTEM RAM binaries. */
        . = (1 == _RZN_ORDINAL) ? ALIGN(8) : .;
        _msecondray = .;
    } > FLASH_CONTENTS

/*-----------------------------------------------*/
/*  LOADER PROGRAM                               */
/*-----------------------------------------------*/
    /* Place the functions to be used in the loader program (SSBL). */
    LOADER_TEXT_IMAGE = (1 == _RZN_ORDINAL) ? _mloader_text : LOADER_TEXT_ADDRESS;
    .loader_text LOADER_TEXT_ADDRESS : AT (LOADER_TEXT_IMAGE)
    {
        _loader_text_start = .;
        *(.loader_text)
        */startup_core.o(.text*)
        */system_core.o(.text*)
        */startup.o(.text*)
        */system.o(.text*)
        */all*/bsp_*.o(.text*)
        */r_ioport.o(.text*)
        KEEP(*(.warm_start))
        KEEP(*(.loader_user_data*))
        _loader_text_end = .;
    } > LOADER_TEXT_STACK

    /* Place the variables to be used in the loader program (SSBL). */
    /* Since .loader_data is placed immediately after .loader_text, the start address is set to _loader_text_end.
     * If you move the associated section to another area, you will need to modify this symbol,
     * so check the usage_notes in the bsp and make the modifications. */
    LOADER_DATA_START = ALIGN(_loader_text_end, 0x200);
    LOADER_DATA_IMAGE = (1 == _RZN_ORDINAL) ? _mloader_data : LOADER_DATA_START;
    .loader_data LOADER_DATA_START : AT (LOADER_DATA_IMAGE)
    {
        _loader_data_start = .;
        __loader_data_start = .;
        */startup_core.o(.data*)
        */system_core.o(.data*)
        */startup.o(.data*)
        */system.o(.data*)
        */all*/bsp_*.o(.data*)
        */r_ioport.o(.data*)
        */bsp_mpu_core.o(.rodata*)
        */bsp_mmu_core.o(.rodata*)
        */bsp_register_protection.o(.rodata*)
        */bsp_tzc400.o(.rodata*)
        KEEP(*(.warm_start.data*))
        . = ALIGN(8);
        __loader_data_end = .;

        __loader_bss_start = .;
        */startup_core.o(.bss*)
        */system_core.o(.bss*)
        */startup.o(.bss*)
        */system.o(.bss*)
        */all*/bsp_*.o(.bss*)
        */r_ioport.o(.bss*)
        KEEP(*(.warm_start.bss*))
        KEEP(*(.ttbr))
        */startup_core.o(COMMON)
        */system_core.o(COMMON)
        */startup.o(COMMON)
        */system.o(COMMON)
        */all*/bsp_*.o(COMMON)
        */r_ioport.o(COMMON)
        . = ALIGN(8);
        __loader_bss_end = . ;

        KEEP(*(.software_reset)) /* Do not initialize */
        _loader_data_end = .;
    } > LOADER_DATA_STACK

/*-----------------------------------------------*/
/*  AArch32 STACK                                */
/*-----------------------------------------------*/
    /* Place the stacks used in AArch32. */
    .loader_stack (NOLOAD) :
    {
        /* Place the sys_stack here. */
        . = ALIGN(8);
        __SysStackBase = .;
        KEEP(*(.sys_stack))
        __SysStackLimit = .;

        /* Place the svc_stack here. */
        . = ALIGN(8);
        __SvcStackBase = .;
        KEEP(*(.svc_stack))
        __SvcStackLimit = .;

        /* Place the irq_stack here. */
        . = ALIGN(8);
        __IrqStackBase = .;
        KEEP(*(.irq_stack))
        __IrqStackLimit = .;

        /* Place the fiq_stack here. */
        . = ALIGN(8);
        __FiqStackBase = .;
        KEEP(*(.fiq_stack))
        __FiqStackLimit = .;

        /* Place the und_stack here. */
        . = ALIGN(8);
        __UndStackBase = .;
        KEEP(*(.und_stack))
        __UndStackLimit = .;

        /* Place the abt_stack here. */
        . = ALIGN(8);
        __AbtStackBase = .;
        KEEP(*(.abt_stack))
        __AbtStackLimit = .;
    } > LOADER_DATA_STACK

/*-----------------------------------------------*/
/*  VECTOR TABLE                                 */
/*-----------------------------------------------*/
    /* Place the vector table. */
    INTVEC_IMAGE = (1 == _RZN_ORDINAL) ? _mfvector : INTVEC_ADDRESS;
    .intvec INTVEC_ADDRESS : AT (INTVEC_IMAGE)
    {
        _fvector_start = .;
        KEEP(*(.intvec))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x080))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x100))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x180))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x200))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x280))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x300))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x380))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x400))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x480))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x500))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x580))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x600))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x680))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x700))
        . = ALIGN(0x80);
        KEEP(*(.intvec_0x780))
        _fvector_end = .;
    } > RAM_TEXT

/*-----------------------------------------------*/
/*  IMAGE INFO                                   */
/*-----------------------------------------------*/
    /* Place the variable that identifies the core of the secondary. */
    .image_info IMAGE_INFO_ADDRESS :
    {
        _image_info_start = .;
        KEEP(*(.image_info))
        _image_info_end = .;
    } > RAM_DATA

/*-----------------------------------------------*/
/*  USER PROGRAM                                 */
/*-----------------------------------------------*/
    /* Place the functions to be used in the user program (Programs other than SSBL). */
    TEXT_IMAGE = (1 == _RZN_ORDINAL) ? _mtext : TEXT_ADDRESS;
    .text TEXT_ADDRESS : AT (TEXT_IMAGE)
    {
        _text_start = .;

        *(.text*)

        KEEP(*(.reset_handler))

        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        _ctor_end = .;

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)
        _dtor_end = .;

        KEEP(*(.eh_frame*))

        /* Place the variable vector table. */
        _rvectors_start = .;
        KEEP(*(.rvectors))
        _rvectors_end = .;

        _text_end = .;
    } > RAM_TEXT

    /* Place the exception table (newlib). */
    EXTAB_IMAGE = (1 == _RZN_ORDINAL) ? _mextab : __extab_start;
    .ARM.extab : AT(EXTAB_IMAGE)
    {
        __extab_start = .;
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        __extab_end = .;
    } > RAM_TEXT

     /* Place the exception index (newlib). */
    EXIDX_IMAGE = (1 == _RZN_ORDINAL) ? _mexidx : __exidx_start;
    .ARM.exidx :  AT(EXIDX_IMAGE)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > RAM_TEXT

    /* Place the variables to be used in the user program (Programs other than SSBL). */
    /* Since .data is placed immediately after .ARM.exidx, the start address is set to __exidx_end.
     * If you move the associated section to another area, you will need to modify this symbol,
     * so check the usage_notes in the bsp and make the modifications. */
    DATA_START = __exidx_end;
    DATA_IMAGE = (1 == _RZN_ORDINAL) ? _mdata : DATA_START;
    .data DATA_START : AT (DATA_IMAGE)
    {
        _data_start = .;

        *(vtable)
        *(.data.*)
        *(.data)

        *(.rodata*)
        _erodata = .;

        . = ALIGN(8);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(8);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(8);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))

        . = ALIGN(8);

        /* Place the addresses of shared library functions. */
        __got_start = .;
        *(.got)
        *(.got.plt)
        __got_end = .;

        /* All data end */
        _data_end = .;
    } > RAM_DATA

    /* Place the variables in the user program that do not have initial values. */
    .bss :
    {
        . = ALIGN(8);
        __bss_start__ = .;
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        __bss_end__ = .;
        _ebss = .;
        _end = .;
    } > RAM_DATA

/*-----------------------------------------------*/
/*  HEAP                                         */
/*-----------------------------------------------*/
    /* Place the variables to secure heap area. */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __HeapBase = .;
        KEEP(*(.heap))
        __HeapLimit = .;
    } > RAM_DATA

/*-----------------------------------------------*/
/*  AArch64 STACK                                */
/*-----------------------------------------------*/
    /* Place the stacks used in AArch64. */
    .aarch64_stack (NOLOAD) :
    {
        . = ALIGN(8);
        __AArch64StackBase = .;
        KEEP(*(.aarch64_stack*))
        __AArch64StackLimit = .;
    } > RAM_DATA

/*-----------------------------------------------*/
/*  THREAD STACK                                 */
/*-----------------------------------------------*/
    /* Place the stack mainly used in free-RTOS. */
    .thread_stack (NOLOAD):
    {
        . = ALIGN(8);
        __ThreadStackBase = .;
        KEEP(*(.stack*))
        __ThreadStackLimit = .;
    } > RAM_DATA

/*-----------------------------------------------*/
/*  NONCACHE                                     */
/*-----------------------------------------------*/
    /* Place the noncache variables. */
    NONCACHE_IMAGE = (1 == _RZN_ORDINAL) ? _mnoncache : NONCACHE_AREA_INIT_START;
    .noncache NONCACHE_AREA_START : AT(NONCACHE_IMAGE)
    {
        _noncache_start = .;

        /* Place the noncache variables. */
        . = ALIGN(8);
        _data_noncache_start = .;
        KEEP(*(.data_noncache*))
        _data_noncache_end = .;

        /* Place the noncache variables to be used in DMAC_LINK_MODE. */
        . = ALIGN(8);
        _dmac_link_mode_start = .;
        KEEP(*(.dmac_link_mode*))
        _dmac_link_mode_end = .;

        /* Place the core non-common noncache variables. */
        . = ALIGN(32);
        _ncbuffer_start = .;
        KEEP(*(.noncache_buffer*))
        _ncbuffer_end = .;

        _noncache_end = .;

        /* Place the secondary noncache section binaries. */
        . = ALIGN(32);
        _secondary_noncache_start = .;
        KEEP(*(.secondary_noncache))
        _secondary_noncache_end = .;
    } > NONCACHE

    /* Place the core common noncache variables. */
    SHARED_NONCACHE_BUFFER_IMAGE = (1 == _RZN_ORDINAL) ? _msncbuffer : SHARED_NONCACHE_BUFFER_INIT_START;
    .shared_noncache_buffer SHARED_NONCACHE_BUFFER_START : AT(SHARED_NONCACHE_BUFFER_IMAGE)
    {
        _sncbuffer_start = .;
        . = ALIGN(32);
        KEEP(*(.shared_noncache_buffer*))
        _sncbuffer_end = .;
    } > SHARED_NONCACHE_BUFFER

    /* Place the noncache variables to be used in LCDC. */
    .lcdc_frame_buffer (NOLOAD) :
    {
        . = ALIGN(512);
        _lcdc_frame_buffer_start = .;
        KEEP(*(.lcdc_frame_buffer*))
        _lcdc_frame_buffer_end = .;
    } > LCDC_FRAME_BUFFER

/*-----------------------------------------------*/
/*  SECONDARY                                    */
/*-----------------------------------------------*/
    /* Places the secondary binaries that will be used to SYSTEM_RAM. */
    SECONDARY_START = (DEFINED(ATCM_CR52_0_START) || DEFINED(ATCM_CR52_1_START)) ?  SYSTEM_RAM_ALIGN_START : ALIGN(__ThreadStackLimit, 0x00020000);
    SECONDARY_IMAGE = (1 == _RZN_ORDINAL) ? _msecondray : SECONDARY_START;
    .secondary SECONDARY_START : AT (SECONDARY_IMAGE)
    {
        . = ALIGN(0x20000);
        _secondary_start = .;
        KEEP(*(.secondary))
        _secondary_end = .;
    } > SYSTEM_RAM
}

/*============================================================================*/
/* D D S C   S Y M B O L                                                      */
/*============================================================================*/

/* Defines the ddsc symbol, which is used in multi-core projects and to calculate the usage of each region.
 * If you change the memory layout, modify each __ddsc_***_END value to be the end address of that region. */

/* xSPI0_CS0_SPACE region */
__ddsc_xSPI0_CS0_SPACE_START  = ORIGIN(xSPI0_CS0_SPACE);
__ddsc_xSPI0_CS0_SPACE_END    = (1 == _RZN_ORDINAL) ? flash_contents_end : __ddsc_xSPI0_CS0_SPACE_START;

/* ATCM_CR52_0 region */
__ddsc_ATCM_CR52_0_START      = ORIGIN(ATCM);

/******************************
__ddsc_ATCM_CR52_0_END        = DEFINED(ATCM_CR52_0_START) ? __ThreadStackLimit : __ddsc_ATCM_CR52_0_START;
*******************************/
__ddsc_ATCM_CR52_0_END        = __ddsc_ATCM_CR52_0_START;
/* BTCM_CR52_0 region */
__ddsc_BTCM_CR52_0_START      = ORIGIN(BTCM);
__ddsc_BTCM_CR52_0_END        = DEFINED(BTCM_CR52_0_START) ? __AbtStackLimit : __ddsc_BTCM_CR52_0_START;

/* ATCM_CR52_1 region */
__ddsc_ATCM_CR52_1_START      = ORIGIN(ATCM);
__ddsc_ATCM_CR52_1_END        = DEFINED(ATCM_CR52_1_START) ? __ThreadStackLimit : __ddsc_ATCM_CR52_1_START;

/* BTCM_CR52_1 region */
__ddsc_BTCM_CR52_1_START      = ORIGIN(BTCM);
__ddsc_BTCM_CR52_1_END        = DEFINED(BTCM_CR52_1_START) ? __AbtStackLimit : __ddsc_BTCM_CR52_1_START;

/* SYSTEM_RAM region */
__ddsc_SYSTEM_RAM_START       = ORIGIN(SYSTEM_RAM);
/*********************************************************
__ddsc_SYSTEM_RAM_END         = (DEFINED(ATCM_CR52_0_START) || DEFINED(ATCM_CR52_1_START)) ? __ddsc_SYSTEM_RAM_START : __ThreadStackLimit;
**********************************************************/
__ddsc_SYSTEM_RAM_END        = __ThreadStackLimit;

/* NONCACHE region */
__ddsc_NONCACHE_START         = NONCACHE_AREA_START - SYSTEM_RAM_MIRROR_OFFSET;
__ddsc_NONCACHE_END           = DEFINED(NONCACHE_START) ? (_noncache_end - SYSTEM_RAM_MIRROR_OFFSET) : __ddsc_NONCACHE_START;

/* Used to determine whether a project is the primary project or not in a multi-core project. */
__ddsc_SECONDARY_START        = (1 == _RZN_ORDINAL) ? 0 : 1;
__ddsc_SECONDARY_END          = 1;