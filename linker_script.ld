/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-13
 * (c) NXP Semiconductors 2013-2015
 * Generated linker script file for LPC11U24/401
 * Created from generic_c.ld (LPCXpresso v7.5 (0 [Build 254] [2014-10-31] ))
 * By LPCXpresso v7.5.0 [Build 254] [2014-10-31]  on Sat Apr 04 17:36:49 BST 2015
 */


INCLUDE "mem.ld"

ENTRY(ResetISR)

SECTIONS
{

    /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))

        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */


        *(.after_vectors*)

    } >MFlash32

    .text : ALIGN(4)
    {
         *(.text*)

	KEEP(*(.init))
	KEEP(*(.fini))

	/* .ctors */
	*crtbegin.o(.ctors)
	*crtbegin?.o(.ctors)
	*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
	*(SORT(.ctors.*))
	*(.ctors)

	/* .dtors */
	*crtbegin.o(.dtors)
	*crtbegin?.o(.dtors)
	*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
	*(SORT(.dtors.*))
	*(.dtors)

        *(.rodata .rodata.* .constdata .constdata.*)
        . = ALIGN(4);

    } > MFlash32

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlash32
    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > MFlash32
    __exidx_end = .;

    _etext = .;


    /* DATA section for RamUsb2 */
    .data_RAM2 : ALIGN(4)
    {
       FILL(0xff)
       *(.ramfunc.$RAM2)
       *(.ramfunc.$RamUsb2)
    	*(.data.$RAM2*)
    	*(.data.$RamUsb2*)
       . = ALIGN(4) ;
    } > RamUsb2 AT>MFlash32

    /* MAIN DATA SECTION */


    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc8


	/* Main DATA section (RamLoc8) */
	.data : ALIGN(4)
	{
	   FILL(0xff)
	   _data = . ;
	   *(vtable)
	   *(.ramfunc*)
	   *(.data*)
	   . = ALIGN(4) ;

           /* preinit data */
           PROVIDE_HIDDEN (__preinit_array_start = .);
           KEEP(*(.preinit_array))
           PROVIDE_HIDDEN (__preinit_array_end = .);

           . = ALIGN(4);
           /* init data */
           PROVIDE_HIDDEN (__init_array_start = .);
           KEEP(*(SORT(.init_array.*)))
           KEEP(*(.init_array))
           PROVIDE_HIDDEN (__init_array_end = .);

           . = ALIGN(4);
           /* finit data */
           PROVIDE_HIDDEN (__fini_array_start = .);
           KEEP(*(SORT(.fini_array.*)))
           KEEP(*(.fini_array))
           PROVIDE_HIDDEN (__fini_array_end = .);
	   _edata = . ;
	} > RamLoc8 AT>MFlash32

    /* BSS section for RamUsb2 */
    .bss_RAM2 : ALIGN(4)
    {
    	*(.bss.$RAM2*)
    	*(.bss.$RamUsb2*)
       . = ALIGN(4) ;
    } > RamUsb2

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        __bss_start__ = .;
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        __bss_end__ = .;
        PROVIDE(end = .);
    } > RamLoc8

    /* NOINIT section for RamUsb2 */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
    	*(.noinit.$RAM2*)
    	*(.noinit.$RamUsb2*)
       . = ALIGN(4) ;
    } > RamUsb2

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RamLoc8

    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);
    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_RamLoc8 - 0);
}
