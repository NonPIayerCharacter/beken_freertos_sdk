/* 
 * Script for GNU linker.
 * Describes layout of sections, location of stack.
 *
 * In this case vectors are at location 0 (reset @ 0x08)
 *
 * +------------+  0x00400020
 * data         |
 * end
 * |(heap)      |
 * .            .
 * .            .
 * |(heap limit)|
 *
 * |- - - - - - |
 * stack bottom    256k
 * +------------+  
 *
 * +------------+  0x0000000
 * |vectors     |
 * |            |
 * |------------+
 * |text        |  
 * |data        |
 * |            |  1024k
 * +------------+
 */


/* Split memory into area for vectors and ram */
MEMORY
{
	flash  (rx) : ORIGIN = 0x00000000, LENGTH = 1M
	itcm   (rwx): ORIGIN = 0x003F0000, LENGTH = 16k
	ram    (rw!x): ORIGIN = 0x00400100, LENGTH = 256k - 0x100
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_vector_start);
_vector_start = ORIGIN(flash);

SECTIONS
{
/* vectors go to vectors region */
	. = ORIGIN(flash);
	.vectors : 
	{ 
		KEEP(*(*.vectors))
		KEEP( *(*.boot))
	} > flash

/* instructions go to the text region*/

    . = ORIGIN(itcm);
    .itcm.code ALIGN(8) :
    {
        *(.text.flash_bypass_wr_sr_cb)
    } > itcm AT>flash
    _itcmcode_flash_begin = LOADADDR(.itcm.code);
    _itcmcode_ram_begin = ADDR(.itcm.code);
    _itcmcode_ram_end = _itcmcode_ram_begin + SIZEOF(.itcm.code);

    . = ALIGN(0x8);
/* code, instructions.for example: i=i+1; */
	.text : 
	{
	    *(.text)
	    *(.text.*)
	    *(.stub)
	    /* .gnu.warning sections are handled specially by elf32.em.  */
	    *(.gnu.warning)
	    *(.gnu.linkonce.t*)
	    *(.glue_7t) *(.glue_7)
	} > flash	

/* read only data.for example: const int rom_data[3]={1,2,3}; */
	.rodata ALIGN(8) : 
	{
		*(.rodata)
		*(.rodata.*)
		*(.gnu.linkonce.r*)
	} > flash
	
	.ARM.exidx :
	{
	    *(.ARM.exidx*)
	    *(.gnu.linkonce.armexidx.*)
	} > flash

	. = ORIGIN(ram);
/* keep stack front of RAM to avoid COLOR reset to zero */
	.stack ALIGN(8):
	{
		*boot_handlers.O(.bss .bss.* .scommon .sbss .dynbss COMMON)
	} > ram						/* in RAM */

/* globals.for example: int ram_data[3]={4,5,6}; */		/* VMA in RAM, but keep LMA in flash */
	_begin_data = .;
	.data ALIGN(8):
	{
		*(.data .data.*)
		*(.sdata)
		*(.gnu.linkonce.d*)
		SORT(CONSTRUCTORS)
	} >ram AT>flash
	_end_data = .;

	/* Loader will copy data from _flash_begin to _ram_begin..ram_end */
	_data_flash_begin = LOADADDR(.data);
	_data_ram_begin = ADDR(.data);
	_data_ram_end = .;

/* uninitialized data section - global   int i; */
	.bss ALIGN(8):
	{
		_bss_start = .;
		*(.bss .bss.*)
		*(.scommon)
		*(.sbss)
		*(.dynbss)
		*(COMMON)
		/* Align here to ensure that the .bss section occupies space up to
		_end.  Align after .bss to ensure correct alignment even if the
		.bss section disappears because there are no input sections.  */
		. = ALIGN(32 / 8);
		_bss_end = .;
	} > ram						/* in RAM */

	. = ALIGN (8);
	_empty_ram = .;

/* This symbol defines end of code/data sections. Heap starts here. */
	PROVIDE(end    	  = .);
}
 GROUP(
   libgcc.a
   libg.a
   libc.a
   libm.a
   libnosys.a
 )
