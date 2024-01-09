MEMORY
{
  FLASH0 (rx) : ORIGIN = 0x00000000, LENGTH = 0x180
  FLASH (rx) : ORIGIN = 0x00000180, LENGTH = 512k - 4k - 0x180
  DSRAM_A (rwx) : ORIGIN = 0x20000000, LENGTH = 16k
  DSRAM_B (rwx) : ORIGIN = 0x20040000, LENGTH = 80k
}
GROUP(libgcc.a libc.a libm.a libnosys.a)
ENTRY(Reset_Handler)
SECTIONS
{
 .vectors :
 {
  KEEP(*(.vectors))
  __Vectors_End = .;
  __Vectors_Size = __Vectors_End - __Vectors;
  __end__ = .;
 } > FLASH0
 .security_options :
 {
  . = ALIGN(4);
  KEEP(*(.security_options))
  . = ALIGN(4);
 } > FLASH
 .text :
 {
  *(.text*)
  KEEP(*(.init))
  KEEP(*(.fini))
  *crtbegin.o(.ctors)
  *crtbegin?.o(.ctors)
  *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
  *(SORT(.ctors.*))
  *(.ctors)
   *crtbegin.o(.dtors)
   *crtbegin?.o(.dtors)
   *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
   *(SORT(.dtors.*))
   *(.dtors)
  *(.rodata*)
  KEEP(*(.eh_frame*))
 } > FLASH
 .ARM.extab :
 {
  *(.ARM.extab* .gnu.linkonce.armextab.*)
 } > FLASH
 __exidx_start = .;
 .ARM.exidx :
 {
  *(.ARM.exidx* .gnu.linkonce.armexidx.*)
 } > FLASH
 __exidx_end = .;
 .copy.table :
 {
  . = ALIGN(4);
  __copy_table_start__ = .;
  LONG (__etext)
  LONG (__data_start__)
  LONG (__data_end__ - __data_start__)
  __copy_table_end__ = .;
 } > FLASH
 .zero.table :
 {
  . = ALIGN(4);
  __zero_table_start__ = .;
  LONG (__bss_start__)
  LONG (__bss_end__ - __bss_start__)
  __zero_table_end__ = .;
 } > FLASH
 __etext = .;
 .data : AT (__etext)
 {
  __data_start__ = .;
  *(vtable)
  *(.data*)
  . = ALIGN(4);
  PROVIDE_HIDDEN (__preinit_array_start = .);
  KEEP(*(.preinit_array))
  PROVIDE_HIDDEN (__preinit_array_end = .);
  . = ALIGN(4);
  PROVIDE_HIDDEN (__init_array_start = .);
  KEEP(*(SORT(.init_array.*)))
  KEEP(*(.init_array))
  PROVIDE_HIDDEN (__init_array_end = .);
  . = ALIGN(4);
  PROVIDE_HIDDEN (__fini_array_start = .);
  KEEP(*(SORT(.fini_array.*)))
  KEEP(*(.fini_array))
  PROVIDE_HIDDEN (__fini_array_end = .);
  KEEP(*(.jcr*))
  . = ALIGN(4);
  __data_end__ = .;
 } > DSRAM_B
 .bss :
 {
  . = ALIGN(4);
  __bss_start__ = .;
  *(.bss*)
  *(COMMON)
  . = ALIGN(4);
  __bss_end__ = .;
 } > DSRAM_B
 .heap (COPY):
 {
  __HeapBase = .;
  __end__ = .;
  end = __end__;
  KEEP(*(.heap*))
  __HeapLimit = .;
 } > DSRAM_A
 .stack_dummy (COPY):
 {
  KEEP(*(.stack*))
 } > DSRAM_A
 __StackTop = ORIGIN(DSRAM_A) + LENGTH(DSRAM_A);
 __StackLimit = __StackTop - SIZEOF(.stack_dummy);
 PROVIDE(__stack = __StackTop);
 ASSERT(__StackLimit >= __HeapLimit, "region DSRAM_A overflowed with stack")
}
