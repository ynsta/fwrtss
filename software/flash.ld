/*
 * Copyright (c) 2014, Stany MARCEL <stanypub@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
OUTPUT_FORMAT("elf32-littlearm")
ENTRY(Reset_Handler)

MEMORY
{
  rom (xr)  : ORIGIN = 0x08000000, LENGTH = 64K
  ram (xrw) : ORIGIN = 0x20000000, LENGTH = 16K
}

SECTIONS
{
  .isr_vectors :
  {
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } > rom

  .text :
  {
    . = ALIGN(4);
    *(.text .text*)
    *(.gnu.warning)
    *(.gnu.linkonce.t*)
    . = ALIGN(4);

     /* These are for running static constructors and destructors under ELF.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))

    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.jcr))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .jcr))
    KEEP (*crtend.o(.jcr))

    *(.rodata .rodata.*)
    *(.gnu.linkonce.r.*)

    . = ALIGN(0x4);
    *(.gcc_except_table)

    . = ALIGN(0x4);
    *(.eh_frame)

    . = ALIGN(0x4);
    KEEP (*(.init))
    . = ALIGN(0x4);
    KEEP (*(.fini))
    . = ALIGN(4);

    *(.lit)

    . = ALIGN(4);

   } > rom

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > rom
  .ARM       : { *(.ARM.exidx*) } > rom

  .data :
  {
    *(.got.plt) *(.got)
    *(.shdata)
    *(.data .data*)
    *(.gnu.linkonce.d*)
    . = ALIGN(4);
  } > ram AT> rom

  .bss :
  {
    *(.shbss)
    *(.bss .bss*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
   } > ram

  .stab 0 (NOLOAD) :
  {
    *(.stab)
  }

  .stabstr 0 (NOLOAD) :
  {
    *(.stabstr)
  }


  .stack :
  {
    . = ORIGIN(ram) + LENGTH(ram);
  } > ram

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libc_s.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  _sisr_vectors		= ADDR(.isr_vectors);
  _nisr_vectors		= SIZEOF(.isr_vectors);
  _eisr_vectors		= _sisr_vectors + _nisr_vectors;

  _stext		= ADDR(.text);
  _ntext		= SIZEOF(.text);
  _etext		= _stext + _ntext;

  _sdata		= ADDR(.data);
  _sidata		= LOADADDR(.data);
  _ndata		= SIZEOF(.data);
  _edata		= _sdata  + _ndata;
  _eidata		= _sidata + _ndata;

  _sbss			= ADDR(.bss);
  _nbss			= SIZEOF(.bss);
  _ebss			= _sbss + _nbss;

  _sstack		= ADDR(.stack);
  _nstack		= SIZEOF(.stack);
  _estack		= _sstack + _nstack;

  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
}
