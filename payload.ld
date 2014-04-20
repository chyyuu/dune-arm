/* Simple linker script for the ucore kernel.
   See the GNU ld 'info' manual ("info ld") to learn the syntax. */

OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(kern_entry)

SECTIONS {
    /* Load the kernel at this address: "." means the current address */
    . = 0xc0008000;
    PROVIDE(__kernel_text_start = .);

    .text : {
/*	  *(.entrytext) */
      *(.text .stub .text.* .gnu.linkonce.t.*)
      *(.init.text)
      . = ALIGN(4);
        PROVIDE(__initcall_start = .);
        *(.initcall1.init)
        *(.initcall2.init)
        *(.initcall3.init)
        *(.initcall4.init)
        *(.initcall5.init)
        *(.initcall6.init)
        *(.initcall7.init)
        PROVIDE(__initcall_end = .);
    }

    .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) }
    PROVIDE_HIDDEN (__exidx_start = .);
    .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
    PROVIDE_HIDDEN (__exidx_end = .);

    . = ALIGN(4096);
    PROVIDE(__kernel_text_end = .);
    PROVIDE(etext = .); /* Define the 'etext' symbol to this value */

    /* Include debugging information in kernel memory */
    .stab : {
        PROVIDE(__STAB_BEGIN__ = .);
        *(.stab);
        PROVIDE(__STAB_END__ = .);
    }

    .stabstr : {
        PROVIDE(__STABSTR_BEGIN__ = .);
        *(.stabstr);
        PROVIDE(__STABSTR_END__ = .);
    }

    /* Adjust the address for the data segment to the next page */
    . = ALIGN(0x1000);

    /* The data segment */
    /* We don't link .initrd into another section because
     * QEMU only load .text and .data
     * However, our own loader will load all sections.
     */
	/* objcopy only copy text and data sections ... */
    .data : {
        *(.data)
	*(.data.rel.local)
        *(.init.data)
        . = ALIGN(4);
        *(.rodata .rodata.* .gnu.linkonce.r.*)
	*(.got.plt) *(.got)

     PROVIDE_HIDDEN (__preinit_array_start = .); 
     KEEP (*(.preinit_array))
     PROVIDE_HIDDEN (__preinit_array_end = .); 
     PROVIDE_HIDDEN (__init_array_start = .); 
     KEEP (*(.init_array*))
     PROVIDE_HIDDEN (__init_array_end = .); 
     PROVIDE_HIDDEN (__fini_array_start = .); 
     KEEP (*(.fini_array*))
     PROVIDE_HIDDEN (__fini_array_end = .); 
        . = ALIGN(4096);
    }

    PROVIDE(edata = .);

    .bss : {
        *(.bss)
    }
    . = ALIGN(0x1000);

    .percpu : {
        PROVIDE(__percpu_start = .);
        *(.percpu)
        PROVIDE(__percpu_end = .);
    }

    PROVIDE(end = .);

    /DISCARD/ : {
        *(.eh_frame .note.GNU-stack)
    }
}
