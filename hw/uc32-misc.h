/*
 * Misc ARM declarations
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the LGPL.
 *
 */

#ifndef ARM_MISC_H
#define ARM_MISC_H 1

#include "memory.h"

/* The CPU is also modeled as an interrupt controller.  */
#define UC32_PIC_CPU_IRQ 0
#define UC32_PIC_CPU_FIQ 1
qemu_irq *uc32_pic_init_cpu(CPUState *env);


/* arm_boot.c */
struct uc32_boot_info {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
    target_phys_addr_t loader_start;
    int nb_cpus;
    int board_id;
    /* Used internally by arm_boot.c */
    int is_linux;
    target_phys_addr_t initrd_size;
    target_phys_addr_t entry;
};
void uc32_load_kernel(CPUState *env, struct uc32_boot_info *info);

/* Multiplication factor to convert from system clock ticks to qemu timer
   ticks.  */
extern int system_clock_scale;

#endif /* !ARM_MISC_H */
