/*
 * Generic ARM Programmable Interrupt Controller support.
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the LGPL
 */

#include "hw.h"
#include "uc32-misc.h"

/* Input 0 is IRQ and input 1 is FIQ.  */
static void uc32_pic_cpu_handler(void *opaque, int irq, int level)
{
    CPUState *env = (CPUState *)opaque;
    switch (irq) {
    case UC32_PIC_CPU_IRQ:
        if (level)
            cpu_interrupt(env, CPU_INTERRUPT_HARD);
        else
            cpu_reset_interrupt(env, CPU_INTERRUPT_HARD);
        break;

    case UC32_PIC_CPU_FIQ:
	fprintf(stderr, "uc32_fiq\n");
#if 0
        if (level)
            cpu_interrupt(env, CPU_INTERRUPT_FIQ);
        else
            cpu_reset_interrupt(env, CPU_INTERRUPT_FIQ);
#endif
        break;
    default:
        hw_error("uc32_pic_cpu_handler: Bad interrupt line %d\n", irq);
    }
}

qemu_irq *uc32_pic_init_cpu(CPUState *env)
{
    return qemu_allocate_irqs(uc32_pic_cpu_handler, env, 2);
}
