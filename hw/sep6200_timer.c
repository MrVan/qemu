
/*
 * ARM PrimeCell Timer modules.
 *
 * Copyright (c) 2005-2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "qemu-timer.h"
#include "sep6200.h"

/* Common timer implementation.  */


#define TIMER_CTRL_ENABLE       (1 << 0)
#define TIMER_CTRL_START        (1 << 1)
#define TIMER_CTRL_ONESHOT      (2 << 2)
#define TIMER_CTRL_PERIODIC	(1 << 2)
#define TIMER_CTRL_FREE		(0 << 2)
#define TIMER_CTRL_IM		(1 << 4)

typedef struct {
    ptimer_state *timer;
    uint32_t control;
    uint32_t limit;
    int freq;
    int int_level;
    qemu_irq irq;
    sep6200_clk clk;
} sep6200_timer_state;

/* Check all active timers, and schedule the next timer interrupt.  */

static void sep6200_timer_update(sep6200_timer_state *s)
{
    /* Update interrupts.  */
    if (s->int_level && ((s->control & TIMER_CTRL_IM) == 0)) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void sep6200_timer_clk_update(void *opaque, int line, int on)
{
	sep6200_timer_state *s = (sep6200_timer_state *)opaque;
	sep6200_timer_update(s);
    	s->freq = on? sep6200_clk_getrate(s->clk) : 0;
}
static uint32_t sep6200_timer_read(void *opaque, target_phys_addr_t offset)
{
    sep6200_timer_state *s = (sep6200_timer_state *)opaque;
    int level;

    //fprintf(stderr, "fanpeng timer_read offset %x\n", offset);
    switch (offset) {
    case 0: /* TimerLoad */
        return s->limit;
    case 4: /* TimerValue */
        return ptimer_get_count(s->timer);
    case 8: /* TimerControl */
        return s->control;
    case 12:
	level = s->int_level;
	s->int_level = 0;
	//fprintf(stderr,"read clear int status\n");
	sep6200_timer_update(s); //fanpeng for read clear int status; or it will trigger int continously
	return level;
    case 16: /* TimerMIS */
        if ((s->control & TIMER_CTRL_IM) == 1)
            	return 0;
	else
		return s->int_level;
    default:
        hw_error("sep6200_timer_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

/* Reset the timer limit after settings have changed.  */
static void sep6200_timer_recalibrate(sep6200_timer_state *s, int reload)
{
    uint32_t limit;

    if (((s->control & (TIMER_CTRL_PERIODIC | TIMER_CTRL_ONESHOT)) == 0) && ((s->control & TIMER_CTRL_ENABLE) == 1)) {
        /* Free running.  */
            limit = 0xffffffff;
    } else {
          /* Periodic.  */
          limit = s->limit;
    }
    ptimer_set_limit(s->timer, limit, reload);
}

static void sep6200_timer_write(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    sep6200_timer_state *s = (sep6200_timer_state *)opaque;
    int freq;
    //fprintf(stderr, "fanpeng timer_write offset %x\n", offset);

    switch (offset) {
    case 0: /* TimerLoad */
        s->limit = value;
        sep6200_timer_recalibrate(s, 1);
        break;
    case 4: /* TimerValue */
        /* ??? Linux seems to want to write to this readonly register.
           Ignore it.  */
        break;
    case 8: /* TimerControl */
        if (s->control & TIMER_CTRL_ENABLE) {
            /* Pause the timer if it is running.  This may cause some
               inaccuracy dure to rounding, but avoids a whole lot of other
               messyness.  */
            ptimer_stop(s->timer);
        }
        s->control = value;
        freq = s->freq;
        /* ??? Need to recalculate expiry time after changing divisor.  */
#if 0
        switch ((value >> 2) & 3) {
        case 1: freq >>= 4; break;
        case 2: freq >>= 8; break;
        }
#endif
        sep6200_timer_recalibrate(s, s->control & TIMER_CTRL_ENABLE);
        ptimer_set_freq(s->timer, freq);
        if (s->control & TIMER_CTRL_ENABLE) {
            /* Restart the timer if still enabled.  */
            ptimer_run(s->timer, (s->control & TIMER_CTRL_ONESHOT) != 0);
        }
        break;
    case 12: 
	fprintf(stderr, "write clear int timer\n");
	//s->int_level = 0; //I do not know but just a try fanpeng qemu
        break;
    case 16:
	break;
    default:
        hw_error("sep6200_timer_write: Bad offset %x\n", (int)offset);
    }
    sep6200_timer_update(s);
}

static void sep6200_timer_tick(void *opaque)
{
    static int i = 0;
    i++;
    sep6200_timer_state *s = (sep6200_timer_state *)opaque;
    //s->int_level = 1;
    s->int_level = 1;
    //s->int_level = i%2; //just a test
//    fprintf(stderr, "sep6200_timer_tick\n");
    sep6200_timer_update(s);
}

static const VMStateDescription vmstate_sep6200_timer = {
    .name = "sep6200_timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(control, sep6200_timer_state),
        VMSTATE_UINT32(limit, sep6200_timer_state),
        VMSTATE_INT32(int_level, sep6200_timer_state),
        VMSTATE_PTIMER(timer, sep6200_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static void sep6200_timer_clk_setup(sep6200_timer_state *s)
{
	sep6200_clk_adduser(s->clk, qemu_allocate_irqs(sep6200_timer_clk_update, s, 1)[0]);
	s->freq = sep6200_clk_getrate(s->clk);
}

static sep6200_timer_state *sep6200_timer_init(sep6200_clk clk)
{
    sep6200_timer_state *s;
    QEMUBH *bh;

    s = (sep6200_timer_state *)g_malloc0(sizeof(sep6200_timer_state));
    s->freq = sep6200_clk_getrate(clk);
    s->control = TIMER_CTRL_IM;
    s->clk = clk;

    bh = qemu_bh_new(sep6200_timer_tick, s);
    s->timer = ptimer_init(bh);
    sep6200_timer_clk_setup(s);
    vmstate_register(NULL, -1, &vmstate_sep6200_timer, s);
    return s;
}

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    sep6200_timer_state *timer[3];
} sep6200_pit_state;

static uint64_t sep6200_pit_read(void *opaque, target_phys_addr_t offset,
                             unsigned size)
{
    sep6200_pit_state *s = (sep6200_pit_state *)opaque;
    int n;

    if (offset >= 0x100)
        hw_error("sp804_write: Bad offset\n");
    n = offset >> 5;
    if (n > 2) {
        hw_error("sp804_write: Bad timer %d\n", n);
    }

    return sep6200_timer_read(s->timer[n], offset - (n << 5));
}

static void sep6200_pit_write(void *opaque, target_phys_addr_t offset,
                          uint64_t value, unsigned size)
{
    sep6200_pit_state *s = (sep6200_pit_state *)opaque;
    int n;

    if (offset >= 0x100)
        hw_error("sp804_write: Bad offset\n");
    n = offset >> 5;
    if (n > 2) {
        hw_error("sp804_write: Bad timer %d\n", n);
    }

    sep6200_timer_write(s->timer[n], offset - (n << 5), value);
}

static const MemoryRegionOps sep6200_pit_ops = {
    .read = sep6200_pit_read,
    .write = sep6200_pit_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int sep6200_pit_init(SysBusDevice *dev)
{
    sep6200_pit_state *s = FROM_SYSBUS(sep6200_pit_state, dev);

    /* Timer 0 runs at the system clock speed (40MHz).  */
    sep6200_clk clk;
    clk = sep6200_findclk(&sep6200_state1, "timer_clk");
    s->timer[0] = sep6200_timer_init(clk); //???40M? not sure
    /* The other two timers run at 1MHz.  */
    s->timer[1] = sep6200_timer_init(clk);
    s->timer[2] = sep6200_timer_init(clk);

    sysbus_init_irq(dev, &s->timer[0]->irq);
    sysbus_init_irq(dev, &s->timer[1]->irq);
    sysbus_init_irq(dev, &s->timer[2]->irq);

    memory_region_init_io(&s->iomem, &sep6200_pit_ops, s, "sep6200_timer", 0x1000);
    sysbus_init_mmio_region(dev, &s->iomem);
    /* This device has no state to save/restore.  The component timers will
       save themselves.  */
    return 0;
}

static void sep6200_timer_register_devices(void)
{
    sysbus_register_dev("sep6200_timer", sizeof(sep6200_pit_state), sep6200_pit_init);
}

device_init(sep6200_timer_register_devices)
