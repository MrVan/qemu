
/*
 * Arm PrimeCell PL190 Vector Interrupt Controller
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"

/* The number of virtual priority levels.  16 user vectors plus the
   unvectored IRQ.  Chained interrupts would require an additional level
   if implemented.  */

#define PL190_NUM_PRIO 17

typedef struct {
    SysBusDevice busdev;
    uint32_t intselect_l;
    uint32_t intselect_h;
    uint32_t int_enable_l;
    uint32_t int_enable_h;
    uint32_t soft_int_l;
    uint32_t soft_int_h;
    uint32_t allmask;
    uint32_t rawintsrc_l;
    uint32_t rawintsrc_h;
    uint32_t irq_table_base;
    uint32_t fiq_table_base;
    int32_t irqnum;
    int32_t fiqnum;
    qemu_irq irq;
    qemu_irq fiq;
} sep612_pic_state;

static const unsigned char sep612_pic_id[] =
{ 0x90, 0x11, 0x04, 0x00, 0x0D, 0xf0, 0x05, 0xb1 };

typedef struct int_level {
	unsigned int l;
	unsigned int h;
}int_level;

static int find_highest_bit(uint32_t a)
{
	int i;
	for (i = 31; i >= 0; i--) {
		if (a & (1 << i))
			return i;
	}
	return -1;

}
static inline int_level sep612_pic_irq_level(sep612_pic_state *s)
{
	int_level l;

	l.l = (s->soft_int_l | s->rawintsrc_l) & (~s->intselect_l) & s->int_enable_l;
	l.h = (s->soft_int_h | s->rawintsrc_h) & (~s->intselect_h) & s->int_enable_h;

	return l;
}

static inline int_level sep612_pic_fiq_level(sep612_pic_state *s)
{
	static int_level l;

	l.l = (s->soft_int_l | s->rawintsrc_l) & (s->intselect_l) & s->int_enable_l;
	l.h = (s->soft_int_h | s->rawintsrc_h) & (s->intselect_h) & s->int_enable_h;

	return l;
}
/* Update interrupts.  */
static void sep612_pic_update(sep612_pic_state *s)
{
    int_level level1 = sep612_pic_irq_level(s);
    int_level level2 = sep612_pic_fiq_level(s);
    int set;

    set = level1.l && ((s->allmask&0x1) ? 0x0:0x1);
    set |= level1.h && ((s->allmask&0x1) ? 0x0:0x1);
    qemu_set_irq(s->irq, set);

    set = level2.l && ((s->allmask&0x2) ? 0x0:0x1);
    set |= level2.h && ((s->allmask&0x2) ? 0x0:0x1);
    qemu_set_irq(s->fiq, set);
}

static void sep612_pic_set_irq(void *opaque, int irq, int level)
{
    sep612_pic_state *s = (sep612_pic_state *)opaque;

    if (irq < 32) {
   	if (level){
		s->soft_int_l |= 1u << irq;
		s->rawintsrc_l |= 1u << irq;
	}
	else {
		s->soft_int_l &= ~(1u << irq);
		s->rawintsrc_l &= ~(1u << irq);
	}
    } else if (irq < 64) {
   	if (level) {
		s->soft_int_h |= 1u << (irq - 32);
		s->rawintsrc_h |= 1u << (irq - 32);
	}
	else {
		s->soft_int_h &= ~(1 << (irq - 32));
		s->rawintsrc_h &= ~(1u << (irq - 32));
	}
    }

    sep612_pic_update(s);
}

#if 0
static void sep612_pic_update_vectors(sep612_pic_state *s)
{
    uint32_t mask;
    int i;
    int n;

    mask = 0;
    for (i = 0; i < 16; i++)
      {
        s->prio_mask[i] = mask;
        if (s->vect_control[i] & 0x20)
          {
            n = s->vect_control[i] & 0x1f;
            mask |= 1 << n;
          }
      }
    s->prio_mask[16] = mask;
    sep612_pic_update(s);
}
#endif

static uint32_t sep612_pic_read(void *opaque, target_phys_addr_t offset)
{
    sep612_pic_state *s = (sep612_pic_state *)opaque;
    int irqnum_l;
    int irqnum_h;
    int fiqnum_l;
    int fiqnum_h;

#if 0// offset > 0x100 in sep612 may not be used right? fanpeng freenix not sure
    if (offset >= 0xfe0 && offset < 0x1000) {
        return sep612_pic_id[(offset - 0xfe0) >> 2];
    }
    if (offset >= 0x100 && offset < 0x140) {
        return s->vect_addr[(offset - 0x100) >> 2];
    }
    if (offset >= 0x200 && offset < 0x240) {
        return s->vect_control[(offset - 0x200) >> 2];
    }
#endif
    switch (offset >> 2) {
    case 0: /* SELECT */
        /* This is a readonly register, but linux tries to write to it
           anyway.  Ignore the write.  */
	return s->intselect_l;
    case 1:
	return s->intselect_h;
    case 2:
	return s->int_enable_l;
    case 3:
	return s->int_enable_h;
    case 4:
    case 5:
	fprintf(stderr, "write only\n");
	return 0;
    case 6:
	return s->soft_int_l;
    case 7:
	return s->soft_int_h;
    case 8:
    case 9:
	fprintf(stderr, "write only\n");
	return 0;
    case 10:
	return s->allmask & 0x3; //fanpeng ????????????????
    case 12:
	return s->rawintsrc_l;
    case 13:
	return s->rawintsrc_h;
    case 14:
	return (s->soft_int_l | s->rawintsrc_l) & (~s->intselect_l);
    case 15:
	return (s->soft_int_h | s->rawintsrc_h) & (~s->intselect_h);
    case 16:
	return (s->soft_int_l | s->rawintsrc_l) & (s->intselect_l);
    case 17:
	return (s->soft_int_h | s->rawintsrc_h) & (s->intselect_h);
    case 18:
	return (s->soft_int_l | s->rawintsrc_l) & (~s->intselect_l) & s->int_enable_l;
    case 19:
	return (s->soft_int_h | s->rawintsrc_h) & (~s->intselect_h) & s->int_enable_h;
    case 20:
	return (s->soft_int_l | s->rawintsrc_l) & (s->intselect_l) & s->int_enable_l;
    case 21:
	return (s->soft_int_h | s->rawintsrc_h) & (s->intselect_h) & s->int_enable_h;
    case 22:
	return (s->soft_int_l | s->rawintsrc_l) & (~s->intselect_l) & s->int_enable_l;
	//return s->irqpending_l; //here do not consider priority mask  may be
	//wrong. will be fixed in future
    case 23:
	return (s->soft_int_h | s->rawintsrc_h) & (~s->intselect_h) & s->int_enable_h;
	//return s->irqpending_h;
    case 24:
	return (s->soft_int_l | s->rawintsrc_l) & (s->intselect_l) & s->int_enable_l;
	//return f->fiqpending_l;
    case 25:
	return (s->soft_int_h | s->rawintsrc_h) & (s->intselect_h) & s->int_enable_h;
	//return f->fiqpending_h;
    case 28:
	return s->irq_table_base;
    case 29:
	return s->fiq_table_base;
    case 30:
	irqnum_l = find_highest_bit((s->soft_int_l | s->rawintsrc_l) & (~s->intselect_l) & s->int_enable_l);
	irqnum_h = find_highest_bit((s->soft_int_h | s->rawintsrc_h) & (~s->intselect_h) & s->int_enable_h);
	if (irqnum_h != -1)
		s->irqnum = 32 + irqnum_h;
	else
		s->irqnum = irqnum_l;
	return s->irqnum;
    case 31:
	fiqnum_l = find_highest_bit((s->soft_int_l | s->rawintsrc_l) & (s->intselect_l) & s->int_enable_l);
	fiqnum_h = find_highest_bit((s->soft_int_h | s->rawintsrc_h) & (s->intselect_h) & s->int_enable_h);
	if (fiqnum_h != -1)
		s->fiqnum = 32 + fiqnum_h;
	else
		s->fiqnum = fiqnum_l;
	return s->fiqnum;
    case 32:
	fprintf(stderr, "irq table num\n");
	if (s->irqnum != -1)
		return s->irq_table_base + (s->irqnum << 2);
	else return 0;
    case 33:
	if (s->fiqnum != -1)
		return s->fiq_table_base + (s->fiqnum << 2);
	else return 0;
#if 0
    case 0: /* IRQSTATUS */
        return sep612_pic_irq_level(s);
    case 1: /* FIQSATUS */
        return (s->level | s->soft_level) & s->fiq_select;
    case 2: /* RAWINTR */
        return s->level | s->soft_level;
    case 3: /* INTSELECT */
        return s->fiq_select;
    case 4: /* INTENABLE */
        return s->int_enable;
    case 6: /* SOFTINT */
        return s->soft_level;
    case 8: /* PROTECTION */
        return s->protected;
    case 12: /* VECTADDR */
        /* Read vector address at the start of an ISR.  Increases the
           current priority level to that of the current interrupt.  */
        for (i = 0; i < s->priority; i++)
          {
            if ((s->level | s->soft_level) & s->prio_mask[i])
              break;
          }
        /* Reading this value with no pending interrupts is undefined.
           We return the default address.  */
        if (i == PL190_NUM_PRIO)
          return s->vect_addr[16];
        if (i < s->priority)
          {
            s->prev_prio[i] = s->priority;
            s->priority = i;
            sep612_pic_update(s);
          }
        return s->vect_addr[s->priority];
    case 13: /* DEFVECTADDR */
        return s->vect_addr[16];
#endif
    default:
        hw_error("sep612_pic_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void sep612_pic_write(void *opaque, target_phys_addr_t offset, uint32_t val)
{
    sep612_pic_state *s = (sep612_pic_state *)opaque;

#if 0 //is this usefull  fanpeng freenix and I think in sep612 it may not access offset big than 0x100
    if (offset >= 0x100 && offset < 0x140) {
        s->vect_addr[(offset - 0x100) >> 2] = val;
        sep612_pic_update_vectors(s);
        return;
    }
    if (offset >= 0x200 && offset < 0x240) {
        s->vect_control[(offset - 0x200) >> 2] = val;
        sep612_pic_update_vectors(s);
        return;
    }
#endif
    switch (offset >> 2) {
    case 0: /* SELECT */
        /* This is a readonly register, but linux tries to write to it
           anyway.  Ignore the write.  */
	    s->intselect_l = val;
        break;
    case 1:
	    s->intselect_h = val;
	break;
    case 2:
	s->int_enable_l = val;
	break;
    case 3:
	s->int_enable_h = val;
	break;
    case 4:
	s->int_enable_l &= ~val; //not sure
	break;
    case 5:
	s->int_enable_h &= ~val;
	break;
    case 6:
	s->soft_int_l = val;
	break;
    case 7:
	s->soft_int_h = val;
	break;
    case 8:
	s->soft_int_l &= ~val;
	break;
    case 9:
	s->soft_int_h &= ~val;
	break;
    case 10:
	s->allmask = val & 0x3; //fanpeng ????????????????
	break;
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
	break;
    case 28:
	s->irq_table_base = val;
	break;
    case 29:
	s->fiq_table_base = val;
    case 30:
    case 31:
    case 32:
    case 33:
	break;
    default:
        hw_error("sep612_pic_write: Bad offset %x\n", (int)offset);
        return;
    }
    sep612_pic_update(s);
}

static CPUReadMemoryFunc * const sep612_pic_readfn[] = {
   sep612_pic_read,
   sep612_pic_read,
   sep612_pic_read
};

static CPUWriteMemoryFunc * const sep612_pic_writefn[] = {
   sep612_pic_write,
   sep612_pic_write,
   sep612_pic_write
};

static void sep612_pic_reset(DeviceState *d)
{
  sep612_pic_state *s = DO_UPCAST(sep612_pic_state, busdev.qdev, d);
    s->intselect_l = 0;
    s->intselect_h = 0;
    s->int_enable_l = 0;
    s->int_enable_h = 0;
    s->soft_int_l = 0;
    s->soft_int_h = 0;
    s->allmask = 0;
    s->rawintsrc_l = 0;
    s->rawintsrc_h = 0;
    s->irq_table_base = 0;
    s->fiq_table_base = 0;
    s->irqnum = -1;
    s->fiqnum = -1;
}

static int sep612_pic_init(SysBusDevice *dev)
{
    sep612_pic_state *s = FROM_SYSBUS(sep612_pic_state, dev);
    int iomemtype;

    iomemtype = cpu_register_io_memory(sep612_pic_readfn,
                                       sep612_pic_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);
    qdev_init_gpio_in(&dev->qdev, sep612_pic_set_irq, 64);
    sysbus_init_irq(dev, &s->irq);
   // sysbus_init_irq(dev, &s->fiq); //freenix not sure
    return 0;
}

static const VMStateDescription vmstate_sep612_pic = {
    .name = "sep612_pic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(intselect_l, sep612_pic_state),
        VMSTATE_UINT32(intselect_h, sep612_pic_state),
        VMSTATE_UINT32(int_enable_l, sep612_pic_state),
        VMSTATE_UINT32(int_enable_h, sep612_pic_state),
        VMSTATE_UINT32(soft_int_l, sep612_pic_state),
        VMSTATE_UINT32(soft_int_h, sep612_pic_state),
        VMSTATE_UINT32(allmask, sep612_pic_state),
        VMSTATE_UINT32(rawintsrc_l, sep612_pic_state),
        VMSTATE_UINT32(rawintsrc_h, sep612_pic_state),
        VMSTATE_UINT32(irq_table_base, sep612_pic_state),
        VMSTATE_UINT32(fiq_table_base, sep612_pic_state),
        VMSTATE_INT32(irqnum, sep612_pic_state),
        VMSTATE_INT32(fiqnum, sep612_pic_state),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo sep612_pic_info = {
    .init = sep612_pic_init,
    .qdev.name = "sep6200_pic",
    .qdev.size = sizeof(sep612_pic_state),
    .qdev.vmsd = &vmstate_sep612_pic,
    .qdev.reset = sep612_pic_reset,
    .qdev.no_user = 1,
};

static void sep612_pic_register_devices(void)
{
    sysbus_register_withprop(&sep612_pic_info);
}

device_init(sep612_pic_register_devices)
