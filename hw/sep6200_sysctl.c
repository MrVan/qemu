/*
 * Status and system control registers for ARM RealView/Versatile boards.
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "hw.h"
#include "qemu-timer.h"
#include "sysbus.h"
#include "primecell.h"
#include "sysemu.h"
#include "sep6200.h"

#define LOCK_VALUE 0xa05f

#if 0
struct sep6200_state {
	MemoryRegion *ram;
	MemoryRegion *esram;
	int remap;
};//fanpeng??
#endif

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t sys_ctrl;
    uint32_t revision;
} sep6200_sysctl_state;

static const VMStateDescription vmstate_sep6200_sysctl = {
    .name = "realview_sysctl",
    .version_id = 3,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(sys_ctrl, sep6200_sysctl_state),
        VMSTATE_UINT32(revision, sep6200_sysctl_state),
        VMSTATE_END_OF_LIST()
    }
};



static void sep6200_sysctl_reset(DeviceState *d)
{
    sep6200_sysctl_state *s = FROM_SYSBUS(sep6200_sysctl_state, sysbus_from_qdev(d));
    s->sys_ctrl = 0x2003e000;
    s->revision = 0x7180630;

}

static uint64_t sep6200_sysctl_read(void *opaque, target_phys_addr_t offset,
                                unsigned size)
{
    sep6200_sysctl_state *s = (sep6200_sysctl_state *)opaque;

    switch (offset) {
    case 0x00: 
        return s->sys_ctrl;
    case 0x04: 
	return s->revision;
    default:
    bad_reg:
        printf ("sep6200_sysctl_read: Bad register offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void sep6200_do_remap(sep6200_sysctl_state *s, MemoryRegion *mem, target_phys_addr_t address)
{
	//sysbus_add_memory_overlap(&s->busdev, address, mem, 1);// do future
	//remap fanpeng
}

static void sep6200_sysctl_write(void *opaque, target_phys_addr_t offset,
                             uint64_t val, unsigned size)
{
    sep6200_sysctl_state *s = (sep6200_sysctl_state *)opaque;

    switch (offset) {
	    case 0x0:
		    fprintf(stderr, "sysctl_write 0x0\n");
    		s->sys_ctrl = val;
		if ((s->sys_ctrl & 0x10000000) != 0) {
			sep6200_do_remap(s, sep6200_state1.ram, 0);
			tlb_flush(cpu_single_env, 1);
		}
		break;
    	case 0x04: 
		break;	
    	default:
    	bad_reg:
       		 printf ("sep6200_sysctl_write: Bad register offset 0x%x\n", (int)offset);
       		 return;
    }
}

static const MemoryRegionOps sep6200_sysctl_ops = {
    .read = sep6200_sysctl_read,
    .write = sep6200_sysctl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int sep6200_sysctl_init1(SysBusDevice *dev)
{
    sep6200_sysctl_state *s = FROM_SYSBUS(sep6200_sysctl_state, dev);

    memory_region_init_io(&s->iomem, &sep6200_sysctl_ops, s, "sep6200-sysctl", 0x1000);
    sysbus_init_mmio_region(dev, &s->iomem);
    //qdev_init_gpio_in(&s->busdev.qdev, sep6200_sysctl_gpio_set, 2);
    //qdev_init_gpio_out(&s->busdev.qdev, &s->pl110_mux_ctrl, 1);
    return 0;
}

/* Legacy helper function.  */
void sep6200_sysctl_init(uint32_t base, uint32_t sys_id, uint32_t proc_id)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "sep6200_sysctl");
    qdev_prop_set_uint32(dev, "sys_id", sys_id);
    qdev_init_nofail(dev);
    qdev_prop_set_uint32(dev, "proc_id", proc_id);
    sysbus_mmio_map(sysbus_from_qdev(dev), 0, base);
}

static SysBusDeviceInfo sep6200_sysctl_info = {
    .init = sep6200_sysctl_init1,
    .qdev.name  = "sep6200_sysctl",
    .qdev.size  = sizeof(sep6200_sysctl_state),
    .qdev.vmsd = &vmstate_sep6200_sysctl,
    .qdev.reset = sep6200_sysctl_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("sys_ctrl", sep6200_sysctl_state, sys_ctrl, 0),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void sep6200_sysctl_register_devices(void)
{
    sysbus_register_withprop(&sep6200_sysctl_info);
}

device_init(sep6200_sysctl_register_devices)
