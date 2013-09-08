#include "sysbus.h"

typedef struct {
    SysBusDevice busdev;
    uint32_t core0_ipi_status;
    uint32_t core0_ipi_enable;
    uint32_t core0_ipi_set;
    uint32_t core0_ipi_clear;
    uint32_t core0_mailbox0;
    uint32_t core0_mailbox1;
    uint32_t core0_mailbox2;
    uint32_t core0_mailbox3;
    uint32_t core1_ipi_status;
    uint32_t core1_ipi_enable;
    uint32_t core1_ipi_set;
    uint32_t core1_ipi_clear;
    uint32_t core1_mailbox0;
    uint32_t core1_mailbox1;
    uint32_t core1_mailbox2;
    uint32_t core1_mailbox3;
    qemu_irq ipi0;
    qemu_irq ipi1;
} sep612_ipi_state;


static void sep612_ipi_update(sep612_ipi_state *s)
{

    int set = 0; 

    set = s->core0_ipi_status && 0xffffffff;
    //fprintf(stderr, "set core 0 = 0x%x\n", set);
    qemu_set_irq(s->ipi0, set);

    set = s->core1_ipi_status && 0xffffffff;
    //fprintf(stderr, "set core 1 = 0x%x\n", set);
    qemu_set_irq(s->ipi1, set);
}

static uint32 sep612_ipi_read(void *opaque, target_phys_addr_t offset)
{
    sep612_ipi_state *s = (sep612_ipi_state *)opaque;

    switch ((offset << 20) >> 22) {
//core0
   	case 0:
		return s->core0_ipi_status;
	case 1:
		return s->core0_ipi_enable;
	case 2:
	case 3:
		fprintf(stderr, "read only\n");
		return 0;
	case 8:
		return s->core0_mailbox0;
	case 10:
		return s->core0_mailbox1;
	case 12:
		return s->core0_mailbox2;
	case 14:
		return s->core0_mailbox3;
//core1
	case 64:
		return s->core1_ipi_status;
	case 65:
		return s->core1_ipi_enable;
	case 66:
	case 67:
		fprintf(stderr, "read only\n");
		return 0;
	case 72:
		return s->core1_mailbox0;
	case 74:
		return s->core1_mailbox1;
	case 76:
		return s->core1_mailbox2;
	case 78:
		return s->core1_mailbox3;

    	default:
       		hw_error("sep612_ipi_read: Bad offset %x\n", (int)offset);
        	return 0;
    }
	
}

static uint32 sep612_ipi_write(void *opaque, target_phys_addr_t offset, uint32_t val)
{
    sep612_ipi_state *s = (sep612_ipi_state *)opaque;
	
    switch ((offset << 20) >> 22) {
//core0
   	case 0:
		break;
	case 1:
		s->core0_ipi_enable = val;
		break;
	case 2:
		if (s->core0_ipi_enable & val)
			s->core0_ipi_status |= val;
		break;
	case 3:
		s->core0_ipi_status &= (~val);
		break;
	case 8:
		s->core0_mailbox0 = val;
		break;
	case 10:
		s->core0_mailbox1 = val;
		break;
	case 12:
		s->core0_mailbox2 = val;
		break;
	case 14:
		s->core0_mailbox3 = val;
		break;
//core1
	case 64:
		break;
	case 65:
		s->core1_ipi_enable = val;
		break;
	case 66:
		if (s->core1_ipi_enable & val)
			s->core1_ipi_status |= val;
		break;
	case 67:
		s->core1_ipi_status &= (~val);
		break;
	case 72:
		s->core1_mailbox0 = val;
		break;
	case 74:
		s->core1_mailbox1 = val;
		break;
	case 76:
		s->core1_mailbox2 = val;
		break;
	case 78:
		s->core1_mailbox3 = val;
		break;

    	default:
       		hw_error("sep612_ipi_read: Bad offset %x\n", (int)offset);
        	return 0;
    }

    sep612_ipi_update(s);
}

static CPUReadMemoryFunc * const sep612_ipi_readfn[] = {
   sep612_ipi_read,
   sep612_ipi_read,
   sep612_ipi_read
};

static CPUWriteMemoryFunc * const sep612_ipi_writefn[] = {
   sep612_ipi_write,
   sep612_ipi_write,
   sep612_ipi_write
};

static void sep612_ipi_reset(DeviceState *d)
{
  sep612_ipi_state *s = DO_UPCAST(sep612_ipi_state, busdev.qdev, d);
  s->core0_ipi_status = 0;
  s->core1_ipi_status = 0;
  s->core0_ipi_enable = 0;
  s->core1_ipi_enable = 0;
  s->core0_ipi_set = 0;
  s->core1_ipi_set = 0;
  s->core0_ipi_clear = 0;
  s->core1_ipi_clear = 0;
  s->core0_mailbox0 = 0;
  s->core0_mailbox1 = 0;
  s->core0_mailbox2 = 0;
  s->core0_mailbox3 = 0;
  s->core1_mailbox0 = 0;
  s->core1_mailbox1 = 0;
  s->core1_mailbox2 = 0;
  s->core1_mailbox3 = 0;
}

static int sep612_ipi_init(SysBusDevice *dev)
{
    sep612_ipi_state *s = FROM_SYSBUS(sep612_ipi_state, dev);
    int iomemtype;

    iomemtype = cpu_register_io_memory(sep612_ipi_readfn,
                                       sep612_ipi_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);
    //qdev_init_gpio_in(&dev->qdev, sep612_ipi_set_irq, 2);
    sysbus_init_irq(dev, &s->ipi0);
    sysbus_init_irq(dev, &s->ipi1); 
    return 0;
}

static const VMStateDescription vmstate_sep612_ipi = {
    .name = "sep612_ipi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(core0_ipi_status, sep612_ipi_state),
        VMSTATE_UINT32(core0_ipi_enable, sep612_ipi_state),
        VMSTATE_UINT32(core0_ipi_set, sep612_ipi_state),
        VMSTATE_UINT32(core0_ipi_clear, sep612_ipi_state),
        VMSTATE_UINT32(core0_mailbox0, sep612_ipi_state),
        VMSTATE_UINT32(core0_mailbox1, sep612_ipi_state),
        VMSTATE_UINT32(core0_mailbox2, sep612_ipi_state),
        VMSTATE_UINT32(core0_mailbox3, sep612_ipi_state),
        VMSTATE_UINT32(core1_ipi_status, sep612_ipi_state),
        VMSTATE_UINT32(core1_ipi_enable, sep612_ipi_state),
        VMSTATE_UINT32(core1_ipi_set, sep612_ipi_state),
        VMSTATE_UINT32(core1_ipi_clear, sep612_ipi_state),
        VMSTATE_UINT32(core1_mailbox0, sep612_ipi_state),
        VMSTATE_UINT32(core1_mailbox1, sep612_ipi_state),
        VMSTATE_UINT32(core1_mailbox2, sep612_ipi_state),
        VMSTATE_UINT32(core1_mailbox3, sep612_ipi_state),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo sep612_ipi_info = {
    .init = sep612_ipi_init,
    .qdev.name = "sep612_ipi",
    .qdev.size = sizeof(sep612_ipi_state),
    .qdev.vmsd = &vmstate_sep612_ipi,
    .qdev.reset = sep612_ipi_reset,
    .qdev.no_user = 1,
};

static void sep612_ipi_register_devices(void)
{
    sysbus_register_withprop(&sep612_ipi_info);
}

device_init(sep612_ipi_register_devices)
