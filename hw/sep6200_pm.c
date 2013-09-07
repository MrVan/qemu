#include "sysbus.h"
#include "sep6200.h"
#include <math.h>
/*may be not implemented as the mannul, just make it work Freenix*/

typedef struct {
	SysBusDevice busdev;
	MemoryRegion iomem;
	uint32_t pllset;
	uint32_t apllcfg;
	uint32_t mpllcfg;
	uint32_t dpllcfg;
	uint32_t pmdr;
	uint32_t clkgtcfg1;
	uint32_t clkgtcfg2;
	uint32_t pwrgtcfg1;
	uint32_t pwrgtcfg2;
	uint32_t clkcfgahb;
	uint32_t clkcfgunicore;
	uint32_t clkcfgddr;
	uint32_t clkcfgpix;
	uint32_t clkcfggpu2x;
	uint32_t divset;
	uint32_t pwrstateall;
	//still many reigster not implemented here, do future;
}sep6200_pmu_state;


static uint64_t sep6200_pmu_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
	sep6200_pmu_state *s = (sep6200_pmu_state *)opaque;
	//fprintf(stderr, "pmu read offset 0x%x\n", offset);
	switch (offset >> 2) {
		case 0:
			return s->pllset;
		case 1:
			return s->apllcfg;
		case 2:
			return s->mpllcfg;
		case 3:
			return s->dpllcfg;
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			return s->clkcfgahb;
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
			fprintf(stderr, "not implemented read fanpeng\n");
			return 0;
		case 23:
			return s->pwrstateall;
		default:
		        hw_error("sep6200_pmu_read:  Bad offset %x\n", (int)offset);

	}
}

static void sep6200_pmu_write(void *opaque, target_phys_addr_t offset,
		uint64_t val, unsigned int size)
{
	sep6200_pmu_state *s = (sep6200_pmu_state *)opaque;
	uint32_t nf, nr, no;
	//fprintf(stderr, "pmu write offset 0x%x\n", offset);
	switch (offset >> 2) {
		case 0:
			if (val & 0x1) {
				//apll	
				nr = ((s->apllcfg >> 5) & 0x1f) + 1;
				nf = ((s->apllcfg >> 10) & 0x7f) + 1;
				switch ((s->apllcfg >> 1) & 3) {
					case 0:
						no = 1;
						break;
					case 1:
						no = 2;
						break;
					case 2:
						no = 4;
						break;
					case 3:
						no = 8;
						break;
					default:
						abort();

				}
				sep6200_clk_onoff(sep6200_findclk(&sep6200_state1, "apll_clk"), 1);
				sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "apll_clk"), nr * no, nf);
			} 
			if (val & 0x2) {
				//mpll	
				nr = ((s->mpllcfg >> 5) & 0x1f) + 1;
				nf = ((s->mpllcfg >> 10) & 0x7f) + 1;
				switch ((s->mpllcfg >> 1) & 3) {
					case 0:
						no = 1;
						break;
					case 1:
						no = 2;
						break;
					case 2:
						no = 4;
						break;
					case 3:
						no = 8;
						break;
					default:
						abort();

				}
				sep6200_clk_onoff(sep6200_findclk(&sep6200_state1, "mpll_clk"), 1);
				sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "mpll_clk"), nr * no, nf);
			}
			if (val & 0x4) {
				//dpll	
				nr = ((s->dpllcfg >> 5) & 0x1f) + 1;
				nf = ((s->dpllcfg >> 10) & 0x7f) + 1;
				switch ((s->dpllcfg >> 1) & 3) {
					case 0:
						no = 1;
						break;
					case 1:
						no = 2;
						break;
					case 2:
						no = 4;
						break;
					case 3:
						no = 8;
						break;
					default:
						abort();

				}
				sep6200_clk_onoff(sep6200_findclk(&sep6200_state1, "dpll_clk"), 1);
				sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "dpll_clk"), nr * no, nf);
			}
			s->pllset = 0;
			s->pwrstateall = 0x7;
			break;
                case 1:
			s->apllcfg = val;
			break;
                case 2:
                	s->mpllcfg = val;	
			break;
                case 3:
                	s->dpllcfg = val;	
			break;
                case 4:
                case 5:
                case 6:
                case 7:
                	//fprintf(stderr, "not implemented fanpeng\n");
			break;
                case 8:
                	s->clkcfgahb = val;
			switch ((val >> 8) & 0x3) {
				case 0:	
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus1_clk"), sep6200_findclk(&sep6200_state1, "apll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus2_clk"), sep6200_findclk(&sep6200_state1, "apll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus3_clk"), sep6200_findclk(&sep6200_state1, "apll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus4_clk"), sep6200_findclk(&sep6200_state1, "apll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus5_clk"), sep6200_findclk(&sep6200_state1, "apll_clk"));
					break;
				case 1:	
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus1_clk"), sep6200_findclk(&sep6200_state1, "mpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus2_clk"), sep6200_findclk(&sep6200_state1, "mpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus3_clk"), sep6200_findclk(&sep6200_state1, "mpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus4_clk"), sep6200_findclk(&sep6200_state1, "mpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus5_clk"), sep6200_findclk(&sep6200_state1, "mpll_clk"));
					break;
				case 2:	
				case 3:	
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus1_clk"), sep6200_findclk(&sep6200_state1, "dpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus2_clk"), sep6200_findclk(&sep6200_state1, "dpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus3_clk"), sep6200_findclk(&sep6200_state1, "dpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus4_clk"), sep6200_findclk(&sep6200_state1, "dpll_clk"));
					sep6200_clk_reparent(sep6200_findclk(&sep6200_state1, "bus5_clk"), sep6200_findclk(&sep6200_state1, "dpll_clk"));
					break;
				default:
					abort();
			}
			sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "bus1_clk"), pow(2, (val & 0xf)), 1);
			sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "bus2_clk"), pow(2, (val & 0xf)), 1);
			sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "bus3_clk"), pow(2, (val & 0xf)), 1);
			sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "bus4_clk"), pow(2, ((val>>4) & 0x3)), 1);
			sep6200_clk_setrate(sep6200_findclk(&sep6200_state1, "bus5_clk"), pow(2, ((val>>6) & 0x3)), 1);
			break;
		case 9:
                case 10:
                case 11:
                case 12:
                case 13:
                	fprintf(stderr, "not implemented write fanpeng\n");
			break;
                default:
		        hw_error("sep6200_pmu_write:  Bad offset %x\n", (int)offset);
	}
}
static const MemoryRegionOps sep6200_pmu_ops = {
    .read = sep6200_pmu_read,
    .write = sep6200_pmu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void sep6200_pmu_reset(sep6200_pmu_state *s)
{
	s->pllset = 0x0;
	s->apllcfg = 0x20;
	s->mpllcfg = 0x20;
	s->dpllcfg = 0x20;
	s->clkcfgahb = 0x0;
	s->pwrstateall = 0;
	fprintf(stderr, "sep6200_pmu_reset\n");
	// others do not care now
}
static int sep6200_pmu_init(SysBusDevice *dev)
{
    sep6200_pmu_state *s = FROM_SYSBUS(sep6200_pmu_state, dev);

    memory_region_init_io(&s->iomem, &sep6200_pmu_ops, s, "sep6200_pmu", 0x1000);
    sysbus_init_mmio_region(dev, &s->iomem);

    sep6200_pmu_reset(s);

    return 0;
}

static void sep6200_timer_register_devices(void)
{
    sysbus_register_dev("sep6200_pmu", sizeof(sep6200_pmu_state), sep6200_pmu_init);
}

device_init(sep6200_timer_register_devices)
