#include "sysbus.h"
#include "hw.h"
#include "pc.h"
#include "fdc.h"
#include "net.h"
#include "boards.h"
#include "smbus.h"
#include "block.h"
#include "flash.h"
#include "mips.h"
#include "mips_cpudevs.h"
#include "pci.h"
#include "usb-uhci.h"
#include "qemu-char.h"
#include "sysemu.h"
#include "audio/audio.h"
#include "qemu-log.h"
#include "loader.h"
#include "mips-bios.h"
#include "ide.h"
#include "elf.h"
#include "vt82c686.h"
#include "mc146818rtc.h"
#include "blockdev.h"
#include "exec-memory.h"
#include "sep6200.h"

#define ENVP_ADDR       0x80001000l
#define ENVP_NB_ENTRIES	 	16
#define ENVP_ENTRY_SIZE	 	256

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;


static struct {
  target_phys_addr_t io_addr;
  int irqn;
} sep612_serial[] = {
  {0xb1000000, 42},
  {0,0},
};


struct sep6200_state sep6200_state1 = {
	NULL,
	NULL,
	0,
};

static void GCC_FMT_ATTR(3, 4) prom_set(uint32_t* prom_buf, int index,
                                        const char *string, ...)
{
    va_list ap;
    int32_t table_addr;

    if (index >= ENVP_NB_ENTRIES)
        return;

    if (string == NULL) {
        prom_buf[index] = 0;
        return;
    }

    table_addr = sizeof(int32_t) * ENVP_NB_ENTRIES + index * ENVP_ENTRY_SIZE;
    prom_buf[index] = tswap32(ENVP_ADDR + table_addr);

    va_start(ap, string);
    vsnprintf((char *)prom_buf + table_addr, ENVP_ENTRY_SIZE, string, ap);
    va_end(ap);
}

static void main_cpu_reset(void *opaque)
{
    CPUState *env = opaque;

    cpu_reset(env);
#if 0
    if (loaderparams.kernel_filename) {
        env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    }
#endif
}

#if 0
static int64_t load_kernel (CPUState *env)
{
    int64_t kernel_entry, kernel_low, kernel_high;
    int index = 0;
    long initrd_size;
    ram_addr_t initrd_offset;
    uint32_t *prom_buf;
    long prom_size;

    if (load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys, NULL,
                 (uint64_t *)&kernel_entry, (uint64_t *)&kernel_low,
                 (uint64_t *)&kernel_high, 0, ELF_MACHINE, 1) < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                loaderparams.kernel_filename);
        exit(1);
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size (loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = (kernel_high + ~TARGET_PAGE_MASK) & TARGET_PAGE_MASK;
            if (initrd_offset + initrd_size > ram_size) {
                fprintf(stderr,
                        "qemu: memory too small for initial ram disk '%s'\n",
                        loaderparams.initrd_filename);
                exit(1);
            }
	    initrd_offset = 0x1000000;
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                     initrd_offset, ram_size - initrd_offset);
	    fprintf(stderr, "initrd_offset 0x%x initrd_size 0x%x\n", initrd_offset, initrd_size);
        }
        if (initrd_size == (target_ulong) -1) {
            fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                    loaderparams.initrd_filename);
            exit(1);
        }
    }

    /* Setup prom parameters. */
    prom_size = ENVP_NB_ENTRIES * (sizeof(int32_t) + ENVP_ENTRY_SIZE);
    prom_buf = g_malloc(prom_size);

    prom_set(prom_buf, index++, "%s", loaderparams.kernel_filename);
    if (initrd_size > 0) {
        prom_set(prom_buf, index++, "rd_start=0x%" PRIx64 " rd_size=%li %s",
                 cpu_mips_phys_to_kseg0(NULL, initrd_offset), initrd_size,
                 loaderparams.kernel_cmdline);
    } else {
        prom_set(prom_buf, index++, "%s", loaderparams.kernel_cmdline);
    }

    /* Setup minimum environment variables */
    prom_set(prom_buf, index++, "busclock=50000000");
    prom_set(prom_buf, index++, "cpuclock=500000000");
    prom_set(prom_buf, index++, "memsize=%i", loaderparams.ram_size/1024/1024);
    prom_set(prom_buf, index++, "modetty0=115200n8r");
    prom_set(prom_buf, index++, NULL);

    rom_add_blob_fixed("prom", prom_buf, prom_size,
                       cpu_mips_kseg0_to_phys(NULL, ENVP_ADDR));

    return kernel_entry;
}
#endif

static void write_bootloader (CPUState *env, uint8_t *base, int64_t kernel_addr)
{
    uint32_t *p;

    /* Small bootloader */
    p = (uint32_t *) base;
    stl_raw(p++, 0x3a0050b1);
    stl_raw(p++, 0x3b000035);
    stl_raw(p++, 0x78080000);
    stl_raw(p++, 0x3a0050b1);
    stl_raw(p++, 0x3b000036);
    stl_raw(p++, 0x78080000);
    stl_raw(p++, 0x3a0050b1);
    stl_raw(p++, 0x3b000038);
    stl_raw(p++, 0x78080000);
    stl_raw(p++, 0x79f80054);
    stl_raw(p++, 0x1a000000);
    stl_raw(p++, 0x79f84050);
    stl_raw(p++, 0x78004000);
    stl_raw(p++, 0x79f8004c);
    stl_raw(p++, 0x1a000000);
    stl_raw(p++, 0x79f84048);
    stl_raw(p++, 0x78004000);
    stl_raw(p++, 0x79f80044);
    stl_raw(p++, 0x1a000000);
    stl_raw(p++, 0x79f84040);
    stl_raw(p++, 0x78004000);
    stl_raw(p++, 0x79f8003c);
    stl_raw(p++, 0x1a000000);
    stl_raw(p++, 0x3a004007);
    stl_raw(p++, 0x78004000);
    stl_raw(p++, 0x79f80030);
    stl_raw(p++, 0x1a000000);
    stl_raw(p++, 0x79004000);
    stl_raw(p++, 0x35080007);
    stl_raw(p++, 0xa2fffffb);
    stl_raw(p++, 0x79ffc020);
    stl_raw(p++, 0xb0001004);
    stl_raw(p++, 0x00010810);
    stl_raw(p++, 0xb0001008);
    stl_raw(p++, 0x00010816);
    stl_raw(p++, 0xb000100c);
    stl_raw(p++, 0x00010812);
    stl_raw(p++, 0xb0001000);
    stl_raw(p++, 0xb000105c);
    stl_raw(p++, 0x40300000);
    stl_raw(p++, 0x00000f41);
    stl_raw(p++, 0x61656100);
    stl_raw(p++, 0x01006962);
    stl_raw(p++, 0x00000005);
}

static void mips_sep612_init(ram_addr_t ram_size, const char *boot_device,
			     const char *kernel_filename, const char * kernel_cmdline,
			     const char *initrd_filename, const char *cpu_model)
{
//  char *filename;
  MemoryRegion *address_space_mem = get_system_memory();
  MemoryRegion *ram = g_new(MemoryRegion, 1);
  MemoryRegion *esram = g_new(MemoryRegion, 1);
  long bios_size;
  int32_t kernel_entry;
//  qemu_irq *intc;
//  qemu_irq *cpu_exit_irq;
//  int via_devfn;
  qemu_irq *cpu_pic;
  qemu_irq pic[64];
  int n;
  CPUState *env;

  DeviceState *dev;

  if (cpu_model == NULL) {
    cpu_model = "UniCore-II";
  }
  
  env = cpu_init(cpu_model);
  if (!env) {
    fprintf(stderr, "Unable to find CPU definition\n");
    exit(1);
  }

  register_savevm(NULL, "cpu", 0, 3, cpu_save, cpu_load, env);
  qemu_register_reset(main_cpu_reset, env);

  sep6200_clk_init(&sep6200_state1);

  ram_size = 256 * 1024 * 1024;

  bios_size = 64 *1024;

  memory_region_init_ram(ram, NULL, "sep6200.ram", ram_size);
  memory_region_init_ram(esram, NULL, "sep6200.bios", bios_size);

  memory_region_add_subregion(address_space_mem, 0x40000000, ram);
  sep6200_state1.ram = ram;
  memory_region_add_subregion(address_space_mem, 0x0, esram);
  sep6200_state1.esram = esram;

  rom_add_file("./u-boot.bin", NULL, 0x40300000, -1);
  rom_add_file("./uImage", NULL, 0x40500000, -1);

#if 0
  if (kernel_filename) {
    loaderparams.ram_size = ram_size;
    loaderparams.kernel_filename = kernel_filename;
    loaderparams.kernel_cmdline = kernel_cmdline;
    loaderparams.initrd_filename = initrd_filename;
    kernel_entry = load_kernel(env);
    write_bootloader(env, memory_region_get_ram_ptr(bios), kernel_entry);
  } else {
    fprintf(stderr, "Please input kernel filename\n");
    exit(1);
  }
#endif

  write_bootloader(env, memory_region_get_ram_ptr(esram), 0);

  cpu_pic = uc32_pic_init_cpu(env);

  dev = sysbus_create_varargs("sep6200_pmu", 0xb0001000, NULL);

  dev = sysbus_create_varargs("sep6200_sysctl", 0xb0008000, NULL);

  dev = sysbus_create_varargs("sep6200_pic", 0xb0000000, cpu_pic[0], NULL);

  for (n = 0; n < 64; n++) {
    pic[n] = qdev_get_gpio_in(dev, n);
  }
     
  int uart_clk = sep6200_clk_getrate(sep6200_findclk(&sep6200_state1, "uart_clk"));
  for (n = 0; n < 1; n++) { //4 there is something wrong with 4 serials ports
    serial_mm_init(address_space_mem, sep612_serial[n].io_addr, 2, qdev_get_gpio_in(dev, sep612_serial[n].irqn), 
		    100000000 / 16, serial_hds[n], DEVICE_NATIVE_ENDIAN);
  }

  dev = sysbus_create_varargs("sep6200_timer", 0xb0003000, pic[56], NULL);
}

QEMUMachine mips_sep612_machine = {
  .name = "sep6200",
  .desc = "SEC sep6200 soc",
  .init = mips_sep612_init,
};

static void mips_sep612_machine_init(void)
{
  qemu_register_machine(&mips_sep612_machine);
}

machine_init(mips_sep612_machine_init);

