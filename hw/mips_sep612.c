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
  {0x1fc50000, 42},
  {0,0},
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
    /* TODO: 2E reset stuff */
    if (loaderparams.kernel_filename) {
        env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    }
}

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

static void write_bootloader (CPUState *env, uint8_t *base, int64_t kernel_addr)
{
    uint32_t *p;

    /* Small bootloader */
    p = (uint32_t *) base;

    stl_raw(p++, 0x0bf00010);                                      /* j 0x1fc00040 */
    stl_raw(p++, 0x00000000);                                      /* nop */

    /* Second part of the bootloader */
    p = (uint32_t *) (base + 0x040);

    stl_raw(p++, 0x3c040000);                                      /* lui a0, 0 */
    stl_raw(p++, 0x34840002);                                      /* ori a0, a0, 2 */
    stl_raw(p++, 0x3c050000 | ((ENVP_ADDR >> 16) & 0xffff));       /* lui a1, high(ENVP_ADDR) */ //in gdb is 3c058000 need debug freenix fanpeng  envp_addr  ok i was wrong
    stl_raw(p++, 0x34a50000 | (ENVP_ADDR & 0xffff));               /* ori a1, a0, low(ENVP_ADDR) */ //34a51000 why? I do not know fanpeng 
    stl_raw(p++, 0x3c060000 | (((ENVP_ADDR + 8) >> 16) & 0xffff)); /* lui a2, high(ENVP_ADDR + 8) */ //3c068000
    stl_raw(p++, 0x34c60000 | ((ENVP_ADDR + 8) & 0xffff));         /* ori a2, a2, low(ENVP_ADDR + 8) */ //34c61008
    stl_raw(p++, 0x3c070000 | (loaderparams.ram_size >> 16));      /* lui a3, high(env->ram_size) *///3c071000
    stl_raw(p++, 0x34e70000 | (loaderparams.ram_size & 0xffff));   /* ori a3, a3, low(env->ram_size) */
    stl_raw(p++, 0x3c1f0000 | ((kernel_addr >> 16) & 0xffff));     /* lui ra, high(kernel_addr) */ //3c1f8020
    stl_raw(p++, 0x37ff0000 | (kernel_addr & 0xffff));             /* ori ra, ra, low(kernel_addr) *///37ff4260
    stl_raw(p++, 0x03e00008);                                      /* jr ra */
    stl_raw(p++, 0x00000000);                                      /* nop */
}


static void mips_sep612_init(ram_addr_t ram_size, const char *boot_device,
			     const char *kernel_filename, const char * kernel_cmdline,
			     const char *initrd_filename, const char *cpu_model)
{
//  char *filename;
  MemoryRegion *address_space_mem = get_system_memory();
  MemoryRegion *ram = g_new(MemoryRegion, 1);
  MemoryRegion *bios = g_new(MemoryRegion, 1);
  long bios_size;
  int32_t kernel_entry;
//  qemu_irq *intc;
//  qemu_irq *cpu_exit_irq;
//  int via_devfn;
  qemu_irq pic[64];
  int n;
  CPUState *env;

  DeviceState *dev;

  if (cpu_model == NULL) {
    cpu_model = "Loongson-3A-seu";
  }
  
  env = cpu_init(cpu_model);
  if (!env) {
    fprintf(stderr, "Unable to find CPU definition\n");
    exit(1);
  }

  register_savevm(NULL, "cpu", 0, 3, cpu_save, cpu_load, env);
  qemu_register_reset(main_cpu_reset, env);

  ram_size = 256 * 1024 * 1024;

  bios_size = 64 *1024;

  memory_region_init_ram(ram, NULL, "sep612.ram", ram_size);
  memory_region_init_ram(bios, NULL, "sep612.bios", bios_size);

  memory_region_add_subregion(address_space_mem, 0, ram);
  memory_region_add_subregion(address_space_mem, 0x1fc00000, bios);

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

  cpu_mips_irq_init_cpu(env);
  cpu_mips_clock_init(env);

  dev = sysbus_create_varargs("sep612_pic", 0x1fc40000, env->irq[2], NULL);

  for (n = 0; n < 64; n++) {
    pic[n] = qdev_get_gpio_in(dev, n);
  }
     
  for (n = 0; n < 1; n++) { //4 there is something wrong with 4 serials ports
    serial_mm_init(address_space_mem, sep612_serial[n].io_addr, 2, qdev_get_gpio_in(dev, sep612_serial[n].irqn), 
		   50000000 / 16, serial_hds[n], DEVICE_NATIVE_ENDIAN);
  }

}

QEMUMachine mips_sep612_machine = {
  .name = "sep612",
  .desc = "SEC sep612 soc",
  .init = mips_sep612_init,
};

static void mips_sep612_machine_init(void)
{
  qemu_register_machine(&mips_sep612_machine);
}

machine_init(mips_sep612_machine_init);

