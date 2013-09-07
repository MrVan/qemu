#include "hw/hw.h"
#include "hw/boards.h"

void cpu_save(QEMUFile *f, void *opaque)
{
    int i;
    CPUState_UniCore32 *env = (CPUState_UniCore32 *)opaque;

    for (i = 0; i < 32; i++) {
        qemu_put_be32(f, env->regs[i]);
    }

    qemu_put_be32(f, cpu_asr_read(env)); 

    qemu_put_be32(f, env->bsr);

    for (i = 0; i < 6; i++) {
        qemu_put_be32(f, env->banked_bsr[i]);
        qemu_put_be32(f, env->banked_r29[i]);
        qemu_put_be32(f, env->banked_r30[i]);
    }


    qemu_put_be32(f, env->cp0.c0_cpuid);
    qemu_put_be32(f, env->cp0.c0_cachetype);
    qemu_put_be32(f, env->cp0.c1_sys);
    qemu_put_be32(f, env->cp0.c2_base);
    qemu_put_be32(f, env->cp0.c3_faultstatus);
    qemu_put_be32(f, env->cp0.c4_faultaddr);
    qemu_put_be32(f, env->cp0.c5_cacheop);
    qemu_put_be32(f, env->cp0.c6_tlbop);


    for (i = 0; i < 16; i++) {
        CPU_DoubleU u;
        u.d = env->ucf64.regs[i];
        qemu_put_be32(f, u.l.upper);
        qemu_put_be32(f, u.l.lower);
    }

    for (i = 0; i < 32; i++) {
   	qemu_put_be32(f, env->ucf64.xregs[i]);
    }



    qemu_put_be32(f, env->features);

}

int cpu_load(QEMUFile *f, void *opaque, int version_id)
{
    CPUState_UniCore32 *env = (CPUState_UniCore32 *)opaque;
    int i;
    uint32_t val;

    if (version_id != CPU_SAVE_VERSION)
        return -EINVAL;

    for (i = 0; i < 32; i++) {
        env->regs[i] = qemu_get_be32(f);
    }
    val = qemu_get_be32(f);
    /* Avoid mode switch when restoring ASR.  */
    env->uncached_asr = val & ASR_M;
    cpu_asr_write(env, val, 0xffffffff);
    env->bsr = qemu_get_be32(f);
    for (i = 0; i < 6; i++) {
        env->banked_bsr[i] = qemu_get_be32(f);
        env->banked_r29[i] = qemu_get_be32(f);
        env->banked_r30[i] = qemu_get_be32(f);
    }

    env->cp0.c0_cpuid = qemu_get_be32(f);
    env->cp0.c0_cachetype = qemu_get_be32(f);
    env->cp0.c1_sys = qemu_get_be32(f);
    env->cp0.c2_base = qemu_get_be32(f);
    env->cp0.c3_faultstatus = qemu_get_be32(f);
    env->cp0.c4_faultaddr = qemu_get_be32(f);
    env->cp0.c5_cacheop = qemu_get_be32(f);
    env->cp0.c6_tlbop = qemu_get_be32(f);

    env->features = qemu_get_be32(f);

    for (i = 0;  i < 16; i++) {
         CPU_DoubleU u;
         u.l.upper = qemu_get_be32(f);
         u.l.lower = qemu_get_be32(f);
         env->ucf64.regs[i] = u.d;
    }
    for (i = 0; i < 32; i++) {
         env->ucf64.xregs[i] = qemu_get_be32(f);
    }

    return 0;
}
