/*
 * Copyright (C) 2010-2011 GUAN Xue-tao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "gdbstub.h"
#include "helper.h"
#include "qemu-common.h"
#include "host-utils.h"

#if !defined(CONFIG_USER_ONLY)
#include "hw/loader.h"
#endif

static inline void set_feature(CPUState *env, int feature)
{
    env->features |= feature;
}

struct uc32_cpu_t {
    uint32_t id;
    const char *name;
};

static const struct uc32_cpu_t uc32_cpu_names[] = {
    { UC32_CPUID_UCV2, "UniCore-II"},
    { UC32_CPUID_ANY, "any"},
    { 0, NULL}
};

/* return 0 if not found */
static uint32_t uc32_cpu_find_by_name(const char *name)
{
    int i;
    uint32_t id;

    id = 0;
    for (i = 0; uc32_cpu_names[i].name; i++) {
        if (strcmp(name, uc32_cpu_names[i].name) == 0) {
            id = uc32_cpu_names[i].id;
            break;
        }
    }
    return id;
}

static void cpu_reset_model_id(CPUState *env, uint32_t id)
{
	env->cp0.c0_cpuid = id;
	set_feature(env, UC32_HWCAP_UCF64);
	env->cp0.c0_cachetype = 0x0;
	fprintf(stderr, "cpu_reset_model_id\n");
}

void cpu_reset(CPUState *env)
{
	uint32_t id;
	if (qemu_loglevel_mask(CPU_LOG_RESET)) {
		qemu_log("CPU Reset (CPU %d)\n", env->cpu_index);	
	}

	id = env->cp0.c0_cpuid;
	memset(env, 0, offsetof(CPUState, breakpoints));
	if (id)
		cpu_reset_model_id(env, id);

#if defined (CONFIG_USER_ONLY)
	fprintf(stderr, "cpu_reset user only\n");
#else
	env->uncached_asr = ASR_MODE_PRIV | ASR_R | ASR_I;
	env->ucf64.xregs[UC32_UCF64_FPSCR] = 0;
#endif
	set_flush_to_zero(1, &env->ucf64.fp_status);
	set_flush_inputs_to_zero(1, &env->ucf64.fp_status);
	set_default_nan_mode(1, &env->ucf64.fp_status);

    	set_float_detect_tininess(float_tininess_before_rounding,
                              &env->ucf64.fp_status);
   	tlb_flush(env, 1);

	env->regs[31] = 0x0;
}

CPUState *uc32_cpu_init(const char *cpu_model)
{
    CPUState *env;
    uint32_t id;
    static int inited = 1;

    env = g_malloc0(sizeof(CPUState));
    cpu_exec_init(env);

    id = uc32_cpu_find_by_name(cpu_model);
    switch (id) {
    case UC32_CPUID_UCV2:
        set_feature(env, UC32_HWCAP_CMOV);
        set_feature(env, UC32_HWCAP_UCF64);
        env->ucf64.xregs[UC32_UCF64_FPSCR] = 0;
        env->cp0.c0_cachetype = 0x1dd20d2;
        env->cp0.c1_sys = 0x00090078;
        break;
    case UC32_CPUID_ANY: /* For userspace emulation.  */
        set_feature(env, UC32_HWCAP_CMOV);
        set_feature(env, UC32_HWCAP_UCF64);
        break;
    default:
        cpu_abort(env, "Bad CPU ID: %x\n", id);
    }

    env->cpu_model_str = cpu_model;
    env->cp0.c0_cpuid = id;
#if !defined(CONFIG_USER_ONLY)
    env->uncached_asr = ASR_MODE_PRIV;
#else
    env->uncached_asr = ASR_MODE_USER;
#endif
    env->regs[31] = 0;

    if (inited) {
        inited = 0;
        uc32_translate_init();
    }

    tlb_flush(env, 1);
    qemu_init_vcpu(env);
    return env;
}

uint32_t HELPER(clo)(uint32_t x)
{
    return clo32(x);
}

uint32_t HELPER(clz)(uint32_t x)
{
    return clz32(x);
}

#if defined(CONFIG_USER_ONLY)
void do_interrupt(CPUState *env)
{
    env->exception_index = -1;
}

int uc32_cpu_handle_mmu_fault(CPUState *env, target_ulong address, int rw,
                              int mmu_idx)
{
    env->exception_index = UC32_EXCP_TRAP;
    env->cp0.c4_faultaddr = address;
    return 1;
}

/* These should probably raise undefined insn exceptions.  */
void HELPER(set_cp)(CPUState *env, uint32_t insn, uint32_t val)
{
    int op1 = (insn >> 8) & 0xf;
    cpu_abort(env, "cp%i insn %08x\n", op1, insn);
    return;
}

uint32_t HELPER(get_cp)(CPUState *env, uint32_t insn)
{
    int op1 = (insn >> 8) & 0xf;
    cpu_abort(env, "cp%i insn %08x\n", op1, insn);
    return 0;
}

void HELPER(set_cp0)(CPUState *env, uint32_t insn, uint32_t val)
{
    cpu_abort(env, "cp0 insn %08x\n", insn);
}

uint32_t HELPER(get_cp0)(CPUState *env, uint32_t insn)
{
    cpu_abort(env, "cp0 insn %08x\n", insn);
    return 0;
}

void switch_mode(CPUState *env, int mode)
{
    if (mode != ASR_MODE_USER) {
        cpu_abort(env, "Tried to switch out of user mode\n");
    }
}

void HELPER(set_r29_banked)(CPUState *env, uint32_t mode, uint32_t val)
{
    cpu_abort(env, "banked r29 write\n");
}

uint32_t HELPER(get_r29_banked)(CPUState *env, uint32_t mode)
{
    cpu_abort(env, "banked r29 read\n");
    return 0;
}

#else
static inline int bank_number(int mode)
{
	switch (mode) {
		case ASR_MODE_USER:
		case ASR_MODE_SUSR:
			return 0;
		case ASR_MODE_INTR:
			return 1;
		case ASR_MODE_PRIV:
			return 2;
		case ASR_MODE_TRAP:
			return 3;
		case ASR_MODE_EXTN:
			return 4;
		case ASR_MODE_REAL:
			return 5;
	}
	cpu_abort(cpu_single_env, "Bad mode %x\n", mode);
	return -1;
}
void switch_mode(CPUState *env, int mode)
{
	int old_mode;
	int i;

	old_mode = env->uncached_asr & ASR_M;
	if (old_mode == mode)
		return;

	i = bank_number(old_mode);
	env->banked_r29[i] = env->regs[29];
	env->banked_r30[i] = env->regs[30];
	env->banked_bsr[i] = env->bsr;

	i = bank_number(mode);
	env->regs[29] = env->banked_r29[i];
	env->regs[30] = env->banked_r30[i];
	env->bsr = env->banked_bsr[i];
}

void do_interrupt(CPUState *env)
{
	uint32_t addr;
	uint32_t mask;
	int new_mode;
	//uint32_t offset;
	int32_t offset;

	switch (env->exception_index) {
		case UC32_EXCP_EEXTN:
			new_mode = ASR_MODE_EXTN;
			fprintf(stderr, "extn-------------------\n");
			addr = 0x04;
			mask = ASR_I;
			offset = 0;
			break;
		case UC32_EXCP_EPRIV:
			new_mode = ASR_MODE_PRIV;
			//fprintf(stderr, "priv-------------------\n");
			addr = 0x08;
			mask = ASR_I;
			//offset = 4;
			offset = 0;
			break;
		case UC32_EXCP_ITRAP:  
			new_mode = ASR_MODE_TRAP;
			//fprintf(stderr, "itrap-------------------\n");
			addr = 0x0c;
			mask = ASR_I;
			offset = 0;
			//offset = 0;
			break;
		case UC32_EXCP_DTRAP:  
			new_mode = ASR_MODE_TRAP;
			//fprintf(stderr, "dtrap-------------------\n");
			addr = 0x10;
			mask = ASR_I;
			offset = 0;
			//offset = 0;
			break;
		case UC32_EXCP_EINTR:  
			new_mode = ASR_MODE_INTR;
			addr = 0x18;
			mask =  ASR_I;
			offset = 0; //I do not know right or wrong just a test fanpeng
			break;
		case UC32_EXCP_EREAL:  
			new_mode = ASR_MODE_REAL;
			fprintf(stderr, "real-------------------\n");
			addr = 0x1c;
			mask = ASR_I | ASR_R;
			offset = 0;
			break;
		default:
			cpu_abort(env, "Unhandled exception 0x%x\n", env->exception_index);
			return;
	}

	if (env->cp0.c1_sys & (1 << 13)) {
		addr += 0xffff0000;	
	}
	switch_mode(env, new_mode);
	env->bsr = cpu_asr_read(env);

	env->uncached_asr = (env->uncached_asr & ~ASR_M) | new_mode;
	env->uncached_asr |= mask;

	env->regs[30] = env->regs[31] + offset;
	env->regs[31] = addr;
	env->interrupt_request |= CPU_INTERRUPT_EXITTB;
}

#if 0
static inline int check_ap(CPUState *env, int ap, int domain, int access_type,
                           int is_user)
{ //fanpeng ????????????
  int prot_ro;


  if (access_type == 1)
      prot_ro = 0;
  else
      prot_ro = PAGE_READ;

  switch (ap) {
  case 0:
      if (access_type == 1)
          return 0;
      switch ((env->cp15.c1_sys >> 8) & 3) {
      case 1:
          return is_user ? 0 : PAGE_READ;
      case 2:
          return PAGE_READ;
      default:
          return 0;
      }
  case 1:
      return is_user ? 0 : PAGE_READ | PAGE_WRITE;
  case 2:
      if (is_user)
          return prot_ro;
      else
          return PAGE_READ | PAGE_WRITE;
  case 3:
      return PAGE_READ | PAGE_WRITE;
  case 4: /* Reserved.  */
      return 0;
  case 5:
      return is_user ? 0 : prot_ro;
  case 6:
      return prot_ro;
  case 7:
      if (!arm_feature (env, ARM_FEATURE_V6K))
          return 0;
      return prot_ro;
  default:
      abort();
  }
}
#endif

static uint32_t get_level1_table_address(CPUState *env, uint32_t address)
{
	uint32_t table;
	//fprintf(stderr, "get_level1_table_address\n"); //fanpeng

	table = (env->cp0.c2_base & (~0x3ff)) | ((address >> 20) & 0xffc);

	return table;
}

static int get_phys_addr_internal(CPUState *env, uint32_t address, int access_type,
			    int is_user, uint32_t *phys_ptr, int *prot,
                            target_ulong *page_size)
{
	int code;
	uint32_t table;
	uint32_t desc;
	int type;
	int rwx;
	uint32_t phys_addr;
	uint32_t prot_ro = 0;
#if 0
	if ((address < 0x1000)&& ((env->cp0.c1_sys & 1)!=0)){//just a test
		cpu_dump_state(env, stderr, fprintf, 0);
        	log_cpu_state(env, 0);
         	qemu_log_flush();
	}
#endif


#if 1//not sure
	if (cpu_mmu_index(env) &&(address >= 0xc0000000)){
		fprintf(stderr, "user access error 0x%x\n", address);
		code = 0x10; // user should not access bigger than 3G
		prot_ro = 0;
		goto do_fault;
	}
#endif

	table = get_level1_table_address(env, address);
	desc = ldl_phys(table);

	type = (desc & 3);

	if (2 == type) {
		fprintf(stderr, "fanpeng type == 2 wrong\n");
		prot_ro = 0;
		code = 5;
		goto do_fault;
	} else if (3 == type) {
		phys_addr = (desc & 0xffc00000) | (address & 0x003fffff);
		rwx = (desc >> 6) & 0x7;
		code = 0; //not sure here
		*page_size = 4 * 1024 * 1024;
		if (((desc >> 2) & 0x1) != 0x1) {
			//not present;
			fprintf(stderr, "code = 0xb\n");
			code = 0xb;//super page no exist
			prot_ro = 0;
			goto do_fault;
		}

	} else {
		//if l2 table present
		if (type == 1) {
			fprintf(stderr, "fanpeng type == 1\n");
			if (((desc >> 2) & 0x1) != 0x1) {
				//l2 table not present;
				code = 0x6;
				prot_ro = 0;
				goto do_fault;
			}
	    		/* Coarse pagetable.  */
			fprintf(stderr, "coarse pagetable\n");
	   	 	table = (desc & 0xfffffc00) | ((address >> 12) & 0x3fc);
		} else if (type ==0) {
			if (((desc >> 2) & 0x1) != 0x1) {
				//l2 table not present;
				//fprintf(stderr, "code = 0x5\n");
				code = 0x5;
				prot_ro = 0;
				goto do_fault;
			}
	   	 /* Fine pagetable.  */
		//fprintf(stderr, "fine pagetable 4k 0x%x\n", address); //fanpeng
	    		table = (desc & 0xfffff000) | ((address >> 10) & 0xffc);
			if (0x0==address)
				fprintf(stderr, "table2 = 0x%x\n", table); //just for test
		} else {
			fprintf(stderr, "page wrong\n");
			abort();
		} 
		

		desc = ldl_phys(table);//lookup

        	switch (desc & 0x3) {
			case 0:
            			phys_addr = (desc & 0xfffff000) | (address & 0xfff);
				rwx = (desc >> 6) & 0x7;
				code = 0; //not sure
            			*page_size = 0x1000;//4k
				if (((desc >> 2) & 0x1) != 0x1) {
					//fprintf(stderr, "code = 0x8\n");
					code = 8;//temporary
					prot_ro = 0;
					goto do_fault;
				}
				//fprintf(stderr, "4k 0x%x\n", phys_addr);
				break;
			case 1:
            			phys_addr = (desc & 0xffffc000) | (address & 0x3fff);
				rwx = (desc >> 6) & 0x7;
            			*page_size = 0x4000; //16k
				code = 0;//not sure
				fprintf(stderr, "16k 0x%x\n", phys_addr);
				if (((desc >> 2) & 0x1) != 0x1) {
					fprintf(stderr, "code = 0x9\n");
					code = 9;//temporary
					prot_ro = 0;
					goto do_fault;
				}
				cpu_dump_state(env, stderr, fprintf, 0); //just for test
				break;
			case 2:
            			phys_addr = (desc & 0xffff0000) | (address & 0xffff);
				rwx = (desc >> 6) & 0x7;
				code = 0; //not sure
            			*page_size = 0x10000; //64k
				if (((desc >> 2) & 0x1) != 0x1) {
					fprintf(stderr, "code = 0xa\n");
					code = 0xa;//temporary
					prot_ro = 0;
					goto do_fault;
				}
				fprintf(stderr, "64k 0x%x\n", phys_addr);
				break;
			default:
				abort();
		}
	}

	if (cpu_mmu_index(env) || ((!cpu_mmu_index(env) && (address < 0x80000000)))) { 
		//user access kernel || kernel access user
		//fprintf(stderr, "rwx = 0x%x address 0x%x\n", rwx, address);
		switch (access_type) {
			case 0:
				if ((rwx & 0x4) == 0x0) {
					fprintf(stderr, "code = 0x11\n");
					code = 0x11;	
					goto do_fault;
				}
				break;
			case 1:
				if ((rwx & 0x2) == 0x0) {
					//fprintf(stderr, "code = 0x12\n");
					code = 0x12;	
					goto do_fault;
				}
				break;
			case 2:
				//fprintf(stderr,"access type %x\n", access_type);
				//break; //just a test fanpeng
#if 1
				if ((rwx & 0x4) == 0x0) {
					fprintf(stderr, "access = 2 code = 0x11\n");
					code = 0x11;	
					goto do_fault;
				}
				if ((rwx & 0x1) == 0x0) {
					fprintf(stderr, "code = 0x13\n");
					code = 0x13;	
					goto do_fault;
				}
				break;
#endif
			default:
				fprintf(stderr, "access_type\n");
				abort();
		}
		if (rwx & 0x4)
			prot_ro |= PAGE_READ;
		if (rwx & 0x2)
			prot_ro |= PAGE_WRITE;
		if (rwx & 0x1)
			prot_ro |= PAGE_EXEC;
	} else {// =================================================here just a test
		prot_ro = PAGE_READ | PAGE_WRITE | PAGE_EXEC;	
		code = 0x0;
	}

#if 1
	*prot = prot_ro;
    	if (!*prot) {
        /* Access permission fault.  */
       	 	goto do_fault;
    	}
#endif
    	//*prot |=PAGE_READ | PAGE_WRITE | PAGE_EXEC; //need to modify in future fanpeng
    	*phys_ptr = phys_addr;
    	return 0;
do_fault:
	//fprintf(stderr, "do_fault mmu code 0x%x addr 0x%x pc 0x%x\n", code, address, env->regs[31]);
	return code;
}

static inline int get_phys_addr(CPUState *env, uint32_t address,
			int access_type, int is_user, uint32_t *phys_ptr, int*prot,
			target_ulong *page_size)
{
	if ((env->cp0.c1_sys & 1) == 0) {
		*phys_ptr = address;
		*prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
		*page_size = TARGET_PAGE_SIZE;
		return 0;
	} else {
		if (((env->uncached_asr & 0x1f) != 0x10) && ((address&0xff000000)== 0x31000000)) {
			fprintf(stderr, "kernel low 2 G mem 0x%x\n", address); //fanpeng
			*prot = PAGE_READ | PAGE_WRITE;		
			*phys_ptr = address | 0x80000000;
			*page_size = TARGET_PAGE_SIZE; // right or wrong? fanpeng
			return 0;
		} else {
			return get_phys_addr_internal(env, address, access_type, is_user, phys_ptr, 
					prot, page_size);	
		}
	}
}
int uc32_cpu_handle_mmu_fault(CPUState *env, target_ulong address, int rw,
                              int mmu_idx)
{
	uint32_t phys_addr;
	target_ulong page_size;
	int prot;
	int ret, is_user;

	is_user = (mmu_idx == MMU_USER_IDX);
	

	ret = get_phys_addr(env, address, rw, is_user, &phys_addr, &prot,
                        &page_size);

    	if (ret == 0) {
		//fprintf(stderr, "handle_mmu_fault1 fanpeng\n");
       		 /* Map a single [sub]page.  */
       		 phys_addr &= ~(uint32_t)0x3ff;
       		 address &= ~(uint32_t)0x3ff;
       		 tlb_set_page (env, address, phys_addr, prot, mmu_idx, page_size);
       		 return 0;
    	}

	if (rw != 2) { //just a test fanpeng
		//fprintf(stderr, "handle_mmu_fault 1 ret %x address %x\n", ret, address);
		//cpu_dump_state(env, stderr, fprintf, 0);
        	env->cp0.c3_faultstatus = ret;
        	env->cp0.c4_faultaddr = address;
        	env->exception_index = UC32_EXCP_DTRAP;
	}
	else if(rw == 2) {
		//fprintf(stderr, "handle_mmu_fault 2 ret %x\n", ret);
		//cpu_dump_state(env, stderr, fprintf, 0);
		env->cp0.c3_faultstatus = ret;
	        //env->cp0.c4_faultaddr = address;
	        env->exception_index = UC32_EXCP_ITRAP;
	}
	return 1;
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
	uint32_t phys_addr;
	target_ulong page_size;
	int prot;
	int ret;

	ret = get_phys_addr(env, addr, 0, 0, &phys_addr, &prot, &page_size);
	fprintf(stderr, "fanpeng cpu_get_phys_page_debug\n");

	if (ret != 0)
		return -1;

	return phys_addr;
}

void HELPER(set_cp)(CPUState *env, uint32_t insn, uint32_t val)
{
	fprintf(stderr, "set_cp\n");
#if 0
    int cp_num = (insn >> 10) & 0xf;
    int cp_info = (insn >> 5) & 7;
    int src = (insn >> 14) & 0x1f;
    int operand = insn & 0xf;

    if (env->cp[cp_num].cp_write)
        env->cp[cp_num].cp_write(env->cp[cp_num].opaque,
                                 cp_info, src, operand, val);
#endif
}

uint32_t HELPER(get_cp)(CPUState *env, uint32_t insn)
{
	fprintf(stderr, "get_cp\n");
#if 0
    int cp_num = (insn >> 8) & 0xf;
    int cp_info = (insn >> 5) & 7;
    int dest = (insn >> 16) & 0xf;
    int operand = insn & 0xf;

    if (env->cp[cp_num].cp_read)
        return env->cp[cp_num].cp_read(env->cp[cp_num].opaque,
                                       cp_info, dest, operand);
    return 0;
#endif
    return 0;
}
void HELPER(set_cp0)(CPUState *env, uint32_t insn, uint32_t val)
{
	int rd;
	int nn;
	int pp;
	int imm;

	pp = (insn >> 10) & 0xf;
	nn = (insn >> 19) & 0x1f;
	rd = (insn >> 14) & 0x1f;
	imm = (insn & 0x1f) | ((insn >> 1) & 0x1e0);

	switch (nn) {
		case 0:
			fprintf(stderr, "read only cp0\n");
			break;
		case 1:
			env->cp0.c1_sys = val;
			tlb_flush(env, 1);
			break;
		case 2:
			//fprintf(stderr, "set c2_base 0x%x\n", val);
			env->cp0.c2_base = val;
			break;
		case 3:
			//fprintf(stderr, "set c3_faultstatus 0x%x\n", val);
			env->cp0.c3_faultstatus = val;
			break;
		case 4:
			//fprintf(stderr, "set c4_faultaddr 0x%x\n", val);
			env->cp0.c4_faultaddr = val;
			break;
		case 5:
			env->cp0.c5_cacheop = val;
			//fprintf(stderr, "cacheop do future\n"); //fanpeng
			break;
		case 6:
			env->cp0.c6_tlbop = val;
			//fprintf(stderr, "tlbop do future\n"); //fanpeng
			break;
		default:
			cpu_abort(env, "Unimplemented cp0 register\n");
	}

	return;
}

uint32_t HELPER(get_cp0)(CPUState *env, uint32_t insn)
{
	int rd;
	int nn;
	int pp;
	int imm;

	pp = (insn >> 10) & 0xf;
	nn = (insn >> 19) & 0x1f;
	rd = (insn >> 14) & 0x1f;
	imm = (insn & 0x1f) | ((insn >> 1) & 0x1e0);

	switch (nn) {
		case 0:
			return env->cp0.c0_cpuid;
		case 1:
			return env->cp0.c1_sys;
		case 2:
			//fprintf(stderr, "get c2_base 0x%x\n", env->cp0.c2_base);
			return env->cp0.c2_base;
		case 3:
			//fprintf(stderr, "get c3_faultstatus 0x%x\n", env->cp0.c3_faultstatus);
			return env->cp0.c3_faultstatus;
		case 4:
			//fprintf(stderr, "get c4_faultaddr 0x%x\n", env->cp0.c4_faultaddr);
			return env->cp0.c4_faultaddr;
		case 5:
			return env->cp0.c5_cacheop;
		case 6:
			return env->cp0.c6_tlbop;
		default:
			cpu_abort(env, "Unimplemented cp0 register\n");
	}

	return -1;
}
void HELPER(set_r29_banked)(CPUState *env, uint32_t mode, uint32_t val)
{
	if ((env->uncached_asr & ASR_M) == mode) {
		env->regs[29] = val;	
	} else {
		env->banked_r29[bank_number(mode)] = val;
	}
}

uint32_t HELPER(get_r29_banked)(CPUState *env, uint32_t mode)
{
    if ((env->uncached_asr & ASR_M) == mode) {
        return env->regs[29];
    } else {
        return env->banked_r29[bank_number(mode)];
    }
}

#endif

/* UniCore-F64 support.  We follow the convention used for F64 instrunctions:
   Single precition routines have a "s" suffix, double precision a
   "d" suffix.  */

/* Convert host exception flags to f64 form.  */
static inline int ucf64_exceptbits_from_host(int host_bits)
{
    int target_bits = 0;

    if (host_bits & float_flag_invalid) {
        target_bits |= UCF64_FPSCR_FLAG_INVALID;
    }
    if (host_bits & float_flag_divbyzero) {
        target_bits |= UCF64_FPSCR_FLAG_DIVZERO;
    }
    if (host_bits & float_flag_overflow) {
        target_bits |= UCF64_FPSCR_FLAG_OVERFLOW;
    }
    if (host_bits & float_flag_underflow) {
        target_bits |= UCF64_FPSCR_FLAG_UNDERFLOW;
    }
    if (host_bits & float_flag_inexact) {
        target_bits |= UCF64_FPSCR_FLAG_INEXACT;
    }
    return target_bits;
}

uint32_t HELPER(ucf64_get_fpscr)(CPUState *env)
{
    int i;
    uint32_t fpscr;

    fpscr = (env->ucf64.xregs[UC32_UCF64_FPSCR] & UCF64_FPSCR_MASK);
    i = get_float_exception_flags(&env->ucf64.fp_status);
    fpscr |= ucf64_exceptbits_from_host(i);
    return fpscr;
}

/* Convert ucf64 exception flags to target form.  */
static inline int ucf64_exceptbits_to_host(int target_bits)
{
    int host_bits = 0;

    if (target_bits & UCF64_FPSCR_FLAG_INVALID) {
        host_bits |= float_flag_invalid;
    }
    if (target_bits & UCF64_FPSCR_FLAG_DIVZERO) {
        host_bits |= float_flag_divbyzero;
    }
    if (target_bits & UCF64_FPSCR_FLAG_OVERFLOW) {
        host_bits |= float_flag_overflow;
    }
    if (target_bits & UCF64_FPSCR_FLAG_UNDERFLOW) {
        host_bits |= float_flag_underflow;
    }
    if (target_bits & UCF64_FPSCR_FLAG_INEXACT) {
        host_bits |= float_flag_inexact;
    }
    return host_bits;
}

void HELPER(ucf64_set_fpscr)(CPUState *env, uint32_t val)
{
    int i;
    uint32_t changed;

    changed = env->ucf64.xregs[UC32_UCF64_FPSCR];
    env->ucf64.xregs[UC32_UCF64_FPSCR] = (val & UCF64_FPSCR_MASK);

    changed ^= val;
    if (changed & (UCF64_FPSCR_RND_MASK)) {
        i = UCF64_FPSCR_RND(val);
        switch (i) {
        case 0:
            i = float_round_nearest_even;
            break;
        case 1:
            i = float_round_to_zero;
            break;
        case 2:
            i = float_round_up;
            break;
        case 3:
            i = float_round_down;
            break;
        default: /* 100 and 101 not implement */
            cpu_abort(env, "Unsupported UniCore-F64 round mode");
        }
        set_float_rounding_mode(i, &env->ucf64.fp_status);
    }

    i = ucf64_exceptbits_to_host(UCF64_FPSCR_TRAPEN(val));
    set_float_exception_flags(i, &env->ucf64.fp_status);
}

float32 HELPER(ucf64_adds)(float32 a, float32 b, CPUState *env)
{
    return float32_add(a, b, &env->ucf64.fp_status);
}

float64 HELPER(ucf64_addd)(float64 a, float64 b, CPUState *env)
{
    return float64_add(a, b, &env->ucf64.fp_status);
}

float32 HELPER(ucf64_subs)(float32 a, float32 b, CPUState *env)
{
    return float32_sub(a, b, &env->ucf64.fp_status);
}

float64 HELPER(ucf64_subd)(float64 a, float64 b, CPUState *env)
{
    return float64_sub(a, b, &env->ucf64.fp_status);
}

float32 HELPER(ucf64_muls)(float32 a, float32 b, CPUState *env)
{
    return float32_mul(a, b, &env->ucf64.fp_status);
}

float64 HELPER(ucf64_muld)(float64 a, float64 b, CPUState *env)
{
    return float64_mul(a, b, &env->ucf64.fp_status);
}

float32 HELPER(ucf64_divs)(float32 a, float32 b, CPUState *env)
{
    return float32_div(a, b, &env->ucf64.fp_status);
}

float64 HELPER(ucf64_divd)(float64 a, float64 b, CPUState *env)
{
    return float64_div(a, b, &env->ucf64.fp_status);
}

float32 HELPER(ucf64_negs)(float32 a)
{
    return float32_chs(a);
}

float64 HELPER(ucf64_negd)(float64 a)
{
    return float64_chs(a);
}

float32 HELPER(ucf64_abss)(float32 a)
{
    return float32_abs(a);
}

float64 HELPER(ucf64_absd)(float64 a)
{
    return float64_abs(a);
}

/* XXX: check quiet/signaling case */
void HELPER(ucf64_cmps)(float32 a, float32 b, uint32_t c, CPUState *env)
{
    int flag;
    flag = float32_compare_quiet(a, b, &env->ucf64.fp_status);
    env->CF = 0;
    switch (c & 0x7) {
    case 0: /* F */
        break;
    case 1: /* UN */
        if (flag == 2) {
            env->CF = 1;
        }
        break;
    case 2: /* EQ */
        if (flag == 0) {
            env->CF = 1;
        }
        break;
    case 3: /* UEQ */
        if ((flag == 0) || (flag == 2)) {
            env->CF = 1;
        }
        break;
    case 4: /* OLT */
        if (flag == -1) {
            env->CF = 1;
        }
        break;
    case 5: /* ULT */
        if ((flag == -1) || (flag == 2)) {
            env->CF = 1;
        }
        break;
    case 6: /* OLE */
        if ((flag == -1) || (flag == 0)) {
            env->CF = 1;
        }
        break;
    case 7: /* ULE */
        if (flag != 1) {
            env->CF = 1;
        }
        break;
    }
    env->ucf64.xregs[UC32_UCF64_FPSCR] = (env->CF << 29)
                    | (env->ucf64.xregs[UC32_UCF64_FPSCR] & 0x0fffffff);
}

void HELPER(ucf64_cmpd)(float64 a, float64 b, uint32_t c, CPUState *env)
{
    int flag;
    flag = float64_compare_quiet(a, b, &env->ucf64.fp_status);
    env->CF = 0;
    switch (c & 0x7) {
    case 0: /* F */
        break;
    case 1: /* UN */
        if (flag == 2) {
            env->CF = 1;
        }
        break;
    case 2: /* EQ */
        if (flag == 0) {
            env->CF = 1;
        }
        break;
    case 3: /* UEQ */
        if ((flag == 0) || (flag == 2)) {
            env->CF = 1;
        }
        break;
    case 4: /* OLT */
        if (flag == -1) {
            env->CF = 1;
        }
        break;
    case 5: /* ULT */
        if ((flag == -1) || (flag == 2)) {
            env->CF = 1;
        }
        break;
    case 6: /* OLE */
        if ((flag == -1) || (flag == 0)) {
            env->CF = 1;
        }
        break;
    case 7: /* ULE */
        if (flag != 1) {
            env->CF = 1;
        }
        break;
    }
    env->ucf64.xregs[UC32_UCF64_FPSCR] = (env->CF << 29)
                    | (env->ucf64.xregs[UC32_UCF64_FPSCR] & 0x0fffffff);
}

/* Helper routines to perform bitwise copies between float and int.  */
static inline float32 ucf64_itos(uint32_t i)
{
    union {
        uint32_t i;
        float32 s;
    } v;

    v.i = i;
    return v.s;
}

static inline uint32_t ucf64_stoi(float32 s)
{
    union {
        uint32_t i;
        float32 s;
    } v;

    v.s = s;
    return v.i;
}

static inline float64 ucf64_itod(uint64_t i)
{
    union {
        uint64_t i;
        float64 d;
    } v;

    v.i = i;
    return v.d;
}

static inline uint64_t ucf64_dtoi(float64 d)
{
    union {
        uint64_t i;
        float64 d;
    } v;

    v.d = d;
    return v.i;
}

/* Integer to float conversion.  */
float32 HELPER(ucf64_si2sf)(float32 x, CPUState *env)
{
    return int32_to_float32(ucf64_stoi(x), &env->ucf64.fp_status);
}

float64 HELPER(ucf64_si2df)(float32 x, CPUState *env)
{
    return int32_to_float64(ucf64_stoi(x), &env->ucf64.fp_status);
}

/* Float to integer conversion.  */
float32 HELPER(ucf64_sf2si)(float32 x, CPUState *env)
{
    return ucf64_itos(float32_to_int32(x, &env->ucf64.fp_status));
}

float32 HELPER(ucf64_df2si)(float64 x, CPUState *env)
{
    return ucf64_itos(float64_to_int32(x, &env->ucf64.fp_status));
}

/* floating point conversion */
float64 HELPER(ucf64_sf2df)(float32 x, CPUState *env)
{
    return float32_to_float64(x, &env->ucf64.fp_status);
}

float32 HELPER(ucf64_df2sf)(float64 x, CPUState *env)
{
    return float64_to_float32(x, &env->ucf64.fp_status);
}
