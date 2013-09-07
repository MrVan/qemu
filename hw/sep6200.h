#include "memory.h"
typedef struct clk *sep6200_clk;

struct sep6200_state {
	MemoryRegion *ram;
	MemoryRegion *esram;
	int remap;
	sep6200_clk clks;
};

extern struct sep6200_state sep6200_state1;
void sep6200_clk_init(struct sep6200_state *s);
void sep6200_clk_adduser(struct clk *clk, qemu_irq user);
int64_t sep6200_clk_getrate(sep6200_clk clk);
void sep6200_clk_get(struct clk *clk);
struct clk *sep6200_findclk(struct sep6200_state *s, const char *name);
void sep6200_clk_put(struct clk *clk);
void sep6200_clk_reparent(struct clk *clk, struct clk *parent);
void sep6200_clk_onoff(struct clk *clk ,int on);
void sep6200_clk_canidle(struct clk *clk, int can);
void sep6200_clk_setrate(struct clk *clk, int divide, int multiply);
void sep6200_sysctl_init(uint32_t base, uint32_t sys_id, uint32_t proc_id);


qemu_irq *uc32_pic_init_cpu(CPUState *env);
