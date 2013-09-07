#include "hw.h"
#include "sep6200.h"

struct clk {
	const char *name;
	const char *alias;
	struct clk *parent;
	struct clk *child1;
	struct clk *sibling;
#define ALWAYS_ENABLED		(1 << 0)
#define CLOCK_IN_SEP6200 	(1 << 10)
	uint32_t flags;
	int id;

	int running;
	int enabled;
	unsigned long rate;
	unsigned int divisor;
	unsigned int multiplier;
	qemu_irq users[16];
	int usecount;
};

static struct clk ref_clk = {
	.name = "ref_clk",
	.flags = CLOCK_IN_SEP6200,
	.rate = 12000000, //12mhz
	.enabled = 1,
};

static struct clk apll = {
	.name = "apll_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &ref_clk,
	.enabled = 1,
};

static struct clk dpll = {
	.name = "dpll_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &ref_clk,
	.enabled = 1,
};

static struct clk mpll = {
	.name = "mpll_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &ref_clk,
	.enabled = 1,
};

static struct clk bus1 = {
	.name = "bus1_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &mpll,
	.enabled = 1,
};

static struct clk bus2 = {
	.name = "bus2_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &mpll,
	.enabled = 1,
};

static struct clk bus3 = {
	.name = "bus3_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &mpll,
	.enabled = 1,
};

static struct clk bus4 = {
	.name = "bus4_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &mpll,
	.enabled = 1,
};

static struct clk bus5 = {
	.name = "bus5_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &mpll,
	.enabled = 1,
};

static struct clk timer = {
	.name = "timer_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &bus4,
	//.rate = 12000000,
	.enabled = 1,
};

static struct clk uart = {
	.name = "uart_clk",
	.flags = CLOCK_IN_SEP6200,
	.parent = &bus5,
	.enabled = 1,
};

static struct clk *onchip_clks[] = {
	&ref_clk,
	&apll,
	&dpll,
	&mpll,
	&bus1,
	&bus2,
	&bus3,
	&bus4,
	&bus5,
	&timer,
	&uart,
	NULL
};

void sep6200_clk_adduser(struct clk *clk, qemu_irq user)
{
	qemu_irq *i;

	for (i = clk->users; *i; i++);
	*i = user;
}

struct clk *sep6200_findclk(struct sep6200_state *s, const char *name)
{
	struct clk *i;

	for (i = s->clks; i->name; i++)
		if (!strcmp(i->name, name) || (i->alias && !strcmp(i->alias, name)))
			return i;
	hw_error("%s: %s not fount\n", __FUNCTION__, name);
}

void sep6200_clk_get(struct clk *clk)
{
	clk->usecount++;
}

void sep6200_clk_put(struct clk *clk)
{
	if (!(clk->usecount--))
		hw_error("%s: %s is not in use\n", __FUNCTION__, clk->name);
}

static void sep6200_clk_update(struct clk *clk)
{
	int parent, running;
	qemu_irq *user;
	struct clk *i;

	if (clk->parent)
		parent = clk->parent->running;
	else 
		parent = 1;

	running = parent && (clk->enabled ||
			((clk->flags & ALWAYS_ENABLED) && clk->usecount));
	
    	if (clk->running != running) {
       		clk->running = running;
        	for (user = clk->users; *user; user ++)
            		qemu_set_irq(*user, running);
        	for (i = clk->child1; i; i = i->sibling)
            		sep6200_clk_update(i);
    }
}

static void sep6200_clk_rate_update_full(struct clk *clk, unsigned long int rate,\
		unsigned long int div, unsigned long int mult)
{
	struct clk *i;
	qemu_irq *user;

	clk->rate = muldiv64(rate, mult, div);
	if (clk->running) {
		for (user = clk->users; *user; user++)	
			qemu_irq_raise(*user);
	}
	for (i = clk->child1; i; i= i->sibling)
		sep6200_clk_rate_update_full(i, rate, div * i->divisor, mult * i->multiplier);
}

static void sep6200_clk_rate_update(struct clk *clk)
{
	struct clk *i;
	unsigned long int div, mult = div = 1;

	for (i = clk; i->parent; i = i->parent) {
		div *= i->divisor;
		mult *= i->multiplier;
	}

	sep6200_clk_rate_update_full(clk, i->rate, div, mult);
}

void sep6200_clk_reparent(struct clk *clk, struct clk *parent)
{
	struct clk **p;

	if (clk->parent) {
		for (p = &clk->parent->child1; *p != clk; p = &(*p)->sibling);	
		*p = clk->sibling;
	}

	clk->parent = parent;
	if (parent) {
		clk->sibling = parent->child1;
		parent->child1 = clk;
		sep6200_clk_update(clk);
		sep6200_clk_rate_update(clk);
	} else
		clk->sibling = NULL;
}

void sep6200_clk_onoff(struct clk *clk ,int on)
{
	clk->enabled = on;
	sep6200_clk_update(clk);
}

void sep6200_clk_canidle(struct clk *clk, int can)
{
	if (can) 
		sep6200_clk_put(clk);
	else 
		sep6200_clk_get(clk);
}

void sep6200_clk_setrate(struct clk *clk, int divide, int multiply)
{
	clk->divisor = divide;
	clk->multiplier = multiply;
	sep6200_clk_rate_update(clk);
}

int64_t sep6200_clk_getrate(sep6200_clk clk)
{
	return clk->rate;
}

void sep6200_clk_init(struct sep6200_state *s)
{
	struct clk **i, *j, *k;
	int count;
	int flags;

	flags = CLOCK_IN_SEP6200;

	for (i = onchip_clks, count = 0; *i; i++) {
		if ((*i)->flags & flags)	
			count++;
	}

	s->clks = (struct clk *)g_malloc0(sizeof(struct clk) * (count + 1));

	for (i = onchip_clks, j = s->clks; *i; i++) {
		if ((*i)->flags & flags) {
			memcpy(j, *i, sizeof(struct clk));
			for (k = s->clks; k < j; k++) {
				if (j->parent && !strcmp(j->parent->name, k->name)) {
					j->parent = k;
					j->sibling = k->child1;
					k->child1 = j;
				} else if (k->parent && !strcmp(k->parent->name, j->name)) {
					k->parent = j;
					k->sibling = j->child1;
					j->child1 = k;
				}
			}
			j->divisor = j->divisor ?: 1;
			j->multiplier = j->multiplier ?: 1;
			j++;
		}
	}
	for (j = s->clks; count--; j++) {
		sep6200_clk_update(j);
		sep6200_clk_rate_update(j);
	}
}
