/*
 * arch/arm/mach-tegra/tegra_mpdecision.c
 *
 * This program features:
 * -cpu auto-hotplug/unplug based on system load (runqueue) for tegra quadcore
 *   -automatic decision wether to switch to low power core or not
 * -low power single core while screen is off
 * -extensive sysfs tuneables
 *
 * Copyright (c) 2012-2013, Dennis Rassmann <showp1984@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <asm-generic/cputime.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
#include <linux/input.h>
#include <linux/slab.h>
#endif

#include "clock.h"
#include "cpu-tegra.h"
#include "pm.h"

#define DEBUG 0

#define MPDEC_TAG                       "[MPDEC]: "
#define TEGRA_MPDEC_STARTDELAY            20000
#define TEGRA_MPDEC_DELAY                 130
#define TEGRA_MPDEC_PAUSE                 10000

/* will be overwritten later by lpcpu max clock */
#define TEGRA_MPDEC_IDLE_FREQ             475000

/* This rq value will be used if we only have the lpcpu online */
#define TEGRA_MPDEC_LPCPU_RQ_DOWN         36

/*
 * This will replace TEGRA_MPDEC_DELAY in each case.
 */
#define TEGRA_MPDEC_LPCPU_UPDELAY         130
#define TEGRA_MPDEC_LPCPU_DOWNDELAY       2000

/*
 * LPCPU hysteresis default values
 * we need at least 5 requests to go into lpmode and
 * we need at least 3 requests to come out of lpmode.
 * This does not affect frequency overrides
 */
#define TEGRA_MPDEC_LPCPU_UP_HYS        4
#define TEGRA_MPDEC_LPCPU_DOWN_HYS      2

#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
#define TEGRA_MPDEC_BOOSTTIME             1000
#define TEGRA_MPDEC_BOOSTFREQ_CPU0        910000
#define TEGRA_MPDEC_BOOSTFREQ_CPU1        910000
#define TEGRA_MPDEC_BOOSTFREQ_CPU2        760000
#define TEGRA_MPDEC_BOOSTFREQ_CPU3        620000
#endif

enum {
    TEGRA_MPDEC_DISABLED = 0,
    TEGRA_MPDEC_IDLE,
    TEGRA_MPDEC_DOWN,
    TEGRA_MPDEC_UP,
    TEGRA_MPDEC_LPCPU_UP,
    TEGRA_MPDEC_LPCPU_DOWN,
};

struct tegra_mpdec_cpudata_t {
    struct mutex hotplug_mutex;
    int online;
    int device_suspended;
    cputime64_t on_time;
    cputime64_t on_time_total;
    long long unsigned int times_cpu_hotplugged;
    long long unsigned int times_cpu_unplugged;
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    struct mutex boost_mutex;
    struct mutex unboost_mutex;
    unsigned long int norm_min_freq;
    unsigned long int boost_freq;
    cputime64_t boost_until;
    bool is_boosted;
    bool revib_wq_running;
#endif
};
static DEFINE_PER_CPU(struct tegra_mpdec_cpudata_t, tegra_mpdec_cpudata);
static struct tegra_mpdec_cpudata_t tegra_mpdec_lpcpudata;

static struct delayed_work tegra_mpdec_work;
static struct workqueue_struct *tegra_mpdec_workq;
static struct delayed_work tegra_mpdec_suspended_work;
static struct workqueue_struct *tegra_mpdec_suspended_workq;

static DEFINE_MUTEX(mpdec_tegra_cpu_lock);
static DEFINE_MUTEX(mpdec_tegra_cpu_suspend_lock);
static DEFINE_MUTEX(mpdec_tegra_lpcpu_lock);

#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
static struct workqueue_struct *mpdec_input_wq;
static DEFINE_PER_CPU(struct work_struct, mpdec_input_work);
static struct workqueue_struct *tegra_mpdec_revib_workq;
static DEFINE_PER_CPU(struct delayed_work, tegra_mpdec_revib_work);
#endif

static struct tegra_mpdec_tuners {
    unsigned int startdelay;
    unsigned int delay;
    unsigned int pause;
    bool scroff_single_core;
    unsigned long int idle_freq;
    unsigned int lp_cpu_up_hysteresis;
    unsigned int lp_cpu_down_hysteresis;
    unsigned int max_cpus;
    unsigned int min_cpus;
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    bool boost_enabled;
    unsigned int boost_time;
    unsigned long int boost_freq[4];
#endif
} tegra_mpdec_tuners_ins = {
    .startdelay = TEGRA_MPDEC_STARTDELAY,
    .delay = TEGRA_MPDEC_DELAY,
    .pause = TEGRA_MPDEC_PAUSE,
    .scroff_single_core = true,
    .idle_freq = TEGRA_MPDEC_IDLE_FREQ,
    .lp_cpu_up_hysteresis = TEGRA_MPDEC_LPCPU_UP_HYS,
    .lp_cpu_down_hysteresis = TEGRA_MPDEC_LPCPU_DOWN_HYS,
    .max_cpus = CONFIG_NR_CPUS,
    .min_cpus = 1,
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    .boost_enabled = true,
    .boost_time = TEGRA_MPDEC_BOOSTTIME,
    .boost_freq = {
        TEGRA_MPDEC_BOOSTFREQ_CPU0,
        TEGRA_MPDEC_BOOSTFREQ_CPU1,
        TEGRA_MPDEC_BOOSTFREQ_CPU2,
        TEGRA_MPDEC_BOOSTFREQ_CPU3
    },
#endif
};

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *cpu_lp_clk;

static unsigned int idle_top_freq;
static unsigned int idle_bottom_freq;

static unsigned int NwNs_Threshold[8] = {16, 10, 24, 12, 30, 16, 0, 18};
static unsigned int TwTs_Threshold[8] = {140, 0, 140, 190, 140, 190, 0, 190};

extern unsigned int get_rq_info(void);

unsigned int state = TEGRA_MPDEC_IDLE;
bool was_paused = false;
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
bool is_screen_on = true;
static int update_cpu_min_freq(struct cpufreq_policy *cpu_policy,
                               int cpu, int new_freq);
static void unboost_cpu(int cpu);
#endif
static cputime64_t mpdec_paused_until = 0;

static unsigned long get_rate(int cpu) {
    return tegra_getspeed(cpu);
}

static int get_slowest_cpu(void) {
    int i, cpu = 0;
    unsigned long rate, slow_rate = 0;

    for (i = 0; i < CONFIG_NR_CPUS; i++) {
        if (!cpu_online(i))
                continue;
        rate = get_rate(i);
        if (slow_rate == 0) {
                slow_rate = rate;
        }
        if ((rate <= slow_rate) && (slow_rate != 0)) {
            if (i == 0)
                    continue;
            cpu = i;
            slow_rate = rate;
        }
    }
    return cpu;
}

static int get_slowest_cpu_rate(void) {
    int i = 0;
    unsigned long rate, slow_rate = 0;

    for (i = 0; i < CONFIG_NR_CPUS; i++) {
        rate = get_rate(i);
        if ((rate < slow_rate) && (slow_rate != 0)) {
            slow_rate = rate;
        }
        if (slow_rate == 0) {
            slow_rate = rate;
        }
    }
    return slow_rate;
}

static bool lp_possible(void) {
    int i = 0;
    unsigned int speed;

    for (i = 1; i < CONFIG_NR_CPUS; i++) {
            if (cpu_online(i))
                return false;
    }
    speed = tegra_getspeed(0);
    if (speed > idle_top_freq)
            return false;

    return true;
}

static void mpdec_cpu_up(int cpu) {
    if (!cpu_online(cpu)) {
        mutex_lock(&per_cpu(tegra_mpdec_cpudata, cpu).hotplug_mutex);
        cpu_up(cpu);
        per_cpu(tegra_mpdec_cpudata, cpu).on_time = ktime_to_ms(ktime_get());
        per_cpu(tegra_mpdec_cpudata, cpu).online = true;
        per_cpu(tegra_mpdec_cpudata, cpu).times_cpu_hotplugged += 1;
        pr_info(MPDEC_TAG"CPU[%d] off->on | Mask=[%d.%d%d%d%d]\n",
                cpu, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
                cpu_online(1), cpu_online(2), cpu_online(3));
        mutex_unlock(&per_cpu(tegra_mpdec_cpudata, cpu).hotplug_mutex);
    }
}
EXPORT_SYMBOL_GPL(mpdec_cpu_up);

static void mpdec_cpu_down(int cpu) {
    cputime64_t on_time = 0;
    if (cpu_online(cpu)) {
        mutex_lock(&per_cpu(tegra_mpdec_cpudata, cpu).hotplug_mutex);
        cpu_down(cpu);
        on_time = (ktime_to_ms(ktime_get()) - per_cpu(tegra_mpdec_cpudata, cpu).on_time);
        per_cpu(tegra_mpdec_cpudata, cpu).online = false;
        per_cpu(tegra_mpdec_cpudata, cpu).on_time_total += on_time;
        per_cpu(tegra_mpdec_cpudata, cpu).times_cpu_unplugged += 1;
        pr_info(MPDEC_TAG"CPU[%d] on->off | Mask=[%d.%d%d%d%d] | time online: %llu\n",
                cpu, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
                cpu_online(1), cpu_online(2), cpu_online(3), on_time);
        mutex_unlock(&per_cpu(tegra_mpdec_cpudata, cpu).hotplug_mutex);
    }
}
EXPORT_SYMBOL_GPL(mpdec_cpu_down);

static int mp_decision(void) {
    static bool first_call = true;
    int new_state = TEGRA_MPDEC_IDLE;
    int nr_cpu_online;
    int index;
    unsigned int rq_depth;
    static cputime64_t total_time = 0;
    static cputime64_t last_time;
    cputime64_t current_time;
    cputime64_t this_time = 0;

    if (state == TEGRA_MPDEC_DISABLED)
        return TEGRA_MPDEC_DISABLED;

    current_time = ktime_to_ms(ktime_get());
    if (current_time <= tegra_mpdec_tuners_ins.startdelay)
        return TEGRA_MPDEC_IDLE;

    if (first_call) {
        first_call = false;
    } else {
        this_time = current_time - last_time;
    }
    total_time += this_time;

    rq_depth = get_rq_info();
    nr_cpu_online = num_online_cpus();

    if (nr_cpu_online) {
        index = (nr_cpu_online - 1) * 2;
        if ((nr_cpu_online < CONFIG_NR_CPUS) && (rq_depth >= NwNs_Threshold[index])) {
            if (total_time >= TwTs_Threshold[index]) {
                if ((!is_lp_cluster()) && (nr_cpu_online < tegra_mpdec_tuners_ins.max_cpus))
                    new_state = TEGRA_MPDEC_UP;
                else if (rq_depth > TEGRA_MPDEC_LPCPU_RQ_DOWN)
                    new_state = TEGRA_MPDEC_LPCPU_DOWN;

                if ((get_slowest_cpu_rate() <= tegra_mpdec_tuners_ins.idle_freq)
                     && (new_state != TEGRA_MPDEC_LPCPU_DOWN))
                    new_state = TEGRA_MPDEC_IDLE;
            }
        } else if (rq_depth <= NwNs_Threshold[index+1]) {
            if (total_time >= TwTs_Threshold[index+1] ) {
                if ((nr_cpu_online > 1) && (nr_cpu_online > tegra_mpdec_tuners_ins.min_cpus))
                    new_state = TEGRA_MPDEC_DOWN;
                else if ((get_rate(0) <= idle_top_freq) && (!is_lp_cluster()))
                    new_state = TEGRA_MPDEC_LPCPU_UP;
                else if ((get_rate(0) <= idle_top_freq) && (is_lp_cluster()))
                    new_state = TEGRA_MPDEC_IDLE;

                if (get_slowest_cpu_rate() > tegra_mpdec_tuners_ins.idle_freq)
                    new_state = TEGRA_MPDEC_IDLE;
            }
        } else {
            new_state = TEGRA_MPDEC_IDLE;
            total_time = 0;
        }
    } else {
        total_time = 0;
    }

    if (new_state != TEGRA_MPDEC_IDLE) {
        total_time = 0;
    }

    last_time = ktime_to_ms(ktime_get());

#if DEBUG
    pr_info(MPDEC_TAG"[DEBUG] rq: %u, new_state: %i | Mask=[%d.%d%d%d%d]\n",
            rq_depth, new_state, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
            cpu_online(1), cpu_online(2), cpu_online(3));
#endif
    return new_state;
}

static int tegra_lp_cpu_handler(bool state, bool notifier) {
    bool err = false;
    cputime64_t on_time = 0;

    /* robustness checks */
    if (!cpu_clk) {
        printk(KERN_INFO "[MPDEC]: re-setting cpu_clk");
        cpu_clk = clk_get_sys(NULL, "cpu");
    }
    if (!cpu_lp_clk) {
        printk(KERN_INFO "[MPDEC]: re-setting cpu_lp_clk");
        cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");
    }
    if (!cpu_g_clk) {
        printk(KERN_INFO "[MPDEC]: re-setting cpu_g_clk");
        cpu_g_clk = clk_get_sys(NULL, "cpu_g");
    }
    if (IS_ERR(cpu_clk) || IS_ERR(cpu_lp_clk) || IS_ERR(cpu_g_clk)) {
        printk(KERN_INFO "[MPDEC]: Error, cpu_clk/lp_lck/g_clk still not set");
        return 0;
    }

    if (!mutex_trylock(&mpdec_tegra_lpcpu_lock))
            return 0;

    /* true = up, false = down */
    switch (state) {
    case true:
        if(!clk_set_parent(cpu_clk, cpu_lp_clk)) {
            /* catch-up with governor target speed */
            tegra_cpu_set_speed_cap(NULL);

            pr_info(MPDEC_TAG"CPU[LP] off->on | Mask=[%d.%d%d%d%d]\n",
                    is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
                    cpu_online(1), cpu_online(2), cpu_online(3));
            tegra_mpdec_lpcpudata.on_time = ktime_to_ms(ktime_get());
            tegra_mpdec_lpcpudata.online = true;
            tegra_mpdec_lpcpudata.times_cpu_hotplugged += 1;
            on_time = ktime_to_ms(ktime_get()) - per_cpu(tegra_mpdec_cpudata, 0).on_time;
            per_cpu(tegra_mpdec_cpudata, 0).on_time_total += on_time;
            per_cpu(tegra_mpdec_cpudata, 0).times_cpu_unplugged += 1;
        } else {
            pr_err(MPDEC_TAG" %s (up): clk_set_parent fail\n", __func__);
            err = true;
        }
        break;
    case false:
        if (!clk_set_parent(cpu_clk, cpu_g_clk)) {
            /* catch-up with governor target speed */
            tegra_cpu_set_speed_cap(NULL);

            on_time = ktime_to_ms(ktime_get()) - tegra_mpdec_lpcpudata.on_time;
            tegra_mpdec_lpcpudata.online = false;
            tegra_mpdec_lpcpudata.times_cpu_unplugged += 1;
            tegra_mpdec_lpcpudata.on_time_total += on_time;
            per_cpu(tegra_mpdec_cpudata, 0).on_time = ktime_to_ms(ktime_get());
            per_cpu(tegra_mpdec_cpudata, 0).times_cpu_hotplugged += 1;

            /* was this called because the freq is too high for the lpcpu? */
            if (!notifier)
                pr_info(MPDEC_TAG"CPU[LP] on->off | Mask=[%d.%d%d%d%d] | time on: %llu\n",
                        is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
                        cpu_online(1), cpu_online(2), cpu_online(3), on_time);
            else
                pr_info(MPDEC_TAG"CPU[LP] on->off (freq) | Mask=[%d.%d%d%d%d] | time on: %llu\n",
                        is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
                        cpu_online(1), cpu_online(2), cpu_online(3), on_time);
        } else {
            pr_err(MPDEC_TAG" %s (down): clk_set_parent fail\n", __func__);
            err = true;
        }
        break;
    }

    mutex_unlock(&mpdec_tegra_lpcpu_lock);

    if (err)
        return 0;
    else
        return 1;
}

int mpdecision_gmode_notifier(void) {
    if (!is_lp_cluster())
        return 0;

    if (!mutex_trylock(&mpdec_tegra_cpu_lock))
        return 0;

    if (tegra_lp_cpu_handler(false, true)) {
        /* if we are suspended, start lp checks */
        if ((per_cpu(tegra_mpdec_cpudata, 0).device_suspended == true)) {
            queue_delayed_work(tegra_mpdec_suspended_workq, &tegra_mpdec_suspended_work,
                               TEGRA_MPDEC_LPCPU_UPDELAY);
        } else {
            /* we need to cancel the main workqueue here and restart it
             * with the original delay again. Otherwise it may happen
             * that the lpcpu will jump on/off in < set delay intervals
             */
            cancel_delayed_work_sync(&tegra_mpdec_work);
            was_paused = true;
            queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                               msecs_to_jiffies(TEGRA_MPDEC_LPCPU_DOWNDELAY));
        }
    } else {
        pr_err(MPDEC_TAG"CPU[LP] error, cannot power down.\n");
        mutex_unlock(&mpdec_tegra_cpu_lock);
        return 0;
    }

    mutex_unlock(&mpdec_tegra_cpu_lock);
    return 1;
}
EXPORT_SYMBOL_GPL(mpdecision_gmode_notifier);

static void tegra_mpdec_suspended_work_thread(struct work_struct *work) {
    unsigned int rq_depth;
    rq_depth = get_rq_info();

    if (!mutex_trylock(&mpdec_tegra_cpu_suspend_lock))
        goto out;

    if ((rq_depth <= NwNs_Threshold[1]) &&
        (get_rate(0) <= idle_top_freq) &&
        (!is_lp_cluster()) && (lp_possible())) {
        if (!tegra_lp_cpu_handler(true, false)) {
            pr_err(MPDEC_TAG"CPU[LP] error, cannot power up.\n");
        } else {
            mutex_unlock(&mpdec_tegra_cpu_suspend_lock);
            return;
        }
    }

    mutex_unlock(&mpdec_tegra_cpu_suspend_lock);

out:
    /* LP CPU is not up again, reschedule for next check.
       Since we are suspended, double the delay to save resources */
    queue_delayed_work(tegra_mpdec_suspended_workq, &tegra_mpdec_suspended_work,
                          (TEGRA_MPDEC_DELAY * 2));

    return;
}

static void tegra_mpdec_work_thread(struct work_struct *work) {
    unsigned int cpu = nr_cpu_ids;
    static int lpup_req = 0;
    static int lpdown_req = 0;
    bool suspended = false;

    if (ktime_to_ms(ktime_get()) <= tegra_mpdec_tuners_ins.startdelay)
        goto out;

    /* Check if we are paused */
    if (mpdec_paused_until >= ktime_to_ms(ktime_get()))
        goto out;

    for_each_possible_cpu(cpu)
        if ((per_cpu(tegra_mpdec_cpudata, cpu).device_suspended == true))
                suspended = true;

    if (suspended == true)
        goto out;

    if (!mutex_trylock(&mpdec_tegra_cpu_lock))
        goto out;

    /* if sth messed with the cpus, update the check vars so we can proceed */
    if (was_paused) {
        for_each_possible_cpu(cpu) {
            if (cpu_online(cpu))
                per_cpu(tegra_mpdec_cpudata, cpu).online = true;
            else if (!cpu_online(cpu))
                per_cpu(tegra_mpdec_cpudata, cpu).online = false;
        }
        if (is_lp_cluster())
                tegra_mpdec_lpcpudata.online = true;
        else
                tegra_mpdec_lpcpudata.online = false;
        was_paused = false;
    }

    state = mp_decision();
    switch (state) {
    case TEGRA_MPDEC_IDLE:
        lpup_req = 0;
        lpdown_req = 0;
    case TEGRA_MPDEC_DISABLED:
        break;
    case TEGRA_MPDEC_DOWN:
        lpup_req = 0;
        lpdown_req = 0;
        cpu = get_slowest_cpu();
        if (cpu < nr_cpu_ids) {
            if ((per_cpu(tegra_mpdec_cpudata, cpu).online == true) && (cpu_online(cpu))) {
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
                unboost_cpu(cpu);
#endif
                mpdec_cpu_down(cpu);
            } else if (per_cpu(tegra_mpdec_cpudata, cpu).online != cpu_online(cpu)) {
                pr_info(MPDEC_TAG"CPU[%d] was controlled outside of mpdecision! | pausing [%d]ms\n",
                        cpu, tegra_mpdec_tuners_ins.pause);
                mpdec_paused_until = ktime_to_ms(ktime_get()) + tegra_mpdec_tuners_ins.pause;
                was_paused = true;
            }
        }
        break;
    case TEGRA_MPDEC_UP:
        lpup_req = 0;
        lpdown_req = 0;
        cpu = cpumask_next_zero(0, cpu_online_mask);
        if (cpu < nr_cpu_ids) {
            if ((per_cpu(tegra_mpdec_cpudata, cpu).online == false) && (!cpu_online(cpu))) {
                mpdec_cpu_up(cpu);
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
                unboost_cpu(cpu);
#endif
            } else if (per_cpu(tegra_mpdec_cpudata, cpu).online != cpu_online(cpu)) {
                pr_info(MPDEC_TAG"CPU[%d] was controlled outside of mpdecision! | pausing [%d]ms\n",
                        cpu, tegra_mpdec_tuners_ins.pause);
                mpdec_paused_until = ktime_to_ms(ktime_get()) + tegra_mpdec_tuners_ins.pause;
                was_paused = true;
            }
        }
        break;
    case TEGRA_MPDEC_LPCPU_DOWN:
        lpup_req = 0;
        if (is_lp_cluster()) {
            /* hysteresis loop for lpcpu powerdown
               this prevents the lpcpu to kick out too early and produce lags
               we need at least 3 requests in order to power down the lpcpu */
            lpdown_req++;
            if (lpdown_req > tegra_mpdec_tuners_ins.lp_cpu_down_hysteresis) {
                if(!tegra_lp_cpu_handler(false, false))
                    pr_err(MPDEC_TAG"CPU[LP] error, cannot power down.\n");
                lpdown_req = 0;
            }
        }
        break;
    case TEGRA_MPDEC_LPCPU_UP:
        lpdown_req = 0;
        if ((!is_lp_cluster()) && (lp_possible())) {
            /* hysteresis loop for lpcpu powerup
               this prevents the lpcpu to kick in too early and produce lags
               we need at least 5 requests in order to power up the lpcpu */
            lpup_req++;
            if (lpup_req > tegra_mpdec_tuners_ins.lp_cpu_up_hysteresis) {
                if(!tegra_lp_cpu_handler(true, false))
                    pr_err(MPDEC_TAG"CPU[LP] error, cannot power up.\n");
                lpup_req = 0;
            }
        }
        break;
    default:
        pr_err(MPDEC_TAG"%s: invalid mpdec hotplug state %d\n",
               __func__, state);
    }
    mutex_unlock(&mpdec_tegra_cpu_lock);

out:
    if (state != TEGRA_MPDEC_DISABLED) {
        /* This is being used if the lpcpu up/down delay values are different
         * than the default mpdecision delay. */
        switch (state) {
        case TEGRA_MPDEC_LPCPU_DOWN:
            queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                               msecs_to_jiffies(TEGRA_MPDEC_LPCPU_DOWNDELAY));
            break;
        case TEGRA_MPDEC_LPCPU_UP:
                queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                                   msecs_to_jiffies(TEGRA_MPDEC_LPCPU_UPDELAY));
            break;
        default:
            queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                               msecs_to_jiffies(tegra_mpdec_tuners_ins.delay));
        }
    }
    return;
}

#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
static int update_cpu_min_freq(struct cpufreq_policy *cpu_policy,
                               int cpu, int new_freq) {
    int ret = 0;

    if (!cpu_policy)
        return -EINVAL;

    cpufreq_verify_within_limits(cpu_policy, new_freq, cpu_policy->max);
    cpu_policy->user_policy.min = new_freq;

    ret = cpufreq_update_policy(cpu);
    if (!ret) {
        pr_debug(MPDEC_TAG"Touch event! Setting CPU%d min frequency to %d\n",
            cpu, new_freq);
    }
    return ret;
}

static void unboost_cpu(int cpu) {
    struct cpufreq_policy *cpu_policy = NULL;

    if (cpu_online(cpu)) {
        if (per_cpu(tegra_mpdec_cpudata, cpu).is_boosted) {
            if (mutex_trylock(&per_cpu(tegra_mpdec_cpudata, cpu).unboost_mutex)) {
                cpu_policy = cpufreq_cpu_get(cpu);
                if (!cpu_policy) {
                    pr_debug(MPDEC_TAG"NULL policy on cpu %d\n", cpu);
                    return;
                }
#if DEBUG
                pr_info(MPDEC_TAG"un boosted cpu%i to %lu", cpu, per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq);
#endif
                per_cpu(tegra_mpdec_cpudata, cpu).is_boosted = false;
                per_cpu(tegra_mpdec_cpudata, cpu).revib_wq_running = false;
                if ((cpu_policy->min != per_cpu(tegra_mpdec_cpudata, cpu).boost_freq) &&
                    (cpu_policy->min != per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq)) {
                    pr_info(MPDEC_TAG"cpu%u min was changed while boosted (%lu->%u), using new min",
                            cpu, per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq, cpu_policy->min);
                    per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq = cpu_policy->min;
                }
                update_cpu_min_freq(cpu_policy, cpu, per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq);
                cpufreq_cpu_put(cpu_policy);
                mutex_unlock(&per_cpu(tegra_mpdec_cpudata, cpu).unboost_mutex);
            }
        }
    }

    return;
}

static void tegra_mpdec_revib_work_thread(struct work_struct *work) {
    int cpu = smp_processor_id();

    if (ktime_to_ms(ktime_get()) > per_cpu(tegra_mpdec_cpudata, cpu).boost_until) {
        unboost_cpu(cpu);
    } else {
        queue_delayed_work_on(cpu,
                              tegra_mpdec_revib_workq,
                              &per_cpu(tegra_mpdec_revib_work, cpu),
                              msecs_to_jiffies((per_cpu(tegra_mpdec_cpudata, cpu).boost_until - ktime_to_ms(ktime_get()))));
    }
    return;
}

static void mpdec_input_callback(struct work_struct *unused) {
    struct cpufreq_policy *cpu_policy = NULL;
    int cpu = smp_processor_id();
    bool boosted = false;

    if (!per_cpu(tegra_mpdec_cpudata, cpu).is_boosted) {
        if (mutex_trylock(&per_cpu(tegra_mpdec_cpudata, cpu).boost_mutex)) {
            cpu_policy = cpufreq_cpu_get(cpu);
            if (!cpu_policy) {
                pr_debug(MPDEC_TAG"NULL policy on cpu %d\n", cpu);
                return;
            }
            per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq = cpu_policy->min;
            update_cpu_min_freq(cpu_policy, cpu, per_cpu(tegra_mpdec_cpudata, cpu).boost_freq);
#if DEBUG
            pr_info(MPDEC_TAG"boosted cpu%i to %lu", cpu, per_cpu(tegra_mpdec_cpudata, cpu).boost_freq);
#endif
            per_cpu(tegra_mpdec_cpudata, cpu).is_boosted = true;
            per_cpu(tegra_mpdec_cpudata, cpu).boost_until = ktime_to_ms(ktime_get()) + TEGRA_MPDEC_BOOSTTIME;
            boosted = true;
            cpufreq_cpu_put(cpu_policy);
            mutex_unlock(&per_cpu(tegra_mpdec_cpudata, cpu).boost_mutex);
        }
    } else {
        boosted = true;
    }
    if (boosted && !per_cpu(tegra_mpdec_cpudata, cpu).revib_wq_running) {
        per_cpu(tegra_mpdec_cpudata, cpu).revib_wq_running = true;
        queue_delayed_work_on(cpu,
                              tegra_mpdec_revib_workq,
                              &per_cpu(tegra_mpdec_revib_work, cpu),
                              msecs_to_jiffies(TEGRA_MPDEC_BOOSTTIME));
    } else if (boosted && per_cpu(tegra_mpdec_cpudata, cpu).revib_wq_running) {
        per_cpu(tegra_mpdec_cpudata, cpu).boost_until = ktime_to_ms(ktime_get()) + TEGRA_MPDEC_BOOSTTIME;
    }

    return;
}

static void mpdec_input_event(struct input_handle *handle, unsigned int type,
        unsigned int code, int value) {
    int i = 0;

    if (!tegra_mpdec_tuners_ins.boost_enabled)
        return;

    if (!is_screen_on)
        return;

    for_each_online_cpu(i) {
        queue_work_on(i, mpdec_input_wq, &per_cpu(mpdec_input_work, i));
    }
}

static int input_dev_filter(const char *input_dev_name) {
    if (strstr(input_dev_name, "touch") ||
        strstr(input_dev_name, "keypad")) {
        return 0;
    } else {
        return 1;
    }
}

static int mpdec_input_connect(struct input_handler *handler,
        struct input_dev *dev, const struct input_device_id *id) {
    struct input_handle *handle;
    int error;

    if (input_dev_filter(dev->name))
        return -ENODEV;

    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle)
        return -ENOMEM;

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "mpdec";

    error = input_register_handle(handle);
    if (error)
        goto err2;

    error = input_open_device(handle);
    if (error)
        goto err1;

    return 0;
err1:
    input_unregister_handle(handle);
err2:
    kfree(handle);
    return error;
}

static void mpdec_input_disconnect(struct input_handle *handle) {
    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
}

static const struct input_device_id mpdec_ids[] = {
    { .driver_info = 1 },
    { },
};

static struct input_handler mpdec_input_handler = {
    .event        = mpdec_input_event,
    .connect      = mpdec_input_connect,
    .disconnect   = mpdec_input_disconnect,
    .name         = "mpdec_inputreq",
    .id_table     = mpdec_ids,
};
#endif

static void tegra_mpdec_early_suspend(struct early_suspend *h) {
    int cpu = nr_cpu_ids;

#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    is_screen_on = false;
#endif

    if (!tegra_mpdec_tuners_ins.scroff_single_core) {
        pr_info(MPDEC_TAG"Screen -> off\n");
        return;
    }

    /* main work thread can sleep now */
    cancel_delayed_work_sync(&tegra_mpdec_work);

    /* power down all cpus except 0 and switch to lp mode */
    for_each_possible_cpu(cpu) {
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
        unboost_cpu(cpu);
#endif
        if ((cpu >= 1) && (cpu_online(cpu))) {
            mpdec_cpu_down(cpu);
        }
        per_cpu(tegra_mpdec_cpudata, cpu).device_suspended = true;
    }

    if ((lp_possible()) && (!is_lp_cluster())) {
        if(!tegra_lp_cpu_handler(true, false))
            pr_err(MPDEC_TAG"CPU[LP] error, cannot power up.\n");
    } else if (!is_lp_cluster()) {
        queue_delayed_work(tegra_mpdec_suspended_workq, &tegra_mpdec_suspended_work,
                           TEGRA_MPDEC_LPCPU_UPDELAY);
    }
    pr_info(MPDEC_TAG"Screen -> off. Deactivated mpdecision.\n");
}

static void tegra_mpdec_late_resume(struct early_suspend *h) {
    int cpu = nr_cpu_ids;
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    is_screen_on = true;
#endif

    for_each_possible_cpu(cpu)
        per_cpu(tegra_mpdec_cpudata, cpu).device_suspended = false;

    /* always switch back to g mode on resume */
    if (is_lp_cluster())
        if(!tegra_lp_cpu_handler(false, false))
            pr_err(MPDEC_TAG"CPU[LP] error, cannot power down.\n");

    if (tegra_mpdec_tuners_ins.scroff_single_core) {
        was_paused = true;
        /* wake up main work thread */
        queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                           msecs_to_jiffies(tegra_mpdec_tuners_ins.delay));

        /* restore min/max cpus limits */
        for (cpu=1; cpu<CONFIG_NR_CPUS; cpu++) {
            if (cpu < tegra_mpdec_tuners_ins.min_cpus) {
                if (!cpu_online(cpu))
                    mpdec_cpu_up(cpu);
            } else if (cpu > tegra_mpdec_tuners_ins.max_cpus) {
                if (cpu_online(cpu))
                    mpdec_cpu_down(cpu);
            }
        }
        pr_info(MPDEC_TAG"Screen -> on. Activated mpdecision. | Mask=[%d.%d%d%d%d]\n",
                is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
                cpu_online(1), cpu_online(2), cpu_online(3));
    } else {
        pr_info(MPDEC_TAG"Screen -> on\n");
    }
}

static struct early_suspend tegra_mpdec_early_suspend_handler = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = tegra_mpdec_early_suspend,
    .resume = tegra_mpdec_late_resume,
};

/**************************** SYSFS START ****************************/
struct kobject *tegra_mpdec_kobject;

#define show_one(file_name, object)                    \
static ssize_t show_##file_name                        \
(struct kobject *kobj, struct attribute *attr, char *buf)               \
{                                    \
    return sprintf(buf, "%u\n", tegra_mpdec_tuners_ins.object);    \
}

show_one(startdelay, startdelay);
show_one(delay, delay);
show_one(pause, pause);
show_one(lpcpu_up_hysteresis, lp_cpu_up_hysteresis);
show_one(lpcpu_down_hysteresis, lp_cpu_down_hysteresis);
show_one(scroff_single_core, scroff_single_core);
show_one(min_cpus, min_cpus);
show_one(max_cpus, max_cpus);
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
show_one(boost_enabled, boost_enabled);
show_one(boost_time, boost_time);
#endif

#define show_one_twts(file_name, arraypos)                              \
static ssize_t show_##file_name                                         \
(struct kobject *kobj, struct attribute *attr, char *buf)               \
{                                                                       \
    return sprintf(buf, "%u\n", TwTs_Threshold[arraypos]);          \
}
show_one_twts(twts_threshold_0, 0);
show_one_twts(twts_threshold_1, 1);
show_one_twts(twts_threshold_2, 2);
show_one_twts(twts_threshold_3, 3);
show_one_twts(twts_threshold_4, 4);
show_one_twts(twts_threshold_5, 5);
show_one_twts(twts_threshold_6, 6);
show_one_twts(twts_threshold_7, 7);

#define store_one_twts(file_name, arraypos)                             \
static ssize_t store_##file_name                                        \
(struct kobject *a, struct attribute *b, const char *buf, size_t count) \
{                                                                       \
    unsigned int input;                                             \
    int ret;                                                        \
    ret = sscanf(buf, "%u", &input);                                \
    if (ret != 1)                                                   \
        return -EINVAL;                                         \
    TwTs_Threshold[arraypos] = input;                               \
    return count;                                                   \
}                                                                       \
define_one_global_rw(file_name);
store_one_twts(twts_threshold_0, 0);
store_one_twts(twts_threshold_1, 1);
store_one_twts(twts_threshold_2, 2);
store_one_twts(twts_threshold_3, 3);
store_one_twts(twts_threshold_4, 4);
store_one_twts(twts_threshold_5, 5);
store_one_twts(twts_threshold_6, 6);
store_one_twts(twts_threshold_7, 7);

#define show_one_nwns(file_name, arraypos)                              \
static ssize_t show_##file_name                                         \
(struct kobject *kobj, struct attribute *attr, char *buf)               \
{                                                                       \
    return sprintf(buf, "%u\n", NwNs_Threshold[arraypos]);          \
}
show_one_nwns(nwns_threshold_0, 0);
show_one_nwns(nwns_threshold_1, 1);
show_one_nwns(nwns_threshold_2, 2);
show_one_nwns(nwns_threshold_3, 3);
show_one_nwns(nwns_threshold_4, 4);
show_one_nwns(nwns_threshold_5, 5);
show_one_nwns(nwns_threshold_6, 6);
show_one_nwns(nwns_threshold_7, 7);

#define store_one_nwns(file_name, arraypos)                             \
static ssize_t store_##file_name                                        \
(struct kobject *a, struct attribute *b, const char *buf, size_t count) \
{                                                                       \
    unsigned int input;                                             \
    int ret;                                                        \
    ret = sscanf(buf, "%u", &input);                                \
    if (ret != 1)                                                   \
        return -EINVAL;                                         \
    NwNs_Threshold[arraypos] = input;                               \
    return count;                                                   \
}                                                                       \
define_one_global_rw(file_name);
store_one_nwns(nwns_threshold_0, 0);
store_one_nwns(nwns_threshold_1, 1);
store_one_nwns(nwns_threshold_2, 2);
store_one_nwns(nwns_threshold_3, 3);
store_one_nwns(nwns_threshold_4, 4);
store_one_nwns(nwns_threshold_5, 5);
store_one_nwns(nwns_threshold_6, 6);
store_one_nwns(nwns_threshold_7, 7);

static ssize_t store_lpcpu_up_hysteresis(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    long unsigned int input;
    int ret;
    ret = sscanf(buf, "%lu", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.lp_cpu_up_hysteresis = input;

    return count;
}

static ssize_t store_lpcpu_down_hysteresis(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    long unsigned int input;
    int ret;
    ret = sscanf(buf, "%lu", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.lp_cpu_down_hysteresis = input;

    return count;
}

static ssize_t show_idle_freq (struct kobject *kobj, struct attribute *attr,
                                   char *buf)
{
    return sprintf(buf, "%lu\n", tegra_mpdec_tuners_ins.idle_freq);
}

static ssize_t show_enabled(struct kobject *a, struct attribute *b,
                   char *buf)
{
    unsigned int enabled;
    switch (state) {
    case TEGRA_MPDEC_DISABLED:
        enabled = 0;
        break;
    case TEGRA_MPDEC_IDLE:
    case TEGRA_MPDEC_DOWN:
    case TEGRA_MPDEC_UP:
        enabled = 1;
        break;
    default:
        enabled = 333;
    }
    return sprintf(buf, "%u\n", enabled);
}

static ssize_t store_max_cpus(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret, cpu;
    ret = sscanf(buf, "%u", &input);
    if ((ret != 1) || input > CONFIG_NR_CPUS || input < tegra_mpdec_tuners_ins.min_cpus)
        return -EINVAL;

    tegra_mpdec_tuners_ins.max_cpus = input;
    if (num_online_cpus() > input) {
        for (cpu=CONFIG_NR_CPUS; cpu>0; cpu--) {
            if (num_online_cpus() <= input)
                break;
            if (!cpu_online(cpu))
                continue;
            mpdec_cpu_down(cpu);
        }
        pr_info(MPDEC_TAG"max_cpus set to %u. Affected CPUs were unplugged!\n", input);
    }

    return count;
}

static ssize_t store_min_cpus(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret, cpu;
    ret = sscanf(buf, "%u", &input);
    if ((ret != 1) || input < 1 || input > tegra_mpdec_tuners_ins.max_cpus)
        return -EINVAL;

    tegra_mpdec_tuners_ins.min_cpus = input;
    if (num_online_cpus() < input) {
        for (cpu=1; cpu<CONFIG_NR_CPUS; cpu++) {
            if (num_online_cpus() >= input)
                break;
            if (cpu_online(cpu))
                continue;
            mpdec_cpu_up(cpu);
        }
        pr_info(MPDEC_TAG"min_cpus set to %u. Affected CPUs were hotplugged!\n", input);
    }

    return count;
}

static ssize_t store_startdelay(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.startdelay = input;

    return count;
}

static ssize_t store_delay(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.delay = input;

    return count;
}

static ssize_t store_pause(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.pause = input;

    return count;
}

static ssize_t store_idle_freq(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    long unsigned int input;
    int ret;
    ret = sscanf(buf, "%lu", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.idle_freq = input;

    return count;
}

static ssize_t store_scroff_single_core(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;
    switch (buf[0]) {
        case '0':
        case '1':
            tegra_mpdec_tuners_ins.scroff_single_core = input;
            break;
        default:
            ret = -EINVAL;
    }
    return count;
}

static ssize_t store_enabled(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int cpu, input, enabled;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    switch (state) {
    case TEGRA_MPDEC_DISABLED:
        enabled = 0;
        break;
    case TEGRA_MPDEC_IDLE:
    case TEGRA_MPDEC_DOWN:
    case TEGRA_MPDEC_UP:
        case TEGRA_MPDEC_LPCPU_UP:
        case TEGRA_MPDEC_LPCPU_DOWN:
        enabled = 1;
        break;
    default:
        enabled = 333;
    }

    if (buf[0] == enabled)
        return -EINVAL;

    switch (buf[0]) {
    case '0':
        state = TEGRA_MPDEC_DISABLED;
        pr_info(MPDEC_TAG"nap time... Hot plugging offline CPUs...\n");

        if (is_lp_cluster())
            if(!tegra_lp_cpu_handler(false, false))
                    pr_err(MPDEC_TAG"CPU[LP] error, cannot power down.\n");

        for (cpu = 1; cpu < CONFIG_NR_CPUS; cpu++)
            if (!cpu_online(cpu))
                mpdec_cpu_up(cpu);
        break;
    case '1':
        state = TEGRA_MPDEC_IDLE;
        was_paused = true;
        queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                           msecs_to_jiffies(tegra_mpdec_tuners_ins.delay));
        pr_info(MPDEC_TAG"firing up mpdecision...\n");
        break;
    default:
        ret = -EINVAL;
    }
    return count;
}

#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
static ssize_t store_boost_enabled(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.boost_enabled = input;

    return count;
}

static ssize_t store_boost_time(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    tegra_mpdec_tuners_ins.boost_time = input;

    return count;
}

static ssize_t show_boost_freqs(struct kobject *a, struct attribute *b,
                   char *buf)
{
    ssize_t len = 0;
    int cpu = 0;

    for_each_present_cpu(cpu) {
        len += sprintf(buf + len, "%lu\n", per_cpu(tegra_mpdec_cpudata, cpu).boost_freq);
    }
    return len;
}
static ssize_t store_boost_freqs(struct kobject *a, struct attribute *b,
                   const char *buf, size_t count)
{
    int i = 0;
    unsigned int cpu = 0;
    long unsigned int hz = 0;
    const char *chz = NULL;

    for (i=0; i<count; i++) {
        if (buf[i] == ' ') {
            sscanf(&buf[(i-1)], "%u", &cpu);
            chz = &buf[(i+1)];
        }
    }
    sscanf(chz, "%lu", &hz);
    per_cpu(tegra_mpdec_cpudata, cpu).boost_freq = hz;
    return count;
}
define_one_global_rw(boost_freqs);
#endif

define_one_global_rw(lpcpu_up_hysteresis);
define_one_global_rw(lpcpu_down_hysteresis);
define_one_global_rw(startdelay);
define_one_global_rw(delay);
define_one_global_rw(pause);
define_one_global_rw(scroff_single_core);
define_one_global_rw(idle_freq);
define_one_global_rw(min_cpus);
define_one_global_rw(max_cpus);
define_one_global_rw(enabled);
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
define_one_global_rw(boost_enabled);
define_one_global_rw(boost_time);
#endif

static struct attribute *tegra_mpdec_attributes[] = {
    &lpcpu_up_hysteresis.attr,
    &lpcpu_down_hysteresis.attr,
    &startdelay.attr,
    &delay.attr,
    &pause.attr,
    &scroff_single_core.attr,
    &idle_freq.attr,
    &enabled.attr,
    &min_cpus.attr,
    &max_cpus.attr,
    &twts_threshold_0.attr,
    &twts_threshold_1.attr,
    &twts_threshold_2.attr,
    &twts_threshold_3.attr,
    &twts_threshold_4.attr,
    &twts_threshold_5.attr,
    &twts_threshold_6.attr,
    &twts_threshold_7.attr,
    &nwns_threshold_0.attr,
    &nwns_threshold_1.attr,
    &nwns_threshold_2.attr,
    &nwns_threshold_3.attr,
    &nwns_threshold_4.attr,
    &nwns_threshold_5.attr,
    &nwns_threshold_6.attr,
    &nwns_threshold_7.attr,
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    &boost_freqs.attr,
    &boost_enabled.attr,
    &boost_time.attr,
#endif
    NULL
};


static struct attribute_group tegra_mpdec_attr_group = {
    .attrs = tegra_mpdec_attributes,
    .name = "conf",
};

/********* STATS START *********/

static ssize_t show_time_cpus_on(struct kobject *a, struct attribute *b,
                   char *buf)
{
    ssize_t len = 0;
    int cpu = 0;

    if (is_lp_cluster() == 1) {
        len += sprintf(buf + len, "LP %llu\n",
                       (tegra_mpdec_lpcpudata.on_time_total +
                        (ktime_to_ms(ktime_get()) -
                         tegra_mpdec_lpcpudata.on_time)));
    } else
        len += sprintf(buf + len, "LP %llu\n", tegra_mpdec_lpcpudata.on_time_total);
    for_each_possible_cpu(cpu) {
        if (cpu_online(cpu) && (is_lp_cluster() == 0)) {
            len += sprintf(buf + len, "%i %llu\n", cpu,
                           (per_cpu(tegra_mpdec_cpudata, cpu).on_time_total +
                            (ktime_to_ms(ktime_get()) -
                            per_cpu(tegra_mpdec_cpudata, cpu).on_time)));
        } else
            len += sprintf(buf + len, "%i %llu\n", cpu, per_cpu(tegra_mpdec_cpudata, cpu).on_time_total);
    }

    return len;
}
define_one_global_ro(time_cpus_on);

static ssize_t show_times_cpus_hotplugged(struct kobject *a, struct attribute *b,
                   char *buf)
{
    ssize_t len = 0;
    int cpu = 0;

    len += sprintf(buf + len, "LP %llu\n", tegra_mpdec_lpcpudata.times_cpu_hotplugged);
    for_each_possible_cpu(cpu) {
        len += sprintf(buf + len, "%i %llu\n", cpu, per_cpu(tegra_mpdec_cpudata, cpu).times_cpu_hotplugged);
    }

    return len;
}
define_one_global_ro(times_cpus_hotplugged);

static ssize_t show_times_cpus_unplugged(struct kobject *a, struct attribute *b,
                   char *buf)
{
    ssize_t len = 0;
    int cpu = 0;

    len += sprintf(buf + len, "LP %llu\n", tegra_mpdec_lpcpudata.times_cpu_unplugged);
    for_each_possible_cpu(cpu) {
        len += sprintf(buf + len, "%i %llu\n", cpu, per_cpu(tegra_mpdec_cpudata, cpu).times_cpu_unplugged);
    }

    return len;
}
define_one_global_ro(times_cpus_unplugged);

static struct attribute *tegra_mpdec_stats_attributes[] = {
    &time_cpus_on.attr,
    &times_cpus_hotplugged.attr,
    &times_cpus_unplugged.attr,
    NULL
};


static struct attribute_group tegra_mpdec_stats_attr_group = {
    .attrs = tegra_mpdec_stats_attributes,
    .name = "stats",
};
/**************************** SYSFS END ****************************/

static int __init tegra_mpdec_init(void) {
    int cpu, rc, err = 0;
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    int i;
    unsigned long int boost_freq = 0;
#endif

    cpu_clk = clk_get_sys(NULL, "cpu");
    cpu_g_clk = clk_get_sys(NULL, "cpu_g");
    cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");

    if (IS_ERR(cpu_clk) || IS_ERR(cpu_g_clk) || IS_ERR(cpu_lp_clk))
        return -ENOENT;

    idle_top_freq = clk_get_max_rate(cpu_lp_clk) / 1000;
    idle_bottom_freq = clk_get_min_rate(cpu_g_clk) / 1000;

    /* overwrite idle frequency with lpcpu max clock */
    tegra_mpdec_tuners_ins.idle_freq = idle_top_freq;

    for_each_possible_cpu(cpu) {
        mutex_init(&(per_cpu(tegra_mpdec_cpudata, cpu).hotplug_mutex));
        per_cpu(tegra_mpdec_cpudata, cpu).device_suspended = false;
        per_cpu(tegra_mpdec_cpudata, cpu).online = true;
        per_cpu(tegra_mpdec_cpudata, cpu).on_time_total = 0;
        per_cpu(tegra_mpdec_cpudata, cpu).times_cpu_unplugged = 0;
        per_cpu(tegra_mpdec_cpudata, cpu).times_cpu_hotplugged = 0;
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
        per_cpu(tegra_mpdec_cpudata, cpu).norm_min_freq = 102000;
        switch (cpu) {
            case 0:
            case 1:
            case 2:
                boost_freq = tegra_mpdec_tuners_ins.boost_freq[cpu];
                break;
            default:
                boost_freq = tegra_mpdec_tuners_ins.boost_freq[3];
                break;
        }
        per_cpu(tegra_mpdec_cpudata, cpu).boost_freq = boost_freq;
        per_cpu(tegra_mpdec_cpudata, cpu).is_boosted = false;
        per_cpu(tegra_mpdec_cpudata, cpu).revib_wq_running = false;
        per_cpu(tegra_mpdec_cpudata, cpu).boost_until = 0;
        mutex_init(&(per_cpu(tegra_mpdec_cpudata, cpu).boost_mutex));
        mutex_init(&(per_cpu(tegra_mpdec_cpudata, cpu).unboost_mutex));
#endif
    }
    per_cpu(tegra_mpdec_cpudata, 0).on_time = ktime_to_ms(ktime_get());
    tegra_mpdec_lpcpudata.on_time_total = 0;
    tegra_mpdec_lpcpudata.times_cpu_hotplugged = 0;
    tegra_mpdec_lpcpudata.times_cpu_unplugged = 0;

    was_paused = true;

    tegra_mpdec_workq = alloc_workqueue(
        "mpdec", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
    if (!tegra_mpdec_workq)
        return -ENOMEM;
    INIT_DELAYED_WORK(&tegra_mpdec_work, tegra_mpdec_work_thread);

    tegra_mpdec_suspended_workq = alloc_workqueue(
        "mpdec_sus", WQ_UNBOUND | WQ_RESCUER, 1);
    if (!tegra_mpdec_suspended_workq)
        return -ENOMEM;
    INIT_DELAYED_WORK(&tegra_mpdec_suspended_work,
                      tegra_mpdec_suspended_work_thread);

#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    mpdec_input_wq = create_workqueue("mpdeciwq");
    if (!mpdec_input_wq) {
        printk(KERN_ERR "%s: Failed to create mpdeciwq workqueue\n", __func__);
        return -EFAULT;
    }
    tegra_mpdec_revib_workq = create_workqueue("mpdecribwq");
    if (!tegra_mpdec_revib_workq) {
        printk(KERN_ERR "%s: Failed to create mpdecrevibwq workqueue\n", __func__);
        return -EFAULT;
    }
    for_each_possible_cpu(i) {
        INIT_WORK(&per_cpu(mpdec_input_work, i), mpdec_input_callback);
        INIT_DELAYED_WORK(&per_cpu(tegra_mpdec_revib_work, i), tegra_mpdec_revib_work_thread);
    }
    rc = input_register_handler(&mpdec_input_handler);
#endif

    if (state != TEGRA_MPDEC_DISABLED)
        queue_delayed_work(tegra_mpdec_workq, &tegra_mpdec_work,
                           msecs_to_jiffies(tegra_mpdec_tuners_ins.delay));

    register_early_suspend(&tegra_mpdec_early_suspend_handler);

    tegra_mpdec_kobject = kobject_create_and_add("tegra_mpdecision", kernel_kobj);
    if (tegra_mpdec_kobject) {
        rc = sysfs_create_group(tegra_mpdec_kobject,
                                &tegra_mpdec_attr_group);
        if (rc) {
            pr_warn(MPDEC_TAG"sysfs: ERROR, could not create sysfs group");
        }
        rc = sysfs_create_group(tegra_mpdec_kobject,
                            &tegra_mpdec_stats_attr_group);
        if (rc) {
            pr_warn(MPDEC_TAG"sysfs: ERROR, could not create sysfs stats group");
        }
    } else
        pr_warn(MPDEC_TAG"sysfs: ERROR, could not create sysfs kobj");

    pr_info(MPDEC_TAG"%s init complete.", __func__);

    return err;
}

late_initcall(tegra_mpdec_init);

void tegra_mpdec_exit(void) {
#ifdef CONFIG_TEGRA_MPDECISION_INPUTBOOST_CPUMIN
    input_unregister_handler(&mpdec_input_handler);
    destroy_workqueue(tegra_mpdec_revib_workq);
    destroy_workqueue(mpdec_input_wq);
#endif
    destroy_workqueue(tegra_mpdec_workq);
    destroy_workqueue(tegra_mpdec_suspended_workq);
}
