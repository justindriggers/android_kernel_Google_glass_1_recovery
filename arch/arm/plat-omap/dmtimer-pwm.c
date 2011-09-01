/*
 * dmtimer-pwm.c
 * Driver for OMAP4430 Dual-Mode Timers as Pulse Width Modulated outputs.
 *
 * Copyright (C) 2011 Google Inc.
 * Author: John Hawley <madsci@google.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include <plat/clock.h>
#include <plat/dmtimer.h>
#include <plat/dmtimer-pwm.h>

static struct pwm_device *pwm_id_map[MAX_PWM_ID + 1];
static unsigned int next_available_id = 0;

/*
 * Convert period (in ns) to clock cycles based on the fclk of the timer used.
 */
static unsigned long period_to_cycles(struct pwm_device *pwm, int period_ns) {
  unsigned long timer_clk_rate;

  /* We save the clock rate because clk_get_rate() is an expensive call. */
  if (pwm->divisor == 0) {
    timer_clk_rate = clk_get_rate(omap_dm_timer_get_fclk(pwm->timer));
    if (timer_clk_rate == 0) {
      printk(KERN_ERR "%s %s: Failed to get sys_clkin_ck rate",
             __FILE__, __func__);
      return 0;
    }
    pwm->divisor = 1000000000UL / timer_clk_rate;
    if (pwm->divisor == 0) {
      printk(KERN_ERR "%s %s: Timer clock greater than 1GHz not supported",
             __FILE__, __func__);
      return 0;
    }
  }

  return (unsigned long)period_ns / pwm->divisor;
}

struct pwm_device *pwm_request_dmtimer(int timer_id, const char *label,
                                       struct dmtimer_pwm_ops *ops) {
  struct omap_dm_timer *timer;

  if (next_available_id > MAX_PWM_ID) {
    printk(KERN_ERR "%s %s: Insufficient PWM IDs", __FILE__, __func__);
    return NULL;
  }

  timer = omap_dm_timer_request_specific(timer_id);
  if (!timer) {
    printk(KERN_ERR "%s %s: Request failed for dmtimer %i", __FILE__,
           __func__, timer_id);
    return NULL;
  }

  omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
  omap_dm_timer_set_pwm(timer, 1, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

  pwm_id_map[next_available_id] = kzalloc(sizeof(struct pwm_device),
                                          GFP_KERNEL);
  pwm_id_map[next_available_id]->label = label;
  pwm_id_map[next_available_id]->pwm_id = next_available_id;
  pwm_id_map[next_available_id]->dmtimer_id = timer_id;
  pwm_id_map[next_available_id]->timer = timer;
  pwm_id_map[next_available_id]->ops = ops;
  pwm_id_map[next_available_id]->divisor = 0;

  return pwm_id_map[next_available_id++];
}
EXPORT_SYMBOL(pwm_request_dmtimer);

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns) {
  const unsigned int overflow = 0xffffffff;
  unsigned int duty_cycles;
  unsigned int period_cycles;

  if (!pwm) {
    printk(KERN_ERR "%s %s: NULL pwm_device", __FILE__, __func__);
    return -EINVAL;
  }

  if (!pwm->timer) {
    printk(KERN_ERR "%s %s: pwm_device with id %i has NULL timer",
            __FILE__, __func__, pwm->pwm_id);
    return -EINVAL;
  }

  /* Convert nanoseconds to cycles based on the clock.  */
  duty_cycles = period_to_cycles(pwm, duty_ns);
  period_cycles = period_to_cycles(pwm, period_ns);

  /*
   * period_cycles must be at least 3 because duty_cycles must be
   * less than period_cycles and must also be at least 2.
   */
  if (period_cycles < 3) {
    printk(KERN_ERR "%s %s: Invalid period %i ns, too short",
          __FILE__, __func__, period_ns);
    return -EINVAL;
  }

  if (duty_cycles >= period_cycles) {
    duty_cycles = period_cycles - 1;
  } else if (duty_cycles < 2) {
    duty_cycles = 2;
  }

  omap_dm_timer_set_load_start(pwm->timer, 1, overflow - period_cycles);
  omap_dm_timer_set_match(pwm->timer, 1, overflow - duty_cycles);
  return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm) {
  if (!pwm) {
    printk(KERN_ERR "%s %s: NULL pwm_device", __FILE__, __func__);
    return -EINVAL;
  }

  if (!pwm->timer) {
    printk(KERN_ERR "%s %s: pwm_device with id %i has NULL timer",
            __FILE__, __func__, pwm->pwm_id);
    return -EINVAL;
  }

  omap_dm_timer_enable(pwm->timer);

  /* Call board-specific enable function now that the timer is started. */
  if (pwm->ops && pwm->ops->enable) {
    /*
     * We need a short delay here to be certain that the timer has actually
     * started.  If the timer hasn't yet started, the output will be high which
     * corresponds to a full-on pwm, which is likely undesirable.
     * Experimentally, 10 ms is long enough, but this may need to be adjusted
     * in the future.
     */
    msleep(10);
    pwm->ops->enable();
  }
  return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm) {
  if (!pwm) {
    printk(KERN_ERR "%s %s: NULL pwm_device", __FILE__, __func__);
    return;
  }

  if (!pwm->timer) {
    printk(KERN_ERR "%s %s: pwm_device with id %i has NULL timer",
            __FILE__, __func__, pwm->pwm_id);
    return;
  }

  /* Call board-specific disable function before stopping the timer. */
  if (pwm->ops && pwm->ops->disable) {
    pwm->ops->disable();
  }

  omap_dm_timer_disable(pwm->timer);
}
EXPORT_SYMBOL(pwm_disable);

struct pwm_device *pwm_request(int pwm_id, const char *label) {
  if (pwm_id < 0 || pwm_id > MAX_PWM_ID) {
    printk(KERN_ERR "%s %s: pwm_id %i out of range [0, %i]",
            __FILE__, __func__, pwm_id, MAX_PWM_ID);
    return NULL;
  }

  if (!pwm_id_map[pwm_id]) {
    printk(KERN_ERR "%s %s: Request for unallocated pwm_id %i, call "
            "pwm_request_dmtimer() first", __FILE__, __func__, pwm_id);
    return NULL;
  }

  if (strcmp(pwm_id_map[pwm_id]->label, label)) {
    printk(KERN_WARNING "%s %s: Requested label (%s) for pwm_id %i does not "
            "match existing label (%s)", __FILE__, __func__, label, pwm_id,
            pwm_id_map[pwm_id]->label);
  }

  return pwm_id_map[pwm_id];
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm) {
  int i;

  if (!pwm) {
    return;
  }

  for (i = 0; i < next_available_id; ++i) {
    if (pwm_id_map[pwm->pwm_id] == pwm) {
      pwm_id_map[pwm->pwm_id] = NULL;
      break;
    }
  }

  pwm_disable(pwm);
  if (pwm->timer) {
    omap_dm_timer_free(pwm->timer);
  }

  kfree(pwm);
  return;
}
EXPORT_SYMBOL(pwm_free);
