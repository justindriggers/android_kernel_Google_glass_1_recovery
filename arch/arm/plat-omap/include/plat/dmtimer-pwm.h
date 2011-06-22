// Copyright 2011 Google Inc. All Rights Reserved.
// Author: madsci@google.com (John Hawley)

#ifndef __PLAT_DMTIMER_PWM_H_
#define __PLAT_DMTIMER_PWM_H_

#define MAX_PWM_ID      0xf

/*
 * Ops used to enable/disable dmtimers for pwm use.  The dmtimer-pwm driver will
 * automatically start/stop the dmtimer and configure it for pwm mode, but there
 * may be other board-specific configuration required, ie mux configuration.
 * Any such board-specific configuration should be done in these ops fucntions.
 */
struct dmtimer_pwm_ops {
  void (*enable)(void);
  void (*disable)(void);
};

struct pwm_device {
  const char *label;
  unsigned int pwm_id;
  int dmtimer_id;
  struct omap_dm_timer *timer;
  struct dmtimer_pwm_ops *ops;
  /*
   * Used to convert period (ns) to clock cycles, this value will be
   * automatically obtained by examining the fref_clk frequency used
   * by the dmtimer.
   * */
  unsigned long divisor;
};

struct pwm_device *pwm_request_dmtimer(int timer_id, const char *label,
                                       struct dmtimer_pwm_ops *ops);

#endif
