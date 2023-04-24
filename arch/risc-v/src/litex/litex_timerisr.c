/****************************************************************************
 * arch/risc-v/src/litex/litex_timerisr.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/init.h>

#include <debug.h>
#include "riscv_internal.h"

#include "litex.h"
#include "litex_clockconfig.h"
#include "hardware/litex_timer.h"
#include "riscv_mtimer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (litex_get_hfclk() / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  litex_timerisr
 ****************************************************************************/

#ifdef CONFIG_LITEX_CORE_VEXRISCV_SMP
static void litex_mtimer_initialise(void)
{
  struct oneshot_lowerhalf_s *lower = riscv_mtimer_initialize(
    LITEX_CLINT_MTIME, LITEX_CLINT_MTIMECMP,
    RISCV_IRQ_TIMER, litex_get_hfclk());

  DEBUGASSERT(lower);

  up_alarm_set_lowerhalf(lower);
}

#else

static int litex_timerisr(int irq, void *context, void *arg)
{
  /* Clear timer interrupt */

  putreg32(0xffffffff, LITEX_TIMER0_EV_PENDING);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

static void litex_timer0_initialize(void)
{
  /* Disable the timer and clear any pending interrupt */

  putreg32(0, LITEX_TIMER0_EN);
  putreg32(getreg32(LITEX_TIMER0_EV_PENDING), LITEX_TIMER0_EV_PENDING);

  /* Set the timer period */

  putreg32(TICK_COUNT, LITEX_TIMER0_RELOAD);
  putreg32(getreg32(LITEX_TIMER0_RELOAD), LITEX_TIMER0_LOAD);

  /* Attach timer interrupt handler */

  irq_attach(LITEX_IRQ_TIMER0, litex_timerisr, NULL);

  /* Enable the timer */

  putreg32(1, LITEX_TIMER0_EN);

  /* And enable the timer interrupt */

  putreg32(1, LITEX_TIMER0_EV_ENABLE);
  up_enable_irq(LITEX_IRQ_TIMER0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
#ifdef CONFIG_LITEX_CORE_VEXRISCV_SMP
  litex_mtimer_initialise();
#else
  litex_timer0_initialize();
#endif
}
