/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_audio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/signal.h>
#include <arch/chip/audio.h>

#ifdef CONFIG_AUDIO_CXD56
#include "arch/chip/cxd56_audio_lower.h"
#endif

#include "chip.h"
#include "arm_internal.h"

#include <arch/board/board.h>
#include "cxd56_pmic.h"
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef CXD5247_XRST
#  error "CXD5247_XRST must be defined in board.h !!"
#endif
#ifndef CXD5247_AVDD
#  error "CXD5247_AVDD must be defined in board.h !!"
#endif
#ifndef CXD5247_DVDD
#  error "CXD5247_DVDD must be defined in board.h !!"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: check_pin_i2s_mode
 *
 * Description:
 *   Check if the pin is I2S.
 *
 ****************************************************************************/

#if defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1)
static bool check_pin_i2s_mode(uint32_t pin)
{
  bool                res = false;
  cxd56_pin_status_t  pstat;

  if (cxd56_pin_status(pin, &pstat) >= 0)
    {
      if (pstat.mode == 1)
        {
          res = true;
        }
    }

  return res;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_audio_power_control
 *
 * Description:
 *   Power on/off the audio device on the board.
 *
 ****************************************************************************/

bool board_audio_power_control(bool en)
{
  if (en)
    {
      /* Enable I2S pin. */

      cxd56_audio_en_i2s_io();

      /* Enable speaker output. */

      cxd56_audio_set_spout(true);

      /* Power on Audio driver */

      if (cxd56_audio_poweron() != 0)
        {
          return false;
        }

      /* Enable BaseBand driver output */

      if (cxd56_audio_en_output() != 0)
        {
          return false;
        }

      /* Cancel output mute. */

      board_external_amp_mute_control(false);
    }
    else
    {
      /* Set output mute. */

      board_external_amp_mute_control(true);

      /* Disable speaker output. */

      cxd56_audio_set_spout(false);

      /* Disable I2S pin. */

      cxd56_audio_dis_i2s_io();

      /* Power off Audio driver */

      if (cxd56_audio_dis_output() != 0)
        {
          return false;
        }

      /* Disable BaseBand driver output */

      if (cxd56_audio_poweroff() != 0)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: board_aca_power_control
 *
 * Description:
 *   Power on/off the Aca device on the board.
 *
 ****************************************************************************/

int board_aca_power_control(int target, bool en)
{
  int ret = 0;
  static int first = 1;
  static bool avdd_on = false;
  static bool dvdd_on = false;

  if (first)
    {
      /* gpio configuration (output disabled yet) */

      cxd56_gpio_config(CXD5247_XRST, false);

      first = 0;
    }

  if (en)
    {
      if (!dvdd_on && (target & CXD5247_DVDD))
        {
          /* reset assert */

          cxd56_gpio_write(CXD5247_XRST, false);
        }

      /* power on */

      if (!avdd_on && (target & CXD5247_AVDD))
        {
          board_power_control(POWER_AUDIO_AVDD, true);
          avdd_on = true;
        }

      if (!dvdd_on && (target & CXD5247_DVDD))
        {
          board_power_control(POWER_AUDIO_DVDD, true);
          dvdd_on = true;

          /* reset release */

          cxd56_gpio_write(CXD5247_XRST, true);
        }
    }
  else
    {
      if (dvdd_on && (target & CXD5247_DVDD))
        {
          /* reset assert */

          cxd56_gpio_write(CXD5247_XRST, false);
        }

      /* power off */

      if (avdd_on && (target & CXD5247_AVDD))
        {
          board_power_control(POWER_AUDIO_AVDD, false);
          avdd_on = false;
        }

      if (dvdd_on && (target & CXD5247_DVDD))
        {
          board_power_control(POWER_AUDIO_DVDD, false);
          dvdd_on = false;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_aca_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Aca device on the board.
 *
 ****************************************************************************/

bool board_aca_power_monitor(int target)
{
  bool avdd_stat = true;
  bool dvdd_stat = true;

  if (target & CXD5247_AVDD)
    {
      avdd_stat = board_power_monitor(POWER_AUDIO_AVDD);
    }

  if (target & CXD5247_DVDD)
    {
      dvdd_stat = board_power_monitor(POWER_AUDIO_DVDD);
    }

  return avdd_stat && dvdd_stat;
}

#define MUTE_OFF_DELAY  (1250 * 1000) /* ms */
#define MUTE_ON_DELAY   (150 * 1000)  /* ms */

/****************************************************************************
 * Name: board_external_amp_mute_control
 *
 * Description:
 *   External Amp. Mute on/off.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

int board_external_amp_mute_control(bool en)
{
  int ret = 0;

  if (en)
    {
      /* Mute ON */

      ret = board_power_control(POWER_AUDIO_MUTE, false);
      nxsig_usleep(MUTE_ON_DELAY);
    }
  else
    {
      /* Mute OFF */

      nxsig_usleep(MUTE_OFF_DELAY);
      ret = board_power_control(POWER_AUDIO_MUTE, true);
    }

  return ret;
}

/****************************************************************************
 * Name: board_external_amp_mute_monitor
 *
 * Description:
 *   Get External Amp. Mute status.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

bool board_external_amp_mute_monitor(void)
{
  bool mute = board_power_monitor(POWER_AUDIO_MUTE);
  return !mute;
}

/****************************************************************************
 * Name: board_audio_i2s_enable
 *
 * Description:
 *   Enable I2S on the board.
 *
 ****************************************************************************/

void board_audio_i2s_enable(void)
{
#ifdef CONFIG_CXD56_I2S0
  /* Select I2S0_BCK, I2S0_LRCK, I2S0_DATA_IN, I2S0_DATA_OUT. */

#  ifdef CONFIG_CXD56_AUDIO_I2S_DEVICE_1_MASTER
  /* I2S0 Master. */

#    ifdef CONFIG_CXD56_AUDIO_I2S_LOWEMI_2MA
  CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_NORM);
#    else
  CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_HIGH);
#    endif
#  else
  /* I2S0 Slave. */

#    ifdef CONFIG_CXD56_AUDIO_I2S_LOWEMI_2MA
  CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_NORM);
#    else
  CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_HIGH);
#    endif
#  endif /* CONFIG_CXD56_AUDIO_I2S_DEVICE_1_MASTER */
#endif /* CONFIG_CXD56_I2S0 */

#ifdef CONFIG_CXD56_I2S1
  /* Select I2S1_BCK, I2S1_LRCK, I2S1_DATA_IN, I2S1_DATA_OUT. */

#  ifdef CONFIG_CXD56_AUDIO_I2S_DEVICE_2_MASTER
  /* I2S1 Master. */

#    ifdef CONFIG_CXD56_AUDIO_I2S_LOWEMI_2MA
  CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_NORM);
#    else
  CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_HIGH);
#    endif
#  else
  /* I2S1 Slave. */

#    ifdef CONFIG_CXD56_AUDIO_I2S_LOWEMI_2MA
  CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_NORM);
#    else
  CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_HIGH);
#    endif
#  endif /* CONFIG_CXD56_AUDIO_I2S_DEVICE_2_MASTER */
#endif /* CONFIG_CXD56_I2S1 */
}

/****************************************************************************
 * Name: board_audio_i2s_disable
 *
 * Description:
 *   Disable I2S on the board.
 *
 ****************************************************************************/

void board_audio_i2s_disable(void)
{
#ifdef CONFIG_CXD56_I2S0
  /* Select GPIO(P1v_00/01/02/03) */

  if (check_pin_i2s_mode(PIN_I2S0_BCK))
    {
      CXD56_PIN_CONFIGS(PINCONFS_I2S0_GPIO);
    }
#endif

#ifdef CONFIG_CXD56_I2S1
  /* Select GPIO(P1v_00/01/02/03) */

  if (check_pin_i2s_mode(PIN_I2S1_BCK))
    {
      CXD56_PIN_CONFIGS(PINCONFS_I2S1_GPIO);
    }
#endif
}

/****************************************************************************
 * Name: board_audio_initialize
 *
 * Description:
 *   Initialize audio I/O on the board.
 *
 ****************************************************************************/

void board_audio_initialize(void)
{
  /* Select MCLK. */

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  CXD56_PIN_CONFIGS(PINCONFS_MCLK);
#endif

  /* Select PDM_CLK, PDM_IN, PDM_OUT. */

#ifdef CONFIG_CXD56_AUDIO_PDM_LOWEMI_2MA
  CXD56_PIN_CONFIGS(PINCONFS_PDM_NORM);
#else
  CXD56_PIN_CONFIGS(PINCONFS_PDM_HIGH);
#endif
}

/****************************************************************************
 * Name: board_audio_finalize
 *
 * Description:
 *   Finalize audio I/O on the board.
 *
 ****************************************************************************/

void board_audio_finalize(void)
{
  /* Select GPIO(P1x_00). */

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  CXD56_PIN_CONFIGS(PINCONFS_MCLK_GPIO);
#endif

  /* Select GPIO(P1y_00/01/02). */

  CXD56_PIN_CONFIGS(PINCONFS_PDM_GPIO);

  /* Disable I2S. */

  board_audio_i2s_disable();
}

#ifdef CONFIG_AUDIO_CXD56
/****************************************************************************
 * Name: board_audio_initialize_driver
 *
 * Description:
 *   Initialize and register the CXD56 audio driver.
 *
 ****************************************************************************/

int board_audio_initialize_driver(void)
{
  struct audio_lowerhalf_s *dev;
  int ret;

  /* Initialize CXD56 Audio System */

  ret = cxd56_audsystem_initialize(cxd56_audio_lower());
  ASSERT(ret == OK);

  /* Initialize MIC Input device */

  dev = cxd56_aud_miclower();
  ret = audio_register("pcm_in0", dev);
  ASSERT(ret == OK);

  /* Initialize SPK Output device */

  dev = cxd56_aud_spk0out();
  ret = audio_register("pcm0", dev);
  ASSERT(ret == OK);

  /* Initialize I2S Output device */

  dev = cxd56_aud_spk1out();
  ret = audio_register("pcm1", dev);
  ASSERT(ret == OK);

  /* Initialize I2S Input device */

  dev = cxd56_aud_i2sin();
  ret = audio_register("pcm_in1", dev);
  ASSERT(ret == OK);

  return ret;
}
#endif /* CONFIG_AUDIO_CXD56 */
