/*****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_audio_lower.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>
#include <arch/board/cxd56_power.h>
#include "arch/chip/cxd56_audio_lower.h"
#include "hardware/cxd5602_topreg.h"

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static uint8_t *get_board_micmaps(void);
static bool get_is_mclk24m(void);
static int get_audclocksrc(void);
static int get_audclockdiv(void);
static int ext_mute(bool en);
static int ext_micen(bool en);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static struct cxd56_audio_lower_s g_cxd56_audio_lower =
{
  get_board_micmaps,
  get_audclocksrc,
  get_audclockdiv,
  get_is_mclk24m,
  ext_mute,
  ext_micen
};

static const uint8_t g_mic_chmaps[8] =
{
  CONFIG_CXD5247_MICID_CH0,
  CONFIG_CXD5247_MICID_CH1,
  CONFIG_CXD5247_MICID_CH2,
  CONFIG_CXD5247_MICID_CH3,
  CONFIG_CXD5247_MICID_CH4,
  CONFIG_CXD5247_MICID_CH5,
  CONFIG_CXD5247_MICID_CH6,
  CONFIG_CXD5247_MICID_CH7
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

static uint8_t *get_board_micmaps(void)
{
  /* Return Channel maps of mics on CXD5247 on your board. */

  return (FAR uint8_t *)g_mic_chmaps;
}

static bool get_is_mclk24m(void)
{
  /* If your board uses 49.152MHz for audio MCLK on CXD5247,
   * return false, othewise return true.
   */

#ifdef CONFIG_CXD5247_OSC_FREQ_24MHZ
  return true;
#else
  return false;
#endif
}

static int get_audclocksrc(void)
{
  /* Select Clock source of MCLK in CXD5602.
   * Usualy it is external clock source as
   * 24.576MHz or 49.152MHz. So select AUD_MCLK_EXT.
   *
   * If your platform has no external clock, you can also use
   * internal clock source as audio MCLK.
   * In that case, the value shoule use AUD_MCLK_XOSC or AUD_MCLK_RCOSC.
   */

  return AUD_MCLK_EXT;
}

static int get_audclockdiv(void)
{
  /* If you have selected internal clock source for MCLK,
   * you also need to select clock divide value.
   * In case of AUD_MCLK_EXT, this value is no meaning and return 0.
   */

  return 0;
}

static int ext_mute(bool en)
{
  /* If your board has external circuit for audio out,
   * you can add some control here for enabling output audio.
   *
   * In spresense case, there is an amp and turn it on/off here.
   */

  return board_power_control(POWER_AUDIO_MUTE, en ? false : true);
}

static int ext_micen(bool en)
{
  /* If your board has external circuit for controlling mic input.
   * Do some operation here.
   *
   * In spresense case, no external circuit, so do nothing.
   */

  return 0;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

struct cxd56_audio_lower_s *cxd56_audio_lower(void)
{
  return &g_cxd56_audio_lower;
}
