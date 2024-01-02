/*****************************************************************************
 * arch/arm/src/cxd56xx/cxd5247.c
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
#include "hardware/cxd5247.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* For CXD5247 Control */

/* CXD5247 Control IDs for external fw_as_acacontrol */

#define CXD5247_CTL_CHECK_ID          0
#define CXD5247_CTL_POWER_ON_COMMON   1
#define CXD5247_CTL_POWER_ON_INPUT    2
#define CXD5247_CTL_POWER_ON_OUTPUT   3
#define CXD5247_CTL_SET_SERDES        4
#define CXD5247_CTL_SET_SMASTER       5
#define CXD5247_CTL_POWER_OFF_COMMON  6
#define CXD5247_CTL_POWER_OFF_INPUT   7
#define CXD5247_CTL_POWER_OFF_OUTPUT  8
#define CXD5247_CTL_POWER_ON_MICBIAS  9
#define CXD5247_CTL_INIT_AMIC         11
#define CXD5247_CTL_SET_OUTPUT_DEVICE 13

/* Parameters of CXD5247_CTL_SET_OUTPUT_DEVICE */

#define CXD5247_AOUT_OFF (0)
#define CXD5247_AOUT_ON  (1)

/* Audio Master Clock Frequency */

#define CXD56_XTAL_24_576MHZ  (0)
#define CXD56_XTAL_49_152MHZ  (1)

/* CXD5247 Audio Parameter definition */

/* Parameters of CXD5247_CTL_POWER_ON_COMMON */

#define CXD5247_OSC_24576KHZ        (1)
#define CXD5247_OSC_24576KHZ_HIRES  (2)
#define CXD5247_OSC_49152KHZ        (3)
#define CXD5247_OSC_49152KHZ_HIRES  (4)

/* Parameters of CXD5247_CTL_SET_SERDES */

#define CXD5247_CHMODE_8CH  (1)
#define CXD5247_CHMODE_4CH  (2)

#define CXD5247_FSMODE_128  (1)
#define CXD5247_FSMODE_64   (2)

/* Parameters of CXD5247_CTL_POWER_ON_INPUT and _INIT_AMIC */

#define CXD5247_MICBIAS_2_0V  (1)
#define CXD5247_MICBIAS_2_8V  (2)

/* Parameters of CXD5247_CTL_SET_SMASTER */

#define CXD5247_SMSTRFS_16  (1)
#define CXD5247_SMSTRFS_32  (2)

#define CXD5247_SMSTRMCKFS_512  (1)
#define CXD5247_SMSTRMCKFS_1024 (2)

#define CXD5247_SMSTRPWM_SINGLE     (1)
#define CXD5247_SMSTRPWM_BOTH       (2)
#define CXD5247_SMSTRPWM_SINGLEALT  (3)
#define CXD5247_SMSTRPWM_BOTHALT    (4)

#define CXD5247_SMSTRSWAP_NOSWAP    (1)
#define CXD5247_SMSTRSWAP_SWAPLR    (2)

/* Parameters of CXD5247_CTL_POWER_ON_OUTPUT */

#define CXD5247_TGTOUTDEV_HP    (1) /* Headphone output */
#define CXD5247_TGTOUTDEV_EP    (2) /* Ear Speaker output */
#define CXD5247_TGTOUTDEV_PWM   (3) /* PWM output */
#define CXD5247_TGTOUTDEV_HPPWM (4) /* Headphone and PWM output */
#define CXD5247_TGTOUTDEV_EPPWM (5) /* Ear Speaker and PWM output */
#define CXD5247_TGTOUTDEV_OFF   (6) /* Disable output */

#define CXD5247_PWMOUTSEL_NOSEL (1)
#define CXD5247_PWMOUTSEL_LN    (2)
#define CXD5247_PWMOUTSEL_LP    (3)
#define CXD5247_PWMOUTSEL_RN    (4)
#define CXD5247_PWMOUTSEL_RP    (5)

#define CXD5247_SPKDELAY_NO     (1)
#define CXD5247_SPKDELAY_SHORT  (2)
#define CXD5247_SPKDELAY_MIDDLE (3)
#define CXD5247_SPKDELAY_LONG   (4)

#define CXD5247_SPKLOOP_EN  (1)
#define CXD5247_SPKLOOP_DIS (2)

#define CXD5247_SPKDLYFREE_OFF  (1)
#define CXD5247_SPKDLYFREE_ON   (2)

#define CXD5247_SPSPLITON_1 (1) /* Short */
#define CXD5247_SPSPLITON_2 (2)
#define CXD5247_SPSPLITON_3 (3)
#define CXD5247_SPSPLITON_4 (4) /* Long */

#define CXD5247_SPDRV_4     (1)
#define CXD5247_SPDRV_2     (2)
#define CXD5247_SPDRV_1     (3)
#define CXD5247_SPDRV_LINE  (4)

#define IO_CURRENT_2MA  (2)

#define CONVERT_GAIN(g) ((CXD5247_MIC_GAIN_MAX + CXD5247_MIC_PGAGAIN_MAX) * \
                         (g) / 1000)

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

extern uint32_t fw_as_acacontrol(uint8_t type, uint32_t param);

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Parameters of CXD5247_CTL_POWER_ON_COMMON */

typedef struct
{
  uint8_t osc_mode;     /* CXD5247_OSC_xxxx */
  uint8_t mic_dev;      /* CXD5247_MIC_xxxx */
  uint8_t gpo_ds;       /* 1 ~ 4: 1 weakest */
  uint8_t ad_data_ds;   /* 1 ~ 4: 1 weakest */
  uint8_t dmic_clk_ds;  /* 1 ~ 4: 1 weakest */
  uint8_t mclk_ds;      /* 1 ~ 4: 1 weakest */
} cxd5247_auparam_poweron_t;

/* Parameters of CXD5247_CTL_SET_SERDES */

typedef struct
{
  uint8_t ch_mode;              /* CXD5247_CHMODE_xxxx */
  uint8_t fs_mode;              /* CXD5247_FSMODE_xxxx */
  uint8_t in[CXD56_MIC_MAXCH];  /* CXD5247_MICID_xxxx */
} cxd5247_auparam_serdes_t;

/* Parameters of CXD5247_CTL_POWER_ON_INPUT and _INIT_AMIC */

typedef struct
{
  uint8_t mic_dev;      /* CXD5247_MIC_xxxx */
  uint8_t mic_bias;     /* CXD5247_MICBIAS_xxxx */
  uint32_t mic_gain[CXD56_AMIC_MAXCH];
  uint32_t pga_gain[CXD56_AMIC_MAXCH];
  int32_t  vgain[CXD56_AMIC_MAXCH];
} cxd5247_auparam_input_t;

/* Parameters of CXD5247_CTL_SET_SMASTER */

typedef struct
{
  uint8_t mode;     /* CXD5247_SMSTRFS_xxxx */
  uint8_t mck_fs;   /* CXD5247_SMSTRMCKFS_xxxx */
  uint8_t pwm_mode; /* CXD5247_SMSTRPWM_xxxx */
  uint8_t swap_lr;  /* CXD5247_SMSTRSWAP_xxxx */
  uint8_t out2dly;
} cxd5247_auparam_smaster_t;

/* Parameters of CXD5247_CTL_POWER_ON_OUTPUT */

typedef struct
{
  uint8_t out_dev;      /* CXD5247_TGTOUTDEV_xxxx */
  uint8_t pwm_out[2];   /* Fixed 0 in both (CXD5247_PWMOUTSEL_xxxx) */
  uint8_t sp_delay;     /* Fixed 0 (CXD5247_SPKDELAY_xxxx) */
  uint8_t loop_mode;    /* Fixed 0 (CXD5247_SPKLOOP_xxxx) */
  uint8_t mode;         /* CXD5247_SMSTRFS_xxxx */
  uint8_t sp_dly_free;  /* Fixed 0 (CXD5247_SPKDLYFREE_xxxx) */
  uint8_t sp_spliton;   /* CXD5247_SPSPLITON_xxxx */
  uint8_t sp_drv;       /* CXD5247_SPDRV_xxxx */
} cxd5247_auparam_outpon_t;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/** name: convert_iomamps_idx() */

static int convert_iomamps_idx(int miliamps)
{
  switch (miliamps)
    {
      case 2:  miliamps = 1; break;
      case 4:  miliamps = 2; break;
      case 6:  miliamps = 3; break;
      case 8:  miliamps = 4; break;
      default: miliamps = 1; break;
    }

  return miliamps;
}

/** name: cxd5247_audio_outputon() */

static int cxd5247_audio_outputon(uint32_t samplerate, int spk_drv)
{
  cxd5247_auparam_outpon_t param;

  param.mode = (samplerate > 48000) ? CXD5247_SMSTRFS_16 :
                                      CXD5247_SMSTRFS_32;
  param.out_dev     = CXD5247_TGTOUTDEV_OFF;
  param.pwm_out[0]  = 0;
  param.pwm_out[1]  = 0;
  param.sp_delay    = 0;
  param.loop_mode   = 0;
  param.sp_dly_free = 0;
  param.sp_spliton  = CXD5247_SPSPLITON_1;
  param.sp_drv      = (spk_drv > 3) ? 3 : spk_drv;

  return fw_as_acacontrol(CXD5247_CTL_POWER_ON_OUTPUT, (uint32_t)&param);
}

/** name: cxd5247_audio_samplerate() */

static int cxd5247_audio_samplerate(uint32_t samplerate, bool is_mclk24m)
{
  cxd5247_auparam_smaster_t param;

  if (samplerate > 48000 && !is_mclk24m)
    {
      param.mode = CXD5247_SMSTRFS_32;
      param.mck_fs = CXD5247_SMSTRMCKFS_1024;
    }
  else if (samplerate <= 48000)
    {
      param.mode = CXD5247_SMSTRFS_16;
      param.mck_fs = CXD5247_SMSTRMCKFS_512;
    }
  else
    {
      /* CXD5247 Master clock must be 49.152MHz for HI Res Audio. */

      return -1;
    }

  param.swap_lr  = CXD5247_SMSTRSWAP_NOSWAP;
  param.out2dly  = 0;
  param.pwm_mode = CXD5247_SMSTRPWM_BOTH;

  return fw_as_acacontrol(CXD5247_CTL_SET_SMASTER, (uint32_t)&param);
}

/** name: cxd5247_audio_poweron() */

static int cxd5247_audio_poweron(uint32_t samplerate, int io_miliamps,
                                 bool is_mclk24m, uint8_t *chmaps)
{
  cxd5247_auparam_poweron_t param;

  if (samplerate <= 48000)
    {
      param.osc_mode = is_mclk24m ? CXD5247_OSC_24576KHZ :
                                    CXD5247_OSC_49152KHZ;
    }
  else
    {
      param.osc_mode = is_mclk24m ? CXD5247_OSC_24576KHZ_HIRES :
                                    CXD5247_OSC_49152KHZ_HIRES;
    }

  param.mic_dev     = cxd5247_get_micdev(chmaps);
  param.gpo_ds      = convert_iomamps_idx(io_miliamps);
  param.ad_data_ds  = convert_iomamps_idx(io_miliamps);
  param.dmic_clk_ds = convert_iomamps_idx(io_miliamps);
  param.mclk_ds     = convert_iomamps_idx(io_miliamps);

  return fw_as_acacontrol(CXD5247_CTL_POWER_ON_COMMON, (uint32_t)&param);
}

/** name: cxd5247_audio_inputon() */

static int cxd5247_audio_inputon(int gain, uint8_t mic_dev,
                                 bool hi_bias)
{
  int i;
  cxd5247_auparam_input_t param;
  uint32_t mic_gain;
  uint32_t pga_gain;

  mic_gain = (gain >= CXD5247_MIC_GAIN_MAX) ? CXD5247_MIC_GAIN_MAX :
                                              (gain / 30) * 30;
  pga_gain = gain - mic_gain;
  pga_gain = (pga_gain >= CXD5247_MIC_PGAGAIN_MAX) ?
              CXD5247_MIC_PGAGAIN_MAX : pga_gain;

  param.mic_dev  = mic_dev;
  param.mic_bias = hi_bias ? CXD5247_MICBIAS_2_8V :
                             CXD5247_MICBIAS_2_0V;

  for (i = 0; i < CXD56_AMIC_MAXCH; i++)
    {
      param.mic_gain[i] = mic_gain;
      param.pga_gain[i] = pga_gain;
      param.vgain[i]    = 0;
    }

  fw_as_acacontrol(CXD5247_CTL_INIT_AMIC, (uint32_t)&param);
  fw_as_acacontrol(CXD5247_CTL_POWER_ON_INPUT, (uint32_t)&param);

  return 0;
}

/** name: cxd5247_audio_micassign() */

static int cxd5247_audio_micassign(FAR uint8_t *chmaps)
{
  int i;
  cxd5247_auparam_serdes_t param;
  uint8_t mic_dev;

  mic_dev = cxd5247_get_micdev(chmaps);

  param.ch_mode = mic_dev == CXD5247_MIC_AMIC ? CXD5247_CHMODE_4CH :
                                                CXD5247_CHMODE_8CH;
  param.fs_mode = mic_dev == CXD5247_MIC_AMIC ? CXD5247_FSMODE_128 :
                                                CXD5247_FSMODE_64;

  for (i = 0; i < CXD56_MIC_MAXCH; i++)
    {
      param.in[i] = chmaps[i];
    }

  return fw_as_acacontrol(CXD5247_CTL_SET_SERDES, (uint32_t)&param);
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/* name: cxd5247_get_micdev()
 * Confirm Mic device type.
 */

uint8_t cxd5247_get_micdev(FAR uint8_t *chmaps)
{
  int i;
  uint8_t dev = 0;

  for (i = 0; i < CXD56_MIC_MAXCH; i++)
    {
      switch (chmaps[i])
        {
          /* Analog mic */

          case CXD5247_MICID_AMIC(0):
          case CXD5247_MICID_AMIC(1):
          case CXD5247_MICID_AMIC(2):
          case CXD5247_MICID_AMIC(3):
            dev |= 0x01;
            break;

          /* Digital mic */

          case CXD5247_MICID_DMIC(0):
          case CXD5247_MICID_DMIC(1):
          case CXD5247_MICID_DMIC(2):
          case CXD5247_MICID_DMIC(3):
          case CXD5247_MICID_DMIC(4):
          case CXD5247_MICID_DMIC(5):
          case CXD5247_MICID_DMIC(6):
          case CXD5247_MICID_DMIC(7):
            dev |= 0x02;
            break;
        }

      if (dev == 3)
        {
          return CXD5247_MIC_ANADIG;
        }
    }

  return dev == 1 ? CXD5247_MIC_AMIC : CXD5247_MIC_DMIC;
}

/* For CXD5247 Control */

/** name: cxd5247_audio_config()
 *  Configure CXD5247 setting.
 */

int cxd5247_audio_config(bool is_mclk24m, uint32_t samplerate,
                         uint8_t *chmaps, int mic_gain)
{
  int ret;
  mic_gain = CONVERT_GAIN(mic_gain);
  ret = cxd5247_audio_poweron(samplerate, IO_CURRENT_2MA, is_mclk24m, chmaps);
  ret += cxd5247_audio_outputon(samplerate, CXD5247_SPDRV_4);
  ret += cxd5247_audio_samplerate(samplerate, is_mclk24m);
  ret += cxd5247_audio_micassign(chmaps);
  ret += cxd5247_audio_inputon(mic_gain, cxd5247_get_micdev(chmaps), false);
  ret += fw_as_acacontrol(CXD5247_CTL_POWER_ON_MICBIAS, 0);
  return ret;
}

/** name: cxd5247_audio_micgain()
 *  Set Analog Mic gain.
 */

int cxd5247_audio_micgain(int mic_gain, uint8_t *chmaps)
{
  mic_gain = CONVERT_GAIN(mic_gain);
  return cxd5247_audio_inputon(mic_gain, cxd5247_get_micdev(chmaps), false);
}

/** name: cxd5247_audio_mute()
 *  Set CXD5247 Spk out mute control.
 */

int cxd5247_audio_mute(bool en)
{
  return fw_as_acacontrol(CXD5247_CTL_SET_OUTPUT_DEVICE,
                          en ? CXD5247_AOUT_OFF : CXD5247_AOUT_ON);
}

/** name: cxd5247_audio_poweroff()
 *  Internal Power OFF of CXD5247.
 */

int cxd5247_audio_poweroff(void)
{
  return fw_as_acacontrol(CXD5247_CTL_POWER_OFF_COMMON, 0);
}

/** name: cxd5247_audio_inputoff()
 *  CXD5247 Audio Input block power off.
 */

int cxd5247_audio_inputoff(void)
{
  return fw_as_acacontrol(CXD5247_CTL_POWER_OFF_INPUT, 0);
}

/** name: cxd5247_audio_outputoff()
 *  CXD5247 Audio Output block power off.
 */

int cxd5247_audio_outputoff(void)
{
  return fw_as_acacontrol(CXD5247_CTL_POWER_OFF_OUTPUT, 0);
}
