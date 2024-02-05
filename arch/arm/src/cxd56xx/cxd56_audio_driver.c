/*****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_audio_driver.c
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
#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/kmalloc.h>

#include <debug.h>
#include <math.h>
#include <semaphore.h>

#include <arch/chip/audio.h>
#include <arch/board/cxd56_clock.h>
#include <arch/board/board.h>

#include "hardware/cxd5247.h"
#include "hardware/cxd5602_topreg.h"
#include "arch/chip/cxd56_audio_lower.h"

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_audio_reg.h"
#include "cxd56_audio_driver.h"

#include "cxd56_audio_dma.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Register parameters */

#define CXD56_AUDPUP_EN  (0)
#define CXD56_AUDPUP_DIS (1)

#define CXD56_AUDBLK_DIS (0)
#define CXD56_AUDBLK_EN  (1)

#define CXD56_AUDDECFS_4CH (0)
#define CXD56_AUDDECFS_8CH (1)

#define CXD56_AUDDSPMICSEL_MIC01  (0)
#define CXD56_AUDDSPMICSEL_MIC23  (1)
#define CXD56_AUDDSPMICSEL_MIC45  (2)
#define CXD56_AUDDSPMICSEL_MIC67  (3)

#define CXD56_AUDOUTSEL_MIC01 (0)
#define CXD56_AUDOUTSEL_MIC23 (1)
#define CXD56_AUDOUTSEL_MIC45 (2)
#define CXD56_AUDOUTSEL_MIC67 (3)
#define CXD56_AUDOUTSEL_DMA   (4)

#define CXD56_AUDSPKDATSEL_SRC0     (0)
#define CXD56_AUDSPKDATSEL_SRC1     (1)
#define CXD56_AUDSPKDATSEL_AUDOUT0  (2)
#define CXD56_AUDSPKDATSEL_AUDOUT1  (3)

#define CXD56_AUDRESOLITION_LOW   (0)
#define CXD56_AUDRESOLITION_HIGH  (1)

#define CXD56_I2SCLKS_EN  (0)
#define CXD56_I2SCLKS_DIS (1)

#define CXD56_AUDI2S_SLAVE  (0)
#define CXD56_AUDI2S_MASTER (1)

#define CXD56_AUDI2SFMT_ORG  (0)
#define CXD56_AUDI2SFMT_LEFT (1)

#define CXD56_AUDI2SINDATA_I2S0DMA (0)
#define CXD56_AUDI2SINDATA_I2S1DMA (1)
#define CXD56_AUDI2SINDATA_ADC     (2)
#define CXD56_AUDI2SINDATA_MIXED   (3)

#define CXD56_AUDI2SSRC_NOSRC  (0)
#define CXD56_AUDI2SSRC_UT48K  (1)
#define CXD56_AUDI2SSRC_UT96K  (2)
#define CXD56_AUDI2SSRC_UT192K (3)

#define CXD56_AUDRAMP_VOLCTL_OFF (0)
#define CXD56_AUDRAMP_VOLCTL_ON  (1)

#define CXD56_AUDCICCLK_64FS  (0)
#define CXD56_AUDCICCLK_128FS (1)
#define CXD56_AUDCICCLK_256FS (2)

#define CXD56_AUDCICDATAINV_NOINV    (0)
#define CXD56_AUDCICDATAINV_INVERSE  (1)

#define CXD56_AUDCIC_BOOST_OFF  (0)
#define CXD56_AUDCIC_BOOST_ON   (1)

#define CXD56_AUDCICMIC_DEV_CXD5247 (0)
#define CXD56_AUDCICMIC_DEV_DIGITAL (1)

#define CXD56_AUDCIC_GAINMODE_MATSU (0)
#define CXD56_AUDCIC_GAINMODE_ORG   (1)

#define CXD56_AUDCICIN_NOSWAP (0)
#define CXD56_AUDCICIN_SWAPLR (1)

#define CXD56_AUDCIC_HPFMODE_OFF  (0)
#define CXD56_AUDCIC_HPFMODE_LOW  (1)
#define CXD56_AUDCIC_HPFMODE_MID  (2)
#define CXD56_AUDCIC_HPFMODE_HI   (3)

#define CXD56_CICGAIN(x)  (0x4000 + (x))
#define CXD56_AUD_CICGAIN_MIN (0)
#define CXD56_AUD_CICGAIN_MAX (0x4000)

#define CXD56_MAXGAIN (1000)
#define CONVERT_CICGAIN(g)  \
  CXD56_CICGAIN(((CXD56_AUD_CICGAIN_MAX - CXD56_AUD_CICGAIN_MIN) / \
   CXD56_MAXGAIN) * g)

/* Volume Values 8'h18(d24):+12dB ~ 8'h34(d-204):-102dB */

#define CXD56_AUDVOLUME_LOG1001 (6.908755)
#define CXD56_AUDVOLUME_MAX (0)    /* 6.908755 */
#define CXD56_AUDVOLUME_MIN (-205) /* 0x33: Mute 0 */

#define CXD56_AUDVOL_CONVERT(v) \
          ((CXD56_AUDVOLUME_MIN + \
            (int)(((float)(CXD56_AUDVOLUME_MAX - CXD56_AUDVOLUME_MIN) / \
            CXD56_AUDVOLUME_LOG1001) * logf(v+1))) & 0xff)

#define CXD56_AUREG_I2STRANSMODE_NORM  (0)
#define CXD56_AUREG_I2STRANSMODE_LRDIF (1)

#define CXD56_AUDDESMODE_8CH  (0)
#define CXD56_AUDDESMODE_4CH  (1)

#define CXD56_AUDDECIM_INFS_4FS (0)
#define CXD56_AUDDECIM_INFS_8FS (1)

#define CXD56_AUDDECIM_OUTFS_1FS (0)
#define CXD56_AUDDECIM_OUTFS_2FS (1)
#define CXD56_AUDDECIM_OUTFS_4FS (2)
#define CXD56_AUDDECIM_OUTFS_8FS (3)

#define CXD56_AUDDECIMFS_4FS (1)
#define CXD56_AUDDECIMFS_8FS (2)

#define CXD56_AUDIO_DMA_START       (0x01)
#define CXD56_AUDIO_DMA_STOP_WOINTR (0x04)
#define CXD56_AUDIO_DMA_STOP        (0x00)

#define CXD56_DMABITW24 (0)
#define CXD56_DMABITW16 (1)

/* Dithering Speed setting parameters */

#define CXD56_VOLCTRLSTEP_1 (1) /* Fastest */
#define CXD56_VOLCTRLSTEP_2 (2)
#define CXD56_VOLCTRLSTEP_3 (3)
#define CXD56_VOLCTRLSTEP_4 (4)
#define CXD56_VOLCTRLSTEP_5 (5)
#define CXD56_VOLCTRLSTEP_6 (6)
#define CXD56_VOLCTRLSTEP_7 (7) /* Slowest */

#define CXD56_I2SDATARATE_48KHZ   (0)
#define CXD56_I2SDATARATE_192KHZ  (1)

/* CXD56_MICCHMAP_BITW16
 * Odd channels (L-ch side) assigned on MSB side in 32bit data
 * like big-endian. Data of lower channel number should store
 * lower address.
 * To fix this, swap channel assign odd and even channels.
 */

#define CXD56_MICCHMAP_BITW16 (0x10325476)
#define CXD56_MICCHMAP_BITW24 (0x01234567)

#define CXD56_NSDD_DITHER_VALUE (0x07FD5)

#define CXD56_SAMPRATE_48K  (48000)
#define CXD56_SAMPRATE_96K  (96000)
#define CXD56_SAMPRATE_192K (192000)
#define CXD56_SAMPRATE_INVALID (0xffffffff)
#define CXD56_DEFAULT_SYSTEMVOL (1000)

#define BEEP_FREQ_MAX        4085
#define BEEP_FREQ_MIN        94

/** NX Audio framework related ***********************************************/

/* Audio Multi settion configuration wrapper */

#ifdef CONFIG_AUDIO_MULTI_SESSION
#define MSESSION_ARG  , FAR void *session
#else
#define MSESSION_ARG
#endif

/*****************************************************************************
 * Private Types
 *****************************************************************************/

struct cxd56_system_s
{
  sem_t lock;
  int usr_cnt;
};

struct cxd56_ext_mutectl_s
{
  sem_t lock;
  int mute_cnt;
};

struct cxd56_hwresource_s
{
  int spkout_cnt;
  int i2susr_cnt;
  int outfs_cnt;

  uint16_t volume;
  bool is_slave;
  int samprate;
};

struct cxd56_aud_beep_s
{
  sem_t lock;
  bool is_inited;
  struct beep_ctl_s ctl;
};

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

extern void cxd56_audio_clock_enable(uint32_t clk, uint32_t div);
extern void cxd56_audio_clock_disable(void);
extern bool cxd56_audio_clock_is_enabled(void);

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

/* NX Audio I/F functions */

static int cxd56audio_setup(FAR struct audio_lowerhalf_s *lower, int cnt);
static int cxd56audio_getcaps(FAR struct audio_lowerhalf_s *lower,
                              int type, FAR struct audio_caps_s *caps);
static int cxd56audio_config(FAR struct audio_lowerhalf_s *lower
                             MSESSION_ARG,
                             FAR const struct audio_caps_s *caps);
static int cxd56audio_shutdown(FAR struct audio_lowerhalf_s *lower, int cnt);
static int cxd56audio_start(FAR struct audio_lowerhalf_s *lower
                            MSESSION_ARG);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int cxd56audio_stop(FAR struct audio_lowerhalf_s *lower
                           MSESSION_ARG);
#endif  /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int cxd56audio_pause(FAR struct audio_lowerhalf_s *lower
                            MSESSION_ARG);
static int cxd56audio_resume(FAR struct audio_lowerhalf_s *lower
                             MSESSION_ARG);
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int cxd56audio_enqbuff(FAR struct audio_lowerhalf_s *lower,
                              FAR struct ap_buffer_s *apb);
static int cxd56audio_cancelbuff(FAR struct audio_lowerhalf_s *lower,
                                 FAR struct ap_buffer_s *apb);
static int cxd56audio_ioctl(FAR struct audio_lowerhalf_s *lower, int cmd,
                            unsigned long arg);
static int cxd56audio_reserve(FAR struct audio_lowerhalf_s *lower
                              MSESSION_ARG);
static int cxd56audio_release(FAR struct audio_lowerhalf_s *lower
                              MSESSION_ARG);

#define CONFIG_CXD56_BEEEPDEV
#define CONFIG_CXD56_BEEPDEV_NAME "/dev/audio/beep"

#ifdef CONFIG_CXD56_BEEEPDEV
static int cxd56_beep_open(FAR struct file *filep);
static int cxd56_beep_close(FAR struct file *filep);
static ssize_t cxd56_beep_read(FAR struct file *filep,
                               FAR char *buffer, size_t buflen);
static ssize_t cxd56_beep_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int cxd56_beep_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#endif

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static FAR struct cxd56_audio_lower_s *g_lower = NULL;
static struct cxd56_system_s    g_system;
static struct cxd56_hwresource_s g_hwres;
static struct cxd56_ext_mutectl_s g_mutectl;
static int mic_samprate = CXD56_SAMPRATE_48K;
#ifdef CONFIG_CXD56_BEEEPDEV
static FAR struct cxd56_aud_beep_s *g_beep = NULL;
static const uint16_t g_beepfreqtable[] =
{
  120,  127,  134,  142,  151,  160,  169,  180,  190,  201,  214,  226,
  240,  254,  269,  285,  302,  320,  339,  360,  381,  403,  428,  453,
  480,  509,  539,  571,  606,  642,  681,  719,  762,  810,  857,  910,
  965,  1021, 1079, 1143, 1215, 1289, 1362, 1444, 1536, 1627, 1714, 1829,
  1939, 2043, 2182, 2313, 2400, 2560, 2704, 2866, 3048, 3200, 3429, 3623,
  3840, 4085,   94
};
#endif

/* NuttX Audio Operations */

static const struct audio_ops_s g_audioops =
{
  cxd56audio_setup,       /* setup          */
  cxd56audio_getcaps,     /* getcaps        */
  cxd56audio_config,      /* configure      */
  cxd56audio_shutdown,    /* shutdown       */
  cxd56audio_start,       /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  cxd56audio_stop,        /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  cxd56audio_pause,       /* pause          */
  cxd56audio_resume,      /* resume         */
#endif
  NULL,                   /* alloc_buffer   */
  NULL,                   /* free_buffer    */
  cxd56audio_enqbuff,     /* enqueue_buffer */
  cxd56audio_cancelbuff,  /* cancel_buffer  */
  cxd56audio_ioctl,       /* ioctl          */
  NULL,                   /* read           */
  NULL,                   /* write          */
  cxd56audio_reserve,     /* reserve        */
  cxd56audio_release      /* release        */
};

/* File Operations for BEEP device */

#ifdef CONFIG_CXD56_BEEEPDEV
static struct file_operations g_cxd56_beep_fops =
{
  cxd56_beep_open,
  cxd56_beep_close,
  cxd56_beep_read,
  cxd56_beep_write,
  NULL, /* seek */
  cxd56_beep_ioctl,
  NULL, /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL, /* unlink */
#endif
};
#endif

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/* CXD5602 Audio HW controls */

static void cxd56_i2s0_config(bool en, uint32_t insrc, uint32_t blf,
                              bool is_slave, bool high_drv)
{
  uint8_t mode = (is_slave ? 1 : 0) + (high_drv ? 2 : 0);

  if (en)
    {
      /* IO Pin configuration */

      switch (mode)
        {
          case 0: /* Master & Low Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_NORM);
            break;

          case 1: /* Slave & Low Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_NORM);
            break;

          case 2: /* Master & High Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_HIGH);
            break;

          case 3: /* Slave & High Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_HIGH);
            break;
        }

      cxd56_waureg(AUREG_SD1MASTER, is_slave ? CXD56_AUDI2S_SLAVE :
                                               CXD56_AUDI2S_MASTER);
      cxd56_waureg(AUREG_I2S0_FMT,        CXD56_AUDI2SFMT_ORG);
      cxd56_waureg(AUREG_I2S1_FILTBYPASS, CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_SDOUT1_EN,       CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_SDIN1_EN,        CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_M_SPCLKERR1,     CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_SRC1IN_SEL,      insrc);

      /* Sampling Rate Converter Frequency range setting */

      cxd56_waureg(AUREG_SRC1,            blf);
    }
  else
    {
      cxd56_waureg(AUREG_SD1MASTER,       CXD56_AUDI2S_MASTER);
      cxd56_waureg(AUREG_I2S0_FMT,        CXD56_AUDI2SFMT_ORG);
      cxd56_waureg(AUREG_I2S1_FILTBYPASS, CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_SDOUT1_EN,       CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_SDIN1_EN,        CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_M_SPCLKERR1,     CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_SRC1IN_SEL,      CXD56_AUDI2SINDATA_I2S0DMA);
      cxd56_waureg(AUREG_SRC1,            CXD56_AUDI2SSRC_NOSRC);
      CXD56_PIN_CONFIGS(PINCONFS_I2S0_GPIO);
    }
}

static void cxd56_i2s1_config(bool en, uint32_t insrc, uint32_t blf,
                              bool is_slave, bool high_drv)
{
  uint8_t mode = (is_slave ? 1 : 0) + (high_drv ? 2 : 0);

  if (en)
    {
      switch (mode)
        {
          case 0: /* Master & Low Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_NORM);
            break;

          case 1: /* Slave & Low Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_NORM);
            break;

          case 2: /* Master & High Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_HIGH);
            break;

          case 3: /* Slave & High Drive */
            CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_HIGH);
            break;
        }

      cxd56_waureg(AUREG_SD2MASTER, is_slave ? CXD56_AUDI2S_SLAVE :
                                               CXD56_AUDI2S_MASTER);
      cxd56_waureg(AUREG_I2S1_FMT,        CXD56_AUDI2SFMT_ORG);
      cxd56_waureg(AUREG_I2S2_FILTBYPASS, CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_SDOUT2_EN,       CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_SDIN2_EN,        CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_M_SPCLKERR2,     CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_SRC2IN_SEL,      insrc);

      /* Sampling Rate Converter Frequency range setting */

      cxd56_waureg(AUREG_SRC2,            blf);
    }
  else
    {
      cxd56_waureg(AUREG_SD2MASTER,       CXD56_AUDI2S_MASTER);
      cxd56_waureg(AUREG_I2S1_FMT,        CXD56_AUDI2SFMT_ORG);
      cxd56_waureg(AUREG_I2S2_FILTBYPASS, CXD56_AUDBLK_EN);
      cxd56_waureg(AUREG_SDOUT2_EN,       CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_SDIN2_EN,        CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_M_SPCLKERR2,     CXD56_AUDBLK_DIS);
      cxd56_waureg(AUREG_SRC2IN_SEL,      CXD56_AUDI2SINDATA_I2S1DMA);
      cxd56_waureg(AUREG_SRC2,            CXD56_AUDI2SSRC_NOSRC);
      CXD56_PIN_CONFIGS(PINCONFS_I2S1_GPIO);
    }
}

static void cxd5247_power_en(bool en)
{
  board_power_control(POWER_AUDIO_AVDD, en);
  board_power_control(POWER_AUDIO_DVDD, en);
}

static void cxd5247_reset(bool en)
{
  cxd56_gpio_write(CXD5247_XRST, en ? 0 : 1);
}

static void cxd56_cxd5247_pinconfig(bool en, bool high_drv)
{
  cxd56_gpio_config(CXD5247_XRST, false);
  cxd5247_reset(true);

  if (en)
    {
      CXD56_PIN_CONFIGS(PINCONFS_MCLK);
      if (high_drv)
        {
          CXD56_PIN_CONFIGS(PINCONFS_PDM_HIGH);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_PDM_NORM);
        }
    }
  else
    {
      CXD56_PIN_CONFIGS(PINCONFS_MCLK_GPIO);
      CXD56_PIN_CONFIGS(PINCONFS_PDM_GPIO);
    }
}

static void cxd56_audio_if_initialize(void)
{
  cxd56_cxd5247_pinconfig(true, false);

  cxd5247_reset(true);

  cxd5247_power_en(true);
  up_mdelay(10);

  cxd5247_reset(false);

  up_mdelay(10);

  cxd5247_audio_config(g_lower->is_mclk24m(), CXD56_SAMPRATE_48K,
                       g_lower->chmaps(), CXD5247_MIC_GAIN_MAX);
}

/* CXD5602 Internal Audio HW control functions */

static void cxd56_audioblk_reset(void)
{
  cxd56_waureg(AUREG_DSPRAM_CLR,  1);
  cxd56_waureg(AUREG_DSPRAM1_CLR, 1);
  cxd56_waureg(AUREG_DSPRAM2_CLR, 1);
  usleep(1);
  cxd56_waureg(AUREG_DSPRAM_CLR,  0);
  cxd56_waureg(AUREG_DSPRAM1_CLR, 0);
  cxd56_waureg(AUREG_DSPRAM2_CLR, 0);

  cxd56_waureg(AUREG_S_RST, 1);
  usleep(1);
  cxd56_waureg(AUREG_S_RST, 0);
}

static void cxd56_audio_block_poweren(bool en)
{
  uint32_t pdn = en ? CXD56_AUDPUP_EN : CXD56_AUDPUP_DIS;
  uint32_t blken = en ? CXD56_AUDBLK_EN : CXD56_AUDBLK_DIS;

  cxd56_waureg(AUREG_PDN_DSPC,    pdn);
  cxd56_waureg(AUREG_PDN_DSPB,    pdn);
  cxd56_waureg(AUREG_PDN_DSPS1,   pdn);
  cxd56_waureg(AUREG_PDN_DSPS2,   pdn);
  cxd56_waureg(AUREG_PDN_SMSTR,   pdn);
  cxd56_waureg(AUREG_PDN_AMIC1,   pdn);
  cxd56_waureg(AUREG_PDN_AMIC2,   pdn);
  cxd56_waureg(AUREG_PDN_AMICEXT, pdn);

  cxd56_waureg(AUREG_PDN_MIC,     pdn);
  cxd56_waureg(AUREG_PDN_LINEIN,  pdn);
  cxd56_waureg(AUREG_PDN_DAC,     pdn);

  cxd56_waureg(AUREG_AHBMIC_CLKEN,  blken);
  cxd56_waureg(AUREG_AHBI2S1_CLKEN, blken);
  cxd56_waureg(AUREG_AHBI2S2_CLKEN, blken);

  cxd56_waureg(AUREG_MCK_AHBMSTR_EN, blken);
  cxd56_waureg(AUREG_SDES_EN,        blken);
  cxd56_waureg(AUREG_DECIM1_EN,      blken);
  cxd56_waureg(AUREG_DECIM0_EN,      blken);

  /* Fixed Disable non use HW block */

  cxd56_waureg(AUREG_PDN_DMIC,    CXD56_AUDPUP_DIS);

  cxd56_waureg(AUREG_PDN_DNC2,    CXD56_AUDPUP_DIS);
  cxd56_waureg(AUREG_PDN_DNC1,    CXD56_AUDPUP_DIS);
  cxd56_waureg(AUREG_PDN_ANC,     CXD56_AUDPUP_DIS);
}

static void cxd56_audio_nouseblk_init(void)
{
  /* No use block setting */

  /* Auto volume controls
   *  ALC(AudoLevelControl)
   *  SPC(SoundPressurControl)
   *  ARC(AutoRangeControl)
   */

  cxd56_waureg(AUREG_ALC_EN,      CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_ALC_REC,     CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_ALCTARGET,   CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_ALC_KNEE,    CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_SPC_EN,      CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_SPC_LIMIT,   CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_ARC,         CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_INV_DIGIPOL, CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_ARC_VOL,     CXD56_AUDBLK_DIS);

  cxd56_waureg(AUREG_INV_DMIC1R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC1L, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC2R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC2L, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC3R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC3L, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC4R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_DMIC4L, CXD56_AUDCICDATAINV_NOINV);

  cxd56_waureg(AUREG_CS_SIGN,   0); /* Always 0 */
  cxd56_waureg(AUREG_CS_VOL,    0); /* Allways 7'b0 */

  cxd56_waureg(AUREG_INTMASK_SMASLOVF, CXD56_AUDBLK_EN);
  cxd56_waureg(AUREG_INTMASK_SMASROVF, CXD56_AUDBLK_EN);

  cxd56_waureg(AUREG_ADC2R_VOL, 0);  /* Used Codec block. Unknown usage */
  cxd56_waureg(AUREG_ADC2L_VOL, 0);  /* Used Codec block. Unknown usage */
  cxd56_waureg(AUREG_ADC1R_VOL, 0);  /* Used Codec block. Unknown usage */
  cxd56_waureg(AUREG_ADC1L_VOL, 0);  /* Used Codec block. Unknown usage */

  cxd56_waureg(AUREG_DNC1_START, 0);
  cxd56_waureg(AUREG_DNC1_MUTE,  CXD56_AUDBLK_EN);
  cxd56_waureg(AUREG_DNC1_START, 0);
  cxd56_waureg(AUREG_DNC1_MUTE,  CXD56_AUDBLK_EN);
  cxd56_waureg(AUREG_SMS_INTIM,  0);
}

static void cxd56_audio_beep_ctl(bool en, uint32_t freq, uint32_t vol)
{
  cxd56_waureg(AUREG_BEEP_ON,   en ? CXD56_AUDBLK_EN : CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_BEEP_FREQ, freq);
  cxd56_waureg(AUREG_BEEP_VOL, CXD56_AUDVOL_CONVERT(vol));
}

static void cxd56_audio_fixedregval_init(void)
{
  cxd56_waureg(AUREG_TRANS_MODE, CXD56_AUREG_I2STRANSMODE_NORM);
  cxd56_waureg(AUREG_DEQ_EN, CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_LR_SWAP2, CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_LR_SWAP1, CXD56_AUDBLK_DIS);

  /* CIC Filter setting */

  cxd56_waureg(AUREG_CIC1_GAIN_MODE, CXD56_AUDCIC_GAINMODE_ORG);
  cxd56_waureg(AUREG_CIC2_GAIN_MODE, CXD56_AUDCIC_GAINMODE_ORG);

  /* CIC Input device select */

  cxd56_waureg(AUREG_CIC1IN_SEL, CXD56_AUDCICMIC_DEV_CXD5247);
  cxd56_waureg(AUREG_CIC2IN_SEL, CXD56_AUDCICMIC_DEV_CXD5247);
  cxd56_waureg(AUREG_CIC3IN_SEL, CXD56_AUDCICMIC_DEV_CXD5247);
  cxd56_waureg(AUREG_CIC4IN_SEL, CXD56_AUDCICMIC_DEV_CXD5247);

  /* No Inverse signal */

  cxd56_waureg(AUREG_INV_AMIC1R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC1L, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC2R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC2L, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC3R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC3L, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC4R, CXD56_AUDCICDATAINV_NOINV);
  cxd56_waureg(AUREG_INV_AMIC4L, CXD56_AUDCICDATAINV_NOINV);

  /* No LR Swap in CIC filter block */

  cxd56_waureg(AUREG_CIC1IN_SWAP, CXD56_AUDCICIN_NOSWAP);
  cxd56_waureg(AUREG_CIC2IN_SWAP, CXD56_AUDCICIN_NOSWAP);
  cxd56_waureg(AUREG_CIC3IN_SWAP, CXD56_AUDCICIN_NOSWAP);
  cxd56_waureg(AUREG_CIC4IN_SWAP, CXD56_AUDCICIN_NOSWAP);

  /* BLF is always on */

  cxd56_waureg(AUREG_BLF_EN, CXD56_AUDBLK_EN);

  /* Mic input is connected to CODEC directoly but not used this path.
   * So fixed the mic input selector.
   */

  cxd56_waureg(AUREG_COD_INSEL1, CXD56_AUDDSPMICSEL_MIC01);

  /* LINEIN and SDOUT path is not used. Both volumes should be 0. */

  cxd56_waureg(AUREG_SDOUT_VOL,  CXD56_AUDVOL_CONVERT(0));
  cxd56_waureg(AUREG_LINEIN_VOL, CXD56_AUDVOL_CONVERT(0));

  /* Fade in/out Volume control registers */

  cxd56_waureg(AUREG_DIGSFT, CXD56_AUDRAMP_VOLCTL_ON);
  cxd56_waureg(AUREG_DSR_RATE, CXD56_VOLCTRLSTEP_1);

  cxd56_waureg(AUREG_MUTE_B, CXD56_AUDBLK_DIS);
  cxd56_waureg(AUREG_FS_CLK_EN,  CXD56_AUDBLK_EN);
  cxd56_waureg(AUREG_PDM_OUT_EN, CXD56_AUDBLK_EN);

  cxd56_waureg(AUREG_NSDD, CXD56_NSDD_DITHER_VALUE);
  cxd56_waureg(AUREG_NSX2, 0);
  cxd56_waureg(AUREG_NSPMUTE, CXD56_AUDBLK_DIS);
}

static void cxd56_audio_set_miccicgain(int gain)
{
  uint8_t *chmaps = g_lower->chmaps();

  gain = (gain >= 1000) ? CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX) :
                          CONVERT_CICGAIN(gain);

  cxd56_waureg(AUREG_CIC1_LGAIN,
               chmaps[0] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC1_RGAIN,
               chmaps[1] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC2_LGAIN,
               chmaps[2] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC2_RGAIN,
               chmaps[3] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC3_LGAIN,
               chmaps[4] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC3_RGAIN,
               chmaps[5] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC4_LGAIN,
               chmaps[6] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
  cxd56_waureg(AUREG_CIC4_RGAIN,
               chmaps[7] > CXD5247_MICID_AMIC(3) ? gain :
               CXD56_CICGAIN(CXD56_AUD_CICGAIN_MAX));
}

static void cxd56_audio_set_micboost(bool en)
{
  uint32_t boost = en ? CXD56_AUDCIC_BOOST_ON : CXD56_AUDCIC_BOOST_OFF;
  cxd56_waureg(AUREG_ADC1_BOOST, boost);
  cxd56_waureg(AUREG_ADC2_BOOST, boost);
  cxd56_waureg(AUREG_ADC3_BOOST, boost);
  cxd56_waureg(AUREG_ADC4_BOOST, boost);
}

static void cxd56_audio_set_micfilter_strangth(uint32_t val)
{
  cxd56_waureg(AUREG_HPF1_MODE, val);
  cxd56_waureg(AUREG_HPF2_MODE, val);
  cxd56_waureg(AUREG_HPF3_MODE, val);
  cxd56_waureg(AUREG_HPF4_MODE, val);
}

static void cxd56_audio_set_mic_en(bool en)
{
  uint32_t mic_en = en ? CXD56_AUDBLK_EN : CXD56_AUDBLK_DIS;

  cxd56_waureg(AUREG_OUTEN_MIC1R_A, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC1L_A, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC2R_A, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC2L_A, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC1R_B, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC1L_B, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC2R_B, mic_en);
  cxd56_waureg(AUREG_OUTEN_MIC2L_B, mic_en);
}

static void cxd56_audio_spkin1_select(uint32_t sel)
{
  cxd56_waureg(AUREG_COD_INSEL2, sel);
}

static void cxd56_audio_spkin2_select(uint32_t sel)
{
  cxd56_waureg(AUREG_COD_INSEL3, sel);
}

static void cxd56_audio_dma1out_select(uint32_t sel)
{
  cxd56_waureg(AUREG_AU_DAT_SEL1, sel);
}

static void cxd56_audio_dma2out_select(uint32_t sel)
{
  cxd56_waureg(AUREG_AU_DAT_SEL2, sel);
}

static void cxd56_i2sclock_out(bool en)
{
  cxd56_waureg(AUREG_SDCK_OUTENX, en ? CXD56_AUDBLK_EN :
                                       CXD56_AUDBLK_DIS);
}

static void cxd56_audio_outresolution(uint32_t res)
{
  cxd56_waureg(AUREG_HI_RES_MODE, res);
  cxd56_waureg(AUREG_I2SALL_DATARATE, res == CXD56_AUDRESOLITION_LOW ?
                                      CXD56_I2SDATARATE_48KHZ :
                                      CXD56_I2SDATARATE_192KHZ);
}

static void cxd56_audio_micrate(uint32_t rate)
{
  /* CXD56_AUDDECIM_OUTFS_1FS for 48k
   * CXD56_AUDDECIM_OUTFS_2FS for 96k
   * CXD56_AUDDECIM_OUTFS_4FS for 192k
   */

  cxd56_waureg(AUREG_SEL_DECIM, CXD56_AUDBLK_EN);
  cxd56_waureg(AUREG_SEL_OUTF,  rate);
}

static void cxd56_audio_set_pdm(bool is_mclk24m, FAR uint8_t *chmap)
{
  if (!is_mclk24m && cxd5247_get_micdev(chmap) == CXD5247_MIC_AMIC)
    {
      /* CXD5247 supports Hi Res Audio with only Analog MIC and 49MHz
       * master clock.
       * In this case, PDM Clock is configured 128FS,
       * 4Ch(Max of Analog MIC chs) and 8FS data after de-serialized.
       */

      cxd56_waureg(AUREG_FS_FS,    CXD56_AUDDECFS_4CH);
      cxd56_waureg(AUREG_ADC_FS,   CXD56_AUDCICCLK_128FS);
      cxd56_waureg(AUREG_SEL_INF,  CXD56_AUDDECIM_INFS_8FS);
      cxd56_waureg(AUREG_SER_MODE, CXD56_AUDDESMODE_4CH);
      cxd56_waureg(AUREG_DCMFS12,  CXD56_AUDDECIMFS_8FS);
      cxd56_waureg(AUREG_DCMFS34,  CXD56_AUDDECIMFS_8FS);
    }
  else
    {
      cxd56_waureg(AUREG_FS_FS,    CXD56_AUDDECFS_8CH);
      cxd56_waureg(AUREG_ADC_FS,   CXD56_AUDCICCLK_64FS);
      cxd56_waureg(AUREG_SEL_INF,  CXD56_AUDDECIM_INFS_4FS);
      cxd56_waureg(AUREG_SER_MODE, CXD56_AUDDESMODE_8CH);
      cxd56_waureg(AUREG_DCMFS12,  CXD56_AUDDECIMFS_4FS);
      cxd56_waureg(AUREG_DCMFS34,  CXD56_AUDDECIMFS_4FS);
    }
}

static void cxd56_set_spkin0_vol(uint32_t vol)
{
  vol = (vol > 1000) ? 1000 : vol;
  cxd56_waureg(AUREG_SDIN1_VOL, CXD56_AUDVOL_CONVERT(vol));
}

static void cxd56_set_spkin1_vol(uint32_t vol)
{
  vol = (vol > 1000) ? 1000 : vol;
  cxd56_waureg(AUREG_SDIN2_VOL, CXD56_AUDVOL_CONVERT(vol));
}

static void cxd56_set_linin_vol(uint32_t vol)
{
  vol = (vol > 1000) ? 1000 : vol;
  cxd56_waureg(AUREG_LINEIN_VOL, CXD56_AUDVOL_CONVERT(vol));
}

static void cxd56_set_spkout_vol(uint32_t vol)
{
  vol = (vol > 1000) ? 1000 : vol;
  cxd56_waureg(AUREG_DAC_VOL, CXD56_AUDVOL_CONVERT(vol));
}

static void cxd56_set_novol(uint32_t vol)
{
  /* Do nothing */
}

static int external_mute_control(bool en)
{
  int ret = 0;

  if (en)
    {
      /* Mute ON */

      ret = g_lower->ext_mute(true);
      usleep(150000);
      cxd5247_audio_mute(true);
    }
  else
    {
      /* Mute OFF */

      cxd5247_audio_mute(false);
      usleep(150000);
      ret = g_lower->ext_mute(false);
    }

  return ret;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/** test functions **/

static int cxd56_audio_initialize(void)
{
  cxd56_audio_if_initialize();
  cxd56_audio_clock_enable(g_lower->clksrc(), g_lower->clkdiv());
  external_mute_control(true);

  /* Global setting */

  cxd56_audio_block_poweren(true);

  /* Reset Assert */

  cxd56_audioblk_reset();

  /* System Setting */

  cxd56_audio_nouseblk_init();
  cxd56_audio_fixedregval_init();

  /* CXD5247 I/F setting */

  cxd56_audio_set_pdm(g_lower->is_mclk24m(), g_lower->chmaps());

  /* MIC Out */

  cxd56_audio_micrate(CXD56_AUDDECIM_OUTFS_1FS);

  cxd56_audio_set_miccicgain(0);
  cxd56_audio_set_micboost(false);
  cxd56_audio_set_micfilter_strangth(CXD56_AUDCIC_HPFMODE_LOW);
  cxd56_audio_set_mic_en(true);

  /* Output Volume Control */

  cxd56_set_spkin0_vol(0);
  cxd56_set_spkin1_vol(0);
  cxd56_set_spkout_vol(0);
  cxd56_set_linin_vol(0);

  /* I2S Setting */

  cxd56_i2sclock_out(false);

  cxd56_i2s0_config(false, CXD56_AUDI2SINDATA_I2S0DMA,
                    CXD56_AUDI2SSRC_UT48K, false, false);
  cxd56_i2s1_config(false, CXD56_AUDI2SINDATA_I2S1DMA,
                    CXD56_AUDI2SSRC_UT48K, false, false);

  /* Output from CXD5602 resolution */

  cxd56_audio_outresolution(CXD56_AUDRESOLITION_LOW);

  /* Data path selection */

  cxd56_audio_spkin1_select(CXD56_AUDSPKDATSEL_AUDOUT0);
  cxd56_audio_spkin2_select(CXD56_AUDSPKDATSEL_AUDOUT1);

  cxd56_audio_dma1out_select(CXD56_AUDOUTSEL_DMA);
  cxd56_audio_dma2out_select(CXD56_AUDOUTSEL_DMA);

  /* Beep Signal Setting */

  cxd56_audio_beep_ctl(false, 0, 0);

  return 0;
}

static int cxd56_audio_deinitialize(void)
{
  g_mutectl.mute_cnt = 0;
  g_hwres.spkout_cnt = 0;
  g_hwres.i2susr_cnt = 0;
  g_hwres.outfs_cnt = 0;
  g_hwres.volume = 1000;
  g_hwres.is_slave = false;
  g_hwres.samprate = CXD56_SAMPRATE_48K;

  cxd56_audio_block_poweren(false);
  external_mute_control(true);
  cxd56_audio_clock_disable();

  cxd5247_audio_poweroff();
  cxd5247_reset(true);
  cxd5247_power_en(false);
  cxd56_cxd5247_pinconfig(false, false);

  return OK;
}

/* System resource control functions *****************************************/

/* Control system power */

static void cxd56_system_power_on(FAR struct cxd56_dmachannel_s *dmach)
{
  nxsem_wait_uninterruptible(&g_system.lock);

  if (!dmach->sysinited)
    {
      g_system.usr_cnt++;
      if (g_system.usr_cnt == 1)
        {
          cxd56_audio_initialize();
        }

      dmach->sysinited = true;
    }

  nxsem_post(&g_system.lock);
}

static void cxd56_system_power_off(FAR struct cxd56_dmachannel_s *dmach)
{
  nxsem_wait_uninterruptible(&g_system.lock);

  if (dmach->sysinited)
    {
      g_system.usr_cnt--;
      if (g_system.usr_cnt == 0)
        {
          cxd56_audio_deinitialize();
        }

      dmach->sysinited = false;
    }

  nxsem_post(&g_system.lock);
}

/* Control terminal volumme */

static void cxd56_update_systemvol(uint16_t volume)
{
  irqstate_t flags;

  flags = enter_critical_section();
  g_hwres.volume = volume;
  cxd56_set_spkout_vol(g_hwres.volume);
  leave_critical_section(flags);
}

static void cxd56_use_mute_off(bool use)
{
  if (use)
    {
      nxsem_wait_uninterruptible(&g_mutectl.lock);

      g_mutectl.mute_cnt++;
      if (g_mutectl.mute_cnt == 1)
        {
          external_mute_control(false);
        }

      nxsem_post(&g_mutectl.lock);
    }
  else
    {
      nxsem_wait_uninterruptible(&g_mutectl.lock);

      g_mutectl.mute_cnt--;
      if (g_mutectl.mute_cnt == 0)
        {
          external_mute_control(true);
        }

      nxsem_post(&g_mutectl.lock);
    }
}

static int cxd56_use_spkout_nolock(bool use)
{
  int ret;

  if (use)
    {
      ret = g_hwres.spkout_cnt++;
      if (ret == 0)
        {
          cxd56_set_spkout_vol(g_hwres.volume);
        }
    }
  else
    {
      ret = g_hwres.spkout_cnt--;
      if (ret <= 1)
        {
          cxd56_set_spkout_vol(0);
        }
    }

  return ret;
}

static int cxd56_use_spkout(bool use)
{
  int ret;
  irqstate_t flags;

  flags = enter_critical_section();
  ret = cxd56_use_spkout_nolock(use);
  leave_critical_section(flags);

  return ret;
}

/* Management System sampling frequency except mic input */

static int cxd56_set_audoutfs(int samprate)
{
  int ret;
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_hwres.outfs_cnt == 0)
    {
      samprate = samprate >= CXD56_SAMPRATE_192K ? CXD56_SAMPRATE_192K :
                                                   CXD56_SAMPRATE_48K;
      g_hwres.samprate = samprate;
    }

  ret = g_hwres.samprate;

  leave_critical_section(flags);

  return ret;
}

static int cxd56_use_audoutfs_nolock(bool use)
{
  int samprate = 0;

  if (use)
    {
      g_hwres.outfs_cnt++;
      if (g_hwres.outfs_cnt == 1)
        {
          cxd56_audio_outresolution(g_hwres.samprate > CXD56_SAMPRATE_48K ?
                                    CXD56_AUDRESOLITION_HIGH :
                                    CXD56_AUDRESOLITION_LOW);
          samprate = g_hwres.samprate;
        }
    }
  else
    {
      g_hwres.outfs_cnt--;
    }

  return samprate;
}

static int cxd56_use_audoutfs(bool use)
{
  int samprate = 0;
  irqstate_t flags;

  flags = enter_critical_section();
  samprate = cxd56_use_audoutfs_nolock(use);
  leave_critical_section(flags);

  return samprate;
}

/* Management I2S Hardware */

static bool cxd56_set_i2sslave(bool slave)
{
  bool ret = false;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Only when I2S user does not exist,
   * Master or Slave mode can be changed.
   */

  if (g_hwres.i2susr_cnt == 0)
    {
      g_hwres.is_slave = slave;
      ret = true;
    }

  leave_critical_section(flags);

  return ret;
}

static bool cxd56_use_i2sdevice_nolock(bool use)
{
  bool ret = false;
  int samprate;

  if (use)
    {
      samprate = cxd56_use_audoutfs_nolock(true);
      g_hwres.i2susr_cnt++;
      if (g_hwres.i2susr_cnt == 1)
        {
          ret = true;
          cxd56_i2s1_config(true, CXD56_AUDI2SINDATA_I2S1DMA,
                            samprate >= CXD56_SAMPRATE_192K ?
                                        CXD56_AUDI2SSRC_UT192K :
                            samprate >= CXD56_SAMPRATE_96K ?
                                        CXD56_AUDI2SSRC_UT96K :
                                        CXD56_AUDI2SSRC_UT48K,
                            g_hwres.is_slave, false);
          cxd56_i2sclock_out(g_hwres.is_slave ? false : true);
        }
    }
  else
    {
      cxd56_use_audoutfs_nolock(false);
      g_hwres.i2susr_cnt--;
      if (g_hwres.i2susr_cnt == 0)
        {
          cxd56_i2s1_config(false, CXD56_AUDI2SINDATA_I2S1DMA,
                            CXD56_AUDI2SSRC_UT48K,
                            g_hwres.is_slave, false);
          cxd56_i2sclock_out(false);
        }
    }

  return ret;
}

static bool cxd56_use_i2sdevice(bool use)
{
  bool ret;
  irqstate_t flags;
  flags = enter_critical_section();
  ret = cxd56_use_i2sdevice_nolock(use);
  leave_critical_section(flags);
  return ret;
}

/* Utility functions for configuration ***************************************/

static void cxd56_aud_setvolgain(FAR struct cxd56_dmachannel_s *dmach,
                                 uint16_t vol)
{
  if (AUDSTATE_IS_STARTED(dmach))
    {
      dmach->setvolume(vol);
    }
}

static int cxd56_aud_setconfig(FAR struct cxd56_dmachannel_s *dmach,
                               uint32_t samprate, int chs, uint16_t bitw)
{
  int ret = -EINVAL;

  if (!AUDSTATE_IS_INITIAL(dmach) && !AUDSTATE_IS_CONFIGURED(dmach))
    {
      return -EBUSY;
    }

  if ((bitw != 32 && bitw != 16) || (chs <= 0 || chs > dmach->maxch(dmach)))
    {
      AUDSTATECHG_INITIAL(dmach);
      return ret;
    }

  if (IS_AUDIO_MICDEV(dmach))
    {
      if (samprate == CXD56_SAMPRATE_48K || samprate == CXD56_SAMPRATE_192K ||
          samprate == CXD56_SAMPRATE_96K)
        {
          ret = OK;
          mic_samprate = samprate;
          dmach->bitwidth = bitw;
          dmach->channels = chs;
          AUDSTATECHG_CONFIGURED(dmach);
        }
      else
        {
          AUDSTATECHG_INITIAL(dmach);
        }
    }
  else
    {
      if (samprate == CXD56_SAMPRATE_48K || samprate == CXD56_SAMPRATE_192K)
        {
          if (samprate != cxd56_set_audoutfs(samprate))
            {
              ret = -EBUSY;
            }
          else
            {
              ret = OK;
              dmach->bitwidth = bitw;
              dmach->channels = chs;
              AUDSTATECHG_CONFIGURED(dmach);
            }
        }
      else
        {
          AUDSTATECHG_INITIAL(dmach);
        }
    }

  return ret;
}

/* Inherited methods *********************************************************/

/* For Mic DMA instance */

static void cxd56_aud_micgain(uint32_t vol)
{
  cxd56_audio_set_miccicgain(vol);
  cxd5247_audio_micgain(vol, g_lower->chmaps());
}

static uint8_t cxd56_aud_micmaxch(FAR struct cxd56_dmachannel_s *dmach)
{
  int i;
  uint8_t ret = 0;
  uint8_t *chs = g_lower->chmaps();

  for (i = 0; i < 8; i++)
    {
      if (chs[i] != CXD5247_MICID_NOTUSED)
        {
          ret++;
        }
    }

  return ret;
}

static int cxd56_aud_michwres(FAR struct cxd56_dmachannel_s *dmach,
                                 bool en)
{
  int ret = -EAGAIN;

  if (en)
    {
      switch (mic_samprate)
        {
          case CXD56_SAMPRATE_192K:
            cxd56_audio_micrate(CXD56_AUDDECIM_OUTFS_4FS);
            break;
          case CXD56_SAMPRATE_96K:
            cxd56_audio_micrate(CXD56_AUDDECIM_OUTFS_2FS);
            break;
          case CXD56_SAMPRATE_48K:
            cxd56_audio_micrate(CXD56_AUDDECIM_OUTFS_1FS);
            break;
        }

      cxd56_audio_set_micboost(false);
      cxd56_audio_set_micfilter_strangth(CXD56_AUDCIC_HPFMODE_LOW);
      cxd56_audio_set_mic_en(true);

      dmach->setvolume(dmach->volgain);

      ret = OK;
    }
  else
    {
      cxd56_audio_set_mic_en(false);
      ret = OK;
    }

  return ret;
}

/* For I2S DMA */

static uint8_t cxd56_aud_generic_maxch(FAR struct cxd56_dmachannel_s *dmach)
{
  return 2;
}

static int cxd56_aud_i2s0outhwres(FAR struct cxd56_dmachannel_s *dmach,
                                     bool en)
{
  if (en)
    {
      /* I2S Output DMA is fixed to speaker.
       * So I2S0 HW block is always disabled
       */

      cxd56_i2s0_config(false, CXD56_AUDI2SINDATA_I2S0DMA,
                        CXD56_AUDI2SSRC_UT48K, false, false);

      cxd56_set_spkin0_vol(dmach->volgain);
      cxd56_use_audoutfs(true);
      cxd56_use_spkout(true);
    }
  else
    {
      cxd56_use_audoutfs(false);
      cxd56_use_spkout(false);
    }

  return OK;
}

static int cxd56_aud_i2s0outhwres_irq(FAR struct cxd56_dmachannel_s *dmach,
                                         bool en)
{
  cxd56_use_audoutfs_nolock(en);
  cxd56_use_spkout_nolock(en);
  return OK;
}

static int cxd56_aud_i2s1outhwres(FAR struct cxd56_dmachannel_s *inst,
                                     bool en)
{
  if (inst->distination == AUDDISTINATION_I2S)
    {
      cxd56_use_i2sdevice(en);
    }
  else
    {
      cxd56_set_spkin1_vol(en ? inst->volgain : 0);
      cxd56_use_audoutfs(en);
      cxd56_use_spkout(en);
    }

  return OK;
}

static int cxd56_aud_i2s1outhwres_irq(FAR struct cxd56_dmachannel_s *inst,
                                         bool en)
{
  if (inst->distination == AUDDISTINATION_I2S)
    {
      cxd56_use_i2sdevice_nolock(en);
    }
  else
    {
      cxd56_set_spkin1_vol(en ? inst->volgain : 0);
      cxd56_use_audoutfs_nolock(en);
      cxd56_use_spkout_nolock(en);
    }

  return OK;
}

static int cxd56_aud_i2s1inhwres(FAR struct cxd56_dmachannel_s *inst,
                                    bool en)
{
  cxd56_use_i2sdevice(en);
  return OK;
}

static int cxd56_aud_i2s1inhwres_irq(FAR struct cxd56_dmachannel_s *inst,
                                        bool en)
{
  cxd56_use_i2sdevice_nolock(en);
  return OK;
}

/* NuttX Audio operations implementations ************************************/

static int cxd56audio_setup(FAR struct audio_lowerhalf_s *lower, int cnt)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  if (cnt == 1)
    {
      cxd56_system_power_on(dmach);
      AUDSTATECHG_INITIAL(dmach);

      if (IS_AUDIO_OUTPUT(dmach))
        {
          cxd56_use_mute_off(true);
        }
    }

  return OK;
}

static int cxd56audio_getcaps(FAR struct audio_lowerhalf_s *lower,
                              int type, FAR struct audio_caps_s *caps)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  caps->ac_format.hw = 0;
  caps->ac_controls.w = 0;

  if (AUDSTATE_IS_SHUTDOWN(dmach))
    {
      return -EAGAIN;
    }

  switch (type)
    {
      case AUDIO_TYPE_QUERY:
        caps->ac_channels = dmach->maxch(dmach);
        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              caps->ac_controls.b[0] = dmach->caps;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      case AUDIO_TYPE_OUTPUT:
        if (IS_AUDIO_OUTPUT(dmach))
          {
            caps->ac_channels = dmach->maxch(dmach);
            switch (caps->ac_subtype)
              {
                case AUDIO_TYPE_QUERY:
                  caps->ac_controls.hw[0] = dmach->supfs;
                  break;
                default:
                  break;
              }
          }

        break;

      case AUDIO_TYPE_INPUT:
        if (IS_AUDIO_INPUT(dmach))
          {
            caps->ac_channels = dmach->maxch(dmach);
            switch (caps->ac_subtype)
              {
                case AUDIO_TYPE_QUERY:
                  caps->ac_controls.hw[0] = dmach->supfs;
                  break;
                default:
                  break;
              }
          }

        break;

      case AUDIO_TYPE_FEATURE:
        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            caps->ac_controls.hw[0] = dmach->fcaps;
          }

        break;

      default:
        caps->ac_subtype = 0;
        caps->ac_channels = 0;
        break;
    }

  return caps->ac_len;
}

static int cxd56audio_config(FAR struct audio_lowerhalf_s *lower
                             MSESSION_ARG,
                             FAR const struct audio_caps_s *caps)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;
  int samprate;
  int bitw;
  int chs;
  int ret = OK;

  if (AUDSTATE_IS_SHUTDOWN(dmach))
    {
      return -EAGAIN;
    }

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_FEATURE:
        switch (caps->ac_format.hw)
          {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
            case AUDIO_FU_VOLUME:
              if (IS_AUDIO_OUTPUT(dmach))
                {
                  dmach->volgain = caps->ac_controls.hw[0];
                  dmach->volgain = (dmach->volgain > 1000) ? 1000 :
                                                         dmach->volgain;
                  if (dmach->distination == AUDDISTINATION_SPK)
                    {
                      cxd56_aud_setvolgain(dmach, dmach->volgain);
                    }
                }
              else
                {
                  auderr("ERROR: No volume control. Device is not aout\n");
                  ret = -EINVAL;
                }

              break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
            case AUDIO_FU_MUTE:
              if (IS_AUDIO_OUTPUT(dmach))
                {
                  bool mute = (bool) caps->ac_controls.hw[0];
                  audinfo("    Mute: %d\n", mute);

                  if (mute)
                    {
                      cxd56_aud_setvolgain(dmach, 0);
                    }
                  else
                    {
                      cxd56_aud_setvolgain(dmach, dmach->volgain);
                    }
                }
              else
                {
                  auderr("ERROR: No Mute control. Device is input\n");
                  ret = -EINVAL;
                }

              break;
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */

            case AUDIO_FU_INP_GAIN:
              if (IS_AUDIO_INPUT(dmach))
                {
                  dmach->volgain = caps->ac_controls.hw[0];
                  dmach->volgain = (dmach->volgain > 1000) ? 1000 :
                                                         dmach->volgain;
                  cxd56_aud_setvolgain(dmach, dmach->volgain);
                }
              else
                {
                  auderr("ERROR: No Gain control. Device is output\n");
                  ret = -EINVAL;
                }

              break;

            default:
              auderr("ERROR: Unknown feature unit: %d\n", caps->ac_format.hw);
              ret = -ENOTTY;
          }

        break;

      case AUDIO_TYPE_OUTPUT:
        if (!IS_AUDIO_OUTPUT(dmach))
          {
            auderr("ERROR: Can't configure, device is output\n");
            ret = -EINVAL;
          }
        else
          {
            samprate = (uint32_t)caps->ac_controls.hw[0] +
                      ((uint32_t)caps->ac_controls.b[3] << 16);
            chs = caps->ac_channels;
            bitw = caps->ac_controls.b[2];

            ret = cxd56_aud_setconfig(dmach, samprate, chs, bitw);
            if (ret != OK)
              {
                  auderr("ERROR: Output Configuration Error : %d\n", ret);
              }
          }

        break;

      case AUDIO_TYPE_INPUT:
        if (!IS_AUDIO_INPUT(dmach))
          {
              auderr("ERROR: Can't configure, device is input\n");
              ret = -EINVAL;
          }
        else
          {
            samprate = (uint32_t)caps->ac_controls.hw[0] +
                       ((uint32_t)caps->ac_controls.b[3] << 16);
            chs = caps->ac_channels;
            bitw = caps->ac_controls.b[2];

            ret = cxd56_aud_setconfig(dmach, samprate, chs, bitw);
            if (ret != OK)
              {
                  auderr("ERROR: Input Configuration Error %d \n", ret);
              }
          }

        break;

      default:
        ret = -ENOTSUP;
        break;
    }

  return ret;
}

static int cxd56audio_shutdown(FAR struct audio_lowerhalf_s *lower, int cnt)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  if (cnt == 0)
    {
      /* Reference counter is zero, means closing the device file.
       * So audio system power is shutdown.
       */

      if (cxd56_auddma_shutdown(dmach))
        {
          dmach->hwresource(dmach, false);
        }

      if (IS_AUDIO_OUTPUT(dmach))
        {
          cxd56_use_mute_off(false);
        }

      cxd56_system_power_off(dmach);
    }
  else
    {
      /* Othewise, user is requesting to re-initialize the device. */

      if (cxd56_auddma_shutdown(dmach))
        {
          dmach->hwresource(dmach, false);
        }

      AUDSTATECHG_INITIAL(dmach);
    }

  return OK;
}

static int cxd56audio_start(FAR struct audio_lowerhalf_s *lower
                            MSESSION_ARG)
{
  int ret = -EAGAIN;
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  if (AUDSTATE_IS_CONFIGURED(dmach))
    {
      dmach->hwresource(dmach, true);
      ret = cxd56_auddma_start(dmach);
      if (ret != OK)
        {
          dmach->hwresource(dmach, false);
        }
    }

  return ret;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int cxd56audio_stop(FAR struct audio_lowerhalf_s *lower
                           MSESSION_ARG)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  if (AUDSTATE_IS_SHUTDOWN(dmach))
    {
      return -EAGAIN;
    }
  else if (cxd56_auddma_stop(dmach))
    {
      /* cxd56_auddma_stop() returned if HW resource is needed to
       * release here or not.
       */

      dmach->hwresource(dmach, false);
    }

  return OK;
}
#endif  /* CONFIG_AUDIO_EXCLUDE_STOP */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int cxd56audio_pause(FAR struct audio_lowerhalf_s *lower
                            MSESSION_ARG)
{
  int ret = -EAGAIN;
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  if (!AUDSTATE_IS_SHUTDOWN(dmach))
    {
      ret = cxd56_auddma_pause(dmach);
    }

  return ret;
}

static int cxd56audio_resume(FAR struct audio_lowerhalf_s *lower
                             MSESSION_ARG)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  /* At first, make sure the DMA is not going to stop */

  while (AUDSTATE_IS_PAUSING(dmach) ||
         AUDSTATE_IS_COMPLETING(dmach) ||
         AUDSTATE_IS_STOPPING(dmach))
    {
      usleep(1);
    }

  return cxd56_auddma_resume(dmach);
}
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

static int cxd56audio_enqbuff(FAR struct audio_lowerhalf_s *lower,
                              FAR struct ap_buffer_s *apb)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  if (AUDSTATE_IS_SHUTDOWN(dmach))
    {
      return -EAGAIN;
    }

  return cxd56_auddma_enqueue(dmach, apb);
}

static int cxd56audio_cancelbuff(FAR struct audio_lowerhalf_s *lower,
                                 FAR struct ap_buffer_s *apb)
{
  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;

  /* When app wants to cancel buffer, it is not for a specific buffer.
   * Because canceling it is to stop playing or recording, and to get back
   * buffer(s) which is/are already enqueued.
   * So avoid the apb argument and remove all buffer(s) not run yet.
   */

  return cxd56_auddma_cancelbuff(dmach);
}

static int cxd56audio_ioctl(FAR struct audio_lowerhalf_s *lower, int cmd,
                            unsigned long arg)
{
  int ret = -EINVAL;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  FAR struct cxd56_dmachannel_s *dmach =
        (FAR struct cxd56_dmachannel_s *)lower;
  FAR struct cxd56_audio_ioctl_s *acmd =
        (FAR struct cxd56_audio_ioctl_s *)arg;

  if (AUDSTATE_IS_SHUTDOWN(dmach) || AUDSTATE_IS_CLOSING(dmach))
    {
      return -EAGAIN;
    }

  switch (cmd)
    {
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_CXD56_AUDIO_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_CXD56_AUDIO_NUM_BUFFERS;
          ret = OK;
        }
        break;
#endif

      case AUDIOIOC_VENDORSPECIFIC:
        switch (acmd->cmd)
          {
            case CXD56AUD_SET_SYSVOLUME:
              if (IS_AUDIO_OUTPUT(dmach))
                {
                  cxd56_update_systemvol((uint16_t)acmd->arg);
                  ret = OK;
                }

              break;

            case CXD56AUD_SET_AUDIOPATH:

              /* To implement to make I2SDown DMA to I2S out or SPK out */

              ret = -ENOTSUP;
              break;

            case CXD56AUD_SET_I2SMODE:
              if (IS_AUDIO_I2SDEV(dmach))
                {
                  ret = -EBUSY;
                  if (cxd56_set_i2sslave((bool)acmd->arg))
                    {
                      ret = OK;
                    }
                }

              break;

            case CXD56AUD_SET_SAMPLERATE:
              ret = cxd56_aud_setconfig(dmach, acmd->arg,
                                        dmach->channels, dmach->bitwidth);
              break;
          }

        break; /* of case AUDIOIOC_VENDORSPECIFIC */
    }

  return ret;
}

static int cxd56audio_reserve(FAR struct audio_lowerhalf_s *lower
                              MSESSION_ARG)
{
  /* Do nothing for this driver */

  return OK;
}

static int cxd56audio_release(FAR struct audio_lowerhalf_s *lower
                              MSESSION_ARG)
{
  /* Do nothing for this driver */

  return OK;
}

/* CXD56 BEEP device driver operations ***************************************/

#ifdef CONFIG_CXD56_BEEEPDEV
uint32_t convert_beep_freq(uint32_t freq)
{
  uint32_t prev;
  uint32_t i;

  /* Clip frequency */

  freq = freq > BEEP_FREQ_MAX ? BEEP_FREQ_MAX :
         freq < BEEP_FREQ_MIN ? BEEP_FREQ_MIN : freq;

  for (i = 0; i < sizeof(g_beepfreqtable) / sizeof(uint16_t); i++)
    {
      prev = (i + 62) % 63;
      if (freq < g_beepfreqtable[i])
        {
          if (prev == 62)
            {
              break;
            }
          else if(g_beepfreqtable[prev] <= freq)
            {
              break;
            }
        }
    }

  return prev;
}

static int cxd56_beep_open(FAR struct file *filep)
{
  int ret = -EBUSY;
  FAR struct cxd56_aud_beep_s *beep;
  beep = (FAR struct cxd56_aud_beep_s *)filep->f_inode->i_private;

  if (!beep->is_inited)
    {
      nxsem_wait_uninterruptible(&g_system.lock);

      g_system.usr_cnt++;
      if (g_system.usr_cnt == 1)
        {
          cxd56_audio_initialize();
        }

      nxsem_post(&g_system.lock);

      cxd56_use_mute_off(true);
      cxd56_use_spkout(true);
      beep->is_inited = true;
      ret = OK;
    }

  return ret;
}

static int cxd56_beep_close(FAR struct file *filep)
{
  FAR struct cxd56_aud_beep_s *beep;
  beep = (FAR struct cxd56_aud_beep_s *)filep->f_inode->i_private;

  if (beep->is_inited)
    {
      cxd56_use_spkout(false);
      cxd56_use_mute_off(false);

      nxsem_wait_uninterruptible(&g_system.lock);

      g_system.usr_cnt--;
      if (g_system.usr_cnt == 0)
        {
          cxd56_audio_deinitialize();
        }

      nxsem_post(&g_system.lock);

      beep->is_inited = false;
    }

  return OK;
}

static ssize_t cxd56_beep_read(FAR struct file *filep,
                               FAR char *buffer, size_t buflen)
{
  ssize_t ret = -EINVAL;
  FAR struct cxd56_aud_beep_s *beep;
  beep = (FAR struct cxd56_aud_beep_s *)filep->f_inode->i_private;

  if (buflen >= sizeof(struct beep_ctl_s))
    {
      memcpy(buffer, &beep->ctl, sizeof(struct beep_ctl_s));
      ret = sizeof(struct beep_ctl_s);
    }

  return ret;
}

static ssize_t cxd56_beep_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  int ret = -EINVAL;
  FAR struct cxd56_aud_beep_s *beep;
  beep = (FAR struct cxd56_aud_beep_s *)filep->f_inode->i_private;

  if (buflen >= sizeof(struct beep_ctl_s))
    {
      memcpy(&beep->ctl, buffer, sizeof(struct beep_ctl_s));
      cxd56_audio_beep_ctl(beep->ctl.en, convert_beep_freq(beep->ctl.freq),
                           beep->ctl.vol);
      ret = sizeof(struct beep_ctl_s);
    }

  return ret;
}

static int cxd56_beep_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  int ret = -EINVAL;

  switch (cmd)
    {
      case CXD56AUD_SET_SYSVOLUME:
        cxd56_update_systemvol((uint16_t)arg);
        ret = OK;
        break;
    }

  return ret;
}
#endif

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int cxd56_audsystem_initialize(FAR cxd56_audio_lower_t *low)
{
  if (low == NULL)
    {
      return -EINVAL;
    }

  g_lower = low;

#ifdef CONFIG_CXD56_BEEEPDEV
  g_beep = (FAR struct cxd56_aud_beep_s *)
                kmm_zalloc(sizeof(struct cxd56_aud_beep_s));

  if (g_beep == NULL)
    {
      g_lower = NULL;
      return -ENOMEM;
    }
#endif

  /* Initialize HW resource management variables */

  nxsem_init(&g_system.lock, 0, 1);
  g_system.usr_cnt = 0;

  nxsem_init(&g_mutectl.lock, 0, 1);
  g_mutectl.mute_cnt = 0;

  g_hwres.spkout_cnt = 0;
  g_hwres.i2susr_cnt = 0;
  g_hwres.outfs_cnt = 0;
  g_hwres.volume = 1000;
  g_hwres.is_slave = false;
  g_hwres.samprate = CXD56_SAMPRATE_48K;

#ifdef CONFIG_CXD56_BEEEPDEV
  nxsem_init(&g_beep->lock, 0, 1);
  register_driver(CONFIG_CXD56_BEEPDEV_NAME, &g_cxd56_beep_fops,
                  0666, g_beep);
#endif

  return OK;
}

struct audio_lowerhalf_s *cxd56_aud_miclower(void)
{
  struct cxd56_dmachannel_s *dmach = NULL;

  if (g_lower)
    {
      dmach = cxd56_get_micdmach();
      dmach->maxch = cxd56_aud_micmaxch;
      dmach->setvolume = cxd56_aud_micgain;
      dmach->hwresource = cxd56_aud_michwres;
      dmach->hwresource_irq = cxd56_aud_michwres;
      dmach->lower.ops = &g_audioops;
    }

  return (FAR struct audio_lowerhalf_s *)dmach;
}

struct audio_lowerhalf_s *cxd56_aud_spk0out(void)
{
  struct cxd56_dmachannel_s *dmach = NULL;

  if (g_lower)
    {
      dmach = cxd56_get_i2s0out_dmach();
      dmach->maxch = cxd56_aud_generic_maxch;
      dmach->setvolume = cxd56_set_spkin0_vol;
      dmach->hwresource = cxd56_aud_i2s0outhwres;
      dmach->hwresource_irq = cxd56_aud_i2s0outhwres_irq;
      dmach->lower.ops = &g_audioops;
    }

  return (FAR struct audio_lowerhalf_s *)dmach;
}

struct audio_lowerhalf_s *cxd56_aud_spk1out(void)
{
  struct cxd56_dmachannel_s *dmach = NULL;

  if (g_lower)
    {
      dmach = cxd56_get_i2s1out_dmach();
      dmach->maxch = cxd56_aud_generic_maxch;
      dmach->setvolume = cxd56_set_spkin1_vol;
      dmach->hwresource = cxd56_aud_i2s1outhwres;
      dmach->hwresource_irq = cxd56_aud_i2s1outhwres_irq;
      dmach->lower.ops = &g_audioops;
    }

  return (FAR struct audio_lowerhalf_s *)dmach;
}

struct audio_lowerhalf_s *cxd56_aud_i2sin(void)
{
  struct cxd56_dmachannel_s *dmach = NULL;

  if (g_lower)
    {
      dmach = cxd56_get_i2s1in_dmach();
      dmach->maxch = cxd56_aud_generic_maxch;
      dmach->setvolume = cxd56_set_novol;
      dmach->hwresource = cxd56_aud_i2s1inhwres;
      dmach->hwresource_irq = cxd56_aud_i2s1inhwres_irq;
      dmach->lower.ops = &g_audioops;
    }

  return (FAR struct audio_lowerhalf_s *)dmach;
}
