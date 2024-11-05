/*****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_audio_dma.c
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
#include <debug.h>
#include <math.h>

#include <arch/chip/chip.h>
#include <arch/board/cxd56_clock.h>
#include <arch/board/board.h>
#include "hardware/cxd5602_topreg.h"

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#include "cxd56_audio_reg.h"
#include "hardware/cxd5247.h"
#include "cxd56_audio_driver.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define CXD56_AUDIO_DMA_START       (0x01)
#define CXD56_AUDIO_DMA_STOP_WOINTR (0x04)
#define CXD56_AUDIO_DMA_STOP        (0x00)

#define CXD56_DMABITW24 (0)
#define CXD56_DMABITW16 (1)

#define CXD56_INTRBIT_MIC_DONE    (1u << 0)
#define CXD56_INTRBIT_MIC_ERR     (1u << 1)
#define CXD56_INTRBIT_MIC_SMPL    (1u << 2)
#define CXD56_INTRBIT_MIC_CMB     (1u << 3)
#define CXD56_INTRBIT_I2SOUT_DONE (1u << 0)
#define CXD56_INTRBIT_I2SOUT_ERR  (1u << 1)
#define CXD56_INTRBIT_I2SIN_DONE  (1u << 2)
#define CXD56_INTRBIT_I2SIN_ERR   (1u << 3)
#define CXD56_INTRBIT_I2S_SMPL    (1u << 4)
#define CXD56_INTRBIT_I2S_CMB     (1u << 5)

#define CXD56_MICCHMAP_BITW16 (0x10325476)
#define CXD56_MICCHMAP_BITW24 (0x01234567)

#define CXD56_IRQ_AUDIOMIC   CXD56_IRQ_AUDIO_0
#define CXD56_IRQ_AUDIOI2S0  CXD56_IRQ_AUDIO_1
#define CXD56_IRQ_AUDIOI2S1  CXD56_IRQ_AUDIO_2

#define INT_IRQ_AUDINTBIT_MIC   (1<<6)
#define INT_IRQ_AUDINTBIT_I2S0  (1<<7)
#define INT_IRQ_AUDINTBIT_I2S1  (1<<8)

#define DMA_INTBITS(dmach) (cxd56_raureg32(*(dmach)->intr_stat) & \
                            ~cxd56_raureg32(*(dmach)->intr_mask))
#define IS_DMADONE(dmach) (cxd56_raureg32(*(dmach)->intr_stat) & \
                           (dmach)->intrbit_done)
#define IS_DMAERROR(dmach) (cxd56_raureg32(*(dmach)->intr_stat) & \
                            (dmach)->intrbit_err)
#define IS_DMASAMPLE(dmach) (cxd56_raureg32(*(dmach)->intr_stat) & \
                             (dmach)->intrbit_smp)
#define IS_DMACMB(dmach) (cxd56_raureg32(*(dmach)->intr_stat) & \
                          (dmach)->intrbit_cmb)

#define CXD56_DMABUG_DMA_SMP_WAIT (40) /* usec */
#define CXD56_DMABUG_WOKARND_RETRY (10000)

#define CXD56_AUD_MAXDMASAMPLE (1024)

#define dq_get(q)   ((FAR struct ap_buffer_s *)dq_remfirst(&(q)))
#define dq_put(q,n) ({dq_addlast((dq_entry_t*)&(n),&(q));})

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

extern void cxd56_audio_clock_enable(uint32_t clk, uint32_t div);
extern void cxd56_audio_clock_disable(void);
extern bool cxd56_audio_clock_is_enabled(void);

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static void dma_mic_chassign(FAR struct cxd56_dmachannel_s *dmach);
static void dma_i2s_chassign(FAR struct cxd56_dmachannel_s *dmach);
static int audio_micdma_hdlr(int irq, FAR void *context, FAR void *arg);
static int audio_i2sdma_hdlr(int irq, FAR void *context, FAR void *arg);
static int cxd56_audio_irq_en(FAR struct cxd56_dmachannel_s *dmach, bool en);
static int cxd56_audio_i2sirq_en(FAR struct cxd56_dmachannel_s *dmach,
                                 bool en);
static int inject_dma(FAR struct cxd56_dmachannel_s *dmach,
                      FAR struct ap_buffer_s *apb);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

struct cxd56_aud_dmaref_s i2s1_dmaref =
{
  0, /* uselock */
  0  /* usecnt */
};

static struct cxd56_dmachannel_s mic_dma =
{
  {NULL, NULL, NULL},       /* Instance of audio_lowerhalf_s */
  AUDIO_TYPE_INPUT | CXD56_AUD_MICDEV, /* type */
  16,                       /* bitwidth */
  2,                        /* channels */
  0,                        /* dma_lock */
  AUDSTATE_SHUTDOWN,        /* state */
  {NULL, NULL},             /* dma_pendq */
  {NULL, NULL},             /* dma_runq */
  CXD56_IRQ_AUDIOMIC,       /* cpu_intr_no */
  INT_IRQ_AUDINTBIT_MIC,    /* cpu_intr_bit */
  audio_micdma_hdlr,        /* isr */
  &AUREG_MIC_DMA_ADR,       /* addr */
  &AUREG_MIC_DMA_SAMPLES,   /* smpls */
  &AUREG_MIC_DMA_CMD,       /* cmd */
  &AUREG_MIC_BITWT,         /* bitw */
  &AUREG_MIC_DMA_CMDMON,    /* fifolen */
  &AUREG_MIC_DMA_SETUPERR,  /* errmon */
  &AUREG_MIC_DMA_STATUS,    /* status */
  &AUREG_MIC_CH8SEL,        /* chsel */
  CXD56_INTRBIT_MIC_DONE,   /* intrbit_done */
  CXD56_INTRBIT_MIC_ERR,    /* intrbit_err */
  CXD56_INTRBIT_MIC_SMPL,   /* intrbit_smp */
  CXD56_INTRBIT_MIC_CMB,    /* intrbit_cmb */
  &AUREG_MIC_INTR_STATUS,   /* intr_stat */
  &AUREG_MIC_INTR_MASK,     /* intr_mask */
  NULL,                     /* hwresource */
  NULL,                     /* hwresource_irq */
  dma_mic_chassign,         /* chassign */
  cxd56_audio_irq_en,       /* irqen */
  NULL,                     /* irq_arg */
  {{{{0}}}},                /* delay_work */
  NULL,                     /* setvolumme */
  NULL,                     /* maxch */
  AUDIO_TYPE_INPUT | AUDIO_TYPE_FEATURE, /* caps */
  AUDIO_SAMP_RATE_48K | AUDIO_SAMP_RATE_96K | AUDIO_SAMP_RATE_192K, /* supfs */
  AUDIO_FU_INP_GAIN,        /* fcaps */
  false,                    /* sysinited */
  1000,                     /* volgain */
  AUDDISTINATION_MEM        /* distination */
};

static struct cxd56_i2sdmachannel_s i2s0out_dma =
{
  {
    {NULL, NULL, NULL},            /* Instance of audio_lowerhalf_s */
    AUDIO_TYPE_OUTPUT | CXD56_AUD_I2SDEV | CXD56_AUD_I2S0, /* type */
    16,                            /* bitwidth */
    2,                             /* channels */
    0,                             /* dma_lock */
    AUDSTATE_SHUTDOWN,             /* state */
    {NULL, NULL},                  /* dma_pendq */
    {NULL, NULL},                  /* dma_runq */
    CXD56_IRQ_AUDIOI2S0,           /* cpu_intr_no */
    INT_IRQ_AUDINTBIT_I2S0,        /* cpu_intr_bit */
    audio_i2sdma_hdlr,             /* isr */
    &AUREG_I2S1OUT_DMA_ADR,        /* addr */
    &AUREG_I2S1OUT_DMA_SAMPLES,    /* smpls */
    &AUREG_I2S1OUT_DMA_CMD,        /* cmd */
    &AUREG_I2S1OUT_BITWT,          /* bitw */
    &AUREG_I2S1OUT_DMA_CMDMON,     /* fifolen */
    &AUREG_I2S1OUT_DMA_SETUPERR,   /* errmon */
    &AUREG_I2S1OUT_DMA_STATUS,     /* status */
    &AUREG_I2S1OUT_CHSEL,          /* chsel */
    CXD56_INTRBIT_I2SOUT_DONE,     /* intrbit_done */
    CXD56_INTRBIT_I2SOUT_ERR,      /* intrbit_err */
    CXD56_INTRBIT_I2S_SMPL,        /* intrbit_smp */
    CXD56_INTRBIT_I2S_CMB,         /* intrbit_cmb */
    &AUREG_I2S1_INTR_STATUS,       /* intr_stat */
    &AUREG_I2S1_INTR_MASK,         /* intr_mask */
    NULL,                          /* hwresource */
    NULL,                          /* hwresource_irq */
    dma_i2s_chassign,              /* chassign */
    cxd56_audio_i2sirq_en,         /* irqen */
    NULL,                          /* irq_arg */
    {{{{0}}}},                     /* delay_work */
    NULL,                          /* setvolumme */
    NULL,                          /* maxch */
    AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE,     /* caps */
    AUDIO_SAMP_RATE_48K | AUDIO_SAMP_RATE_192K, /* supfs */
    AUDIO_FU_VOLUME | AUDIO_FU_MUTE,            /* fcaps */
    false,                         /* sysinited */
    1000,                          /* volgain */
    AUDDISTINATION_SPK             /* distination */
  },
  NULL
};

static struct cxd56_i2sdmachannel_s i2s1out_dma =
{
  {
    {NULL, NULL, NULL},            /* Instance of audio_lowerhalf_s */
    AUDIO_TYPE_OUTPUT | CXD56_AUD_I2SDEV | CXD56_AUD_I2S1, /* type */
    16,                            /* bitwidth */
    2,                             /* channels */
    0,                             /* dma_lock */
    AUDSTATE_SHUTDOWN,             /* state */
    {NULL, NULL},                  /* dma_pendq */
    {NULL, NULL},                  /* dma_runq */
    CXD56_IRQ_AUDIOI2S1,           /* cpu_intr_no */
    INT_IRQ_AUDINTBIT_I2S1,        /* cpu_intr_bit */
    audio_i2sdma_hdlr,             /* isr */
    &AUREG_I2S2OUT_DMA_ADR,        /* addr */
    &AUREG_I2S2OUT_DMA_SAMPLES,    /* smpls */
    &AUREG_I2S2OUT_DMA_CMD,        /* cmd */
    &AUREG_I2S2OUT_BITWT,          /* bitw */
    &AUREG_I2S2OUT_DMA_CMDMON,     /* fifolen */
    &AUREG_I2S2OUT_DMA_SETUPERR,   /* errmon */
    &AUREG_I2S2OUT_DMA_STATUS,     /* status */
    &AUREG_I2S2OUT_CHSEL,          /* chsel */
    CXD56_INTRBIT_I2SOUT_DONE,     /* intrbit_done */
    CXD56_INTRBIT_I2SOUT_ERR,      /* intrbit_err */
    CXD56_INTRBIT_I2S_SMPL,        /* intrbit_smp */
    CXD56_INTRBIT_I2S_CMB,         /* intrbit_cmb */
    &AUREG_I2S2_INTR_STATUS,       /* intr_stat */
    &AUREG_I2S2_INTR_MASK,         /* intr_mask */
    NULL,                          /* hwresource */
    NULL,                          /* hwresource_irq */
    dma_i2s_chassign,              /* chassign */
    cxd56_audio_i2sirq_en,         /* irqen */
    NULL,                          /* irq_arg */
    {{{{0}}}},                     /* delay_work */
    NULL,                          /* setvolumme */
    NULL,                          /* maxch */
    AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE,     /* caps */
    AUDIO_SAMP_RATE_48K | AUDIO_SAMP_RATE_192K, /* supfs */
    AUDIO_FU_VOLUME | AUDIO_FU_MUTE,            /* fcaps */
    false,                         /* sysinited */
    1000,                          /* volgain */
    AUDDISTINATION_SPK             /* distination */
  },
  &i2s1_dmaref
};

static struct cxd56_i2sdmachannel_s i2s1in_dma =
{
  {
    {NULL, NULL, NULL},            /* Instance of audio_lowerhalf_s */
    AUDIO_TYPE_INPUT | CXD56_AUD_I2SDEV | CXD56_AUD_I2S1, /* type */
    16,                            /* bitwidth */
    2,                             /* channels */
    0,                             /* dma_lock */
    AUDSTATE_SHUTDOWN,             /* state */
    {NULL, NULL},                  /* dma_pendq */
    {NULL, NULL},                  /* dma_runq */
    CXD56_IRQ_AUDIOI2S1,           /* cpu_intr_no */
    INT_IRQ_AUDINTBIT_I2S1,        /* cpu_intr_bit */
    audio_i2sdma_hdlr,             /* isr */
    &AUREG_I2S2IN_DMA_ADR,         /* addr */
    &AUREG_I2S2IN_DMA_SAMPLES,     /* smpls */
    &AUREG_I2S2IN_DMA_CMD,         /* cmd */
    &AUREG_I2S2IN_BITWT,           /* bitw */
    &AUREG_I2S2IN_DMA_CMDMON,      /* fifolen */
    &AUREG_I2S2IN_DMA_SETUPERR,    /* errmon */
    &AUREG_I2S2IN_DMA_STATUS,      /* status */
    &AUREG_I2S2IN_CHSEL,           /* chsel */
    CXD56_INTRBIT_I2SIN_DONE,      /* intrbit_done */
    CXD56_INTRBIT_I2SIN_ERR,       /* intrbit_err */
    CXD56_INTRBIT_I2S_SMPL,        /* intrbit_smp */
    CXD56_INTRBIT_I2S_CMB,         /* intrbit_cmb */
    &AUREG_I2S2_INTR_STATUS,       /* intr_stat */
    &AUREG_I2S2_INTR_MASK,         /* intr_mask */
    NULL,                          /* hwresource */
    NULL,                          /* hwresource_irq */
    dma_i2s_chassign,              /* chassign */
    cxd56_audio_i2sirq_en,         /* irqen */
    NULL,                          /* irq_arg */
    {{{{0}}}},                     /* delay_work */
    NULL,                          /* setvolumme */
    NULL,                          /* maxch */
    AUDIO_TYPE_INPUT | AUDIO_TYPE_FEATURE,      /* caps */
    AUDIO_SAMP_RATE_48K | AUDIO_SAMP_RATE_192K, /* supfs */
    0,                             /* fcaps */
    false,                         /* sysinited */
    1000,                          /* volgain */
    AUDDISTINATION_MEM             /* distination */
  },
  &i2s1_dmaref
};

static struct cxd56_dmachannel_s *i2s_dmaarg[][2] =
{
  /* for SPK OUT */

  {
    (FAR struct cxd56_dmachannel_s *)NULL,
    (FAR struct cxd56_dmachannel_s *)&i2s0out_dma
  },

  /* for I2S IN/OUT */

  {
    (FAR struct cxd56_dmachannel_s *)&i2s1in_dma,
    (FAR struct cxd56_dmachannel_s *)&i2s1out_dma
  },
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/* CXD5602 Audio DMA Controller control functions */

static void dma_intr_clear(FAR struct cxd56_dmachannel_s *dmach,
                           uint32_t bits)
{
  cxd56_waureg32(*dmach->intr_stat, bits);
}

static void dma_intr_mask(FAR struct cxd56_dmachannel_s *dmach, uint32_t bits)
{
  bits |= cxd56_raureg32(*dmach->intr_mask);
  cxd56_waureg32(*dmach->intr_mask, bits);
}

static void dma_intr_unmask(FAR struct cxd56_dmachannel_s *dmach,
                            uint32_t bits)
{
  bits = cxd56_raureg32(*dmach->intr_mask) & ~bits;
  cxd56_waureg32(*dmach->intr_mask, bits);
}

static void dequeue_apb_to_app(FAR struct cxd56_dmachannel_s *dmach,
                               FAR struct ap_buffer_s *apb)
{
  apb->flags &= ~(AUDIO_APB_OUTPUT_ENQUEUED | AUDIO_APB_OUTPUT_PROCESS);
  dmach->lower.upper(dmach->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
}

static void avoid_allapbs(FAR struct cxd56_dmachannel_s *dmach,
                          FAR irqstate_t *flags)
{
  FAR struct ap_buffer_s *apb;

  while ((apb = dq_get(dmach->dma_runq)) != NULL)
    {
      spin_unlock_irqrestore(&dmach->dma_lock, *flags);
      dequeue_apb_to_app(dmach, apb);
      *flags = spin_lock_irqsave(&dmach->dma_lock);
    }

  while ((apb = dq_get(dmach->dma_pendq)) != NULL)
    {
      spin_unlock_irqrestore(&dmach->dma_lock, *flags);
      dequeue_apb_to_app(dmach, apb);
      *flags = spin_lock_irqsave(&dmach->dma_lock);
    }
}

static void send_message_underrun(FAR struct cxd56_dmachannel_s *dmach)
{
  struct audio_msg_s msg;

  msg.msg_id = AUDIO_MSG_UNDERRUN;
  msg.u.ptr = NULL;
  dmach->lower.upper(dmach->lower.priv, AUDIO_CALLBACK_MESSAGE,
                     (FAR struct ap_buffer_s *)&msg, OK);
}

static void release_all_apb_for_stopping(FAR void *arg)
{
  FAR struct cxd56_dmachannel_s *dmach =
    (FAR struct cxd56_dmachannel_s *)arg;

  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);
  if (AUDSTATE_IS_STOPPING(dmach) || AUDSTATE_IS_COMPLETING(dmach))
    {
      while ((apb = dq_get(dmach->dma_pendq)) != NULL)
        {
          spin_unlock_irqrestore(&dmach->dma_lock, flags);
          dequeue_apb_to_app(dmach, apb);
          flags = spin_lock_irqsave(&dmach->dma_lock);
        }
    }

  AUDSTATECHG_CONFIGURED(dmach);
  spin_unlock_irqrestore(&dmach->dma_lock, flags);
}

static void release_all_apb_for_shutdown(FAR void *arg)
{
  FAR struct cxd56_dmachannel_s *dmach =
    (FAR struct cxd56_dmachannel_s *)arg;

  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);
  while ((apb = dq_get(dmach->dma_pendq)) != NULL)
    {
      spin_unlock_irqrestore(&dmach->dma_lock, flags);
      dequeue_apb_to_app(dmach, apb);
      flags = spin_lock_irqsave(&dmach->dma_lock);
    }

  AUDSTATECHG_SHUTDOWN(dmach);
  spin_unlock_irqrestore(&dmach->dma_lock, flags);
}

static void handle_actual_dmaaction(FAR struct cxd56_dmachannel_s *dmach,
                                    uint32_t intbit)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);
  if ((AUDSTATE_IS_STOPPING(dmach) || AUDSTATE_IS_CLOSING(dmach) ||
       AUDSTATE_IS_COMPLETING(dmach) || AUDSTATE_IS_PAUSING(dmach)) &&
       cxd56_raureg(*dmach->status) == 0)
    {
      /* Final DMA is Done */

      cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP_WOINTR);
      dmach->irqen(dmach, false);  /* Stop DMA Interrupt */
      dma_intr_mask(dmach, dmach->intrbit_done |
                           dmach->intrbit_err  |
                           dmach->intrbit_smp  |
                           dmach->intrbit_cmb);

      /* Dequeue all audio buffers from runq */

      while ((apb = dq_get(dmach->dma_runq)) != NULL)
        {
          spin_unlock_irqrestore(&dmach->dma_lock, flags);
          dequeue_apb_to_app(dmach, apb);
          flags = spin_lock_irqsave(&dmach->dma_lock);
        }

      if (AUDSTATE_IS_STOPPING(dmach) || AUDSTATE_IS_COMPLETING(dmach))
        {
          spin_unlock_irqrestore(&dmach->dma_lock, flags);

          dmach->lower.upper(dmach->lower.priv,
                            AUDIO_CALLBACK_COMPLETE, NULL, OK);

          flags = enter_critical_section();
          dmach->hwresource_irq(dmach, false);
          leave_critical_section(flags);

          if (work_available(&dmach->delay_work))
            {
              /* Set delay worker for releasing all apb in pendq */

              work_queue(LPWORK, &dmach->delay_work, release_all_apb_for_stopping,
                        (FAR void *)dmach, 0);
            }

          flags = spin_lock_irqsave(&dmach->dma_lock);
        }
      else if (AUDSTATE_IS_PAUSING(dmach))
        {
          AUDSTATECHG_PAUSED(dmach);
        }
      else
        {
          spin_unlock_irqrestore(&dmach->dma_lock, flags);

          flags = enter_critical_section();
          dmach->hwresource_irq(dmach, false);
          leave_critical_section(flags);

          /* Release buffers not processed */

          if (work_available(&dmach->delay_work))
            {
              /* Set delay worker for releasing all apb in pendq */

              work_queue(LPWORK, &dmach->delay_work, release_all_apb_for_shutdown,
                        (FAR void *)dmach, 0);
            }

          flags = spin_lock_irqsave(&dmach->dma_lock);
        }
    }
  else if (intbit & dmach->intrbit_err || intbit & dmach->intrbit_cmb)
    {
      /* Handle Error */

      /* Check if the state is STARTED or STOPPING
       * to care conflict by I2S IN and OUT DMA
       * because they are sharing one DMA Master.
       */

      if (AUDSTATE_IS_STARTED(dmach))
        {
          cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP);
          AUDSTATECHG_STOPPING(dmach);
          spin_unlock_irqrestore(&dmach->dma_lock, flags);
          dmach->lower.upper(dmach->lower.priv,
                             AUDIO_CALLBACK_IOERR, NULL, EIO);
          flags = spin_lock_irqsave(&dmach->dma_lock);
          avoid_allapbs(dmach, &flags);
        }
    }
  else if (intbit & dmach->intrbit_done)
    {
      /* Handle DMA Done */

      apb = (FAR struct ap_buffer_s *)dq_get(dmach->dma_runq);
      if (apb)
        {
          spin_unlock_irqrestore(&dmach->dma_lock, flags);
          dequeue_apb_to_app(dmach, apb);
          flags = spin_lock_irqsave(&dmach->dma_lock);
        }

      if (AUDSTATE_IS_STARTED(dmach))
        {
          /* If DMA is still running, inject next data */

          apb = (FAR struct ap_buffer_s *)dq_get(dmach->dma_pendq);
          if (apb != NULL)
            {
              inject_dma(dmach, apb);
              if ((apb->flags & AUDIO_APB_FINAL) != 0)
                {
                  /* If this apb is final, going to DMA Stopping state */

                  cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP);
                  AUDSTATECHG_STOPPING(dmach);
                }
            }
          else if (cxd56_raureg(*dmach->fifolen) >= 2)
            {
              /* No enough buffer to exec: Under run is occured */

              spin_unlock_irqrestore(&dmach->dma_lock, flags);
              send_message_underrun(dmach);
              flags = spin_lock_irqsave(&dmach->dma_lock);
              cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP);
              AUDSTATECHG_STOPPING(dmach);
            }
        }
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);
}

static int audio_micdma_hdlr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cxd56_dmachannel_s *dmach =
      (FAR struct cxd56_dmachannel_s *)arg;
  uint32_t intbit = DMA_INTBITS(dmach);

  dma_intr_clear(dmach, intbit);
  handle_actual_dmaaction(dmach, intbit);

  return 0;
}

static int audio_i2sdma_hdlr(int irq, FAR void *context, FAR void *arg)
{
  int i;
  FAR struct cxd56_dmachannel_s **dmachs =
        (FAR struct cxd56_dmachannel_s **)arg;
  uint32_t intbit = dmachs[0] != NULL ? DMA_INTBITS(dmachs[0]) :
                                        DMA_INTBITS(dmachs[1]);

  for (i = 0; i < 2; i++)
    {
      if (intbit != 0 && dmachs[i])
        {
          dma_intr_clear(dmachs[i], intbit);
          handle_actual_dmaaction(dmachs[i], intbit);
        }
    }

  return 0;
}

/**************************** Instance methds ********************************/

static int cxd56_audio_irq_en(FAR struct cxd56_dmachannel_s *dmach,
                              bool en)
{
  if (en)
    {
      cxd56_waureg32(AUREG_INT_EN1,
                     cxd56_raureg32(AUREG_INT_EN1) | dmach->cpu_intr_bit);
      irq_attach(dmach->cpu_intr_no, dmach->isr, dmach->irq_arg);
      up_enable_irq(dmach->cpu_intr_no);
    }
  else
    {
      up_disable_irq(dmach->cpu_intr_no);
      irq_detach(dmach->cpu_intr_no);
      cxd56_waureg32(AUREG_INT_EN1,
                     cxd56_raureg32(AUREG_INT_EN1) & ~dmach->cpu_intr_bit);
    }

  return 0;
}

static int cxd56_audio_i2sirq_en(FAR struct cxd56_dmachannel_s *inst,
                                 bool en)
{
  FAR struct cxd56_i2sdmachannel_s *dmach =
    (FAR struct cxd56_i2sdmachannel_s *)inst;
  int usecnt;
  irqstate_t flags;

  if (dmach->ref != NULL)
    {
      if (en)
        {
          flags = spin_lock_irqsave(&dmach->ref->uselock);
          usecnt = dmach->ref->usecnt++;
          spin_unlock_irqrestore(&dmach->ref->uselock, flags);
          if (usecnt == 0)
            {
              cxd56_audio_irq_en((FAR struct cxd56_dmachannel_s *)dmach, en);
            }
        }
      else
        {
          flags = spin_lock_irqsave(&dmach->ref->uselock);

          usecnt = dmach->ref->usecnt--;
          if (dmach->ref->usecnt < 0)
            {
              dmach->ref->usecnt = 0;
            }

          spin_unlock_irqrestore(&dmach->ref->uselock, flags);

          if (usecnt == 1)
            {
              cxd56_audio_irq_en((FAR struct cxd56_dmachannel_s *)dmach, en);
            }
        }
    }
  else
    {
      cxd56_audio_irq_en((FAR struct cxd56_dmachannel_s *)dmach, en);
    }

  return OK;
}

static void dma_i2s_chassign(FAR struct cxd56_dmachannel_s *dmach)
{
  cxd56_waureg(*dmach->chsel, 0x10);
}

static void dma_mic_chassign(FAR struct cxd56_dmachannel_s *dmach)
{
  uint32_t i;
  uint8_t mic_num;

  /* Each 4bit is for each ch assing. 8 means No assign */

  uint32_t mic_dmach    = 0x88888888;
  uint32_t mic_chselmsk = 0xf0000000;
  uint32_t mic_chassign;

  if (dmach->bitwidth == 16)
    {
      mic_chassign = CXD56_MICCHMAP_BITW16;

      if (dmach->channels == 1)
        {
          /* DMA needs even channels, because of bus is 32bits.
           * In case of 1 channel, duplicate even channel data on
           * a half word on odd side.
           */

          mic_chassign = mic_chassign & 0x00fffffff;
        }

      mic_num = (dmach->channels + 1) / 2;
      mic_num = (mic_num > (CXD56_MIC_MAXCH / 2)) ?
                (CXD56_MIC_MAXCH / 2) : mic_num;

      for (i = 0; i < mic_num; i++, mic_chselmsk >>= 4)
        {
          mic_dmach = (mic_dmach & ~mic_chselmsk) +
                      (i << ((CXD56_MIC_MAXCH - 1 - i) * 4));
        }
    }
  else
    {
      mic_chassign = CXD56_MICCHMAP_BITW24;

      mic_num = dmach->channels;
      for (i = 0; i < mic_num; i++, mic_chselmsk >>= 4)
        {
          mic_dmach = (mic_dmach & ~mic_chselmsk) +
                      (i << ((CXD56_MIC_MAXCH - 1 - i) * 4));
        }
    }

  cxd56_waureg32(AUREG_SEL_OUT1_L, mic_chassign);
  cxd56_waureg32(*dmach->chsel, mic_dmach);
}

/**************************** DMA HW BUG workaround **************************/

static void kick_dma(struct cxd56_dmachannel_s *dmach)
{
  cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_START);
}

static void reset_dma_chsel(struct cxd56_dmachannel_s *dmach)
{
  uint32_t sel;

  sel = cxd56_raureg32(*dmach->chsel);
  cxd56_waureg32(*dmach->chsel, 0xffffffff);
  cxd56_waureg32(*dmach->chsel, sel);
}

static int dma_bug1_workaround_1and2(struct cxd56_dmachannel_s *dmach)
{
  int retry;

  /* Set Interrupt Mask Done and Error */

  dma_intr_clear(dmach, dmach->intrbit_err | dmach->intrbit_smp);

  for (retry = 0; retry < CXD56_DMABUG_WOKARND_RETRY; retry++)
    {
      if (IS_DMASAMPLE(dmach))
        {
          break;
        }
    }

  return retry == CXD56_DMABUG_WOKARND_RETRY ? -1 : 0;
}

static int check_dma_no_error(struct cxd56_dmachannel_s *dmach)
{
  int to;

  if (IS_DMAERROR(dmach))
    {
      cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP_WOINTR);
      dma_intr_clear(dmach, dmach->intrbit_err);

      /* Wait for finishing DMA which is already executed */

      for (to = 0; to < CXD56_DMABUG_WOKARND_RETRY; to++)
        {
          if (IS_DMADONE(dmach))
            {
              dma_intr_clear(dmach, dmach->intrbit_done);
              break;
            }
        }

      return 0;
    }

  return 1;
}

static int kick_dma_workaroundbug(struct cxd56_dmachannel_s *dmach)
{
  int retry;
  irqstate_t flags;

  /* Workaround implementation of start_dma_workaround is refered in
   * nuttx/boards/arm/cxd56xx/drivers/audio/cxd56_audio_dma.c
   */

  /* Execute out-of-sync workaround.
   * 1. Clear smp interrupt status
   * 2. Read until smp interrupt state is true
   * 3. Reset channel select setting
   * 4. Start dma transfer
   * It needs to be less than 9 us by the processing so far.
   * If it does not fit below 9 us, err_int is generated, so retry.
   */

  flags = spin_lock_irqsave(&dmach->dma_lock);

  dma_intr_mask(dmach, dmach->intrbit_done | dmach->intrbit_err);

  for (retry = 0; retry < CXD56_DMABUG_WOKARND_RETRY; retry++)
    {
      if (dma_bug1_workaround_1and2(dmach) != 0)
        {
          spin_unlock_irqrestore(&dmach->dma_lock, flags);
          return -ETIME;
        }

      reset_dma_chsel(dmach);
      kick_dma(dmach);

      /* Wait for one sample transfer */

      up_udelay(CXD56_DMABUG_DMA_SMP_WAIT);
      if (check_dma_no_error(dmach))
        {
          break; /* Exit from loop */
        }
    }

  dma_intr_unmask(dmach, dmach->intrbit_done);
  dma_intr_clear(dmach, dmach->intrbit_err);
  dma_intr_unmask(dmach, dmach->intrbit_err);

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  return 0;
}

/**************************** Enqueue DMA Fifo *******************************/

static int inject_dma(FAR struct cxd56_dmachannel_s *dmach,
                      FAR struct ap_buffer_s *apb)
{
  uint32_t sampls;

  int ret = 0;

  if (cxd56_raureg(*dmach->fifolen) == 0)
    {
      return -EAGAIN;
    }

  if (IS_AUDIO_INPUT(dmach))
    {
      /* DMA size is set as sample number.
       * But it is limited up to 1024 samples.
       * Because size register is only available 10 bits.
       */

      sampls = apb->nmaxbytes / (dmach->bitwidth / 8) / dmach->channels;
      sampls = sampls <= CXD56_AUD_MAXDMASAMPLE ? sampls :
                                                  CXD56_AUD_MAXDMASAMPLE;

      /* In input device case, expected capture bytes is set on nbytes. */

      apb->nbytes = sampls * (dmach->bitwidth / 8) * dmach->channels;
    }
  else
    {
      /* DMA size is set as sample number.
       * But it is limited to 1024 samples.
       * Because size register is only available 10 bits.
       */

      sampls = apb->nbytes / (dmach->bitwidth / 8) / dmach->channels;
      sampls = sampls <= CXD56_AUD_MAXDMASAMPLE ? sampls :
                                                  CXD56_AUD_MAXDMASAMPLE;
    }

  sampls = sampls - 1;

  cxd56_waureg32(*dmach->addr,  CXD56_PHYSADDR(apb->samp));
  cxd56_waureg32(*dmach->smpls, sampls);

  if (AUDSTATE_IS_STARTED(dmach))
    {
      kick_dma(dmach);
      apb->flags |= AUDIO_APB_OUTPUT_PROCESS;
      dq_put(dmach->dma_runq, apb->dq_entry);
    }
  else
    {
      if (kick_dma_workaroundbug(dmach) == 0)
        {
          AUDSTATECHG_STARTED(dmach);
          apb->flags |= AUDIO_APB_OUTPUT_PROCESS;
          dq_put(dmach->dma_runq, apb->dq_entry);
        }
      else
        {
          ret = -EIO;
        }
    }

  return ret;
}

static int start_dma(FAR struct cxd56_dmachannel_s *dmach)
{
  int ret = 0;
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);

  while (cxd56_raureg(*dmach->fifolen) != 0 &&
         (apb = (FAR struct ap_buffer_s *)dq_get(dmach->dma_pendq)))
    {
      ret = inject_dma(dmach, apb);
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  return ret;
}

static void dma_initialize(struct cxd56_dmachannel_s *dmach)
{
  dmach->chassign(dmach);
  cxd56_waureg(*dmach->bitw, dmach->bitwidth == 16 ?
                              CXD56_DMABITW16 : CXD56_DMABITW24);
  cxd56_waureg(*dmach->addr, 0);
  cxd56_waureg(*dmach->smpls, 0);

  dma_intr_clear(dmach, dmach->intrbit_done |
                        dmach->intrbit_err  |
                        dmach->intrbit_smp  |
                        dmach->intrbit_cmb);
  dma_intr_mask(dmach, dmach->intrbit_done |
                       dmach->intrbit_err  |
                       dmach->intrbit_smp  |
                       dmach->intrbit_cmb);

  if (cxd56_raureg(*dmach->errmon))
    {
      auderr("ERROR: CXD5602 DMA Set error: %ld\n",
             cxd56_raureg(*dmach->errmon));
    }

  dmach->irqen(dmach, true);
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int cxd56_auddma_enqueue(FAR struct cxd56_dmachannel_s *dmach,
                         FAR struct ap_buffer_s *apb)
{
  int ret = 0;
  irqstate_t flags;
  bool do_start = false;

  flags = spin_lock_irqsave(&dmach->dma_lock);
  if (!AUDSTATE_IS_INITIAL(dmach) &&
      !AUDSTATE_IS_STOPPING(dmach) &&
      !AUDSTATE_IS_CLOSING(dmach))
    {
      apb->flags = (apb->flags & ~AUDIO_APB_DEQUEUED) |
                   AUDIO_APB_OUTPUT_ENQUEUED;
      dq_put(dmach->dma_pendq, apb->dq_entry);
      if (AUDSTATE_IS_STARTING(dmach) &&
          dq_count(&dmach->dma_pendq) >= CONFIG_CXD56_START_ENQUEUE_APB)
        {
          dma_initialize(dmach);
          do_start = true;
        }
    }
  else
    {
      ret = -EAGAIN;
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  if (do_start)
    {
      ret = start_dma(dmach);
    }

  return ret;
}

int cxd56_auddma_start(FAR struct cxd56_dmachannel_s *dmach)
{
  int ret = -EALREADY;
  irqstate_t flags;
  bool do_start = false;

  flags = spin_lock_irqsave(&dmach->dma_lock);

  if (AUDSTATE_IS_CONFIGURED(dmach))
    {
      if (dq_count(&dmach->dma_pendq) >= CONFIG_CXD56_START_ENQUEUE_APB)
        {
          dma_initialize(dmach);
          do_start = true;
        }
      else
        {
          AUDSTATECHG_STARTING(dmach);
        }

      ret = OK;
    }
  else if (AUDSTATE_IS_INITIAL(dmach))
    {
      ret = -EAGAIN;
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  if (do_start)
    {
      ret = start_dma(dmach);
    }

  return ret;
}

bool cxd56_auddma_stop(struct cxd56_dmachannel_s *dmach)
{
  bool need_release_resource = false;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);

  switch (dmach->state)
    {
      case AUDSTATE_STARTED:

        /* Triger to stop DMA */

        cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP);
        AUDSTATECHG_STOPPING(dmach);
        break;

      case AUDSTATE_PAUSING:

        /* DMA is going to pause already,
         * but stop request is higher proiritized.
         */

        AUDSTATECHG_STOPPING(dmach);
        break;

      case AUDSTATE_STARTING:
      case AUDSTATE_PAUSED:

        /* On these state, DMA is not working but HW resource is aquired.
         * So HW resource should be released by the task because
         * HW resource is not released in DMA interrupt handler.
         */

        need_release_resource = true;
        AUDSTATECHG_CONFIGURED(dmach);
        break;

      case AUDSTATE_STOPPING:
      case AUDSTATE_COMPLETING:
      case AUDSTATE_CONFIGURED:
      case AUDSTATE_INITIAL:
      case AUDSTATE_SHUTDOWN:

        /* Do nothing */

        break;

      case AUDSTATE_CLOSING:

        /* To enter CLOSING state, shutdown() must be called.
         * The function is waiting for going to SHUTDOWN state.
         * So this case never happened.
         */

        break;
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  while (!AUDSTATE_IS_INITIAL(dmach) &&
         !AUDSTATE_IS_CONFIGURED(dmach) &&
         !AUDSTATE_IS_SHUTDOWN(dmach))
    {
      usleep(1);
    }

  flags = spin_lock_irqsave(&dmach->dma_lock);
  avoid_allapbs(dmach, &flags);
  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  return  need_release_resource;
}

bool cxd56_auddma_shutdown(struct cxd56_dmachannel_s *dmach)
{
  irqstate_t flags;
  bool need_release_resource = false;

  flags = spin_lock_irqsave(&dmach->dma_lock);

  switch (dmach->state)
    {
      case AUDSTATE_STARTED:

        /* Triger to stop DMA */

        cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP);
        AUDSTATECHG_CLOSING(dmach);
        break;

      case AUDSTATE_STOPPING:
      case AUDSTATE_COMPLETING:
      case AUDSTATE_PAUSING:

        /* DMA is going to stop/pause already,
         * but shutdown request is higher proiritized.
         */

        AUDSTATECHG_CLOSING(dmach);
        break;

      case AUDSTATE_STARTING:
      case AUDSTATE_PAUSED:

        /* On these state, DMA is not working but HW resource is aquired.
         * So HW resource should be released by the task because
         * HW resource is not released in DMA interrupt handler.
         */

        need_release_resource = true;
        AUDSTATECHG_SHUTDOWN(dmach);
        break;

      case AUDSTATE_CONFIGURED:
      case AUDSTATE_INITIAL:
        AUDSTATECHG_SHUTDOWN(dmach);
        break;

      case AUDSTATE_CLOSING:
      case AUDSTATE_SHUTDOWN:

        /* Do nothing */

        break;
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  /* Blocking until go to SHUTDOWN */

  while (!AUDSTATE_IS_SHUTDOWN(dmach))
    {
      usleep(1);
    }

  flags = spin_lock_irqsave(&dmach->dma_lock);
  avoid_allapbs(dmach, &flags);
  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  return  need_release_resource;
}

int cxd56_auddma_pause(struct cxd56_dmachannel_s *dmach)
{
  int ret = -EAGAIN;

  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);

  if (AUDSTATE_IS_STARTED(dmach))
    {
      /* Pause request is accepted only when it is playing */

      cxd56_waureg(*dmach->cmd, CXD56_AUDIO_DMA_STOP);
      AUDSTATECHG_PAUSING(dmach);
      ret = OK;
    }
  else if (AUDSTATE_IS_STARTING(dmach))
    {
      AUDSTATECHG_PAUSED(dmach);
      ret = OK;
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  return ret;
}

int cxd56_auddma_resume(struct cxd56_dmachannel_s *dmach)
{
  bool do_start = false;
  int ret = -EAGAIN;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);

  if (AUDSTATE_IS_PAUSED(dmach))
    {
      if (dq_count(&dmach->dma_pendq) >= CONFIG_CXD56_START_ENQUEUE_APB)
        {
          dma_initialize(dmach);
          do_start = true;
        }
      else
        {
          AUDSTATECHG_STARTING(dmach);
        }

      ret = OK;
    }

  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  if (do_start)
    {
      ret = start_dma(dmach);
    }

  return ret;
}

int cxd56_auddma_cancelbuff(struct cxd56_dmachannel_s *dmach)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&dmach->dma_lock);
  while (dq_get(dmach->dma_pendq) != NULL);
  spin_unlock_irqrestore(&dmach->dma_lock, flags);

  return OK;
}

struct cxd56_dmachannel_s *cxd56_get_micdmach(void)
{
  mic_dma.irq_arg     = &mic_dma;
  dq_init(&mic_dma.dma_pendq);
  dq_init(&mic_dma.dma_runq);

  return (struct cxd56_dmachannel_s *)&mic_dma;
}

struct cxd56_dmachannel_s *cxd56_get_i2s0out_dmach(void)
{
  i2s0out_dma.dmach.irq_arg = &i2s_dmaarg[0][0];
  dq_init(&i2s0out_dma.dmach.dma_pendq);
  dq_init(&i2s0out_dma.dmach.dma_runq);

  return (struct cxd56_dmachannel_s *)&i2s0out_dma;
}

struct cxd56_dmachannel_s *cxd56_get_i2s1out_dmach(void)
{
  i2s1out_dma.dmach.irq_arg = &i2s_dmaarg[1][0];
  dq_init(&i2s1out_dma.dmach.dma_pendq);
  dq_init(&i2s1out_dma.dmach.dma_runq);

  return (struct cxd56_dmachannel_s *)&i2s1out_dma;
}

struct cxd56_dmachannel_s *cxd56_get_i2s1in_dmach(void)
{
  i2s1in_dma.dmach.irq_arg  = &i2s_dmaarg[1][0];
  dq_init(&i2s1in_dma.dmach.dma_pendq);
  dq_init(&i2s1in_dma.dmach.dma_runq);

  return (struct cxd56_dmachannel_s *)&i2s1in_dma;
}
