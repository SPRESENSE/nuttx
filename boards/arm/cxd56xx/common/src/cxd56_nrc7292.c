/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_nrc7292.c
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

#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/nrc7292.h>
#include <arch/chip/pin.h>
#include <arch/board/board.h>

#include "cxd56_pinconfig.h"
#include "cxd56_spi.h"
#include "cxd56_dmac.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CXD56_NRC_HALOW_SPI4)
#  define SPI_CH           (4)
#  if  defined(CONFIG_CXD56_NRC_HALOW_SPI4_DMAC)
#    define DMA_TXCH       (CONFIG_CXD56_DMAC_SPI4_TX_CH)
#    define DMA_RXCH       (CONFIG_CXD56_DMAC_SPI4_RX_CH)
#    define DMA_TXCH_CFG   (CXD56_DMA_PERIPHERAL_SPI4_TX)
#    define DMA_RXCH_CFG   (CXD56_DMA_PERIPHERAL_SPI4_RX)
#    define SPI_TX_MAXSIZE (CONFIG_CXD56_DMAC_SPI4_TX_MAXSIZE)
#    define SPI_RX_MAXSIZE (CONFIG_CXD56_DMAC_SPI4_RX_MAXSIZE)
#  endif
#elif defined(CONFIG_CXD56_NRC_HALOW_SPI5)
#  define SPI_CH           (5)
#  if  defined(CONFIG_CXD56_NRC_HALOW_SPI5_DMAC)
#    define DMA_TXCH       (CONFIG_CXD56_DMAC_SPI5_TX_CH)
#    define DMA_RXCH       (CONFIG_CXD56_DMAC_SPI5_RX_CH)
#    define DMA_TXCH_CFG   (CXD56_DMA_PERIPHERAL_SPI5_TX)
#    define DMA_RXCH_CFG   (CXD56_DMA_PERIPHERAL_SPI5_RX)
#    define SPI_TX_MAXSIZE (CONFIG_CXD56_DMAC_SPI5_TX_MAXSIZE)
#    define SPI_RX_MAXSIZE (CONFIG_CXD56_DMAC_SPI5_RX_MAXSIZE)
#  endif
#else
#  error "Select NRC Wi-Fi Halow SPI 4 or 5"
#endif

#define NRC7292_EIRQ (PIN_RTC_IRQ_OUT)
#define POWER_NRC    (PMIC_GPO(2))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  nrc7292_irq_attach(xcpt_t, void *);
static void nrc7292_irq_enable(void);
static void nrc7292_irq_disable(void);
static void nrc7292_reset(bool);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct nrc7292_lower_s g_nrc7292_lower =
{
  .attach  = nrc7292_irq_attach,
  .enable  = nrc7292_irq_enable,
  .disable = nrc7292_irq_disable,
  .reset   = nrc7292_reset
};

static void *g_devhandle = NULL;
static volatile int32_t  _enable_count = 0;
static volatile uint32_t _n_called;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrc7292_irq_attach
 ****************************************************************************/

static int nrc7292_irq_attach(xcpt_t handler, void *arg)
{
  if (handler)
    {
      cxd56_gpioint_config(NRC7292_EIRQ,
                           GPIOINT_LEVEL_HIGH | GPIOINT_NOISE_FILTER_DISABLE,
                           handler,
                           arg);
    }
  else
    {
      cxd56_gpioint_config(NRC7292_EIRQ, 0, NULL, NULL);
    }

  return 0;
}

/****************************************************************************
 * Name: nrc7292_irq_enable
 ****************************************************************************/

static void nrc7292_irq_enable(void)
{
  irqstate_t flags = enter_critical_section();

  wlinfo("== ec:%" PRId32 " called=%" PRId32 "\n",
         _enable_count, _n_called++);

  if (0 == _enable_count)
    {
      cxd56_gpioint_enable(NRC7292_EIRQ);
    }

  _enable_count++;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrc7292_irq_disable
 ****************************************************************************/

static void nrc7292_irq_disable(void)
{
  irqstate_t flags = enter_critical_section();

  wlinfo("== ec:%" PRId32 " called=%" PRId32 "\n",
         _enable_count, _n_called++);

  _enable_count--;

  if (0 == _enable_count)
    {
      cxd56_gpioint_disable(NRC7292_EIRQ);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrc7292_reset
 ****************************************************************************/

static void nrc7292_reset(bool reset)
{
  board_power_control(POWER_NRC, reset);
}

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 ****************************************************************************/

static void spi_pincontrol(int bus, bool on)
{
  switch (bus)
    {
#ifdef CONFIG_CXD56_SPI4
      case 4:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_SPI4 */

#ifdef CONFIG_CXD56_SPI5
      case 5:
#ifdef CONFIG_CXD56_SPI5_PINMAP_EMMC
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
          }
#endif /* CONFIG_CXD56_SPI5_PINMAP_EMMC */

#ifdef CONFIG_CXD56_SPI5_PINMAP_SDIO
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
          }
#endif /* CONFIG_CXD56_SPI5_PINMAP_SDIO */
        break;
#endif /* CONFIG_CXD56_SPI5 */
      default:
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_nrc7292_initialize
 ****************************************************************************/

int board_nrc7292_initialize(const char *devpath)
{
  struct spi_dev_s *spi;
  DMA_HANDLE    hdl;
  dma_config_t  conf;
  int bus = SPI_CH;

  wlinfo("Initializing NRC7292..\n");

  if (!g_devhandle)
    {
      nrc7292_reset(true);

      cxd56_gpio_config(NRC7292_EIRQ, true);

      spi = cxd56_spibus_initialize(bus);

      if (!spi)
        {
          wlerr("ERROR: Failed to initialize spi%d.\n", bus);
          return -ENODEV;
        }

#if defined(CONFIG_CXD56_NRC_HALOW_SPI4_DMAC) || \
    defined(CONFIG_CXD56_NRC_HALOW_SPI5_DMAC)

      hdl = cxd56_dmachannel(DMA_TXCH, SPI_TX_MAXSIZE);

      if (hdl)
        {
          conf.channel_cfg = DMA_TXCH_CFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(bus, CXD56_SPI_DMAC_CHTYPE_TX, hdl, &conf);
        }

      hdl = cxd56_dmachannel(DMA_RXCH, SPI_RX_MAXSIZE);

      if (hdl)
        {
          conf.channel_cfg = DMA_RXCH_CFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(bus, CXD56_SPI_DMAC_CHTYPE_RX, hdl, &conf);
        }

#endif

      /* Enable SPI */

      spi_pincontrol(bus, true);

      g_devhandle = nrc7292_register(devpath, spi, &g_nrc7292_lower);

      if (!g_devhandle)
        {
          wlerr("ERROR: Failed to register nrc7292 driver.\n");
          return -ENODEV;
        }
    }

  return OK;
}
