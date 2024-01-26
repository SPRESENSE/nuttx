/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_cxd5602pwbimu_spi.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

#include "cxd56_spi.h"
#include <arch/chip/pin.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define PIN_CXD5602PWBIMU_SPI_DRDY PIN_I2S0_DATA_OUT
#define PIN_CXD5602PWBIMU_SPI_CSX PIN_SEN_IRQ_IN
#define SETUP_PIN_INPUT(pin) do { \
  board_gpio_write(pin, -1); \
  board_gpio_config(pin, 0, true, false, PIN_PULLDOWN); \
} while (0)
#define SETUP_PIN_OUTPUT(pin) do { \
  board_gpio_write(pin, -1); \
  board_gpio_config(pin, 0, false, true, PIN_FLOAT); \
} while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_cxd5602pwbimu_intconfig_s
{
  /* Interrupt configuration structure as seen by the CXD5602PWBIMU driver */

  cxd5602pwbimu_intconfig_t config;

  /* Additional private definitions only known to this driver */

  xcpt_t isr;   /* ISR Handler */
};

struct cxd56_cxd5602pwbimu_csxconfig_s
{
  /* Interrupt configuration structure as seen by the CXD5602PWBIMU driver */

  cxd5602pwbimu_csxconfig_t config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  cxd5602pwbimu_irq_attach(cxd5602pwbimu_intconfig_t *state,
                                     xcpt_t isr);
static void cxd5602pwbimu_irq_enable(const cxd5602pwbimu_intconfig_t *state,
                                     bool enable);
static int  cxd5602pwbimu_irq_readlv(const cxd5602pwbimu_intconfig_t *state);
static void cxd5602pwbimu_csx_toggle(const cxd5602pwbimu_csxconfig_t *state,
                                     bool pol);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_cxd5602pwbimu_intconfig_s g_intconfig =
{
  .config =
  {
    .irq_attach = cxd5602pwbimu_irq_attach,
    .irq_enable = cxd5602pwbimu_irq_enable,
    .irq_readlv = cxd5602pwbimu_irq_readlv,
  },
};

static struct cxd56_cxd5602pwbimu_csxconfig_s g_csxconfig =
{
  .config =
  {
    .csx_toggle = cxd5602pwbimu_csx_toggle,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the CXD5602PWBIMU interrupt handler to the GPIO interrupt */

static int cxd5602pwbimu_irq_attach(cxd5602pwbimu_intconfig_t *state,
                                    xcpt_t isr)
{
  struct cxd56_cxd5602pwbimu_intconfig_s *priv =
             (struct cxd56_cxd5602pwbimu_intconfig_s *)state;
  irqstate_t flags;

  sninfo("cxd5602pwbimu_irq_attach\n");

  flags = enter_critical_section();

  priv->isr = isr;

  board_gpio_intconfig(PIN_CXD5602PWBIMU_SPI_DRDY, INT_RISING_EDGE, true,
                       isr);

  leave_critical_section(flags);

  return OK;
}

/* Enable or disable the GPIO interrupt */

static void cxd5602pwbimu_irq_enable(const cxd5602pwbimu_intconfig_t *state,
                                     bool enable)
{
  sninfo("%d\n", enable);

  board_gpio_int(PIN_CXD5602PWBIMU_SPI_DRDY, enable);
}

/* Read interrupt pin level */

static int cxd5602pwbimu_irq_readlv(const cxd5602pwbimu_intconfig_t *state)
{
  int pin_lv;

  pin_lv = board_gpio_read(PIN_CXD5602PWBIMU_SPI_DRDY);

  sninfo("%d\n", pin_lv);

  return pin_lv;
}

/* Toggle the csx pin level */

static void cxd5602pwbimu_csx_toggle(const cxd5602pwbimu_csxconfig_t *state,
                                     bool pol)
{
  sninfo("%d\n", pol);

  /* delay for csx rising edge */

  if (pol == true)
    {
      up_udelay(5);
    }

  /* toggle csx pin */

  board_gpio_write(PIN_CXD5602PWBIMU_SPI_CSX, pol);

  /* delay for csx falling edge */

  if (pol == false)
    {
      up_udelay(5);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_CXD56_SPI) && defined(CONFIG_SENSORS_CXD5602PWBIMU)

int board_cxd5602pwbimu_initialize(int bus)
{
  int ret;
  struct spi_dev_s *spi;

  sninfo("Initializing CXD5602PWBIMU..\n");

  /* Initialize pins */

  /* CSX pin */

  SETUP_PIN_OUTPUT(PIN_CXD5602PWBIMU_SPI_CSX);
  g_csxconfig.config.csx_toggle(&g_csxconfig.config, true);

  /* INT pin */

  SETUP_PIN_INPUT(PIN_CXD5602PWBIMU_SPI_DRDY);

  /* Initialize spi device */

  spi = cxd56_spibus_initialize(bus);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize spi%d.\n", bus);
      return -ENODEV;
    }

  ret = cxd5602pwbimu_register("/dev/cxd5602pwbimu0", spi,
                               &g_intconfig.config, &g_csxconfig.config);
  if (ret < 0)
    {
      snerr("Error registering CXD5602PWBIMU\n");
    }

  return ret;
}

#endif
