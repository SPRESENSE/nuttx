/****************************************************************************
 * drivers/sensors/cxd5602pwbimu.c
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
#include <stdlib.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <poll.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <nuttx/mutex.h>

#if defined(CONFIG_SENSORS_CXD5602PWBIMU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVID               0x5A

/* SPI Address */

#define CXD5602PWBIMU_FSR             (0x01) /* Full Scale */
#define CXD5602PWBIMU_ODR             (0x02) /* Output Data Rate and PMODE*/
#define CXD5602PWBIMU_OUTPUT_ENABLE   (0x03) /* Output Enable */
#define CXD5602PWBIMU_OUTPUT_MODE     (0x04) /* Output Mode */
#define CXD5602PWBIMU_OUTPUT_IF       (0x05) /* Output Interface */
#define CXD5602PWBIMU_IMU_CHOICE      (0x06) /* Enable IMU */
#define CXD5602PWBIMU_DRDY            (0x07) /* Data Ready */
#define CXD5602PWBIMU_UART_ASCII      (0x08) /* UART Output Format */
#define CXD5602PWBIMU_OUTPUT_LATEST   (0x09) /* Output Latest */
#define CXD5602PWBIMU_UART_CLKDEV     (0x20) /* UART Clock Devider */
#define CXD5602PWBIMU_UART_CLKOS      (0x21) /* UART Clock Oversample */
#define CXD5602PWBIMU_UART_INIT       (0x22) /* UART Initialize */
#define CXD5602PWBIMU_BOARD_NUM       (0x30) /* CXD5602PWBIMU Board Number */
#define CXD5602PWBIMU_BOARD_FLAG      (0x31) /* CXD5602PWBIMU Board Flag */
#define CXD5602PWBIMU_BOARD_REGSET    (0x32) /* CXD5602PWBIMU Board Register Set */
#define CXD5602PWBIMU_BOARD_REGCHK    (0x33) /* CXD5602PWBIMU Board Register Check */
#define CXD5602PWBIMU_CALIB_WRITE     (0x52) /* Write Calibration Coef */
#define CXD5602PWBIMU_DUMMY_WRITE     (0x61) /* Write Dummy Data */
#define CXD5602PWBIMU_INT_PERIOD      (0x62) /* Use Interrupt Period for Calib*/
#define CXD5602PWBIMU_CNF_ENABLE      (0x63) /* CompNotchFilter Enable */
#define CXD5602PWBIMU_FIFO_SIZE       (0x70) /* Output FIFO SIZE */
#define CXD5602PWBIMU_ERROR           (0x79) /* ERROR */
#define CXD5602PWBIMU_WHOAMI          (0x7A) /* Device ID */

/* Register 0x01 - ACCEL Full Scale */

#define ACCEL_FS_02_G   (0x00 << 4) /* Set ACCEL FullScale +/-2G */
#define ACCEL_FS_04_G   (0x01 << 4) /* Set ACCEL FullScale +/-4G */
#define ACCEL_FS_08_G   (0x02 << 4) /* Set ACCEL FullScale +/-8G */
#define ACCEL_FS_16_G   (0x03 << 4) /* Set ACCEL FullScale +/-16G */

/* Register 0x01 - GYRO Full Scale */

#define GYRO_FS_0125_DPS   (0x00) /* Set GYRO FullScale +/-125dps */
#define GYRO_FS_0250_DPS   (0x01) /* Set GYRO FullScale +/-250dps */
#define GYRO_FS_0500_DPS   (0x02) /* Set GYRO FullScale +/-500dps */
#define GYRO_FS_1000_DPS   (0x03) /* Set GYRO FullScale +/-1000dps */
#define GYRO_FS_2000_DPS   (0x04) /* Set GYRO FullScale +/-2000dps */
#define GYRO_FS_4000_DPS   (0x05) /* Set GYRO FullScale +/-4000dps */

/* Register 0x02 - ACCEL and GYRO ODR */

#define ODR_0015_HZ (0x00) /* Set ACCEL and GYRO ODR 15Hz */
#define ODR_0030_HZ (0x01) /* Set ACCEL and GYRO ODR 30Hz */
#define ODR_0060_HZ (0x02) /* Set ACCEL and GYRO ODR 60Hz */
#define ODR_0120_HZ (0x03) /* Set ACCEL and GYRO ODR 120Hz */
#define ODR_0240_HZ (0x04) /* Set ACCEL and GYRO ODR 240Hz */
#define ODR_0480_HZ (0x05) /* Set ACCEL and GYRO ODR 480Hz */
#define ODR_0960_HZ (0x06) /* Set ACCEL and GYRO ODR 960Hz */
#define ODR_1920_HZ (0x07) /* Set ACCEL and GYRO ODR 1920Hz */

/* Register 0x03 - Output ENABLE */

#define OUTPUT_DISABLE  (0x00) /* Disable 6axis data output */
#define OUTPUT_ENABLE   (0x01) /* Enable 6axis data output */

/* Register 0x04 - Output Mode */

#define OUTPUT_MODE_SYNTHESIS (0x00) /* Multi IMU Synthesis data mode */
#define OUTPUT_MODE_RAW       (0x02) /* Each IMU raw data mode */

/* Register 0x05 - Output Interface */

#define OUTPUT_IF_SPI   (0x01) /* Set output interface to SPI */

/* Register 0x07 - Data Ready */

#define DRDY_OFF    (0x00) /* Use DRDY signal */
#define DRDY_ON     (0x01) /* Do not use DRDY signal */

/* Register 0x09 - Output Latest */

#define OUTPUT_LATEST_OFF   (0x00) /* FIFO mode */
#define OUTPUT_LATEST_ON    (0x01) /* Resister mode */

/* Address for Data Read */

/* You can read 6axis data at any address
 * in the case of setting CXD5602PWBIMU_OUTPUT_ENABLE reg to OUTPUT_ENABLE
 */
#define CXD5602PWBIMU_DATA_READ_ADD  (0x00) 

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd5602pwbimu_dev_s
{
  FAR struct spi_dev_s *spi;                                  /* SPI interface */
  FAR cxd5602pwbimu_intconfig_t *intconfig;                   /* Interrupt interface */
  FAR cxd5602pwbimu_csxconfig_t *csxconfig;                   /* CSX interface */
  mutex_t devlock;                                            /* Device exclusion control */
  FAR struct pollfd *fds[CONFIG_CXD5602PWBIMU_NPOLLWAITERS];  /* Pooling interface */
  volatile bool cxd5602pwbimu_ready;                          /* Data ready flag */
  cxd5602pwbimu_packet_st_t cxd5602pwbimu_buff;               /* Data buffer */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t cxd5602pwbimu_getreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                     uint8_t regaddr);
static void cxd5602pwbimu_putreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                  uint8_t regaddr, uint8_t regval);
static void cxd5602pwbimu_getregs(FAR struct cxd5602pwbimu_dev_s *priv,
                                  uint8_t regaddr, FAR uint8_t *regval,
                                  int len);

/* Character driver methods */

static int     cxd5602pwbimu_open(FAR struct file *filep);
static int     cxd5602pwbimu_close(FAR struct file *filep);
static ssize_t cxd5602pwbimu_read(FAR struct file *filep, FAR char *buffer,
                                  size_t len);
static int     cxd5602pwbimu_ioctl(FAR struct file *filep, int cmd,
                                   unsigned long arg);
static int     cxd5602pwbimu_poll(FAR struct file *filep,
                                  FAR struct pollfd *fds, bool setup);
static int     cxd5602pwbimu_checkid(FAR struct cxd5602pwbimu_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct cxd5602pwbimu_dev_s *g_priv = NULL;

static uint8_t g_output_6axisdata;

/* This the vtable that supports the character driver interface */

static const struct file_operations g_cxd5602pwbimufops =
{
  cxd5602pwbimu_open,   /* open */
  cxd5602pwbimu_close,  /* close */
  cxd5602pwbimu_read,   /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  cxd5602pwbimu_ioctl,  /* ioctl */
  NULL,                 /* mmap */
  NULL,                 /* truncate */
  cxd5602pwbimu_poll    /* poll */
};

/****************************************************************************
 * Name: cxd5602pwbimu_decoder
 *
 * Description:
 *   decode spi receive data to CXD5602PWBIMU packet data format
 *   designed for unaligned access
 *
 ****************************************************************************/

static void cxd5602pwbimu_decoder(FAR uint8_t *spi_buffer,
                                  FAR cxd5602pwbimu_packet_st_t *data_buffer)
{
  data_buffer->id = spi_buffer[0];
  data_buffer->sensor_time = *(uint32_t *)&spi_buffer[1];
  data_buffer->temp = *(float *)&spi_buffer[5];
  data_buffer->gx = *(float *)&spi_buffer[9];
  data_buffer->gy = *(float *)&spi_buffer[13];
  data_buffer->gz = *(float *)&spi_buffer[17];
  data_buffer->ax = *(float *)&spi_buffer[21];
  data_buffer->ay = *(float *)&spi_buffer[25];
  data_buffer->az = *(float *)&spi_buffer[29];
}

/****************************************************************************
 * Name: cxd5602pwbimu_configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void cxd5602pwbimu_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for CXD5602PWBIMU */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 1000000);
}

/****************************************************************************
 * Name: cxd5602pwbimu_getreg8
 *
 * Description:
 *   Read from an 8-bit CXD5602PWBIMU register
 *
 ****************************************************************************/

static uint8_t cxd5602pwbimu_getreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                     uint8_t regaddr)
{
  uint8_t regval[2];

  /* If CXD5602PWBIMU is in the 6axisdata output mode
   * register read is not available
   */

  if (g_output_6axisdata)
    {
      return EPERM;
    }

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  cxd5602pwbimu_configspi(priv->spi);

  /* Select the CXD5602PWBIMU */

  /* put the code which control csx pin for CXD5602PWBIMU to low */

  priv->csxconfig->csx_toggle(priv->csxconfig, false);

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, 2);

  /* Deselect the CXD5602PWBIMU */

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), false);

  /* put the code which control csx pin for CXD5602PWBIMU to high */

  priv->csxconfig->csx_toggle(priv->csxconfig, true);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  /* The first byte has to be dropped */

  return regval[1];
}

/****************************************************************************
 * Name: cxd5602pwbimu_putreg8
 *
 * Description:
 *   Write a value to an 8-bit CXD5602PWBIMU register
 *
 ****************************************************************************/

static void cxd5602pwbimu_putreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                  uint8_t regaddr, uint8_t regval)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  cxd5602pwbimu_configspi(priv->spi);

  /* Select the CXD5602PWBIMU */

  /* put the code which control csx pin for CXD5602PWBIMU to low */

  priv->csxconfig->csx_toggle(priv->csxconfig, false);

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the CXD5602PWBIMU */

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), false);

  /* put the code which control csx pin for CXD5602PWBIMU to high */

  priv->csxconfig->csx_toggle(priv->csxconfig, true);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: cxd5602pwbimu_getregs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

static void cxd5602pwbimu_getregs(FAR struct cxd5602pwbimu_dev_s *priv,
                                  uint8_t regaddr, FAR uint8_t *regval,
                                  int len)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  cxd5602pwbimu_configspi(priv->spi);

  /* Select the CXD5602PWBIMU */

  /* put the code which control csx pin for CXD5602PWBIMU to low */

  priv->csxconfig->csx_toggle(priv->csxconfig, false);

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), true);

  /* Send register to read and get the next N bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the CXD5602PWBIMU */

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), false);

  /* put the code which control csx pin for CXD5602PWBIMU to high */

  priv->csxconfig->csx_toggle(priv->csxconfig, true);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: cxd5602pwbimu_init
 *
 * Description:
 *   initialize CXD5602PWBIMU
 *
 ****************************************************************************/

static void cxd5602pwbimu_init(FAR struct cxd5602pwbimu_dev_s *priv)
{
  /* Set ACCEL and GYRO FullScale */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_FSR,
                        ACCEL_FS_16_G + GYRO_FS_1000_DPS);
  up_mdelay(100);

  /* Set ACCEL and GYRO ODR */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_ODR, ODR_0120_HZ);
  up_mdelay(100);

  /* Set data output mode */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_OUTPUT_MODE,
                        OUTPUT_MODE_SYNTHESIS);
  up_mdelay(100);

  /* Set data output interface */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_OUTPUT_IF, OUTPUT_IF_SPI);
  up_mdelay(100);

  /* Set enable flag for each imu */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_IMU_CHOICE, 0xff);
  up_mdelay(100);

  /* Set use or do not use DRDY */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_DRDY, DRDY_ON);
  up_mdelay(100);

  /* Set fifo mode */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_OUTPUT_LATEST,
                        OUTPUT_LATEST_OFF);
  up_mdelay(1000);
}

/****************************************************************************
 * Name: cxd5602pwbimu_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int cxd5602pwbimu_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv = inode->i_private;

  /* Initialize CXD5602PWBIMU */

  cxd5602pwbimu_init(priv);

  /* Enable interruput */

  priv->intconfig->irq_enable(priv->intconfig, true);

  /* Start output 6axis data */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_OUTPUT_ENABLE, OUTPUT_ENABLE);
  g_output_6axisdata = 1;
  up_mdelay(100);

  return OK;
}

/****************************************************************************
 * Name: cxd5602pwbimu_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int cxd5602pwbimu_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv = inode->i_private;

  /* Stop output 6axis data and power down */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_OUTPUT_ENABLE, OUTPUT_DISABLE);
  g_output_6axisdata = 0;
  up_mdelay(100);

  return OK;
}

/****************************************************************************
 * Name: cxd5602pwbimu_read_pkt
 *
 * Description:
 *   Obtain a CXD5602PWBIMU data packet from the device.
 *
 ****************************************************************************/

static ssize_t cxd5602pwbimu_read_pkt(FAR cxd5602pwbimu_packet_st_t *buffer,
                                      size_t len)
{
  uint8_t rcv_buffer[40];

  if (len < sizeof(cxd5602pwbimu_packet_st_t))
    {
      snerr("Expected buffer size is %d\n",
            sizeof(cxd5602pwbimu_packet_st_t));
      return 0;
    }

  cxd5602pwbimu_getregs(g_priv, CXD5602PWBIMU_DATA_READ_ADD, rcv_buffer, 33);
  cxd5602pwbimu_decoder(rcv_buffer, buffer);

  return len;
}

/****************************************************************************
 * Name: cxd5602pwbimu_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t cxd5602pwbimu_read(FAR struct file *filep, FAR char *buffer,
                                  size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv = inode->i_private;
  FAR cxd5602pwbimu_packet_st_t *p;
  int ret;
  uint32_t flags;

  p = (FAR cxd5602pwbimu_packet_st_t *)buffer;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  if (len < sizeof(cxd5602pwbimu_packet_st_t))
    {
      snerr("Expected buffer size is %d\n",
            sizeof(cxd5602pwbimu_packet_st_t));
      return 0;
    }

  while (!priv->cxd5602pwbimu_ready)
    {
    }

  flags = enter_critical_section();

  memcpy(p, &priv->cxd5602pwbimu_buff, sizeof(cxd5602pwbimu_packet_st_t));
  priv->cxd5602pwbimu_ready = false;

  leave_critical_section(flags);

  nxmutex_unlock(&priv->devlock);
  return len;
}

/****************************************************************************
 * Name: cxd5602pwbimu_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int cxd5602pwbimu_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: cxd5602pwbimu_poll
 *
 * Description:
 *   Polling method for CXD5602PWBIMU data ready
 *
 ****************************************************************************/

static int cxd5602pwbimu_poll(FAR struct file *filep, FAR struct pollfd *fds,
                              bool setup)
{
  FAR struct inode *inode;
  FAR struct cxd5602pwbimu_dev_s *priv;
  uint32_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_CXD5602PWBIMU_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_CXD5602PWBIMU_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      flags = enter_critical_section();
      if (priv->cxd5602pwbimu_ready)
        {
          poll_notify(priv->fds, CONFIG_CXD5602PWBIMU_NPOLLWAITERS, POLLIN);
        }

      leave_critical_section(flags);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_checkid
 *
 * Description:
 *   Read and verify the CXD5602PWBIMU chip ID
 *
 ****************************************************************************/

static int cxd5602pwbimu_checkid(FAR struct cxd5602pwbimu_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = cxd5602pwbimu_getreg8(priv, CXD5602PWBIMU_WHOAMI);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd5602pwbimu_int_handler
 *
 * Description:
 *   Interrupt handler for CXD5602PWBIMU data ready
 *
 ****************************************************************************/

static int cxd5602pwbimu_int_handler(int irq, FAR void *context,
                                     FAR void *arg)
{
  int ret;
  int result = 0;

  while (g_priv->intconfig->irq_readlv(g_priv->intconfig))
    {
      ret = cxd5602pwbimu_read_pkt(&g_priv->cxd5602pwbimu_buff,
                                   sizeof(cxd5602pwbimu_packet_st_t));
      if (ret != sizeof(cxd5602pwbimu_packet_st_t))
        {
          snerr("Read failed.\n");
          result = -1;
          break;
        }
    }

  g_priv->cxd5602pwbimu_ready = true;
  poll_notify(g_priv->fds,
              CONFIG_CXD5602PWBIMU_NPOLLWAITERS, POLLIN);

  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd5602pwbimu_register
 *
 * Description:
 *   Register the CXD5602PWBIMU character device as 'devpath'
 *
 * Input Parameters:
 *   devpath   - The full path to the driver to register. E.g., "/dev/imu0"
 *   dev       - An instance of the SPI interface to use to communicate with
 *               CXD5602PWBIMU
 *   intconfig - An instance of the interrupt configuration data structure
 *   csxconfig - An instance of the csx pin configuration data structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd5602pwbimu_register(FAR const char *devpath,
                           FAR struct spi_dev_s *dev,
                           FAR cxd5602pwbimu_intconfig_t *intconfig,
                           FAR cxd5602pwbimu_csxconfig_t *csxconfig)
{
  FAR struct cxd5602pwbimu_dev_s *priv;
  size_t dev_size = sizeof(struct cxd5602pwbimu_dev_s);
  int ret;

  priv = (FAR struct cxd5602pwbimu_dev_s *)kmm_malloc(dev_size);
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = dev;
  priv->intconfig = intconfig;
  priv->csxconfig = csxconfig;
  nxmutex_init(&priv->devlock);
  for (int i = 0; i < CONFIG_CXD5602PWBIMU_NPOLLWAITERS; i++)
    {
      priv->fds[i] = NULL;
    }

  priv->cxd5602pwbimu_ready = false;

  g_priv = priv;

  ret = cxd5602pwbimu_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_cxd5602pwbimufops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  priv->intconfig->irq_attach(priv->intconfig, cxd5602pwbimu_int_handler);
  priv->intconfig->irq_enable(priv->intconfig, false);

  sninfo("CXD5602PWBIMU driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_SENSORS_CXD5602PWBIMU */
