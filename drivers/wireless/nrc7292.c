/****************************************************************************
 * drivers/wireless/nrc7292.c
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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/wireless/nrc7292.h>
#include <nuttx/net/nrc7292.h>
#include <nuttx/net/netdev.h>

#include "nrc7292.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_MAXFREQ  CONFIG_WL_NRC7292_SPI_FREQUENCY

#define MAX_PKT_LEN  1500
#define RING_BUFFER_SIZE 4096

#define SPI_TRANS_RETRY_MAX 3
#define SPI_CLEAR_LOOP_MAX 64

#define WLAN_IFNAME       "wlan0"
#define WLAN_IFNAME_LEN   5

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct atcmd_ring_buffer_s
{
  mutex_t lock;
  sem_t full_sem;
  sem_t filled_sem;
  uint8_t buf[RING_BUFFER_SIZE];
  size_t  buf_size;
  int     head;
  int     tail;
  bool    full;
};

struct nrc7292_dev_s
{
  FAR char             *path;
  FAR struct pollfd    *pfd;
  FAR struct spi_dev_s *spi;
  mutex_t              rw_lock;
  mutex_t              refslock;
  uint8_t              crefs;
  bool                 int_enabled;
  int                  recv_thread_pid;

  struct net_driver_s  net_dev;
  struct net_driver_s  net_relay_dev;
  FAR const struct nrc7292_lower_s *lower;

  sem_t                      recv_thread_sem;
  struct spi_slot_info_s     slot_info[2];
  struct spi_queue_status_s  queue_status;
  struct spi_sys_s           sys_info;
  struct spi_hw_status_s     hw_status;
  struct atcmd_ring_buffer_s ring_buf;
  uint8_t                    slot_buf_tx[HSPI_SLOT_SIZE_MAX];
  uint8_t                    slot_buf_rx[HSPI_SLOT_SIZE_MAX];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t nrc7292_open(FAR struct file *filep);
static ssize_t nrc7292_close(FAR struct file *filep);
static ssize_t nrc7292_read(FAR struct file *filep, FAR char *buff,
                            size_t len);
static ssize_t nrc7292_write(FAR struct file *filep, FAR const char *buff,
                             size_t len);
static int     nrc7292_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int     nrc7292_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);

static int     nrc7292_spi_init(FAR struct nrc7292_dev_s *dev);
static int     nrc7292_start(FAR struct nrc7292_dev_s *dev);
static int     nrc7292_start_rxthread(FAR struct nrc7292_dev_s *dev);
static int     nrc7292_irq(int irq, FAR void *context, FAR void *arg);
static void    nrc7292_update_queue_status(FAR struct nrc7292_dev_s *dev);
static int     nrc7292_write_regs(FAR struct nrc7292_dev_s *dev,
                                  uint8_t addr, uint8_t value);
static int     nrc7292_write_slot_data(FAR struct nrc7292_dev_s *dev,
                                       FAR uint8_t *buf, size_t len);
static int     nrc7292_read_slot_data(FAR struct nrc7292_dev_s *dev);
static void    nrc7292_print_queue_status(FAR struct nrc7292_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_nrc7292_fops =
{
  nrc7292_open,  /* open */
  nrc7292_close, /* close */
  nrc7292_read,  /* read */
  nrc7292_write, /* write */
  NULL,          /* seek */
  nrc7292_ioctl, /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  nrc7292_poll   /* poll */
};

FAR struct nrc7292_dev_s *g_nrc7292_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_atcmd_ring_buffer
 ****************************************************************************/

static void init_atcmd_ring_buffer(FAR struct atcmd_ring_buffer_s *ring)
{
  memset(ring->buf, 0, sizeof(ring->buf));
  ring->buf_size = RING_BUFFER_SIZE;
  ring->head = 0;
  ring->tail = 0;
  ring->full = false;
  nxmutex_init(&ring->lock);
  nxsem_init(&ring->full_sem, 0, 0);
  nxsem_init(&ring->filled_sem, 0, 0);
}

/****************************************************************************
 * Name: deinit_atcmd_ring_buffer
 ****************************************************************************/

static void deinit_atcmd_ring_buffer(FAR struct atcmd_ring_buffer_s *ring)
{
  nxmutex_destroy(&ring->lock);
  nxsem_destroy(&ring->full_sem);
  nxsem_destroy(&ring->filled_sem);
}

/****************************************************************************
 * Name: get_buffered_size
 ****************************************************************************/

static size_t get_buffered_size(FAR struct atcmd_ring_buffer_s *ring)
{
  size_t size;

  if (!ring->full)
    {
      if (ring->head <= ring->tail)
        {
          size = (ring->tail - ring->head);
        }
      else
        {
          size = (ring->tail + ring->buf_size - ring->head);
        }
    }
  else
    {
      size = ring->buf_size;
    }

  return size;
}

/****************************************************************************
 * Name: enqueue_data
 ****************************************************************************/

static ssize_t enqueue_data(struct nrc7292_dev_s *dev,
                            FAR uint8_t *data, size_t len)
{
  FAR struct atcmd_ring_buffer_s *ring = &dev->ring_buf;
  size_t buffered_size;
  size_t first_len;
  size_t second_len;
  int semcount;
  bool is_full;
  int ret;

  if (data == NULL || len == 0)
    {
      return -EINVAL;
    }

  while (true)
    {
      nxmutex_lock(&ring->lock);
      is_full = ring->full;
      nxmutex_unlock(&ring->lock);

      if (!is_full)
        {
          break;
        }

      nxsem_wait_uninterruptible(&ring->full_sem);
    }

  ret = nxmutex_lock(&ring->lock);

  if (ret < 0 || ring->full)
    {
      len = -EAGAIN;
      goto error;
    }

  buffered_size = get_buffered_size(ring);

  if (ring->buf_size < buffered_size + len)
    {
      len = ring->buf_size - buffered_size;
    }

  if (ring->tail + len <= ring->buf_size)
    {
      memcpy(&ring->buf[ring->tail], &data[0], len);
    }
  else
    {
      first_len  = ring->buf_size - ring->tail;
      second_len = len - first_len;
      memcpy(&ring->buf[ring->tail], &data[0], first_len);
      memcpy(&ring->buf[0], &data[first_len], second_len);
    }

  ring->tail = (ring->tail + len) % ring->buf_size;
  ring->full = ring->tail == ring->head;

  ret = nxsem_get_value(&ring->filled_sem, &semcount);

  if (ret != OK)
    {
      semcount = 0;
    }

  if (semcount <= 0)
    {
      nxsem_post(&ring->filled_sem);
    }

  if (dev->pfd)
    {
      poll_notify(&dev->pfd, 1, POLLIN);
    }

error:
  nxmutex_unlock(&ring->lock);

  return len;
}

/****************************************************************************
 * Name: dequeue_data
 ****************************************************************************/

static ssize_t dequeue_data(struct nrc7292_dev_s *dev,
                            FAR uint8_t *buf, size_t len)
{
  FAR struct atcmd_ring_buffer_s *ring = &dev->ring_buf;
  size_t buffered_size;
  size_t first_len;
  size_t second_len;
  int semcount;
  bool is_empty;
  int ret;

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  while (true)
    {
      nxmutex_lock(&ring->lock);
      is_empty = !ring->full && (ring->head == ring->tail);
      nxmutex_unlock(&ring->lock);

      if (!is_empty)
        {
          break;
        }

      nxsem_wait_uninterruptible(&ring->filled_sem);
    }

  ret = nxmutex_lock(&ring->lock);

  if (ret < 0 || (!ring->full && (ring->head == ring->tail)))
    {
      len = -EAGAIN;
      goto error;
    }

  buffered_size = get_buffered_size(ring);

  if (buffered_size < len)
    {
      len = buffered_size;
    }

  if (ring->head + len <= ring->buf_size)
    {
      memcpy(&buf[0], &ring->buf[ring->head], len);
    }
  else
    {
      first_len  = ring->buf_size - ring->head;
      second_len = len - first_len;
      memcpy(&buf[0], &ring->buf[ring->head], first_len);
      memcpy(&buf[first_len], &ring->buf[0], second_len);
    }

  ring->head = (ring->head + len) % ring->buf_size;

  if (ring->full)
    {
      ring->full = false;

      ret = nxsem_get_value(&ring->full_sem, &semcount);

      if (ret != OK)
        {
          semcount = 0;
        }

      if (semcount <= 0)
        {
          nxsem_post(&ring->full_sem);
        }
    }

error:
  nxmutex_unlock(&ring->lock);

  return len;
}

/****************************************************************************
 * Name: nrc7292_init
 ****************************************************************************/

static int nrc7292_init(FAR struct nrc7292_dev_s *dev)
{
  int ret;

  nxmutex_init(&dev->rw_lock);

  init_atcmd_ring_buffer(&dev->ring_buf);

  dev->lower->reset(true);
  up_udelay(10 * 1000);
  dev->lower->reset(false);

  ret = nrc7292_start(dev);

  if (ret == 0)
    {
      dev->lower->attach(nrc7292_irq, dev);
      dev->lower->enable();
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_deinit
 ****************************************************************************/

static int nrc7292_deinit(FAR struct nrc7292_dev_s *dev)
{
  dev->lower->disable();
  dev->lower->attach(NULL, dev);

  /* reset board */

  dev->lower->reset(true);

  nxmutex_destroy(&dev->rw_lock);

  deinit_atcmd_ring_buffer(&dev->ring_buf);

  return OK;
}

/****************************************************************************
 * Name: nrc7292_open
 ****************************************************************************/

static ssize_t nrc7292_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct nrc7292_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  inode = filep->f_inode;

  dev = inode->i_private;
  DEBUGASSERT(dev);

  nxmutex_lock(&dev->refslock);

  if (dev->crefs > 0)
    {
      nxmutex_unlock(&dev->refslock);
      return -EPERM;
    }

  /* Increment the count of open references on the driver */

  dev->crefs++;

  nxmutex_unlock(&dev->refslock);

  ret = nrc7292_init(dev);

  if (ret < 0)
    {
      wlerr("Failed to initialize driver: %d\n", ret);
      nxmutex_lock(&dev->refslock);
      dev->crefs--;
      nxmutex_unlock(&dev->refslock);
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_close
 ****************************************************************************/

static ssize_t nrc7292_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct nrc7292_dev_s *dev;
  int ret;

  /* Get our private data structure */

  inode = filep->f_inode;

  dev = inode->i_private;
  DEBUGASSERT(dev);

  nxmutex_lock(&dev->refslock);

  if (dev->crefs == 0)
    {
      nxmutex_unlock(&dev->refslock);
      return -EPERM;
    }

  /* Decrement the count of open references on the driver */

  dev->crefs--;

  nxmutex_unlock(&dev->refslock);

  ret = nrc7292_deinit(dev);

  return ret;
}

/****************************************************************************
 * Name: nrc7292_read
 ****************************************************************************/

static ssize_t nrc7292_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode *inode;
  FAR struct nrc7292_dev_s *dev;
  ssize_t dq_len;

  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  dq_len = dequeue_data(dev, (FAR uint8_t *) buffer, len);

  return dq_len;
}

/****************************************************************************
 * Name: nrc7292_write
 ****************************************************************************/

static ssize_t nrc7292_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  FAR struct nrc7292_dev_s *dev = g_nrc7292_dev;
  int written_len = 0;
  int ret;

  ret = nxmutex_lock(&dev->rw_lock);

  if (ret < 0)
    {
      return ret;
    }

  written_len = nrc7292_write_slot_data(g_nrc7292_dev,
                                        (FAR uint8_t *) buffer, len);
  nxmutex_unlock(&dev->rw_lock);

  return written_len;
}

/****************************************************************************
 * Name: nrc7292_spi_init
 ****************************************************************************/

static int nrc7292_spi_init(FAR struct nrc7292_dev_s *dev)
{
  SPI_LOCK(dev->spi, true);

  /* SPI settings (mode0/8bits/freq) */

  SPI_SETMODE(dev->spi, SPIDEV_MODE0);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, SPI_MAXFREQ);

  SPI_LOCK(dev->spi, false);
  return 0;
}

/****************************************************************************
 * Name: nrc7292_ioctl
 ****************************************************************************/

static int nrc7292_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct ifreq *ifr;
  FAR struct sockaddr_in *inaddr;
  int ret = OK;

  ifr = (FAR struct ifreq *) arg;

  if (strncmp(ifr->ifr_name, WLAN_IFNAME, WLAN_IFNAME_LEN) == 0)
    {
      switch (cmd)
        {
        case SIOCSIFHWADDR:
          memcpy(g_nrc7292_dev->net_dev.d_mac.ether.ether_addr_octet,
                 ifr->ifr_hwaddr.sa_data, 6);
          memcpy(g_nrc7292_dev->net_relay_dev.d_mac.ether.ether_addr_octet,
                 ifr->ifr_hwaddr.sa_data, 6);

          /* flip G/L bit */

          g_nrc7292_dev->net_relay_dev.d_mac.ether.ether_addr_octet[0]
            ^= 0x02;
          break;

        case SIOCSIFADDR:
          inaddr = (FAR struct sockaddr_in *) &ifr->ifr_addr;
          g_nrc7292_dev->net_dev.d_ipaddr = inaddr->sin_addr.s_addr;
          break;

        case SIOCSIFNETMASK:
          inaddr = (FAR struct sockaddr_in *) &ifr->ifr_addr;
          g_nrc7292_dev->net_dev.d_netmask = inaddr->sin_addr.s_addr;
          break;

        case SIOCSIFDSTADDR:
          inaddr = (FAR struct sockaddr_in *) &ifr->ifr_addr;
          g_nrc7292_dev->net_dev.d_draddr = inaddr->sin_addr.s_addr;
          break;

        case SIOCGIFFLAGS:
          ifr->ifr_flags = g_nrc7292_dev->net_dev.d_flags;
          break;

        case SIOCSIFFLAGS:
          g_nrc7292_dev->net_dev.d_flags = ifr->ifr_flags;
          g_nrc7292_dev->net_relay_dev.d_flags = ifr->ifr_flags;
          break;

        default:
          ret = -EINVAL;
          break;
        }
    }
  else if (g_nrc7292_dev->net_relay_dev.d_ifname[0] != '\0')
    {
      switch (cmd)
        {
        case SIOCSIFDELIFACE:
          {
            ret = netdev_unregister(&g_nrc7292_dev->net_relay_dev);

            if (ret < 0)
              {
                wlerr("Failed to delete an interface.\n");
              }
            else
              {
                g_nrc7292_dev->net_relay_dev.d_ifname[0] = '\0';
              }
          }
          break;

        default:
          ret = -EINVAL;
          break;
        }
    }
  else
    {
      switch (cmd)
        {
        case SIOCSIFADDIFACE:
          {
            strncpy(g_nrc7292_dev->net_relay_dev.d_ifname,
                    ifr->ifr_name, IFNAMSIZ);
            ret = netdev_register(&g_nrc7292_dev->net_relay_dev,
                                  NET_LL_IEEE80211);

            if (ret < 0)
              {
                g_nrc7292_dev->net_relay_dev.d_ifname[0] = '\0';
              }
          }
          break;

        default:
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_poll
 ****************************************************************************/

static int nrc7292_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct nrc7292_dev_s *dev;
  size_t buffered_size = 0;
  int ret = OK;

  wlinfo("== setup:%d\n", (int)setup);

  DEBUGASSERT(fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  if (setup)
    {
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      if (dev->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;

      nxmutex_lock(&dev->ring_buf.lock);
      buffered_size = get_buffered_size(&dev->ring_buf);
      nxmutex_unlock(&dev->ring_buf.lock);

      if (0 < buffered_size)
        {
          poll_notify(&dev->pfd, 1, POLLIN);
        }
    }
  else
    {
      dev->pfd = NULL;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: crc7
 ****************************************************************************/

static uint8_t crc7(FAR uint8_t *data, int len)
{
  uint8_t crc = 0;
  int i;
  int j;

  for (i = 0 ; i < len ; i++)
    {
      crc ^= data[i];

      for (j = 0 ; j < 8 ; j++)
        {
          if (crc & 0x80)
            {
              crc ^= 0x89;
            }

          crc <<= 1;
        }
    }

  return crc >> 1;
}

/****************************************************************************
 * Name: put_reverse_order32
 ****************************************************************************/

static void put_reverse_order32(FAR uint8_t *dst, uint32_t value)
{
  *dst++ = (value >> 24) & 0x000000ff;
  *dst++ = (value >> 16) & 0x000000ff;
  *dst++ = (value >> 8) & 0x000000ff;
  *dst++ = value & 0x000000ff;
}

/****************************************************************************
 * Name: _nrc7292_write_regs
 ****************************************************************************/

static int _nrc7292_write_regs(FAR struct nrc7292_dev_s *dev,
                               uint8_t addr, uint8_t value)
{
  uint8_t cmd_tx[8];
  uint8_t cmd_rx[8];
  uint32_t cmd;

  cmd = HSPI_WRITE | HSPI_FIXED | HSPI_ADDR(addr) | HSPI_WDATA(value);

  put_reverse_order32(cmd_tx, cmd);

  cmd_tx[4] = (crc7(cmd_tx, 4) << 1) | 0x1;
  cmd_tx[5] = 0xff;

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_EXCHANGE(dev->spi, cmd_tx, cmd_rx, sizeof(cmd_tx));
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);

  if (addr != HSPI_REG_WAKEUP && addr != HSPI_REG_DEVICE_STATUS
      && cmd_rx[7] != HSPI_ACK)
    {
      wlwarn("[%s] try to read register but SPI ACK is invalid\n",
             __func__);
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: nrc7292_write_regs
 ****************************************************************************/

static int nrc7292_write_regs(FAR struct nrc7292_dev_s *dev,
                              uint8_t addr, uint8_t value)
{
  int ret;
  int i;

  for (i = 0; i < SPI_TRANS_RETRY_MAX; i++)
    {
      ret = _nrc7292_write_regs(dev, addr, value);

      if (ret == 0 || ret != -EIO)
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_read_regs
 ****************************************************************************/

static int _nrc7292_read_regs(FAR struct nrc7292_dev_s *dev,
                              uint8_t addr, FAR uint8_t *buf, size_t len)
{
  uint8_t cmd_tx[8];
  uint8_t cmd_rx[8];
  uint32_t cmd;
  uint32_t crc;

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  cmd = HSPI_READ | HSPI_ADDR(addr);

  if (1 < len)
    {
      cmd |= HSPI_BURST | HSPI_LEN(len);
    }
  else
    {
      cmd |= HSPI_FIXED | HSPI_LEN(0);
    }

  put_reverse_order32(cmd_tx, cmd);

  cmd_tx[4] = (crc7(cmd_tx, 4) << 1) | 0x1;
  cmd_tx[5] = 0xff;

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_EXCHANGE(dev->spi, cmd_tx, cmd_rx, sizeof(cmd_tx));

  if (1 < len)
    {
      SPI_RECVBLOCK(dev->spi, buf, len);
      SPI_RECVBLOCK(dev->spi, &crc, sizeof(crc));
    }

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);

  if (addr != HSPI_REG_WAKEUP && addr != HSPI_REG_DEVICE_STATUS
      && cmd_rx[7] != HSPI_ACK)
    {
      wlwarn("[%s] try to read register but SPI ACK is invalid\n",
             __func__);
      return -EIO;
    }

  if (len == 1)
    {
      buf[0] = cmd_rx[6];
    }

  return 0;
}

/****************************************************************************
 * Name: nrc7292_read_regs
 ****************************************************************************/

static int nrc7292_read_regs(FAR struct nrc7292_dev_s *dev,
                             uint8_t addr, FAR uint8_t *buf, size_t len)
{
  int ret;
  int i;

  for (i = 0; i < SPI_TRANS_RETRY_MAX; i++)
    {
      ret = _nrc7292_read_regs(dev, addr, buf, len);

      if (ret == 0 || ret != -EIO)
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_write_data
 ****************************************************************************/

static int _nrc7292_write_data(FAR struct nrc7292_dev_s *dev,
                               FAR uint8_t *buf, size_t len)
{
  uint8_t cmd_tx[8];
  uint8_t cmd_rx[8];
  uint32_t cmd;
  uint32_t crc;

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  cmd = HSPI_WRITE | HSPI_BURST | HSPI_FIXED
    | HSPI_ADDR(HSPI_REG_RXQ_WINDOW) | HSPI_LEN(len);

  put_reverse_order32(cmd_tx, cmd);

  cmd_tx[4] = (crc7(cmd_tx, 4) << 1) | 0x1;
  cmd_tx[5] = 0xff;

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_EXCHANGE(dev->spi, cmd_tx, cmd_rx, sizeof(cmd_tx));
  SPI_SNDBLOCK(dev->spi, buf, len);
  crc = 0xffffffff;
  SPI_SNDBLOCK(dev->spi, &crc, sizeof(crc));

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);

  if (cmd_rx[7] != HSPI_ACK)
    {
      wlwarn("[%s] try to read register but SPI ACK is invalid\n",
              __func__);
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: nrc7292_write_data()
 ****************************************************************************/

static int nrc7292_write_data(FAR struct nrc7292_dev_s *dev,
                              FAR uint8_t *buf, size_t len)
{
  int ret;
  int i;

  for (i = 0; i < SPI_TRANS_RETRY_MAX; i++)
    {
      ret = _nrc7292_write_data(dev, buf, len);

      if (ret == 0 || ret != -EIO)
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_write_slot_data()
 ****************************************************************************/

static int nrc7292_write_slot_data(FAR struct nrc7292_dev_s *dev,
                                   FAR uint8_t *buf, size_t len)
{
  FAR struct spi_slot_t *slot = (FAR struct spi_slot_t *) dev->slot_buf_tx;
  uint8_t slot_cnt;
  uint16_t slot_size;
  uint16_t slot_num;
  uint16_t rem_len;
  static uint16_t seq;
  int i;
  int ret;
  size_t offset = 0;

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  if (dev->queue_status.rxq.slot_cnt == 0)
    {
      nrc7292_update_queue_status(dev);
    }

  slot_cnt = dev->queue_status.rxq.slot_cnt;
  slot_size = dev->slot_info[RX_SLOT].size;
  slot_num = (len + (slot_size - HSPI_SLOT_HDR_SIZE - 1))
    / (slot_size - HSPI_SLOT_HDR_SIZE);

  if (slot_cnt < slot_num)
    {
      nrc7292_update_queue_status(dev);

      slot_cnt = dev->queue_status.rxq.slot_cnt;
    }

  if (slot_cnt < slot_num)
    {
      slot_num = slot_cnt;
      len = slot_cnt * (slot_size - HSPI_SLOT_HDR_SIZE);
    }

  slot->start[0] = 'H';
  slot->start[1] = 'S';

  rem_len = len;

  for (i = 0; i < slot_num; i++)
    {
      uint16_t _len;

      if (slot_size <= rem_len)
        {
          _len = slot_size - HSPI_SLOT_HDR_SIZE;
        }
      else
        {
          _len = rem_len;
          memset(slot->data + _len, 0,
                 HSPI_SLOT_SIZE_MAX - HSPI_SLOT_HDR_SIZE - _len);
        }

      seq = seq & HSPI_SLOT_SEQ_MAX;
      slot->len_seq = slot_len_seq(_len, seq);
      memcpy(slot->data, &buf[offset], _len);

      ret = nrc7292_write_data(dev, (FAR uint8_t *) slot, slot_size);

      if (ret < 0)
        {
          return ret;
        }

      rem_len -= _len;
      offset += _len;
      seq++;
    }

  dev->queue_status.rxq.slot_cnt -= slot_num;

  return len - rem_len;
}

/****************************************************************************
 * Name: nrc7292_read_data
 ****************************************************************************/

static int _nrc7292_read_data(FAR struct nrc7292_dev_s *dev,
                              FAR uint8_t *buf, size_t len)
{
  uint8_t cmd_tx[8];
  uint8_t cmd_rx[8];
  uint32_t cmd;
  uint32_t crc;

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  cmd = HSPI_READ | HSPI_BURST | HSPI_FIXED
    | HSPI_ADDR(HSPI_REG_TXQ_WINDOW) | HSPI_LEN(len);

  put_reverse_order32(cmd_tx, cmd);

  cmd_tx[4] = (crc7(cmd_tx, 4) << 1) | 0x1;
  cmd_tx[5] = 0xff;

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_EXCHANGE(dev->spi, cmd_tx, cmd_rx, sizeof(cmd_tx));
  SPI_RECVBLOCK(dev->spi, buf, len);
  SPI_RECVBLOCK(dev->spi, &crc, sizeof(crc));
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);

  if (cmd_rx[7] != HSPI_ACK)
    {
      wlwarn("[%s] try to read register but SPI ACK is invalid\n",
             __func__);
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: nrc7292_read_data
 ****************************************************************************/

static int nrc7292_read_data(FAR struct nrc7292_dev_s *dev,
                             FAR uint8_t *buf, size_t len)
{
  int ret;
  int i;

  for (i = 0; i < SPI_TRANS_RETRY_MAX; i++)
    {
      ret = _nrc7292_read_data(dev, buf, len);

      if (ret == 0 || ret != -EIO)
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrc7292_read_slot_data
 ****************************************************************************/

static int nrc7292_read_slot_data(FAR struct nrc7292_dev_s *dev)
{
  FAR struct spi_slot_t *slot = (FAR struct spi_slot_t *) dev->slot_buf_rx;
  uint8_t slot_cnt;
  uint16_t slot_size;
  int ret;
  size_t copied_len = 0;
  uint16_t slot_len;

  slot_cnt = dev->queue_status.txq.slot_cnt;
  slot_size = dev->slot_info[TX_SLOT].size;

  for (int i = 0; i < slot_cnt; i++)
    {
      ret = nrc7292_read_data(dev, (FAR uint8_t *) slot, slot_size);

      if (ret < 0)
        {
          return ret;
        }

      if (slot->start[0] != 'H' || slot->start[1] != 'S')
        {
          return -EBADMSG;
        }

      slot_len = get_slot_len(slot->len_seq);

      if (0 < slot_len)
        {
          enqueue_data(dev, slot->data, slot_len);
        }
      else
        {
          break;
        }
    }

  return copied_len;
}

/****************************************************************************
 * Name: nrc7292_irq
 ****************************************************************************/

static int nrc7292_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrc7292_dev_s *dev;

  DEBUGASSERT(arg != NULL);
  dev = (FAR struct nrc7292_dev_s *)arg;

  wlinfo(">>>>\n");

  nxsem_post(&dev->recv_thread_sem);

  dev->lower->disable();

  return 0;
}

/****************************************************************************
 * Name: nrc7292_set_eirq_config
 ****************************************************************************/

static void nrc7292_set_eirq_config(FAR struct nrc7292_dev_s *dev)
{
  nrc7292_write_regs(dev,
                     HSPI_REG_EIRQ_MODE,
                     HSPI_EIRQ_IO_ENABLE | HSPI_EIRQ_HIGH);
  nrc7292_write_regs(dev, HSPI_REG_EIRQ_ENABLE,
                     HSPI_EIRQ_ALL);
}

/****************************************************************************
 * Name: nrc7292_recvthread
 ****************************************************************************/

static int nrc7292_recvthread(int argc, FAR char *argv[])
{
  struct nrc7292_dev_s *dev = g_nrc7292_dev;
  int ret;

  while (true)
    {
      nxsem_wait_uninterruptible(&dev->recv_thread_sem);

      nrc7292_set_eirq_config(dev);

      while (true)
        {
          nrc7292_update_queue_status(dev);

          if (dev->queue_status.txq.slot_cnt == 0)
            {
              break;
            }

          ret = nxmutex_lock(&dev->rw_lock);

          if (ret < 0)
            {
              continue;
            }

          ret = nrc7292_read_slot_data(dev);

          if (ret < 0)
            {
              wlerr("nrc7292_read_slot_data error! (%d)\n", ret);
            }

          nxmutex_unlock(&dev->rw_lock);
          break;
        }

      dev->lower->enable();
    }

  return 0;
}

/****************************************************************************
 * Name: nrc7292_start_rxthread
 ****************************************************************************/

static int nrc7292_start_rxthread(FAR struct nrc7292_dev_s *dev)
{
  int ret;

  ret = kthread_create("nrc7292_recvthread",
                       SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       nrc7292_recvthread,
                       NULL);

  return ret;
}

/****************************************************************************
 * Name: nrc7292_update_queue_status
 ****************************************************************************/

static void nrc7292_update_queue_status(FAR struct nrc7292_dev_s *dev)
{
  struct spi_queue_status_s status;

  nrc7292_read_regs(dev, HSPI_REG_STATUS, (FAR uint8_t *) &status,
                    sizeof(status));

  status.txq.slot_size = ntohs(status.txq.slot_size);
  status.txq.total_slot_size = ntohs(status.txq.total_slot_size);
  status.rxq.slot_size = ntohs(status.rxq.slot_size);
  status.rxq.total_slot_size = ntohs(status.rxq.total_slot_size);

  dev->queue_status = status;
}

/****************************************************************************
 * Name: nrc7292_print_queue_status
 ****************************************************************************/

static void nrc7292_print_queue_status(FAR struct nrc7292_dev_s *dev)
{
  wlinfo("TX SLOT count:%d\n", dev->queue_status.txq.slot_cnt);
  wlinfo("TX SLOT size:%d\n", dev->queue_status.txq.slot_size);
  wlinfo("TX SLOT total size:%d\n",
          dev->queue_status.txq.total_slot_size);
  wlinfo("RX SLOT count:%d\n", dev->queue_status.rxq.slot_cnt);
  wlinfo("RX SLOT size:%d\n", dev->queue_status.rxq.slot_size);
  wlinfo("RX SLOT total size:%d\n",
          dev->queue_status.rxq.total_slot_size);
}

/****************************************************************************
 * Name: nrc7292_ready
 ****************************************************************************/

static bool nrc7292_ready(FAR struct nrc7292_dev_s *dev)
{
  uint32_t msg[4];
  const size_t MSG_LEN = sizeof(msg);
  uint16_t tx_slot_num;
  uint16_t tx_slot_size;
  uint16_t rx_slot_num;
  uint16_t rx_slot_size;
  int i;

  for (i = 0; i < READY_POLL_COUNT; i++)
    {
      memset(msg, 0, 16);
      nrc7292_read_regs(dev, HSPI_REG_MSG, (FAR uint8_t *) &msg[0], MSG_LEN);
      msg[0] = ntohl(msg[0]);
      msg[1] = ntohl(msg[1]);
      msg[2] = ntohl(msg[2]);
      msg[3] = ntohl(msg[3]);

      nrc7292_update_queue_status(dev);

      if (memcmp("NRC-HSPI", msg, 8) == 0)
        {
          break;
        }
      else
        {
          wlinfo("Invalid FW Signature...\n");
          sleep(1);
        }
    }

  if (READY_POLL_COUNT <= i)
    {
      return false;
    }

  tx_slot_num = (msg[2 + TX_SLOT] >> 16) & 0xffff;
  tx_slot_size = msg[2 + TX_SLOT] & 0xffff;
  rx_slot_num = (msg[2 + RX_SLOT] >> 16) & 0xffff;
  rx_slot_size = msg[2 + RX_SLOT] & 0xffff;

  if (tx_slot_size == 0 || tx_slot_size > HSPI_SLOT_SIZE_MAX
      || rx_slot_size == 0 || rx_slot_size > HSPI_SLOT_SIZE_MAX)
    {
      wlerr("Invalid Slot size ...\n");
      return false;
    }

  dev->slot_info[TX_SLOT].num = tx_slot_num;
  dev->slot_info[TX_SLOT].size = tx_slot_size;
  dev->slot_info[RX_SLOT].num = rx_slot_num;
  dev->slot_info[RX_SLOT].size = rx_slot_size;

  nrc7292_print_queue_status(dev);

  return true;
}

/****************************************************************************
 * Name: nrc7292_start
 ****************************************************************************/

static int nrc7292_start(FAR struct nrc7292_dev_s *dev)
{
  uint8_t latch;
  bool is_ready = false;

  is_ready = nrc7292_ready(dev);

  if (!is_ready)
    {
      return -ENODEV;
    }

  /* clean IRQ register */

  nrc7292_write_regs(dev, HSPI_REG_EIRQ_ENABLE, 0);
  nrc7292_read_regs(dev, HSPI_REG_EIRQ_STATUS_LATCH, &latch, 1);

  /* enable IRQ */

  nrc7292_set_eirq_config(dev);

  nrc7292_read_regs(dev, HSPI_REG_CHIP_ID,
                    (FAR uint8_t *) &dev->sys_info,
                    sizeof(struct spi_sys_s));

  dev->net_dev.d_flags |= IFF_DOWN;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrc7292_register
 ****************************************************************************/

FAR void *nrc7292_register(FAR const char *devpath,
                           FAR struct spi_dev_s *spi,
                           FAR const struct nrc7292_lower_s *lower)
{
  FAR struct nrc7292_dev_s *dev;
  int ret;
  int size;

  size = sizeof(struct nrc7292_dev_s);
  dev = kmm_malloc(size);

  if (!dev)
    {
      wlerr("Failed to allocate instance.\n");
      return NULL;
    }

  g_nrc7292_dev = dev;

  memset(dev, 0, size);

  dev->spi   = spi;
  dev->path  = strdup(devpath);
  dev->lower = lower;
  dev->pfd   = NULL;
  dev->recv_thread_pid = -1;

  nxmutex_init(&dev->refslock);
  nxsem_init(&dev->recv_thread_sem, 0, 0);

  if (!dev->path)
    {
      wlerr("Failed to allocate driver path.\n");
      goto errout;
    }

  ret = nrc7292_start_rxthread(dev);

  if (ret < 0)
    {
      wlerr("Failed to create rxthread.\n");
      goto errout;
    }

  dev->recv_thread_pid = ret;

  nrc7292_spi_init(dev);

  ret = register_driver(devpath, &g_nrc7292_fops, 0666, dev);

  if (ret < 0)
    {
      wlerr("Failed to register driver: %d\n", ret);
      goto errout;
    }

  ret = netdev_register(&dev->net_dev, NET_LL_IEEE80211);

  if (ret < 0)
    {
      unregister_driver(devpath);
      goto errout;
    }

  dev->net_dev.d_pktsize  = MAX_PKT_LEN;
  dev->net_dev.d_llhdrlen = 0;

  dev->net_relay_dev.d_pktsize  = MAX_PKT_LEN;
  dev->net_relay_dev.d_llhdrlen = 0;

  return dev;

errout:
  lib_free(dev->path);
  kmm_free(dev);
  return NULL;
}
