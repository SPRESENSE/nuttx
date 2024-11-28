/****************************************************************************
 * drivers/wireless/nrc7292.h
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

#ifndef __DRIVERS_WIRELESS_NRC7292_H
#define __DRIVERS_WIRELESS_NRC7292_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* nrc7292 hardware definitions */

/* H-SPI Host-Side Registers */

#define HSPI_REG_SYS                    0x00
#define HSPI_REG_WAKEUP                 (HSPI_REG_SYS + 0x00)
#define HSPI_REG_DEVICE_STATUS          (HSPI_REG_SYS + 0x01)
#define HSPI_REG_CHIP_ID                (HSPI_REG_SYS + 0x02)
#define HSPI_REG_MODEM_ID               (HSPI_REG_SYS + 0x04)
#define HSPI_REG_SW_ID                  (HSPI_REG_SYS + 0x08)
#define HSPI_REG_BOARD_ID               (HSPI_REG_SYS + 0x0C

#define HSPI_REG_IRQ                    0x10
#define HSPI_REG_EIRQ_MODE              (HSPI_REG_IRQ + 0x00)
#define HSPI_REG_EIRQ_ENABLE            (HSPI_REG_IRQ + 0x01)
#define HSPI_REG_EIRQ_STATUS_LATCH      (HSPI_REG_IRQ + 0x02)
#define HSPI_REG_EIRQ_STATUS            (HSPI_REG_IRQ + 0x03)
#define HSPI_REG_QUEUE_STATUS           (HSPI_REG_IRQ + 0x04)

#define HSPI_REG_EIRQ                   HSPI_REG_EIRQ_MODE
#define HSPI_REG_STATUS                 HSPI_REG_EIRQ_STATUS_LATCH

#define HSPI_REG_MSG                    0x20
#define HSPI_REG_DEV_MSG_0              (HSPI_REG_MSG + 0x00)
#define HSPI_REG_DEV_MSG_1              (HSPI_REG_MSG + 0x04)
#define HSPI_REG_DEV_MSG_2              (HSPI_REG_MSG + 0x08)
#define HSPI_REG_DEV_MSG_3              (HSPI_REG_MSG + 0x0c)

#define HSPI_REG_RXQ                    0x30
#define HSPI_REG_RXQ_THRES              (HSPI_REG_RXQ + 0x00)
#define HSPI_REG_RXQ_WINDOW             (HSPI_REG_RXQ + 0x01)

#define HSPI_REG_TXQ                    0x40
#define HSPI_REG_TXQ_THRES              (HSPI_REG_TXQ + 0x00)
#define HSPI_REG_TXQ_WINDOW             (HSPI_REG_TXQ + 0x01)

#define HSPI_READ     0x50000000
#define HSPI_WRITE    0x50400000
#define HSPI_BURST    0x00800000
#define HSPI_FIXED    0x00200000
#define HSPI_ADDR(x)  (((x) & 0xff) << 13)
#define HSPI_LEN(x)   ((x) & 0x1fff)
#define HSPI_WDATA(x) ((x) & 0xff)
#define HSPI_ACK      0x47

#define HSPI_EIRQ_LOW           0
#define HSPI_EIRQ_HIGH          1
#define HSPI_EIRQ_LEVEL         0
#define HSPI_EIRQ_EDGE          (1 << 1)
#define HSPI_EIRQ_IO_ENABLE     (1 << 2)

#define HSPI_EIRQ_TXQ           1
#define HSPI_EIRQ_RXQ           (1 << 1)
#define HSPI_EIRQ_READY         (1 << 2)
#define HSPI_EIRQ_SLEEP         (1 << 3)
#define HSPI_EIRQ_ALL           0xf

#define HSPI_SLOT_SIZE_MAX      512
#define HSPI_SLOT_HDR_SIZE      4 /* 2 + 1 + 1 */
#define HSPI_SLOT_START_SIZE    2
#define HSPI_SLOT_START         "HS"
#define HSPI_SLOT_SEQ_MAX       0x3f

#define TX_SLOT 0
#define RX_SLOT 1

#define TXQ_ERROR       (1 << 7)
#define TXQ_SLOT_COUNT  0x7f
#define RXQ_SLOT_COUNT  0x7f

#define HSPI_SLOT_CNT_MAX 0x3f

#define READY_POLL_COUNT  5

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

begin_packed_struct struct spi_sys_s
{
  uint16_t chip_id;
  uint32_t modem_id;
  uint32_t sw_id;
  uint32_t board_id;
} end_packed_struct;

begin_packed_struct struct spi_status_s
{
  uint8_t txq_status[6];
  uint8_t rxq_status[6];
  uint32_t msg[4];
} end_packed_struct;

begin_packed_struct struct spi_queue_s
{
  uint8_t error;
  uint8_t slot_cnt;
  uint16_t slot_size;
  uint16_t total_slot_size;
} end_packed_struct;

begin_packed_struct struct spi_queue_status_s
{
  uint8_t latch;
  uint8_t eirq_status;
  struct spi_queue_s txq;
  struct spi_queue_s rxq;
} end_packed_struct;

begin_packed_struct struct spi_hw_status_s
{
  uint8_t latch;
  uint8_t eirq;
  struct spi_queue_status_s hw;
} end_packed_struct;

begin_packed_struct struct spi_slot_t
{
  uint8_t start[2];
  uint16_t len_seq;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct spi_slot_info_s
{
  uint16_t num;
  uint16_t size;
} end_packed_struct;

static inline uint16_t slot_len_seq(uint16_t len, uint16_t seq)
{
  uint16_t val = (len & 0x03ff) | ((seq << 10) & 0xfc00);
  return val;
}

static inline uint16_t get_slot_len(uint16_t len_seq)
{
  uint16_t val = len_seq & 0x03ff;
  return val;
}

static inline uint16_t get_slot_seq(uint16_t len_seq)
{
  uint16_t val = ((len_seq & 0xfc00) >> 10) & 0x003f;
  return val;
}

#endif /* __DRIVERS_WIRELESS_NRC7292_H */
