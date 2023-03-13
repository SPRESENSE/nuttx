/****************************************************************************
 * drivers/modem/alt1250/alt1250.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <poll.h>
#include <errno.h>
#include <arch/board/board.h>
#include <nuttx/wireless/lte/lte.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/power/pm.h>
#include <assert.h>

#include "altcom_pkt.h"
#include "altcom_hdlr.h"
#include "altmdm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WRITE_OK 0
#define WRITE_NG 1

#define rel_evtbufinst(inst, dev) unlock_evtbufinst(inst, dev)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct alt1250_res_s
{
  sem_t sem;
  int code;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int alt1250_open(FAR struct file *filep);
static int alt1250_close(FAR struct file *filep);
static ssize_t alt1250_read(FAR struct file *filep, FAR char *buffer,
  size_t len);
static int alt1250_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int alt1250_poll(FAR struct file *filep, struct pollfd *fds,
  bool setup);

parse_handler_t alt1250_additional_parsehdlr(uint16_t, uint8_t);
compose_handler_t alt1250_additional_composehdlr(uint32_t,
    FAR uint8_t *, size_t);

#ifdef CONFIG_PM
static int alt1250_pm_prepare(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate);
static void alt1250_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_alt1250fops =
{
  alt1250_open,  /* open */
  alt1250_close, /* close */
  alt1250_read,  /* read */
  0,             /* write */
  0,             /* seek */
  alt1250_ioctl, /* ioctl */
  alt1250_poll,  /* poll */
};
static uint8_t g_recvbuff[ALTCOM_RX_PKT_SIZE_MAX];
static uint8_t g_sendbuff[ALTCOM_PKT_SIZE_MAX];

static struct alt1250_dev_s *g_alt1250_dev;

#ifdef CONFIG_PM
static struct pm_callback_s g_alt1250pmcb =
{
  .notify  = alt1250_pm_notify,
  .prepare = alt1250_pm_prepare,
};

static struct alt1250_res_s g_alt1250_res_s;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: add_list
 ****************************************************************************/

static void add_list(FAR struct alt_queue_s *head,
  FAR struct alt_container_s *list)
{
  FAR struct alt_container_s *next;

  nxsem_wait_uninterruptible(&head->lock);

  while (list != NULL)
    {
      next = (FAR struct alt_container_s *)sq_next(&list->node);

      sq_next(&list->node) = NULL;
      sq_addlast(&list->node, &head->queue);

      list = next;
    }

  nxsem_post(&head->lock);
}

/****************************************************************************
 * Name: remove_list_all
 ****************************************************************************/

static FAR struct alt_container_s *remove_list_all(
  FAR struct alt_queue_s *head)
{
  FAR struct alt_container_s *list;

  nxsem_wait_uninterruptible(&head->lock);

  list = (FAR struct alt_container_s *)sq_peek(&head->queue);
  sq_init(&head->queue);

  nxsem_post(&head->lock);

  return list;
}

/****************************************************************************
 * Name: remove_list
 ****************************************************************************/

static FAR struct alt_container_s *remove_list(FAR struct alt_queue_s *head,
  uint16_t cmdid, uint16_t transid)
{
  FAR struct alt_container_s *list;

  nxsem_wait_uninterruptible(&head->lock);

  list = (FAR struct alt_container_s *)sq_peek(&head->queue);
  while (list != NULL)
    {
      if ((list->altcid == cmdid) && (list->alttid == transid))
        {
          sq_rem(&list->node, &head->queue);
          sq_next(&list->node) = NULL;
          break;
        }

      list = (FAR struct alt_container_s *)sq_next(&list->node);
    }

  nxsem_post(&head->lock);

  return list;
}

/****************************************************************************
 * Name: set_senddisable
 ****************************************************************************/

static void set_senddisable(FAR struct alt1250_dev_s *dev, bool disable)
{
  nxsem_wait_uninterruptible(&dev->senddisablelock);

  dev->senddisable = disable;

  nxsem_post(&dev->senddisablelock);
}

/****************************************************************************
 * Name: is_senddisable
 ****************************************************************************/

static bool is_senddisable(FAR struct alt1250_dev_s *dev)
{
  bool disable;

  nxsem_wait_uninterruptible(&dev->senddisablelock);

  disable = dev->senddisable;

  nxsem_post(&dev->senddisablelock);

  return disable;
}

/****************************************************************************
 * Name: read_evtbitmap
 ****************************************************************************/

static ssize_t read_data(FAR struct alt1250_dev_s *dev,
  FAR struct alt_readdata_s *rdata)
{
  int idx;

  nxsem_wait_uninterruptible(&dev->evtmaplock);

  /* change status to NOT WRITABLE */

  for (idx = 0; idx < (sizeof(uint64_t) * 8); idx++)
    {
      if (dev->evtbitmap & (1ULL << idx))
        {
          if (dev->evtbuff->ninst >= idx)
            {
              FAR alt_evtbuf_inst_t *inst = &dev->evtbuff->inst[idx];

              nxsem_wait_uninterruptible(&inst->stat_lock);

              inst->stat = ALTEVTBUF_ST_NOTWRITABLE;

              nxsem_post(&inst->stat_lock);
            }
        }
    }

  rdata->evtbitmap = dev->evtbitmap;
  rdata->head = remove_list_all(&dev->replylist);

  if (dev->evtbitmap & ALT1250_EVTBIT_RESET)
    {
      /* Resume sending because daemon has been notified of the reset
       * reliably.
       */

      set_senddisable(dev, false);
    }

  dev->evtbitmap = 0ULL;

  nxsem_post(&dev->evtmaplock);

  return sizeof(struct alt_readdata_s);
}

/****************************************************************************
 * Name: write_evtbitmap
 ****************************************************************************/

static void write_evtbitmap(FAR struct alt1250_dev_s *dev,
  uint64_t bitmap, FAR struct alt_container_s *container)
{
  nxsem_wait_uninterruptible(&dev->evtmaplock);

  dev->evtbitmap |= bitmap;

  if (dev->evtbitmap & ALT1250_EVTBIT_RESET)
    {
      dev->evtbitmap = ALT1250_EVTBIT_RESET;
    }

  if (container)
    {
      add_list(&dev->replylist, container);
    }

  m_info("write bitmap: 0x%llx\n", bitmap);

  nxsem_post(&dev->evtmaplock);
}

/****************************************************************************
 * Name: is_evtbitmap_avail
 ****************************************************************************/

static int is_evtbitmap_avail(FAR struct alt1250_dev_s *dev)
{
  int ret;

  nxsem_wait_uninterruptible(&dev->evtmaplock);

  /* 0 means it is not available, otherwise it is available. */

  ret = (0ULL != dev->evtbitmap);

  nxsem_post(&dev->evtmaplock);

  return ret;
}

/****************************************************************************
 * Name: add_evtbuff
 ****************************************************************************/

static void add_evtbuff(FAR struct alt1250_dev_s *dev,
  FAR struct alt_evtbuffer_s *buff)
{
  dev->evtbuff = buff;
}

/****************************************************************************
 * Name: write_evtbuff_byidx
 ****************************************************************************/

static int write_evtbuff_byidx(FAR struct alt1250_dev_s *dev,
  uint64_t idx, void(*write_func)(FAR void *outp[], FAR void *inp),
  FAR void *inp)
{
  int ret = WRITE_NG;

  nxsem_wait_uninterruptible(&dev->evtmaplock);

  if (dev->evtbuff)
    {
      if (dev->evtbuff->ninst >= idx)
        {
          FAR alt_evtbuf_inst_t *inst = &dev->evtbuff->inst[idx];

          nxsem_wait_uninterruptible(&inst->stat_lock);
          if (inst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              write_func(inst->outparam, inp);
              dev->evtbitmap |= (1ULL << idx);
              ret = WRITE_OK;
            }

          nxsem_post(&inst->stat_lock);
        }
    }

  nxsem_post(&dev->evtmaplock);

  return ret;
}

/****************************************************************************
 * Name: lock_evtbuffinst
 ****************************************************************************/

static void lock_evtbuffinst(FAR alt_evtbuf_inst_t *inst,
  FAR struct alt1250_dev_s *dev)
{
  nxsem_wait_uninterruptible(&dev->evtmaplock);
  nxsem_wait_uninterruptible(&inst->stat_lock);
}

/****************************************************************************
 * Name: unlock_evtbufinst
 ****************************************************************************/

static void unlock_evtbufinst(FAR alt_evtbuf_inst_t *inst,
  FAR struct alt1250_dev_s *dev)
{
  nxsem_post(&inst->stat_lock);
  nxsem_post(&dev->evtmaplock);
}

/****************************************************************************
 * Name: search_evtbufinst
 ****************************************************************************/

static FAR alt_evtbuf_inst_t *search_evtbufinst(uint16_t cid,
  FAR uint64_t *bitmap, FAR struct alt1250_dev_s *dev)
{
  FAR alt_evtbuf_inst_t *ret = NULL;
  unsigned int i;

  *bitmap = 0ULL;

  for (i = 0; i < dev->evtbuff->ninst; i++)
    {
      ret = &dev->evtbuff->inst[i];

      if (ret->altcid == cid)
        {
          *bitmap = 1ULL << i;
          return ret;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: cid_to_searchable
 ****************************************************************************/

static uint16_t cid_to_searchable(uint16_t cid, uint8_t altver)
{
  uint16_t cidv1;

  cid &= ~ALTCOM_CMDID_REPLY_BIT;
  if (altver == ALTCOM_VER4)
    {

      /* Change the command ID to Version 1
       * Even if it cannot be converted, try to search the table
       * using the original command ID.
       */

      cidv1 = convert_cid2v1(cid);
      if (cidv1 != APICMDID_UNKNOWN)
        {
          cid = cidv1;
        }
    }

  return cid;
}

/****************************************************************************
 * Name: get_bitmap
 ****************************************************************************/

static uint64_t get_bitmap(FAR struct alt1250_dev_s *dev, uint16_t cid,
  uint8_t altver)
{
  uint64_t bitmap = 0ULL;

  cid = cid_to_searchable(cid, altver);

  search_evtbufinst(cid, &bitmap, dev);

  return bitmap;
}

/****************************************************************************
 * Name: get_evtbuffinst_withlock
 ****************************************************************************/

static FAR alt_evtbuf_inst_t *get_evtbuffinst_withlock(
  FAR struct alt1250_dev_s *dev, uint16_t cid, uint8_t altver,
  FAR uint64_t *bitmap)
{
  FAR alt_evtbuf_inst_t *inst = NULL;
  FAR alt_evtbuf_inst_t *ret = NULL;

  cid = cid_to_searchable(cid, altver);

  if (cid == APICMDID_SOCK_SELECT)
    {
      ret = &dev->select_inst;

      lock_evtbuffinst(ret, dev);

      ret->outparam = dev->select_container->outparam;
      ret->outparamlen = dev->select_container->outparamlen;

      search_evtbufinst(cid, bitmap, dev);
    }
  else
    {
      inst = search_evtbufinst(cid, bitmap, dev);
      if (inst)
        {
          lock_evtbuffinst(inst, dev);

          if (inst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              ret = inst;
            }
          else
            {
              unlock_evtbufinst(inst, dev);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: write_restart_param
 ****************************************************************************/

static void write_restart_param(FAR void *outp[], FAR void *buff)
{
  FAR int *out_reason = (FAR int *)outp[0];
  FAR int *in_reason = (FAR int *)buff;

  *out_reason = *in_reason;
}

/****************************************************************************
 * Name: pollnotify
 ****************************************************************************/

static void pollnotify(FAR struct alt1250_dev_s *dev, uint64_t bitmap,
                       FAR struct alt_container_s *container)
{
  write_evtbitmap(dev, bitmap, container);

  nxsem_wait_uninterruptible(&dev->pfdlock);

  if (dev->pfd)
    {
      /* If poll() waits, notify  */

      dev->pfd->revents |= POLLIN;
      nxsem_post(dev->pfd->sem);
    }

  nxsem_post(&dev->pfdlock);
}

/****************************************************************************
 * Name: get_composehdlr
 ****************************************************************************/

compose_handler_t get_composehdlr(uint32_t cmdid, FAR uint8_t *payload,
  size_t size)
{
  compose_handler_t ret = NULL;

  ret = alt1250_composehdlr(cmdid);

#ifdef CONFIG_MODEM_ALT1250_ADDITIONAL_FUNC
  if (ret == NULL)
    {
      ret = alt1250_additional_composehdlr(cmdid, payload, size);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: get_parsehdlr
 ****************************************************************************/

parse_handler_t get_parsehdlr(uint16_t altcid, uint8_t altver)
{
  parse_handler_t ret = NULL;

  ret = alt1250_parsehdlr(altcid, altver);

#ifdef CONFIG_MODEM_ALT1250_ADDITIONAL_FUNC
  if (ret == NULL)
    {
      ret = alt1250_additional_parsehdlr(altcid, altver);
    }
#endif

  return ret;
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: alt1250_send_daemon_request
 ****************************************************************************/

static int alt1250_send_daemon_request(uint64_t bitmap)
{
  /* Send event for daemon */

  pollnotify(g_alt1250_dev, bitmap, NULL);

  /* Waiting for daemon response */

  nxsem_wait_uninterruptible(&g_alt1250_res_s.sem);

  return g_alt1250_res_s.code;
}

/****************************************************************************
 * Name: alt1250_receive_daemon_response
 ****************************************************************************/

static void alt1250_receive_daemon_response(FAR struct alt_power_s *req)
{
  /* Store daemon response code */

  g_alt1250_res_s.code = req->resp;

  /* Post request semaphore */

  nxsem_post(&g_alt1250_res_s.sem);
}
#endif

/****************************************************************************
 * Name: alt1250_power_control
 ****************************************************************************/

static int alt1250_power_control(FAR struct alt1250_dev_s *dev,
  FAR struct alt_power_s *req)
{
  int ret = OK;

  switch (req->cmdid)
    {
      case LTE_CMDID_POWERON:
        ret = altmdm_poweron();
        break;

      case LTE_CMDID_POWEROFF:
        ret = altmdm_poweroff();
        break;

      case LTE_CMDID_TAKEWLOCK:
        ret = altmdm_take_wlock();
        break;

      case LTE_CMDID_GIVEWLOCK:
        ret = altmdm_give_wlock();
        break;

      case LTE_CMDID_COUNTWLOCK:
        ret = altmdm_count_wlock();
        break;

#ifdef CONFIG_PM
      case LTE_CMDID_STOPAPI:
      case LTE_CMDID_SUSPEND:
        alt1250_receive_daemon_response(req);
        break;
#endif

      case LTE_CMDID_RETRYDISABLE:
        ret = altmdm_set_pm_event(EVENT_RETRYREQ, false);
        break;

      case LTE_CMDID_GET_POWER_STAT:
        ret = altmdm_get_powersupply(dev->lower);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: make_altcomcmd_and_send
 ****************************************************************************/

static int make_altcomcmd_and_send(FAR struct alt1250_dev_s *dev,
  FAR alt_container_t *req)
{
  int ret = OK;
  compose_handler_t handler;
  uint8_t altver;
  uint16_t cid;
  uint16_t tid;
  FAR uint8_t *payload;
  int remainlen;
  int pos;

  m_info("send request: command ID=0x%08lx\n", req->cmdid);

  payload = get_payload((FAR struct altcom_cmdhdr_s *)g_sendbuff);

  handler = get_composehdlr(req->cmdid & ~LTE_CMDOPT_ASYNC_BIT, payload,
    ALTCOM_PAYLOAD_SIZE_MAX);
  if (handler)
    {
      altver = altmdm_get_protoversion();
      if ((altver == ALTCOM_VERX) || is_senddisable(dev))
        {
          ret = -ENETDOWN;
        }
      else
        {
          ret = handler(req->inparam, req->inparamlen, altver, payload,
            ALTCOM_PAYLOAD_SIZE_MAX, &cid);

          ret = (ret > ALTCOM_PAYLOAD_SIZE_MAX) ? -ENOSPC : ret;

          if (ret >= 0)
            {
              tid = altcom_make_header(
                (FAR struct altcom_cmdhdr_s *)g_sendbuff,
                altver, cid, ret);

              req->altcid = cid | ALTCOM_CMDID_REPLY_BIT;
              req->alttid = tid;

              if (req->outparam != NULL)
                {
                  add_list(&dev->waitlist, req);
                }

              remainlen = get_pktlen(altver, (uint16_t)ret);
              pos = 0;

              /* If the modem sleeps during the split transmission,
               * the receive buffer of the modem will be cleared.
               * Therefore, split packets sent before sleep will be
               * discarded. To avoid this, do not enter sleep state
               * until all the packets have been sent.
               */

              altmdm_take_wlock();

              do
                {
                  ret = altmdm_write(&g_sendbuff[pos], remainlen);
                  if (ret < 0)
                    {
                      break;
                    }
                  else
                    {
                      m_info(
                        "write success: size=%d, cid=0x%04x tid=0x%04x\n",
                        ret, cid, tid);
                      remainlen -= ret;
                      pos += ret;
                    }
                }
              while (remainlen > 0);

              altmdm_give_wlock();

              if (ret < 0)
                {
                  m_err("altmdm_write() failed: %d\n", ret);
                  ret = -ENETDOWN;

                  /* If the container is not left in the waitlist,
                   * it has already been processed by the recvthread.
                   * ENETRESET is returned to the caller to indicate that
                   * the container has been processed.
                   */

                  if ((req->outparam != NULL) &&
                    (remove_list(&dev->waitlist, req->altcid, req->alttid)
                      == NULL))
                    {
                      ret = -ENETRESET;
                    }
                }
              else
                {
                  m_info("write success: size=%d, cid=0x%04x tid=0x%04x\n",
                    ret, cid, tid);
                  ret = OK;
                }
            }
          else
            {
              m_err("handler() failed: %d\n", ret);
            }
        }
    }
  else
    {
      ret = -ENOSYS;
    }

  return ret;
}

/****************************************************************************
 * Name: exchange_selectcontainer
 ****************************************************************************/

static int exchange_selectcontainer(FAR struct alt1250_dev_s *dev,
  FAR alt_container_t **container)
{
  FAR alt_container_t *newcontainer;

  if (container == NULL)
    {
      return -EINVAL;
    }

  nxsem_wait_uninterruptible(&dev->select_inst.stat_lock);

  newcontainer = *container;
  *container = dev->select_container;
  dev->select_container = newcontainer;

  nxsem_post(&dev->select_inst.stat_lock);

  return OK;
}

/****************************************************************************
 * Name: parse_altcompkt
 ****************************************************************************/

static int parse_altcompkt(FAR struct alt1250_dev_s *dev, FAR uint8_t *pkt,
                           int pktlen, FAR uint64_t *bitmap,
                           FAR struct alt_container_s **container)
{
  int ret;
  FAR struct altcom_cmdhdr_s *h = (FAR struct altcom_cmdhdr_s *)pkt;
  uint16_t cid = parse_cid(h);
  uint16_t tid = parse_tid(h);
  parse_handler_t parser;
  FAR alt_evtbuf_inst_t *inst;
  FAR void **outparam;
  size_t outparamlen;

  m_info("receive cid:0x%04x tid:0x%04x\n", cid, tid);

  parser = get_parsehdlr(cid, get_altver(h));
  if (is_errind(cid))
    {
      /* Get ALTCOM command ID and transaction ID
       * from payload
       */

      cid = parse_cid4errind(h);
      tid = parse_tid4errind(h);
      m_err("ALT1250 does not support this command. cid:0x%04x tid:0x%04x\n",
            cid, tid);
      cid |= ALTCOM_CMDID_REPLY_BIT;
    }

  *container = remove_list(&dev->waitlist, cid, tid);

  m_info("container %sfound. cid:0x%04x tid:0x%04x\n",
         *container == NULL ? "not " : "", cid, tid);

  if (parser == NULL)
    {
      m_err("parser is not found\n");

      /* If there is a container, use the container to notify
       * daemon of the event. Otherwise, discard the event.
       */

      if (*container)
        {
          (*container)->result = -ENOSYS;
        }

      return *container == NULL ? ERROR: OK;
    }

  /* Obtain outparam and bitmap required for parser execution arguments. */

  if (*container)
    {
      outparam = (*container)->outparam;
      outparamlen = (*container)->outparamlen;
      *bitmap = get_bitmap(dev, cid, get_altver(h));
    }
  else
    {
      /* The outparam is updated in the parser. Lock until the parser
       * returns to prevent outparam from being updated by other tasks.
       */

      inst = get_evtbuffinst_withlock(dev, cid, get_altver(h), bitmap);
      if (inst)
        {
          outparam = inst->outparam;
          outparamlen = inst->outparamlen;
        }
      else
        {
          /* Error return means that the event will be discarded. */

          return ERROR;
        }
    }

  ret = parser(dev, get_payload(h), get_payload_len(h), get_altver(h),
               outparam, outparamlen, bitmap);

  if (*container)
    {
      (*container)->result = ret;
      if (LTE_IS_ASYNC_CMD((*container)->cmdid))
        {
          /* Asynchronous types need to call the callback corresponding
           * to the received event, so the REPLY bit is added to the
           * received event.
           */

          *bitmap |= ALT1250_EVTBIT_REPLY;
        }
      else
        {
          /* Synchronous types do not call a callback,
           * so only the REPLY bit is needed.
           */

          *bitmap = ALT1250_EVTBIT_REPLY;
        }
    }
  else
    {
      /* Unlock outparam because it has been updated. */

      unlock_evtbufinst(inst, dev);
      if (ret < 0)
        {
          /* Error return means that the event will be discarded */

          return ERROR;
        }
    }

  /* OK return means that the event will be notified to the daemon. */

  return OK;
}

/****************************************************************************
 * Name: altcom_recvthread
 ****************************************************************************/

static int altcom_recvthread(int argc, FAR char *argv[])
{
  int ret;
  FAR struct alt1250_dev_s *dev = g_alt1250_dev;
  bool is_running = true;
  FAR struct alt_container_s *head;
  FAR struct alt_container_s *container;
  uint64_t bitmap = 0ULL;
  int recvedlen = 0;
  uint32_t reason;

  m_info("recv thread start\n");

  altmdm_init(dev->spi, dev->lower);

  while (is_running)
    {
      ret = altmdm_read(g_recvbuff + recvedlen,
                        ALTCOM_RX_PKT_SIZE_MAX - recvedlen);

      /* Normal packet received */

      if (ret >= 0)
        {
          m_info("read packet %d bytes\n", ret);

          recvedlen += ret;

          ret = altcom_is_pkt_ok(g_recvbuff, recvedlen);
          if (ret == 0)
            {
              ret = parse_altcompkt(dev, g_recvbuff, recvedlen, &bitmap,
                                    &container);
              if (ret == OK)
                {
                  pollnotify(dev, bitmap, container);
                }
              else
                {
                  dev->discardcnt++;
                  m_err("discard event %lu\n", dev->discardcnt);
                }
            }
          else if (ret > 0)
            {
              /* Cases in which fragmented packets are received.
               * Therefore, the receive process is performed again.
               */

              m_info("This is fragmented packet received. remain len: %d\n",
                     ret);
              continue;
            }
          else
            {
              /* Forced reset of modem due to packet format error detected */

              m_err("[altcom] Forced modem reset due to parse failure\n");

              altmdm_reset();
            }
        }
      else if (ret == ALTMDM_RETURN_RESET_PKT)
        {
          m_info("recieve ALTMDM_RETURN_RESET_PKT\n");
          set_senddisable(dev, true);
        }
      else if (ret == ALTMDM_RETURN_RESET_V1 ||
               ret == ALTMDM_RETURN_RESET_V4)
        {
          reason = altmdm_get_reset_reason();

          m_info("recieve ALTMDM_RETURN_RESET_V%s reason: %d\n",
                 (ret == ALTMDM_RETURN_RESET_V1) ? "1" : "4",
                 reason);

#if defined(CONFIG_MODEM_ALT1250_DISABLE_PV1) && \
    defined(CONFIG_MODEM_ALT1250_DISABLE_PV4)
#  error Unsupported configuration. Do not disable both PV1 and PV4.
#endif

#ifdef CONFIG_MODEM_ALT1250_DISABLE_PV1
          if (ret == ALTMDM_RETURN_RESET_V1)
            {
              m_err("Unsupported ALTCOM Version: V1\n");
              reason = LTE_RESTART_VERSION_ERROR;
            }
#endif

#ifdef CONFIG_MODEM_ALT1250_DISABLE_PV4
          if (ret == ALTMDM_RETURN_RESET_V4)
            {
              m_err("Unsupported ALTCOM Version: V4\n");
              reason = LTE_RESTART_VERSION_ERROR;
            }
#endif

          ret = write_evtbuff_byidx(dev, 0, write_restart_param,
                                    (FAR void *)&reason);

          /* If there is a waiting list,
           * replace it with the replay list.
           */

          head = remove_list_all(&dev->waitlist);
          pollnotify(dev, ALT1250_EVTBIT_RESET, head);
        }
      else if (ret == ALTMDM_RETURN_SUSPENDED)
        {
          m_info("recieve ALTMDM_RETURN_SUSPENDED\n");
          nxsem_post(&dev->rxthread_sem);
          while (1)
            {
              /* After receiving a suspend event, the ALT1250 driver
               * does not accept any requests and must stay alive.
               */

              sleep(1);
            }
        }
      else if (ret == ALTMDM_RETURN_EXIT)
        {
          m_info("recieve ALTMDM_RETURN_EXIT\n");
          is_running = false;
        }
      else
        {
          DEBUGASSERT(0);
        }

      recvedlen = 0;
    }

  nxsem_post(&dev->rxthread_sem);

  m_info("recv thread end\n");

  return 0;
}

/****************************************************************************
 * Name: alt1250_start_rxthread
 ****************************************************************************/

static int alt1250_start_rxthread(FAR struct alt1250_dev_s *dev,
                                  bool senddisable)
{
  int ret = OK;

  nxsem_init(&dev->waitlist.lock, 0, 1);
  nxsem_init(&dev->replylist.lock, 0, 1);
  nxsem_init(&dev->evtmaplock, 0, 1);
  nxsem_init(&dev->pfdlock, 0, 1);
  nxsem_init(&dev->senddisablelock, 0, 1);
  nxsem_init(&dev->select_inst.stat_lock, 0, 1);

  sq_init(&dev->waitlist.queue);
  sq_init(&dev->replylist.queue);

  dev->senddisable = senddisable;

  ret = kthread_create("altcom_recvthread",
                       SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       altcom_recvthread,
                       (FAR char * const *)NULL);

  if (ret < 0)
    {
      m_err("kthread create failed: %d\n", errno);

      nxsem_destroy(&dev->waitlist.lock);
      nxsem_destroy(&dev->replylist.lock);
      nxsem_destroy(&dev->evtmaplock);
      nxsem_destroy(&dev->pfdlock);
      nxsem_destroy(&dev->senddisablelock);
      nxsem_destroy(&dev->select_inst.stat_lock);
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_open
 ****************************************************************************/

static int alt1250_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  nxsem_wait_uninterruptible(&dev->refslock);

  if (dev->crefs > 0)
    {
      nxsem_post(&dev->refslock);
      return -EPERM;
    }

  /* Increment the count of open references on the driver */

  dev->crefs++;

  nxsem_post(&dev->refslock);

  if (dev->rxthread_pid < 0)
    {
      dev->rxthread_pid = alt1250_start_rxthread(dev, true);
    }

  if (dev->rxthread_pid < 0)
    {
      ret = dev->rxthread_pid;
      nxsem_wait_uninterruptible(&dev->refslock);
      dev->crefs--;
      nxsem_post(&dev->refslock);
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_close
 ****************************************************************************/

static int alt1250_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  nxsem_wait_uninterruptible(&dev->refslock);

  if (dev->crefs == 0)
    {
      nxsem_post(&dev->refslock);
      return -EPERM;
    }

  /* Decrement the count of open references on the driver */

  dev->crefs--;

  nxsem_post(&dev->refslock);

  nxsem_destroy(&dev->waitlist.lock);
  nxsem_destroy(&dev->replylist.lock);
  nxsem_destroy(&dev->evtmaplock);
  nxsem_destroy(&dev->pfdlock);
  nxsem_destroy(&dev->senddisablelock);
  nxsem_destroy(&dev->select_inst.stat_lock);

  altmdm_fin();
  dev->rxthread_pid = -1;
  nxsem_wait_uninterruptible(&dev->rxthread_sem);

  return OK;
}

/****************************************************************************
 * Name: alt1250_read
 ****************************************************************************/

static ssize_t alt1250_read(FAR struct file *filep, FAR char *buffer,
  size_t len)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  if (len != sizeof(struct alt_readdata_s))
    {
      return -EINVAL;
    }

  return read_data(dev, (FAR struct alt_readdata_s *)buffer);
}

/****************************************************************************
 * Name: alt1250_ioctl
 ****************************************************************************/

static int alt1250_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  switch (cmd)
    {
      case ALT1250_IOC_POWER:
        {
          FAR struct alt_power_s *req = (FAR struct alt_power_s *)arg;

          /* Performs power control or power consumption control
           * of the modem.
           */

          ret = alt1250_power_control(dev, req);
        }
        break;

      case ALT1250_IOC_SEND:
        {
          FAR alt_container_t *req = (FAR alt_container_t *)arg;

          ret = make_altcomcmd_and_send(dev, req);
        }
        break;

      case ALT1250_IOC_SETEVTBUFF:
        {
          FAR struct alt_evtbuffer_s *buff =
            (FAR struct alt_evtbuffer_s *)arg;
          add_evtbuff(dev, buff);
        }
        break;

      case ALT1250_IOC_EXCHGCONTAINER:
        {
          FAR alt_container_t **container = (FAR alt_container_t **)arg;

          ret = exchange_selectcontainer(dev, container);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_poll
 ****************************************************************************/

static int alt1250_poll(FAR struct file *filep, FAR struct pollfd *fds,
  bool setup)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      nxsem_wait_uninterruptible(&dev->pfdlock);

      if (is_evtbitmap_avail(dev))
        {
          fds->revents |= POLLIN;
          nxsem_post(fds->sem);
        }
      else
        {
          dev->pfd = fds;
        }

      nxsem_post(&dev->pfdlock);
    }
  else
    {
      nxsem_wait_uninterruptible(&dev->pfdlock);
      dev->pfd = NULL;
      nxsem_post(&dev->pfdlock);
    }

errout:
  return ret;
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: alt1250_pm_prepare
 ****************************************************************************/

static int alt1250_pm_prepare(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  int  ret   = OK;
  bool power = false;

  /* ALT1250's power management only support BOARD_PM_APPS */

  if (domain != BOARD_PM_APPS)
    {
      return OK;
    }

  if (pmstate == PM_SLEEP)
    {
      power = altmdm_get_powersupply(g_alt1250_dev->lower);

      if (!power)
        {
          /* If the modem doesn't turned on, system can enter sleep */

          return OK;
        }

      ret = alt1250_send_daemon_request(ALT1250_EVTBIT_STOPAPI);

      if (ret)
        {
          return ERROR;
        }
      else
        {
          return OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_pm_notify
 ****************************************************************************/

static void alt1250_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  bool power;

  /* ALT1250's power management only supports BOARD_PM_APPS */

  if ((domain == BOARD_PM_APPS) && (pmstate == PM_SLEEP))
    {
      power = altmdm_get_powersupply(g_alt1250_dev->lower);

      if (power)
        {
          /* Set retry mode for SPI driver */

          altmdm_set_pm_event(EVENT_RETRYREQ, true);

          /* Send suspend request to daemon */

          alt1250_send_daemon_request(ALT1250_EVTBIT_SUSPEND);

          /* Set suspend mode for SPI driver */

          altmdm_set_pm_event(EVENT_SUSPEND, true);

          /* Waiting for entering sleep state */

          nxsem_wait_uninterruptible(&g_alt1250_dev->rxthread_sem);

          /* Enable LTE hibernation mode for wakeup from LTE */

          g_alt1250_dev->lower->hiber_mode(true);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *alt1250_register(FAR const char *devpath,
  FAR struct spi_dev_s *dev, FAR const struct alt1250_lower_s *lower)
{
  FAR struct alt1250_dev_s *priv;
  int ret;

  priv = (FAR struct alt1250_dev_s *)
    kmm_malloc(sizeof(struct alt1250_dev_s));
  if (!priv)
    {
      m_err("Failed to allocate instance.\n");
      return NULL;
    }

  memset(priv, 0, sizeof(struct alt1250_dev_s));

  priv->spi = dev;
  priv->lower = lower;

  nxsem_init(&priv->refslock, 0, 1);

  nxsem_init(&priv->rxthread_sem, 0, 0);

  g_alt1250_dev = priv;

#ifdef CONFIG_PM
  ret = pm_register(&g_alt1250pmcb);

  if (ret != OK)
    {
      m_err("Failed to register PM: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  nxsem_init(&g_alt1250_res_s.sem, 0, 0);
#endif

  ret = register_driver(devpath, &g_alt1250fops, 0666, priv);
  if (ret < 0)
    {
      m_err("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  priv->rxthread_pid = -1;

#ifdef CONFIG_PM
  if (altmdm_get_powersupply(priv->lower))
    {
      priv->rxthread_pid = alt1250_start_rxthread(priv, false);
    }
#endif

  return (FAR void *)priv;
}

uint64_t get_event_lapibuffer(FAR struct alt1250_dev_s *dev,
  uint32_t lapicmdid, alt_evtbuf_inst_t **inst)
{
  FAR alt_evtbuf_inst_t *evtinst = NULL;
  unsigned int i;
  uint64_t ret = 0ULL;

  for (i = 0; i < dev->evtbuff->ninst; i++)
    {
      evtinst = &dev->evtbuff->inst[i];

      if (evtinst->cmdid == lapicmdid)
        {
          nxsem_wait_uninterruptible(&evtinst->stat_lock);

          if (evtinst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              *inst = evtinst;
              ret = 1ULL << i;
            }

          nxsem_post(&evtinst->stat_lock);
          break;
        }
    }

  return ret;
}
