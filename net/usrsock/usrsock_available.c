/****************************************************************************
 * net/usrsock/usrsock_available.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <sys/socket.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "devif/devif.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t socket_event(FAR struct net_driver_s *dev,
                             FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pstate->conn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      pstate->result = -ENETDOWN;

      /* Stop further callbacks */

      pstate->cb->flags = 0;
      pstate->cb->priv  = NULL;
      pstate->cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }
  else if (flags & USRSOCK_EVENT_REQ_COMPLETE)
    {
      ninfo("request completed.\n");

      pstate->result = conn->resp.result;
      if (pstate->result >= 0)
        {
          /* We might start getting events for this socket right after
           * returning to daemon, so setup 'conn' already here.
           */

          conn->state   = USRSOCK_CONN_STATE_READY;
          conn->usockid = pstate->result;
          if (flags & USRSOCK_EVENT_SENDTO_READY)
            {
              conn->flags |= USRSOCK_EVENT_SENDTO_READY;
            }
        }

      /* Stop further callbacks */

      pstate->cb->flags = 0;
      pstate->cb->priv  = NULL;
      pstate->cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_socket_available
 ****************************************************************************/

static int do_socket_available(FAR struct usrsock_conn_s *conn, int domain,
                               int type, int protocol)
{
  struct usrsock_request_available_s req =
  {
  };

  struct iovec bufs[1];

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_AVAILABLE;
  req.domain = domain;
  req.type = type;
  req.protocol = protocol;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsock_do_request(conn, bufs, nitems(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_available
 *
 * Description:
 *   The available() function retrieves whether the usrsock is available or
 *   not according to Domain, Type, Protocol and Daemon status.
 *
 * Input Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *
 ****************************************************************************/

bool usrsock_available(int domain, int type, int protocol)
{
  struct usrsock_reqstate_s state =
  {
  };

  FAR struct usrsock_conn_s *conn;
  int err;
  bool ret;

  /* Allocate the usrsock socket connection structure and save in the new
   * socket instance.
   */

  conn = usrsock_alloc();
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return false;
    }

  net_lock();

  /* Set up event callback for usrsock. */

  err = usrsock_setup_request_callback(conn, &state, socket_event,
                                       USRSOCK_EVENT_ABORT |
                                       USRSOCK_EVENT_REQ_COMPLETE);
  if (err < 0)
    {
      ret = false;
      goto exit_free_conn;
    }

  /* Request user-space daemon for new socket. */

  err = do_socket_available(conn, domain, type, protocol);
  if (err == -ENETDOWN)
    {
      /* If do_socket_available() returns -ENETDOWN,
       * its means usrsock daemon is not prepared yet.
       * The available() function requires to be prepared
       * a daemon before call.
       */

      ret = true;
      goto exit_teardown_callback;
    }
  else if (err < 0)
    {
      ret = false;
      goto exit_teardown_callback;
    }

  /* Wait for completion of request. */

  net_sem_wait_uninterruptible(&state.recvsem);

  ret = (bool)state.result;

exit_teardown_callback:
  usrsock_teardown_request_callback(&state);
exit_free_conn:
  usrsock_free(conn);

  net_unlock();
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
