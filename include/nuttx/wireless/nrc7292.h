/****************************************************************************
 * include/nuttx/wireless/nrc7292.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_NRC7292_H
#define __INCLUDE_NUTTX_WIRELESS_NRC7292_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

#define SIOCSIWBANDWIDTH  _WLIOC(SIOCIWFIRSTPRIV + 0)
#define SIOCGIWBANDWIDTH  _WLIOC(SIOCIWFIRSTPRIV + 1)
#define SIOCSIFWUPDINIT   _WLIOC(SIOCIWFIRSTPRIV + 2)
#define SIOCSIFWUPDINJECT _WLIOC(SIOCIWFIRSTPRIV + 3)
#define SIOCSIFWUPDEXEC   _WLIOC(SIOCIWFIRSTPRIV + 4)
#define SIOCSIWCONNECT    _WLIOC(SIOCIWFIRSTPRIV + 5)
#define SIOCSIWREGEVTCB   _WLIOC(SIOCIWFIRSTPRIV + 6)

struct nrc7292_lower_s
{
  int  (*attach)(xcpt_t handler, FAR void *arg);
  void (*enable)(void);
  void (*disable)(void);
  void (*reset)(bool);
};

FAR void *nrc7292_register(FAR const char *devpath,
                           FAR struct spi_dev_s *dev,
                           FAR const struct nrc7292_lower_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WIRELESS_NRC7292_H */
