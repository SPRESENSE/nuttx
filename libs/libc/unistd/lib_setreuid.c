/****************************************************************************
 * libs/libc/unistd/lib_setreuid.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setreuid
 *
 * Description:
 *   The setreuid() function sets the real user ID and/or the effective user
 *   ID of the calling task group to ruid and/or euid.
 *
 * Input Parameters:
 *   ruid - Real user identity to set.  The special value (uid_t)-1
 *          indicates that the real user ID should not be changed.
 *   euid - Effective user identity to set.  The special value (uid_t)-1
 *          indicates that the effective user ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setreuid(uid_t ruid, uid_t euid)
{
  /* NuttX only supports the user identity 'root' with a uid value of 0. */

  if ((ruid == (uid_t)-1 || ruid == 0) &&
      (euid == (uid_t)-1 || euid == 0))
    {
      return 0;
    }

  /* All other uid values are considered invalid and not supported by the
   * implementation.
   */

  set_errno(EINVAL);
  return -1;
}
