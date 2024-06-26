/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_irq.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_IRQ_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* CPU interrupt types. */

#define ESP32S3_CPUINT_LEVEL   0
#define ESP32S3_CPUINT_EDGE    1

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32s3_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32s3_cpuint_initialize(void);

/****************************************************************************
 * Name:  esp32s3_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from irq.h to be assigned to
 *              a CPU interrupt.
 *   priority - Interrupt's priority (1 - 5).
 *   type     - Interrupt's type (level or edge).
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int esp32s3_setup_irq(int cpu, int periphid, int priority, int type);

/****************************************************************************
 * Name:  esp32s3_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32s3_setup_irq.
 *   It detaches a peripheral interrupt from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from irq.h to be detached from the
 *              CPU interrupt.
 *   cpuint   - The CPU interrupt from which the peripheral interrupt will
 *              be detached.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_teardown_irq(int cpu, int periphid, int cpuint);

/****************************************************************************
 * Name:  esp32s3_getirq
 *
 * Description:
 *   This function returns the IRQ associated with a CPU interrupt
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   cpuint   - The CPU interrupt associated to the IRQ
 *
 * Returned Value:
 *   The IRQ associated with such CPU interrupt or CPUINT_UNASSIGNED if
 *   IRQ is not yet assigned to a CPU interrupt.
 *
 ****************************************************************************/

int esp32s3_getirq(int cpu, int cpuint);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_IRQ_H */
