/****************************************************************************
 * arch/arm/src/lpc43/lpc43_irq.c
 *
 *   Copyright (C) 2012-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "nvic.h"
#include "ram_vectors.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc43_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (LPC43M4_SYSH_PRIORITY_DEFAULT << 24 | \
   LPC43M4_SYSH_PRIORITY_DEFAULT << 16 | \
   LPC43M4_SYSH_PRIORITY_DEFAULT << 8  | \
   LPC43M4_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/* This is the address of the  exception vector table (determined by the
 * linker script).
 */

extern uint32_t _vectors[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ)
static void lpc43_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
  llerr("NVIC (%s, irq=%d):\n", msg, irq);
  llerr("  INTCTRL:    %08x VECTAB: %08x\n",
        getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
#if 0
  llerr("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
        getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
        getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
#endif
  llerr("  IRQ ENABLE: %08x %08x\n",
        getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE));
  llerr("  SYSH_PRIO:  %08x %08x %08x\n",
        getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
        getreg32(NVIC_SYSH12_15_PRIORITY));
  llerr("  IRQ PRIO:   %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
        getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  llerr("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
        getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
  llerr("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
        getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
  llerr("              %08x %08x %08x\n",
        getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
        getreg32(NVIC_IRQ56_59_PRIORITY));
  leave_critical_section(flags);
}
#else
#  define lpc43_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: lpc43_nmi, lpc43_busfault, lpc43_usagefault, lpc43_pendsv,
 *       lpc43_errmonitor, lpc43_pendsv, lpc43_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int lpc43_nmi(int irq, FAR void *context)
{
  (void)up_irq_save();
  err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int lpc43_busfault(int irq, FAR void *context)
{
  (void)up_irq_save();
  err("PANIC!!! Bus fault recived\n");
  PANIC();
  return 0;
}

static int lpc43_usagefault(int irq, FAR void *context)
{
  (void)up_irq_save();
  err("PANIC!!! Usage fault received\n");
  PANIC();
  return 0;
}

static int lpc43_pendsv(int irq, FAR void *context)
{
  (void)up_irq_save();
  err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int lpc43_errmonitor(int irq, FAR void *context)
{
  (void)up_irq_save();
  err("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int lpc43_reserved(int irq, FAR void *context)
{
  (void)up_irq_save();
  err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: lpc43_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void lpc43_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

/****************************************************************************
 * Name: lpc43_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int lpc43_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                         uintptr_t offset)
{
  DEBUGASSERT(irq >= LPC43_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= LPC43_IRQ_EXTINT)
    {
      /* NOTE: We assume that there are at least 32 interrupts */

      if (irq < (LPC43_IRQ_EXTINT + 32))
        {
           /* Interrupt in range {0-31} */

           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << (irq - LPC43_IRQ_EXTINT);
        }
#if LPC43M4_IRQ_NEXTINT > 95
#  error Extension to interrupt logic needed
#elif LPC43M4_IRQ_NEXTINT > 63
      else if (irq < (LPC43_IRQ_EXTINT + 64))
        {
           /* Interrupt in range {32-63} */

           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (irq - LPC43_IRQ_EXTINT - 32);
        }
      else if (irq < LPC43M4_IRQ_NIRQS)
        {
           /* Interrupt in range {64-LPC43M4_IRQ_NIRQS}, LPC43M4_IRQ_NIRQS <= 95 */

           *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
           *bit     = 1 << (irq - LPC43_IRQ_EXTINT - 64);
        }
#else /* if LPC43M4_IRQ_NEXTINT > 31 */
      else if (irq < LPC43M4_IRQ_NIRQS)
        {
           /* Interrupt in range {32-LPC43M4_IRQ_NIRQS}, LPC43M4_IRQ_NIRQS <= 63 */

           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (irq - LPC43_IRQ_EXTINT - 32);
        }
#endif
      else
        {
           /* Interrupt >= LPC43M4_IRQ_NIRQS */

          return ERROR; /* Invalid interrupt */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == LPC43_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == LPC43_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == LPC43_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == LPC43_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Complete initialization of the interrupt system and enable normal,
 *   interrupt processing.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
#ifdef CONFIG_DEBUG_FEATURES
  uint32_t regval;
#endif
  int num_priority_registers;

  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);
#if LPC43M4_IRQ_NEXTINT > 31
  putreg32(0, NVIC_IRQ32_63_ENABLE);
#if LPC43M4_IRQ_NEXTINT > 63
  putreg32(0, NVIC_IRQ64_95_ENABLE);
#endif
#endif

  /* Make sure that we are using the correct vector table.  The default
   * vector address is 0x0000:0000 but if we are executing code that is
   * positioned in SRAM or in external FLASH, then we may need to reset
   * the interrupt vector so that it refers to the table in SRAM or in
   * external FLASH.
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  up_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(LPC43_IRQ_SVCALL, up_svcall);
  irq_attach(LPC43_IRQ_HARDFAULT, up_hardfault);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
  /* up_prioritize_irq(LPC43_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
  lpc43_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(LPC43_IRQ_MEMFAULT, up_memfault);
  up_enable_irq(LPC43_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(LPC43_IRQ_NMI, lpc43_nmi);
#ifndef CONFIG_ARM_MPU
  irq_attach(LPC43_IRQ_MEMFAULT, up_memfault);
#endif
  irq_attach(LPC43_IRQ_BUSFAULT, lpc43_busfault);
  irq_attach(LPC43_IRQ_USAGEFAULT, lpc43_usagefault);
  irq_attach(LPC43_IRQ_PENDSV, lpc43_pendsv);
  irq_attach(LPC43_IRQ_DBGMONITOR, lpc43_errmonitor);
  irq_attach(LPC43_IRQ_RESERVED, lpc43_reserved);
#endif

  lpc43_dumpnvic("initial", LPC43M4_IRQ_NIRQS);

  /* If a debugger is connected, try to prevent it from catching hardfaults.
   * If CONFIG_ARMV7M_USEBASEPRI, no hardfaults are expected in normal
   * operation.
   */

#if defined(CONFIG_DEBUG_FEATURES) && !defined(CONFIG_ARMV7M_USEBASEPRI)
  regval  = getreg32(NVIC_DEMCR);
  regval &= ~NVIC_DEMCR_VCHARDERR;
  putreg32(regval, NVIC_DEMCR);
#endif

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (lpc43_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= LPC43_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }

  lpc43_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (lpc43_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= LPC43_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }

  lpc43_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  lpc43_clrpend(irq);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= LPC43_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < LPC43_IRQ_EXTINT)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= LPC43_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  lpc43_dumpnvic("prioritize", irq);
  return OK;
}
#endif
