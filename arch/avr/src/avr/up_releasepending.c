/****************************************************************************
 *  arch/avr/src/avr/up_releasepending.c
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_release_pending
 *
 * Description:
 *   Release and ready-to-run tasks that have
 *   collected in the pending task list.  This can call a
 *   context switch if a new task is placed at the head of
 *   the ready to run list.
 *
 ****************************************************************************/

void up_release_pending(void)
{
  struct tcb_s *rtcb = this_task();

  sllerr("From TCB=%p\n", rtcb);

  /* Merge the g_pendingtasks list into the ready-to-run task list */

  /* sched_lock(); */
  if (sched_mergepending())
    {
      /* The currently active task has changed!  We will need to switch
       * contexts.
       */

      /* Update scheduler parameters */

      sched_suspend_scheduler(rtcb);

      if (g_current_regs)
        {
          /* Yes, then we have to do things differently.
           * Just copy the g_current_regs into the OLD rtcb.
           */

           up_savestate(rtcb->xcp.regs);

          /* Restore the exception context of the rtcb at the (new) head
           * of the ready-to-run task list.
           */

          rtcb = this_task();

          /* Update scheduler parameters */

          sched_resume_scheduler(rtcb);

          /* Then switch contexts */

          up_restorestate(rtcb->xcp.regs);
        }

      /* No, then we will need to perform the user context switch */

      else
        {
          struct tcb_s *nexttcb = this_task();

          /* Update scheduler parameters */

          sched_resume_scheduler(nexttcb);

          /* Switch context to the context of the task at the head of the
           * ready to run list.
           */

          up_switchcontext(rtcb->xcp.regs, nexttcb->xcp.regs);

          /* up_switchcontext forces a context switch to the task at the
           * head of the ready-to-run list.  It does not 'return' in the
           * normal sense.  When it does return, it is because the blocked
           * task is again ready to run and has execution priority.
           */
        }
    }
}

