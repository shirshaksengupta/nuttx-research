/****************************************************************************
 * sched/init/os_start.c
 *
 *   Copyright (C) 2007-2014, 2016 Gregory Nutt. All rights reserved.
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

#include  <sys/types.h>
#include  <stdbool.h>
#include  <stdio.h>
#include  <string.h>
#include  <assert.h>
#include  <debug.h>

#include  <nuttx/arch.h>
#include  <nuttx/compiler.h>
#include  <nuttx/sched.h>
#include  <nuttx/fs/fs.h>
#include  <nuttx/net/net.h>
#include  <nuttx/lib.h>
#include  <nuttx/mm/mm.h>
#include  <nuttx/mm/shm.h>
#include  <nuttx/kmalloc.h>
#include  <nuttx/sched_note.h>
#include  <nuttx/init.h>

#include  "sched/sched.h"
#include  "signal/signal.h"
#include  "wdog/wdog.h"
#include  "semaphore/semaphore.h"
#ifndef CONFIG_DISABLE_MQUEUE
# include "mqueue/mqueue.h"
#endif
#ifndef CONFIG_DISABLE_PTHREAD
# include "pthread/pthread.h"
#endif
#include  "clock/clock.h"
#include  "timer/timer.h"
#include  "irq/irq.h"
#ifdef HAVE_TASK_GROUP
#include  "group/group.h"
#endif
#include  "init/init.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP
/* This set of all CPUs */

#  define SCHED_ALL_CPUS         ((1 << CONFIG_SMP_NCPUS) - 1)
#endif /* CONFIG_SMP */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Task Lists ***************************************************************/
/* The state of a task is indicated both by the task_state field of the TCB
 * and by a series of task lists.  All of these tasks lists are declared
 * below. Although it is not always necessary, most of these lists are
 * prioritized so that common list handling logic can be used (only the
 * g_readytorun, the g_pendingtasks, and the g_waitingforsemaphore lists
 * need to be prioritized).
 */

/* This is the list of all tasks that are ready to run.  This is a
 * prioritized list with head of the list holding the highest priority
 * (unassigned) task.  In the non-SMP cae, the head of this list is the
 * currently active task and the tail of this list, the lowest priority
 * task, is always the IDLE task.
 */

volatile dq_queue_t g_readytorun;

#ifdef CONFIG_SMP
/* In order to support SMP, the function of the g_readytorun list changes,
 * The g_readytorun is still used but in the SMP cae it will contain only:
 *
 *  - Only tasks/threads that are eligible to run, but not currently running,
 *    and
 *  - Tasks/threads that have not been assigned to a CPU.
 *
 * Otherwise, the TCB will be reatined in an assigned task list,
 * g_assignedtasks.  As its name suggests, on 'g_assignedtasks queue for CPU
 * 'n' would contain only tasks/threads that are assigned to CPU 'n'.  Tasks/
 * threads would be assigned a particular CPU by one of two mechanisms:
 *
 *  - (Semi-)permanently through an RTOS interfaces such as
 *    pthread_attr_setaffinity(), or
 *  - Temporarily through scheduling logic when a previously unassigned task
 *    is made to run.
 *
 * Tasks/threads that are assigned to a CPU via an interface like
 * pthread_attr_setaffinity() would never go into the g_readytorun list, but
 * would only go into the g_assignedtasks[n] list for the CPU 'n' to which
 * the thread has been assigned.  Hence, the g_readytorun list would hold
 * only unassigned tasks/threads.
 *
 * Like the g_readytorun list in in non-SMP case, each g_assignedtask[] list
 * is prioritized:  The head of the list is the currently active task on this
 * CPU.  Tasks after the active task are ready-to-run and assigned to this
 * CPU. The tail of this assigned task list, the lowest priority task, is
 * always the CPU's IDLE task.
 */

volatile dq_queue_t g_assignedtasks[CONFIG_SMP_NCPUS];
#endif

/* This is the list of all tasks that are ready-to-run, but cannot be placed
 * in the g_readytorun list because:  (1) They are higher priority than the
 * currently active task at the head of the g_readytorun list, and (2) the
 * currently active task has disabled pre-emption.
 */

volatile dq_queue_t g_pendingtasks;

/* This is the list of all tasks that are blocked waiting for a semaphore */

volatile dq_queue_t g_waitingforsemaphore;

/* This is the list of all tasks that are blocked waiting for a signal */

#ifndef CONFIG_DISABLE_SIGNALS
volatile dq_queue_t g_waitingforsignal;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-empty.
 */

#ifndef CONFIG_DISABLE_MQUEUE
volatile dq_queue_t g_waitingformqnotempty;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-full.
 */

#ifndef CONFIG_DISABLE_MQUEUE
volatile dq_queue_t g_waitingformqnotfull;
#endif

/* This is the list of all tasks that are blocking waiting for a page fill */

#ifdef CONFIG_PAGING
volatile dq_queue_t g_waitingforfill;
#endif

/* This the list of all tasks that have been initialized, but not yet
 * activated. NOTE:  This is the only list that is not prioritized.
 */

volatile dq_queue_t g_inactivetasks;

/* These are lists of dayed memory deallocations that need to be handled
 * within the IDLE loop or worker thread.  These deallocations get queued
 * by sched_kufree and sched_kfree() if the OS needs to deallocate memory
 * while it is within an interrupt handler.
 */

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
volatile sq_queue_t g_delayed_kfree;
#endif

#ifndef CONFIG_BUILD_KERNEL
/* REVISIT:  It is not safe to defer user allocation in the kernel mode
 * build.  Why?  Because the correct user context will not be in place
 * when these deferred de-allocations are performed.  In order to make this
 * work, we would need to do something like:  (1) move g_delayed_kufree
 * into the group structure, then traverse the groups to collect garbage
 * on a group-by-group basis.
 */

volatile sq_queue_t g_delayed_kufree;
#endif

/* This is the value of the last process ID assigned to a task */

volatile pid_t g_lastpid;

/* The following hash table is used for two things:
 *
 * 1. This hash table greatly speeds the determination of a new unique
 *    process ID for a task, and
 * 2. Is used to quickly map a process ID into a TCB.
 * It has the side effects of using more memory and limiting
 *
 * the number of tasks to CONFIG_MAX_TASKS.
 */

struct pidhash_s g_pidhash[CONFIG_MAX_TASKS];

/* This is a table of task lists.  This table is indexed by the task stat
 * enumeration type (tstate_t) and provides a pointer to the associated
 * static task list (if there is one) as well as a a set of attribute flags
 * indicating properities of the list, for example, if the list is an
 * ordered list or not.
 */

const struct tasklist_s g_tasklisttable[NUM_TASK_STATES] =
{
  {                                              /* TSTATE_TASK_INVALID */
    NULL,
    0
  },
  {                                              /* TSTATE_TASK_PENDING */
    &g_pendingtasks,
    TLIST_ATTR_PRIORITIZED
  },
#ifdef CONFIG_SMP
  {                                              /* TSTATE_TASK_READYTORUN */
    &g_readytorun,
    TLIST_ATTR_PRIORITIZED
  },
  {                                              /* TSTATE_TASK_ASSIGNED */
    g_assignedtasks,
    TLIST_ATTR_PRIORITIZED | TLIST_ATTR_INDEXED | TLIST_ATTR_RUNNABLE
  },
  {                                              /* TSTATE_TASK_RUNNING */
    g_assignedtasks,
    TLIST_ATTR_PRIORITIZED | TLIST_ATTR_INDEXED | TLIST_ATTR_RUNNABLE
  },
#else
  {                                              /* TSTATE_TASK_READYTORUN */
    &g_readytorun,
    TLIST_ATTR_PRIORITIZED | TLIST_ATTR_RUNNABLE
  },
  {                                              /* TSTATE_TASK_RUNNING */
    &g_readytorun,
    TLIST_ATTR_PRIORITIZED | TLIST_ATTR_RUNNABLE
  },
#endif
  {                                              /* TSTATE_TASK_INACTIVE */
    &g_inactivetasks,
    0
  },
  {                                              /* TSTATE_WAIT_SEM */
    &g_waitingforsemaphore,
    TLIST_ATTR_PRIORITIZED
  }
#ifndef CONFIG_DISABLE_SIGNALS
  ,
  {                                              /* TSTATE_WAIT_SIG */
    &g_waitingforsignal,
    0
  }
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  ,
  {                                              /* TSTATE_WAIT_MQNOTEMPTY */
    &g_waitingformqnotempty,
    TLIST_ATTR_PRIORITIZED
  },
  {                                              /* TSTATE_WAIT_MQNOTFULL */
    &g_waitingformqnotfull,
    TLIST_ATTR_PRIORITIZED
  }
#endif
#ifdef CONFIG_PAGING
  ,
  {                                              /* TSTATE_WAIT_PAGEFILL */
    &g_waitingforfill,
    TLIST_ATTR_PRIORITIZED
  }
#endif
};

/* This is the current initialization state.  The level of initialization
 * is only important early in the start-up sequence when certain OS or
 * hardware resources may not yet be available to the kernel logic.
 */

uint8_t g_os_initstate;  /* See enum os_initstate_e */

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This is an arry of task control block (TCB) for the IDLE thread of each
 * CPU.  For the non-SMP case, this is a a single TCB; For the SMP case,
 * there is one TCB per CPU.  NOTE: The system boots on CPU0 into the IDLE
 * task.  The IDLE task later starts the other CPUs and spawns the user
 * initialization task.  That user initialization task is responsible for
 * bringing up the rest of the system.
 */

#ifdef CONFIG_SMP
static struct task_tcb_s g_idletcb[CONFIG_SMP_NCPUS];
#else
static struct task_tcb_s g_idletcb[1];
#endif

/* This is the name of the idle task */

#ifdef CONFIG_SMP
static const char g_idlename[] = "CPU Idle";
#else
static const char g_idlename[] = "Idle Task";
#endif

/* This the IDLE idle threads argument list.  NOTE: Normally the argument
 * list is created on the stack prior to starting the task.  We have to
 * do things s little differently here for the IDLE tasks.
 */

#ifdef CONFIG_SMP
static FAR char *g_idleargv[CONFIG_SMP_NCPUS][2];
#else
static FAR char *g_idleargv[1][2];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_start
 *
 * Description:
 *   This function is called to initialize the operating system and to spawn
 *   the user initialization thread of execution.  This is the initial entry
 *   point into NuttX
 *
 * Input Parameters:
 *   None
 *
 * Returned value:
 *   Does not return.
 *
 ****************************************************************************/

void os_start(void)
{
#ifdef CONFIG_SMP
  int cpu;
#else
# define cpu 0
#endif
  int i;

  sllinfo("Entry\n");

  /* Boot up is complete */

  g_os_initstate = OSINIT_BOOT;

  /* Initialize RTOS Data ***************************************************/
  /* Initialize all task lists */

  dq_init(&g_readytorun);
  dq_init(&g_pendingtasks);
  dq_init(&g_waitingforsemaphore);
#ifndef CONFIG_DISABLE_SIGNALS
  dq_init(&g_waitingforsignal);
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  dq_init(&g_waitingformqnotfull);
  dq_init(&g_waitingformqnotempty);
#endif
#ifdef CONFIG_PAGING
  dq_init(&g_waitingforfill);
#endif
  dq_init(&g_inactivetasks);
#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
  sq_init(&g_delayed_kfree);
#endif
#ifndef CONFIG_BUILD_KERNEL
  sq_init(&g_delayed_kufree);
#endif

#ifdef CONFIG_SMP
  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      dq_init(&g_assignedtasks[i]);
    }
#endif

  /* Initialize the logic that determine unique process IDs. */

  g_lastpid = 0;
  for (i = 0; i < CONFIG_MAX_TASKS; i++)
    {
      g_pidhash[i].tcb = NULL;
      g_pidhash[i].pid = INVALID_PROCESS_ID;
    }

  /* Initialize the IDLE task TCB *******************************************/

#ifdef CONFIG_SMP
  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++, g_lastpid++)
#endif
    {
      FAR dq_queue_t *tasklist;
      int hashndx;

      /* Assign the process ID(s) of ZERO to the idle task(s) */

      hashndx                = PIDHASH(g_lastpid);
      g_pidhash[hashndx].tcb = &g_idletcb[cpu].cmn;
      g_pidhash[hashndx].pid = g_lastpid;

      /* Initialize a TCB for this thread of execution.  NOTE:  The default
       * value for most components of the g_idletcb are zero.  The entire
       * structure is set to zero.  Then only the (potentially) non-zero
       * elements are initialized. NOTE:  The idle task is the only task in
       * that has pid == 0 and sched_priority == 0.
       */

      bzero((void *)&g_idletcb[cpu], sizeof(struct task_tcb_s));
      g_idletcb[cpu].cmn.pid        = g_lastpid;
      g_idletcb[cpu].cmn.task_state = TSTATE_TASK_RUNNING;

      /* Set the entry point.  This is only for debug purposes.  NOTE: that
       * the start_t entry point is not saved.  That is acceptable, however,
       * becaue it can be used only for restarting a task: The IDLE task
       * cannot be restarted.
       */

#ifdef CONFIG_SMP
      if (cpu > 0)
        {
          g_idletcb[cpu].cmn.start      = os_idle_trampoline;
          g_idletcb[cpu].cmn.entry.main = os_idle_task;
        }
      else
#endif
        {
          g_idletcb[cpu].cmn.start      = (start_t)os_start;
          g_idletcb[cpu].cmn.entry.main = (main_t)os_start;
        }

      /* Set the task flags to indicate that this is a kernel thread and, if
       * configured for SMP, that this task is locked to this CPU.
       */

#ifdef CONFIG_SMP
      g_idletcb[cpu].cmn.flags = (TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_CPU_LOCKED);
      g_idletcb[cpu].cmn.cpu   = cpu;
#else
      g_idletcb[cpu].cmn.flags = TCB_FLAG_TTYPE_KERNEL;
#endif

#ifdef CONFIG_SMP
      /* Set the affinity mask to allow the thread to run on all CPUs.  No,
       * this IDLE thread can only run on its assigned CPU.  That is
       * enforced by the TCB_FLAG_CPU_LOCKED which overrides the affinity
       * mask.  This is essential because all tasks inherit the affinity
       * mask from their parent and, ultimately, the parent of all tasks is
       * the IDLE task.
       */

     g_idletcb[cpu].cmn.affinity = SCHED_ALL_CPUS;
#endif

#if CONFIG_TASK_NAME_SIZE > 0
      /* Set the IDLE task name */

#  ifdef CONFIG_SMP
      snprintf(g_idletcb[cpu].cmn.name, CONFIG_TASK_NAME_SIZE, "CPU%d IDLE", cpu);
#  else
      strncpy(g_idletcb[cpu].cmn.name, g_idlename, CONFIG_TASK_NAME_SIZE);
      g_idletcb[cpu].cmn.name[CONFIG_TASK_NAME_SIZE] = '\0';
#  endif
#endif

      /* Configure the task name in the argument list.  The IDLE task does
       * not really have an argument list, but this name is still useful
       * for things like the NSH PS command.
       *
       * In the kernel mode build, the arguments are saved on the task's
       * stack and there is no support that yet.
       */

#if CONFIG_TASK_NAME_SIZE > 0
      g_idleargv[cpu][0]  = g_idletcb[cpu].cmn.name;
#else
      g_idleargv[cpu][0]  = (FAR char *)g_idlename;
#endif /* CONFIG_TASK_NAME_SIZE */
      g_idleargv[cpu][1]  = NULL;
      g_idletcb[cpu].argv = &g_idleargv[cpu][0];

      /* Then add the idle task's TCB to the head of the corrent ready to
       * run list.
       */

#ifdef CONFIG_SMP
      tasklist = TLIST_HEAD(TSTATE_TASK_RUNNING, cpu);
#else
      tasklist = TLIST_HEAD(TSTATE_TASK_RUNNING);
#endif
      dq_addfirst((FAR dq_entry_t *)&g_idletcb[cpu], tasklist);

      /* Initialize the processor-specific portion of the TCB */

      up_initial_state(&g_idletcb[cpu].cmn);
    }

  /* Task lists are initialized */

  g_os_initstate = OSINIT_TASKLISTS;

  /* Initialize RTOS facilities *********************************************/
  /* Initialize the semaphore facility.  This has to be done very early
   * because many subsystems depend upon fully functional semaphores.
   */

  sem_initialize();

#if defined(MM_KERNEL_USRHEAP_INIT) || defined(CONFIG_MM_KERNEL_HEAP) || defined(CONFIG_MM_PGALLOC)
  /* Initialize the memory manager */

  {
    FAR void *heap_start;
    size_t heap_size;

#ifdef MM_KERNEL_USRHEAP_INIT
    /* Get the user-mode heap from the platform specific code and configure
     * the user-mode memory allocator.
     */

    up_allocate_heap(&heap_start, &heap_size);
    kumm_initialize(heap_start, heap_size);
#endif

#ifdef CONFIG_MM_KERNEL_HEAP
    /* Get the kernel-mode heap from the platform specific code and configure
     * the kernel-mode memory allocator.
     */

    up_allocate_kheap(&heap_start, &heap_size);
    kmm_initialize(heap_start, heap_size);
#endif

#ifdef CONFIG_MM_PGALLOC
    /* If there is a page allocator in the configuration, then get the page
     * heap information from the platform-specific code and configure the
     * page allocator.
     */

    up_allocate_pgheap(&heap_start, &heap_size);
    mm_pginitialize(heap_start, heap_size);
#endif
  }
#endif

  /* The memory manager is available */

  g_os_initstate = OSINIT_MEMORY;

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
  /* Initialize tasking data structures */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (task_initialize != NULL)
#endif
    {
      task_initialize();
    }
#endif

  /* Initialize the interrupt handling subsystem (if included) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (irq_initialize != NULL)
#endif
    {
      irq_initialize();
    }

  /* Initialize the watchdog facility (if included in the link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (wd_initialize != NULL)
#endif
    {
      wd_initialize();
    }

  /* Initialize the POSIX timer facility (if included in the link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (clock_initialize != NULL)
#endif
    {
      clock_initialize();
    }

#ifndef CONFIG_DISABLE_POSIX_TIMERS
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (timer_initialize != NULL)
#endif
    {
      timer_initialize();
    }
#endif

#ifndef CONFIG_DISABLE_SIGNALS
  /* Initialize the signal facility (if in link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (sig_initialize != NULL)
#endif
    {
      sig_initialize();
    }
#endif

#ifndef CONFIG_DISABLE_MQUEUE
  /* Initialize the named message queue facility (if in link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (mq_initialize != NULL)
#endif
    {
      mq_initialize();
    }
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Initialize the thread-specific data facility (if in link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (pthread_initialize != NULL)
#endif
    {
      pthread_initialize();
    }
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
  /* Initialize the file system (needed to support device drivers) */

  fs_initialize();
#endif

#ifdef CONFIG_NET
  /* Initialize the networking system.  Network initialization is
   * performed in two steps:  (1) net_setup() initializes static
   * configuration of the network support.  This must be done prior
   * to registering network drivers by up_initialize().  This step
   * cannot require upon any hardware-depending features such as
   * timers or interrupts.
   */

  net_setup();
#endif

  /* The processor specific details of running the operating system
   * will be handled here.  Such things as setting up interrupt
   * service routines and starting the clock are some of the things
   * that are different for each  processor and hardware platform.
   */

  up_initialize();

  /* Hardware resources are available */

  g_os_initstate = OSINIT_HARDWARE;

#ifdef CONFIG_NET
  /* Complete initialization the networking system now that interrupts
   * and timers have been configured by up_initialize().
   */

  net_initialize();
#endif

#ifdef CONFIG_MM_SHM
  /* Initialize shared memory support */

  shm_initialize();
#endif

  /* Initialize the C libraries.  This is done last because the libraries
   * may depend on the above.
   */

  lib_initialize();

  /* IDLE Group Initialization **********************************************/
  /* Announce that the CPU0 IDLE task has started */

  sched_note_start(&g_idletcb[0].cmn);

#ifdef CONFIG_SMP
  /* Initialize the IDLE group for the IDLE task of each CPU */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
#endif
    {
#ifdef HAVE_TASK_GROUP
      /* Allocate the IDLE group */

      DEBUGVERIFY(group_allocate(&g_idletcb[cpu], g_idletcb[cpu].cmn.flags));
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_SMP
     if (cpu > 0)
        {
          /* Clone stdout, stderr, stdin from the CPU0 IDLE task. */

          DEBUGVERIFY(group_setuptaskfiles(&g_idletcb[cpu]));
        }
      else
#endif
        {
          /* Create stdout, stderr, stdin on the CPU0 IDLE task.  These
           * will be inherited by all of the threads created by the CPU0
           * IDLE task.
           */

          DEBUGVERIFY(group_setupidlefiles(&g_idletcb[cpu]));
        }
#endif

#ifdef HAVE_TASK_GROUP
      /* Complete initialization of the IDLE group.  Suppress retention
       * of child status in the IDLE group.
       */

      DEBUGVERIFY(group_initialize(&g_idletcb[cpu]));
      g_idletcb[cpu].cmn.group->tg_flags = GROUP_FLAG_NOCLDWAIT;
#endif
}

#ifdef CONFIG_SMP
  /* Start all CPUs *********************************************************/

  /* A few basic sanity checks */

  DEBUGASSERT(this_cpu() == 0 && CONFIG_MAX_TASKS > CONFIG_SMP_NCPUS);

  /* Take the memory manager semaphore on this CPU so that it will not be
   * available on the other CPUs until we have finished initialization.
   */

  DEBUGVERIFY(kmm_trysemaphore());

  /* Then start the other CPUs */

  DEBUGVERIFY(os_smp_start());

#endif /* CONFIG_SMP */

  /* Bring Up the System ****************************************************/
  /* The OS is fully initialized and we are beginning multi-tasking */

  g_os_initstate = OSINIT_OSREADY;

  /* Create initial tasks and bring-up the system */

  DEBUGVERIFY(os_bringup());

#ifdef CONFIG_SMP
  /* Let other threads have access to the memory manager */

  kmm_givesemaphore();

#endif /* CONFIG_SMP */

  /* The IDLE Loop **********************************************************/
  /* When control is return to this point, the system is idle. */

  sinfo("CPU0: Beginning Idle Loop\n");
  for (; ; )
    {
      /* Perform garbage collection (if it is not being done by the worker
       * thread).  This cleans-up memory de-allocations that were queued
       * because they could not be freed in that execution context (for
       * example, if the memory was freed from an interrupt handler).
       */

#ifndef CONFIG_SCHED_WORKQUEUE
      /* We must have exclusive access to the memory manager to do this
       * BUT the idle task cannot wait on a semaphore.  So we only do
       * the cleanup now if we can get the semaphore -- this should be
       * possible because if the IDLE thread is running, no other task is!
       *
       * WARNING: This logic could have undesirable side-effects if priority
       * inheritance is enabled.  Imaginee the possible issues if the
       * priority of the IDLE thread were to get boosted!  Moral: If you
       * use priority inheritance, then you should also enable the work
       * queue so that is done in a safer context.
       */

      if (sched_have_garbage() && kmm_trysemaphore() == 0)
        {
          sched_garbage_collection();
          kmm_givesemaphore();
        }
#endif

      /* Perform any processor-specific idle state operations */

      up_idle();
    }
}
