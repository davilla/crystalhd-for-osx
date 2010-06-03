/********************************************************************
 * Copyright(c) 2010 4pi Analysis, Inc.
 *
 *  Name: linux_compatible.cpp
 *
 *  Description: linux compatible functions for dariwn.
 *
 *  AU
 *
 *  HISTORY:
 *
 ********************************************************************
 *
 * This file is part of libcrystalhd.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************/

#include <time.h>
#include <errno.h>
#include <mach/task.h>
#include <mach/mach_init.h>
#include <mach/semaphore.h>

#include "linux_compatible.h"

#undef sem_t
#undef sem_destroy
#undef sem_init
#undef sem_post
#undef sem_trywait
#undef sem_timedwait

#define sem_t semaphore_t

int sem_destroy(sem_t *sem)
{
  int rtn = -1;
  kern_return_t mach_rtn;
  mach_rtn = semaphore_destroy(mach_task_self(), *sem);
  if (mach_rtn == KERN_SUCCESS)
    rtn = 0;

  return(rtn);
}

int sem_init(sem_t *sem, int, unsigned int)
{
  int rtn = -1;
  kern_return_t mach_rtn;
  mach_rtn = semaphore_create(mach_task_self(), sem, SYNC_POLICY_FIFO, 0);
  if (mach_rtn == KERN_SUCCESS)
    rtn = 0;

  return(rtn);
}

int sem_post(sem_t *sem)
{
  int rtn = -1;
  kern_return_t mach_rtn;
  mach_rtn = semaphore_signal(*sem);
  if (mach_rtn == KERN_SUCCESS)
    rtn = 0;

  return(rtn);
}

int sem_trywait(sem_t *sem)
{
  int rtn = -1;
  kern_return_t mach_rtn;
  mach_timespec_t timeout_m_ts = { 0, 0 };
  mach_rtn = semaphore_timedwait(*sem, timeout_m_ts);
  if (mach_rtn == KERN_SUCCESS)
    rtn = 0;
  
  return(rtn);
}

int sem_timedwait(sem_t *sem, struct timespec *ts)
{
  int rtn = -1;
  kern_return_t mach_rtn;
  mach_timespec_t mach_ts;

  mach_ts.tv_sec = ts->tv_sec;
  mach_ts.tv_nsec = ts->tv_nsec;

  mach_rtn = semaphore_timedwait(*sem, mach_ts);
  if (mach_rtn == KERN_SUCCESS)
    rtn = 0;
  else if (mach_rtn == KERN_OPERATION_TIMED_OUT)
    errno = ETIMEDOUT;

  return(rtn);
}
