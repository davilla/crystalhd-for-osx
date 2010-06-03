/********************************************************************
 * Copyright(c) 2010 4pi Analysis, Inc.
 *
 *  Name: linux_compatible.h
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

#ifndef _LINUX_COMPATIBLE_H_
#define _LINUX_COMPATIBLE_H_

#undef sem_t
#undef sem_destroy
#undef sem_init
#undef sem_post
#undef sem_trywait
#undef sem_timedwait

#include <mach/semaphore.h>
#define sem_t semaphore_t

int sem_init(sem_t *sem, int, unsigned int);
int sem_destroy(sem_t *sem);
int sem_post(sem_t *sem);
int sem_trywait(sem_t *sem);
int sem_timedwait(sem_t *sem, struct timespec *ts);

#endif
