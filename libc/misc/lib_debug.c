/****************************************************************************
 * libc/misc/lib_err.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016 Gregory Nutt. All rights reserved.
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

#include <stdarg.h>
#include <debug.h>

#include "libc.h"

#ifndef CONFIG_CPP_HAVE_VARARGS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: err, llerr, info
 *
 * Description:
 *  If the cross-compiler's pre-processor does not support variable
 * length arguments, then these additional APIs will be built.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int err(const char *format, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, format);
  ret = vsyslog(LOG_DEBUG, format, ap);
  va_end(ap);

  return ret;
}

#ifdef CONFIG_ARCH_LOWPUTC
int llerr(const char *format, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, format);
  ret = lowvsyslog(LOG_ERR, format, ap);
  va_end(ap);

  return ret;
}
#endif /* CONFIG_ARCH_LOWPUTC */
#endif /* CONFIG_DEBUG_FEATURES */

#ifdef CONFIG_DEBUG_WARN
int warn(const char *format, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, format);
  ret = vsyslog(LOG_WARNING, format, ap);
  va_end(ap);

  return ret;
}

#ifdef CONFIG_ARCH_LOWPUTC
int llwarn(const char *format, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, format);
  ret = lowvsyslog(LOG_DEBUG, format, ap);
  va_end(ap);

  return ret;
}
#endif /* CONFIG_ARCH_LOWPUTC */
#endif /* CONFIG_DEBUG_INFO */

#ifdef CONFIG_DEBUG_INFO
int info(const char *format, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, format);
  ret = vsyslog(LOG_DEBUG, format, ap);
  va_end(ap);

  return ret;
}

#ifdef CONFIG_ARCH_LOWPUTC
int llinfo(const char *format, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, format);
  ret = lowvsyslog(LOG_DEBUG, format, ap);
  va_end(ap);

  return ret;
}
#endif /* CONFIG_ARCH_LOWPUTC */
#endif /* CONFIG_DEBUG_INFO */

#endif /* CONFIG_CPP_HAVE_VARARGS */
