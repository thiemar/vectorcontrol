/*
Copyright (c) 2014 - 2015 by Thiemar Pty Ltd

This file is part of vectorcontrol.

vectorcontrol is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

vectorcontrol is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
vectorcontrol. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#ifdef NDEBUG
    #define esc_assert(__e) ((void)0)
#else
    #define esc_assert(__e) ((__e) ? (void)0 : __esc_assert_func (__FILE__, __LINE__, \
                                                               __ASSERT_FUNC, #__e))

    #ifndef __ASSERT_FUNC
        /* Use g++'s demangled names in C++.  */
        #if defined __cplusplus && defined __GNUC__
            #define __ASSERT_FUNC __PRETTY_FUNCTION__

        /* C99 requires the use of __func__, gcc also supports it.  */
        #elif defined __GNUC__ || __STDC_VERSION__ >= 199901L
            #define __ASSERT_FUNC __func__
        /* failed to detect __func__ support.  */
        #else
            #define __ASSERT_FUNC ((char *) 0)
        #endif
    #endif
#endif

extern void
__attribute__((noreturn))
__esc_assert_func (
    const char *file,
    int line,
    const char *func,
    const char *failedexpr
);
