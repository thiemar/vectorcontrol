//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// Released under MIT license, see
// http://micro-os-plus.livius.net/wiki/Main_Page

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>


// ----------------------------------------------------------------------------

void
__attribute__((section(".bootloader"),noreturn))
__assert_func (
    const char __attribute__((unused)) *file,
    int __attribute__((unused))  line,
    const char __attribute__((unused))  *func,
    const char __attribute__((unused))  *failedexpr)
{
  abort ();
  /* NOTREACHED */
}

// ----------------------------------------------------------------------------

// This is STM32 specific, but can be used on other platforms too.
// If you need it, add the following to your application header:

//#ifdef  USE_FULL_ASSERT
//#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
//void assert_failed(uint8_t* file, uint32_t line);
//#else
//#define assert_param(expr) ((void)0)
//#endif // USE_FULL_ASSERT

#if defined(USE_FULL_ASSERT)

void
assert_failed (uint8_t* file, uint32_t line);

// Called from the assert_param() macro, usually defined in the stm32f*_conf.h
void
__attribute__((section(".bootloader"),noreturn))
assert_failed (
    uint8_t __attribute__((unused))  *file,
    uint32_t __attribute__((unused))  line
) {
  abort ();
  /* NOTREACHED */
}

#endif // defined(USE_FULL_ASSERT)

// ----------------------------------------------------------------------------
