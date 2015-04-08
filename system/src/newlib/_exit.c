//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// Released under MIT license, see
// http://micro-os-plus.livius.net/wiki/Main_Page


#include <stdlib.h>


void __attribute__((weak,noreturn)) abort(void) {
    while (1);
}
