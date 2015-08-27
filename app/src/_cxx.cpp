/*
Copyright (C) 2014-2015 Thiemar Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

void* __dso_handle = NULL;

namespace __gnu_cxx
{
    void
    __attribute__((noreturn))
    __verbose_terminate_handler();

    void
    __verbose_terminate_handler() {
        while (1);
    }
}

void operator delete (void *) {
}

extern "C" {
    void
    __attribute__((noreturn))
    __cxa_pure_virtual();

    void
    __cxa_pure_virtual() {
        while (1);
    }

    void _sbrk(void) {
    }

    int __aeabi_atexit(
        void *object,
        void (*destructor)(void *),
        void *dso_handle)
    {
        return 0;
    }
}


