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


