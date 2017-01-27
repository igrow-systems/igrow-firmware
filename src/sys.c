
#ifdef __cplusplus
extern "C" {
#endif
void
__cxa_atexit(void (*arg1)(void*), void* arg2, void* arg3)
{
}

extern void * __dso_handle;

__extension__ typedef int __guard __attribute__((mode (__DI__)));

int __cxa_guard_acquire(__guard *);
void __cxa_guard_release (__guard *);
void __cxa_guard_abort (__guard *);

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);}
void __cxa_guard_release (__guard *g) {*(char *)g = 1;}
void __cxa_guard_abort (__guard *g) {(void)g;}

void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}
#ifdef __cplusplus
}
#endif
