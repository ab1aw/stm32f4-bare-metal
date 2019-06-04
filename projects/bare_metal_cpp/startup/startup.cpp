// very simple startup code with definition of handlers for all cortex-m cores

typedef void (*ptr_func_t)();

// main application
extern void main_app();

// location of these variables is defined in linker script
extern unsigned __data_start__;
extern unsigned __data_end__;
extern unsigned __data_load__;

extern unsigned __bss_start__;
extern unsigned __bss_end__;

extern unsigned __heap_start__;

extern ptr_func_t __preinit_array_start[];
extern ptr_func_t __preinit_array_end[];

extern ptr_func_t __init_array_start[];
extern ptr_func_t __init_array_end[];

extern ptr_func_t __fini_array_start[];
extern ptr_func_t __fini_array_end[];


/** Copy default data to DATA section
 */
unsigned __data_load__;
void copy_data() {
    unsigned *src = &__data_load__;
    unsigned *dst = &__data_start__;
    while (dst < &__data_end__) {
        *dst++ = *src++;
    }
}

/** Erase BSS section
 */
void zero_bss() {
    unsigned *dst = &__bss_start__;
    while (dst < &__bss_end__) {
        *dst++ = 0;
    }
}

/** Fill heap memory
 */
unsigned __heap_start__;
void fill_heap(unsigned fill=0x45455246) {
    unsigned *dst = &__heap_start__;
    unsigned *msp_reg;
    __asm__("mrs %0, msp\n" : "=r" (msp_reg) );
    while (dst < msp_reg) {
        *dst++ = fill;
    }
}

/** Call constructors for static objects
 */
void call_init_array() {
    ptr_func_t *array = __preinit_array_start;
    while (array < __preinit_array_end) {
        (*array)();
        array++;
    }

    array = __init_array_start;
    while (array < __init_array_end) {
        (*array)();
        array++;
    }
}

/** Call destructors for static objects
 */
void call_fini_array() {
    ptr_func_t *array = __fini_array_start;
    while (array < __fini_array_end) {
        (*array)();
        array++;
    }
}

// reset handler
void RESET_handler() {
    copy_data();
    zero_bss();
    fill_heap();
    call_init_array();
    // run application
    main_app();
    // call destructors for static instances
    call_fini_array();
    // stop
    while (true);
}
