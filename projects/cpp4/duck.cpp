#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

#ifdef __cplusplus
}
#endif


#include "duck.hpp"


#ifdef __cplusplus

struct duck { };

class Duck : public duck {
public:
    Duck();
    ~Duck();
    void quack();
};

#endif


//__attribute__ ((section(".text")))
Duck::Duck() {}

//__attribute__ ((section(".text")))
Duck::~Duck() {}

//__attribute__ ((section(".text")))
void Duck::quack() {}

//__attribute__ ((section(".data")))
static Duck duck_obj;
static Duck *duck_obj_p;


#ifdef __cplusplus
extern "C" {
#endif

inline Duck* real(duck* d) { return static_cast<Duck*>(d); }

duck* new_duck()
{
	// There is not enough RAM to create sufficiently 
	// large .bss and .stack areas to support C++ new() 
	// dependence on underlying malloc and other services.
	//duck_obj_p = new Duck;
	duck_obj_p = &duck_obj;
	return duck_obj_p;
}

void delete_duck(duck* d) { delete real(d); }
void duck_quack(duck* d) { real(d)->quack(); }

#ifdef __cplusplus
}
#endif


static void* __dso_handle;

