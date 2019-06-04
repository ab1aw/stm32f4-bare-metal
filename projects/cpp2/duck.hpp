

#ifdef __cplusplus
extern "C" {
#endif

typedef struct duck duck;

duck* new_duck(void);
/*void delete_duck(duck* d);*/
void duck_quack(duck* d);

#ifdef __cplusplus
}
#endif

