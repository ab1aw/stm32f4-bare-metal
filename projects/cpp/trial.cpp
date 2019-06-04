

#ifdef __cplusplus
extern "C" {
#endif

typedef struct duck duck;

duck* new_duck(int feet);
void delete_duck(duck* d);
void duck_quack(duck* d, float volume);

#ifdef __cplusplus
}
#endif


struct duck { };

class Duck : public duck {
public:
    Duck(int feet);
    ~Duck();

    void quack(float volume);
};

inline Duck* real(duck* d) { return static_cast<Duck*>(d); }

#ifdef __cplusplus
extern "C" {
#endif
duck* new_duck(int feet) { return new Duck(feet); }
void delete_duck(duck* d) { delete real(d); }
void duck_quack(duck* d, float volume) { real(d)->quack(volume); }
#ifdef __cplusplus
}
#endif
