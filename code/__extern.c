#include "__extern.h"

int sgnf(float x) {return x > 0 ? 1 : x < 0 ? -1 : 0;}
int clip(int x, int low, int up) {return x > up ? up : x < low ? low : x;}
unsigned short clip_u16(unsigned short x, unsigned short low, unsigned short up) {return x > up ? up : x < low ? low : x;}
float fclip(float x, float low, float up) {return x > up ? up : x < low ? low : x;}
