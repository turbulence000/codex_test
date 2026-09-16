#ifndef _extern_h
#define _extern_h

#include "zf_common_headfile.h"


#define Ang2Rad 0.01745329252f
#define Rad2Ang 57.295779513f

int sgnf(float x);
int clip(int x, int low, int up);
unsigned short clip_u16(unsigned short x, unsigned short low, unsigned short up);
float fclip(float x, float low, float up);

#endif
