#ifndef IMAGEDATA_H
#define IMAGEDATA_H
#include <stdint.h>

#define NUM_SPRITES 27

const unsigned int digit_0 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_1 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff
};

const unsigned int digit_2 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_3 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_4 [] = {
0xffffffff, 0xffffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_5 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00000000, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_6 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00000000, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_7 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00ffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_8 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int digit_9 [] = {
0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00ffffff, 0x00ffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int test_sprite [] = {
0xffff0000, 0xffff0000, 0xffff0000, 0xff00ff00, 0xff00ff00, 0xff00ff00, 0xff0000ff, 0xff0000ff, 0xff0000ff, 0xffff00ff, 0xffff00ff, 0xffff00ff, 0xffffff00, 0xffffff00, 0xffffff00, 0xff00ffff, 0xff00ffff, 0xff00ffff, 0xffff0000, 0xffff0000, 0xffff0000, 0xff00ff00, 0xff00ff00, 0xff00ff00, 0xff0000ff, 0xff0000ff, 0xff0000ff, 0xffff00ff, 0xffff00ff, 0xffff00ff, 0xffffff00, 0xffffff00, 0xffffff00, 0xff00ffff, 0xff00ffff, 0xff00ffff, 0xffff0000, 0xffff0000, 0xffff0000, 0xff00ff00, 0xff00ff00, 0xff00ff00, 0xff0000ff, 0xff0000ff, 0xff0000ff, 0xffff00ff, 0xffff00ff, 0xffff00ff, 0xffffff00, 0xffffff00, 0xffffff00, 0xff00ffff, 0xff00ffff, 0xff00ffff, 0xffff0000, 0xffff0000, 0xffff0000, 0xff00ff00, 0xff00ff00, 0xff00ff00, 0xff0000ff, 0xff0000ff, 0xff0000ff, 0xffff00ff, 0xffff00ff, 0xffff00ff, 0xffffff00, 0xffffff00, 0xffffff00, 0xff00ffff, 0xff00ffff, 0xff00ffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

const unsigned int donut [] = {
0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff00f400, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff901c51, 0xff901c51, 0xff901c51, 0xffffffff, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff911c52, 0xff951d54, 0xff981e56, 0xff9a1e57, 0xff9a1e57, 0xff9a1e57, 0xff9a1e57, 0xff9b1e58, 0xff9d1f59, 0xff9e1f5a, 0xffa2205c, 0xffa6205e, 0xffa2205c, 0xffa2205c, 0xffa2205c, 0xffa2205c, 0xffffffff, 0xffa2205c, 0xffa3205c, 0xffa3205c, 0xffa3205c, 0xffa01f5b, 0xff9d1f59, 0xff9d1f59, 0xff0087ff, 0xff0087ff, 0xff9d1f59, 0xff9a1e57, 0xff971e55, 0xff931d53, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff00f400, 0xff00f400, 0xff8f1c51, 0xff901c51, 0xff901c51, 0xff901c52, 0xff901c52, 0xff901c52, 0xff901c52, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff931d53, 0xff991e57, 
0xffa01f5a, 0xffaa2160, 0xffb12364, 0xffab2261, 0xffa8215f, 0xffa5205e, 0xffa2205c, 0xff9f1f5a, 0xff9d1f59, 0xff9d1f59, 0xff9e1f59, 0xff9e1f59, 0xffa01f5b, 0xff0087ff, 0xff921d53, 0xff8f1c51, 0xff8f1c51, 0xffffffff, 0xffaa2160, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbb256a, 0xffb52366, 0xffb12364, 0xffb12364, 0xffb12364, 0xffb42366, 0xffb82468, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffff00, 0xffffff00, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbb256a, 0xffbb256a, 0xffba256a, 0xffba256a, 0xffba2469, 0xffba2469, 0xffbd256b, 0xff00f400, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbc256a, 0xffb72468, 0xffffffff, 0xffb42366, 0xffb42366, 0xffb42366, 0xffb42366, 0xffb82468, 0xffbb256a, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 0xffbd256b, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffb42366, 0xffaf2263, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff00f400, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffff00, 0xffffff00, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff00f400, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffff0017, 0xffbd256b, 0xffbd256b, 
0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff00f400, 0xff00f400, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff00f400, 0xff00f400, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffff00, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff00f400, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff00f400, 0xffbd256b, 
0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xff0087ff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffff00, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffff00, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffff00, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffb82468, 0xffffffff, 0xffae2263, 0xffb12364, 0xffb12364, 0xffb52467, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffff0017, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 
0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffff0017, 0xffbd256b, 0xffba2469, 0xffb72468, 0xffb72468, 0xffbb256a, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffffffff, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffb92469, 0xffb52367, 0xffb22365, 0xffb12364, 0xffb12364, 0xffb42366, 0xffbb256a, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbd256b, 0xffbc256a, 0xffb52366, 0xffb12364, 0xffad2262, 0xffffff00, 0xffffff00, 0xffa5205e, 0xffa5205e, 0xffa6205e, 0xffa6205e, 0xffa92160, 0xffad2262, 0xffb02364, 0xffb82468, 0xffbd256b, 0xffbd256b, 0xffbb256a, 0xffb42366, 0xffb12364, 0xffb32365, 0xffb72468, 0xffb72468, 0xffba256a, 0xffbd256b, 0xffb42366, 0xffae2263, 0xffab2161, 0xffa8215f, 0xffa8215f, 0xffae2263, 0xffff0017, 0xff921d52, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xffff0017, 0xffff0017, 0xffa1205b, 0xffa5205d, 0xffa5205e, 0xffa3205c, 0xff9f1f5a, 0xff991e57, 0xff981e56, 0xff981e56, 0xff981e56, 0xff971e56, 0xff971e56, 0xff9a1e57, 0xff9d1f59, 0xffa2205c, 0xffa3205c, 0xffa3205c, 0xffa6215e, 0xffad2262, 0xffff0017, 0xff971e56, 0xff901c52, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff911c52, 0xff971e55, 0xff9d1f59, 0xffa01f5a, 0xffa3205c, 0xffa6205e, 0xffa2205c, 0xffffff00, 0xffffff00, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff921d53, 0xffffffff, 0xff921d53, 0xff931d53, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff981e56, 0xff961d55, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xffffffff, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 
0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xffff0017, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xffffff00, 0xffffff00, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51, 0xff8f1c51
};

const unsigned int nyan1 [] = {
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0x00000000, 0x00000000, 0x00000000, 0xff000000, 0x00000000, 0xffb86307, 0xffb86307, 0xff950046, 0xff950046, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xffb86307, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0x00000000, 0xff000000, 0xff474747, 0xff000000, 0xffb86307, 0xffff050a, 0xff950046, 0xff000000, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xffff050a, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xfffeff00, 0xfffeff00, 0xff000000, 0xff000000, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0x00000000, 0x00000000, 0xff000000, 0xff474747, 0xff474747, 0xff000000, 0xff000000, 0xff000000, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffff050a, 0xff950046, 0xffb86307, 0xff000000, 0xff000000, 0xff474747, 0xff474747, 0xff000000, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 
0xff00ff23, 0xff000000, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff000000, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff000000, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00deff, 0xff00deff, 0xff00deff, 0x00000000, 0x00000000, 0x00000000, 0xff474747, 0xff474747, 0xff000000, 0xffffffff, 0xff474747, 0xff000000, 0xffffffff, 0xff474747, 0xff474747, 0xff000000, 0xffff050a, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0x00000000, 0xff474747, 0xffff63ac, 0xff000000, 0xff000000, 0xff474747, 0xff000000, 0xff000000, 0xffff63ac, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xffff050a, 0xff950046, 0xff950046, 0xffb86307, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0x00000000, 0x00000000, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff1e1e1e, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xffb86307, 0xff000000, 0xff000000, 0x00000000, 0x00000000, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 
0x00000000, 0x00000000, 0xff000000, 0xff474747, 0xff474747, 0xff1e1e1e, 0xff1e1e1e, 0xff1e1e1e, 0xff474747, 0xff474747, 0xff000000, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xff717171, 0xff717171, 0xff000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xff000000, 0xff000000, 0xff000000, 0xff717171, 0xff717171, 0xff000000, 0xff717171, 0xff000000, 0x00000000, 0x00000000, 0xff000000, 0xff717171, 0xff717171, 0xff000000, 0xff717171, 0xff717171, 0xff000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

const unsigned int nyan2 [] = {
0x00000000, 0xff000000, 0x00000000, 0x00000000, 0xffb86307, 0xffb86307, 0xffb86307, 0xff000000, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xff000000, 0xff474747, 0xff000000, 0xffb86307, 0xffb86307, 0xff950046, 0xff000000, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xffb86307, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000, 0x00000000, 0x00000000, 0xff000000, 0xff474747, 0xff474747, 0xff000000, 0xff000000, 0xff000000, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xffff050a, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0xffff8100, 0x00000000, 0xff000000, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffff050a, 0xff950046, 0xffb86307, 0xff000000, 0xff000000, 0xff000000, 0xfffeff00, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0xfffeff00, 0x00000000, 
0x00000000, 0xff474747, 0xff474747, 0xff000000, 0xffffffff, 0xff474747, 0xff000000, 0xffffffff, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff000000, 0xff474747, 0xff474747, 0xff474747, 0xff000000, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff00ff23, 0xff474747, 0xffff63ac, 0xff000000, 0xff000000, 0xff474747, 0xff000000, 0xff000000, 0xffff63ac, 0xff474747, 0xff000000, 0xffff050a, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xff000000, 0xff000000, 0xff474747, 0xff474747, 0xff000000, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff00deff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff00deff, 0xff00deff, 0xff00deff, 0x00000000, 0x00000000, 0x00000000, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff1e1e1e, 0xff474747, 0xff474747, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xffff050a, 0xff950046, 0xff950046, 0xffb86307, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0xff4b00ff, 0x00000000, 0xff000000, 0xff474747, 0xff474747, 0xff1e1e1e, 0xff1e1e1e, 0xff1e1e1e, 0xff474747, 0xff474747, 0xff000000, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xff950046, 0xffb86307, 0xffb86307, 0xff717171, 0xff717171, 0xff000000, 0xffc500ff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffc500ff, 0xffc500ff, 0xffc500ff, 0xffc500ff, 
0x00000000, 0x00000000, 0x00000000, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xff000000, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xffb86307, 0xff000000, 0xff717171, 0xff717171, 0xff000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xff000000, 0xff717171, 0xff717171, 0xff000000, 0xff717171, 0xff000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xff000000, 0xff717171, 0xff717171, 0xff000000, 0xff000000, 0xff000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

const unsigned int nyan_pop1 [] = {
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

const unsigned int nyan_pop2 [] = {
0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

const unsigned int nyan_pop3 [] = {
0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000
};

const unsigned int nyan_pop4 [] = {
0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x00000000
};

const unsigned int lgbtq [] = {
0xfffe0000, 0xfffe0000, 0xffff8c01, 0xffff8c01, 0xfffeed01, 0xfffeed01, 0xff018112, 0xff018112, 0xff014cff, 0xff800283
};

const unsigned int bi [] = {
0xffa2185c, 0xffa2185c, 0xffa2185c, 0xffa2185c, 0xff4f0096, 0xff4f0096, 0xff000ea3, 0xff000ea3, 0xff000ea3, 0xff000ea3
};

const unsigned int trans [] = {
0xff00deff, 0xff00deff, 0xffff3a96, 0xffff3a96, 0xffffffff, 0xffffffff, 0xffff3a96, 0xffff3a96, 0xff00deff, 0xff00deff
};

const unsigned int nb [] = {
0xffffda00, 0xffffda00, 0xffffda00, 0xffffffff, 0xffffffff, 0xff8000ff, 0xff8000ff, 0xff8000ff, 0xff171717, 0xff171717
};

const unsigned int pan [] = {
0xffee4894, 0xffee4894, 0xffee4894, 0xffee4894, 0xfffdd906, 0xfffdd906, 0xfffdd906, 0xff4ba7da, 0xff4ba7da, 0xff4ba7da
};

const unsigned int queer [] = {
0xffa542ec, 0xffa542ec, 0xffa542ec, 0xffa542ec, 0xffffffff, 0xffffffff, 0xffffffff, 0xff23912b, 0xff23912b, 0xff23912b
};

const unsigned int ace [] = {
0xff000000, 0xff000000, 0xff393939, 0xff393939, 0xff393939, 0xffffffff, 0xffffffff, 0xff800080, 0xff800080, 0xff800080
};

const unsigned int lesbian [] = {
0xffff0055, 0xffff0055, 0xffff4992, 0xffff62af, 0xffffffff, 0xffffffff, 0xffffaad5, 0xffff4b54, 0xffd41701, 0xffd41701
};

const unsigned int gay [] = {
0xff00a338, 0xff00a338, 0xff00bf6e, 0xff66d597, 0xffffffff, 0xffffffff, 0xff4995dd, 0xff313eab, 0xff0f1c78, 0xff0f1c78
};

const uint16_t spriteArrayLength = NUM_SPRITES;
static const uint32_t *spriteArray[NUM_SPRITES] = {
    digit_0,
    digit_1,
    digit_2,
    digit_3,
    digit_4,
    digit_5,
    digit_6,
    digit_7,
    digit_8,
    digit_9,
    test_sprite,
    donut,
    nyan1,
    nyan2,
    nyan_pop1,
    nyan_pop2,
    nyan_pop3,
    nyan_pop4,
    lgbtq,
    bi,
    trans,
    nb,
    pan,
    queer,
    ace,
    lesbian,
    gay
};
const uint16_t spriteSizes[NUM_SPRITES][2] {
    {5,10},
    {3,10},
    {5,10},
    {5,10},
    {5,10},
    {5,10},
    {5,10},
    {5,10},
    {5,10},
    {5,10},
    {18,8},
    {84,10},
    {38,10},
    {38,10},
    {7,7},
    {7,7},
    {7,7},
    {7,7},
    {1,10},
    {1,10},
    {1,10},
    {1,10},
    {1,10},
    {1,10},
    {1,10},
    {1,10},
    {1,10}
};
#endif