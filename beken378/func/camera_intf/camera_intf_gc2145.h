#ifndef __CAMERA_INTF_GC2145_H__
#define __CAMERA_INTF_GC2145_H__

// interface
int gc2145_probe(void);
int gc2145_reset(void);
void gc2145_init(int mode);
void gc2145_grayscale(int mode);
unsigned char gc2145_fps(unsigned char fps);
void gc2145_user_reg_set(unsigned char *user_reg, unsigned char num);

#endif // __CAMERA_INTF_GC2145_H__
