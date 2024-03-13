#ifndef __RC__CONFIG__H__
#define __RC__CONFIG__H__

#define DOF 8
#define FOV_1

#ifdef FOV_1
#define FOV      60
#define FOV_HALF 30
#define FOV_RECT 8
#endif

#ifdef FOV_2
#define FOV      80
#define FOV_HALF 40
#define FOV_RECT 6
#endif

#ifdef FOV_3
#define FOV      96
#define FOV_HALF 48
#define FOV_RECT 5
#endif

#endif
