#ifndef __RC__CONFIG__H__
#define __RC__CONFIG__H__

// PERFORMANCE SETTINGS
#define DOF 16
#define FOV_3
#define REMOVE_FISHEYE

// MOVEMENT SETTINGS
#define INCR_ROTATION    0.0001 // HIGHER IS FASTER
#define INCR_TRANSLATION 0.001  // HIGHER IS FASTER

// ------------------------ DO NOT TOUCH ------------------------
// INTERNAL CONFIGURATION DEFINES
#define M_3PI_2  4.71238898038468985769
#define FOV_INCR 0.0174533

// FOV CONFIGURATION TABLES
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

#ifdef FOV_4
#define FOV      120
#define FOV_HALF 60
#define FOV_RECT 4
#endif

#ifdef FOV_5
#define FOV      240
#define FOV_HALF 120
#define FOV_RECT 2
#endif

#endif
