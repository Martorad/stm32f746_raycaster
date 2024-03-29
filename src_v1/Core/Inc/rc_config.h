#ifndef __RC__CONFIG__H__
#define __RC__CONFIG__H__

// PERFORMANCE SETTINGS
#define RES_5
#define REMOVE_FISHEYE

#define DOF 24

// AESTHETIC SETTINGS
#define LINE_VERTICAL_SCALE 1.5

// MOVEMENT SETTINGS
#define INCR_ROTATION    0.0000004   // HIGHER IS FASTER
#define INCR_TRANSLATION 0.00000012  // HIGHER IS FASTER

// DEBUG SETTINGS
//#define DEBUG_FULLBRIGHT

// <------------------------ DO NOT TOUCH ------------------------>
// INTERNAL CONFIGURATION DEFINES
#define M_3PI_2  4.71238898038468985769
#define R_HIT    0xAAAA
#define L_COEFF  0.166 // Lighting coefficient, should be 1 / (DOF - FOG_OF_WAR_CUTOFF(18))

// RESOLUTION CONFIGURATION TABLES
// FOV_RECT describes the width of each rectangle drawn on screen. Smaller width rectangles have higher fidelity, at the cost of speed. Odd width rectangles mean that symmetrical frames cannot be drawn.
#ifdef  RES_N
#define FOV      10
#define FOV_HALF 5
#define FOV_RECT 48
#define FOV_INCR 0.1047198
#endif

#ifdef  RES_0
#define FOV      30
#define FOV_HALF 15
#define FOV_RECT 16
#define FOV_INCR 0.0349066
#endif

#ifdef  RES_1
#define FOV      60
#define FOV_HALF 30
#define FOV_RECT 8
#define FOV_INCR 0.0174533
#endif

#ifdef  RES_2
#define FOV      80
#define FOV_HALF 40
#define FOV_RECT 6
#define FOV_INCR 0.013089975
#endif

#ifdef  RES_3
#define FOV      96
#define FOV_HALF 48
#define FOV_RECT 5
#define FOV_INCR 0.0109083125
#endif

#ifdef  RES_4
#define FOV      120
#define FOV_HALF 60
#define FOV_RECT 4
#define FOV_INCR 0.00872665
#endif

#ifdef  RES_5
#define FOV      160
#define FOV_HALF 80
#define FOV_RECT 3
#define FOV_INCR 0.005817767
#endif

#ifdef  RES_6
#define FOV      240
#define FOV_HALF 120
#define FOV_RECT 2
#define FOV_INCR 0.004363325
#endif

#ifdef  RES_7
#define FOV      480
#define FOV_HALF 240
#define FOV_RECT 1
#define FOV_INCR 0.0021816625
#endif

#endif
