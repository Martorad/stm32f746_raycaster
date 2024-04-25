#ifndef __RC__CONFIG__H__
#define __RC__CONFIG__H__

// PERFORMANCE SETTINGS
#define RES_3 // Vertical line resolution
#define SBR_0 // Skybox resolution - 0 for 16px, 1 for 32px

#define SHORTEST_LINE 5

// AESTHETIC SETTINGS
#define LINE_VERTICAL_SCALE 1.5

// MOVEMENT SETTINGS
#define LOOK_SPEED    3   // Higher is faster
#define MOVE_SPEED    0.1 // Higher is faster
#define P_HITBOX_SIZE 0.6 // In units of map squares

// DEBUG SETTINGS
//#define DEBUG_FULLBRIGHT

// <------------------------ DO NOT TOUCH ------------------------>
// INTERNAL CONFIGURATION DEFINES

// RESOLUTION CONFIGURATION TABLES
// FOV_RECT describes the width of each rectangle drawn on screen. Smaller width rectangles have higher fidelity, at the cost of speed. Odd width rectangles mean that symmetrical frames cannot be drawn.

#ifdef  RES_1
#define FOV       30
#define FOV_RECT  16
#define FOV_INCR  (2 * M_PI) / (360 * 0.5)
#define FOV_RANGE (360 * 0.5)
#endif

#ifdef  RES_2
#define FOV       60
#define FOV_RECT  8
#define FOV_INCR  (2 * M_PI) / (360 * 1)
#define FOV_RANGE (360 * 1)
#endif

#ifdef  RES_3
#define FOV       120
#define FOV_RECT  4
#define FOV_INCR  (2 * M_PI) / (360 * 2)
#define FOV_RANGE (360 * 2)
#endif

#ifdef  RES_4
#define FOV       240
#define FOV_RECT  2
#define FOV_INCR  (2 * M_PI) / (360 * 4)
#define FOV_RANGE (360 * 4)
#endif

#ifdef  RES_5
#define FOV       480
#define FOV_RECT  1
#define FOV_INCR  (2 * M_PI) / (360 * 8)
#define FOV_RANGE (360 * 8)
#endif

#endif
