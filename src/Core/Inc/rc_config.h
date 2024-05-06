#ifndef __RC__CONFIG__H__
#define __RC__CONFIG__H__

// PERFORMANCE SETTINGS
#define RES_MED // Vertical line resolution
#define SBR_0   // Skybox resolution - 0 for 16px, 1 for 32px

#define SHORTEST_LINE 5

// AESTHETIC SETTINGS
#define LINE_VERTICAL_SCALE 1.5

// MOVEMENT SETTINGS
#define P_LOOK_SPEED  5    // Higher is faster
#define P_HITBOX_SIZE 0.6  // In units of map squares
#define P_ACCEL       0.01
#define P_FRICTION    0.08

// DEBUG SETTINGS
//#define DEBUG_FULLBRIGHT

// <------------------------ DO NOT TOUCH ------------------------>
// INTERNAL CONFIGURATION DEFINES

// RESOLUTION CONFIGURATION TABLES
// RECT_Y describes the width of each rectangle column drawn on screen. Smaller width rectangles have higher fidelity, at the cost of speed. Odd width rectangles mean that symmetrical frames cannot be drawn.
#ifdef  RES_MIN
#define RAYS      160
#define RECT_Y    3
#define ANG_INCR  (2 * M_PI) / (360 * 3)
#define ANG_RANGE (360 * 3)
#endif

#ifdef  RES_MED
#define RAYS      240
#define RECT_Y    2
#define ANG_INCR  (2 * M_PI) / (360 * 4)
#define ANG_RANGE (360 * 4)
#endif

#ifdef  RES_MAX
#define RAYS      480
#define RECT_Y    1
#define ANG_INCR  (2 * M_PI) / (360 * 8)
#define ANG_RANGE (360 * 8)
#endif

#endif
