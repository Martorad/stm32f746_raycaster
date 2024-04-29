# Raycaster engine for STM32F7
## 1. Summary
This is a simple raycasting engine, designed on the STM32F746G-DISCO board. This discovery board features an LCD screen and some audio outputs, making it suitable for this task. The MCU is capable of 216MHz and has 320kb of RAM, along with 1Mb of flash. This represents a bit of a side project, the main purpose being to see just how far I can stretch these three things:

1. The STM32F7 series chip
2. Raycasting as a rendering technique
3. My sanity and sleep schedule

## 2. Features

- 480x272, 16-bit color graphics
- 16x16 texture mapping to walls and floors
- 288x16 skybox texturing
- Fisheye-corrected wall projection, with collision detection
- Fisheye-corrected floor projection
