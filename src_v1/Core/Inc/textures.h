/*
 * textures.h
 *
 *  Created on: Apr 8, 2024
 *      Author: mkiryakov
 */

#ifndef INC_TEXTURES_H_
#define INC_TEXTURES_H_

#define TEXTURE_SIZE 16

// Even textures are brightside, odd textures are darkside
const uint32_t _textures[5][256] = {
  { // Checkerboard
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,
    0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF,0xFFFF00FF
  },
  {
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,
    0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFF0000AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA,0xFFAA00AA
  },
  { // Bars
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,
    1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0
  },
  { // Window
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
  },
  { // Right arrow
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,
    0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,
    0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
  }
};

#endif /* INC_TEXTURES_H_ */
