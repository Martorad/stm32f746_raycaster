/*
 * textures.h
 *
 *  Created on: Apr 8, 2024
 *      Author: mkiryakov
 */

#ifndef INC_TEXTURES_H_
#define INC_TEXTURES_H_

#define TEXTURE_SIZE            16
#define TEXTURE_SIZE_RECIPROCAL 0.0625

#define COLOR_SKY    0xFF55C3D9
#define COLOR_GROUND 0xFF0A8214

// Even textures are brightside, odd textures are darkside
const uint32_t _textures[8][256] = {
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
  { // Some weird muck
    0xFF6E6E6C,0xFF557A55,0xFF557A55,0xFF557A55,0xFF78845A,0xFF767376,0xFF557A55,0xFF746E74,0xFF545454,0xFF545454,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF6E6E6C,
    0xFF6E6E6C,0xFF557A55,0xFF6E6E6C,0xFF78845A,0xFF78845A,0xFF767376,0xFF557A55,0xFF767376,0xFF746E74,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF767376,
    0xFF767376,0xFF767376,0xFF767376,0xFF78845A,0xFF767376,0xFF767376,0xFF78845A,0xFF767376,0xFF767376,0xFF557A55,0xFF746E74,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF545454,
    0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF78845A,0xFF767376,0xFF78845A,0xFF746E74,0xFF746E74,0xFF746E74,0xFF545454,
    0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF6E6E6C,0xFF6E6E6C,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF767376,0xFF767376,0xFF767376,0xFF767376,
    0xFF557A55,0xFF746E74,0xFF746E74,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF557A55,0xFF767376,0xFF767376,0xFF767376,0xFF767376,
    0xFF557A55,0xFF557A55,0xFF767376,0xFF746E74,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF557A55,
    0xFF557A55,0xFF78845A,0xFF767376,0xFF767376,0xFF746E74,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,
    0xFF557A55,0xFF78845A,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF78845A,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF767376,
    0xFF78845A,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF78845A,0xFF767376,0xFF746E74,0xFF746E74,0xFF746E74,0xFF557A55,0xFF545454,0xFF545454,0xFF545454,0xFF767376,
    0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF78845A,0xFF746E74,0xFF545454,0xFF545454,
    0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF78845A,0xFF767376,0xFF767376,0xFF545454,
    0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF6E6E6C,0xFF767376,0xFF767376,
    0xFF767376,0xFF746E74,0xFF746E74,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF557A55,0xFF767376,0xFF767376,0xFF767376,
    0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF746E74,0xFF746E74,0xFF746E74,0xFF545454,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF767376,0xFF767376,0xFF767376,
    0xFF78845A,0xFF767376,0xFF767376,0xFF767376,0xFF6E6E6C,0xFF6E6E6C,0xFF767376,0xFF767376,0xFF767376,0xFF746E74,0xFF545454,0xFF545454,0xFF545454,0xFF767376,0xFF767376,0xFF6E6E6C
  },
  {
    0xFF2F302C,0xFF0C400B,0xFF0C400B,0xFF0C400B,0xFF3D4F12,0xFF3B363B,0xFF0C400B,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF2F302C,
    0xFF2F302C,0xFF0C400B,0xFF2F302C,0xFF3D4F12,0xFF3D4F12,0xFF3B363B,0xFF0C400B,0xFF3B363B,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF3B363B,
    0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,
    0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3D4F12,0xFF3B363B,0xFF3D4F12,0xFF382F38,0xFF382F38,0xFF382F38,0xFF0A0A0A,
    0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF2F302C,0xFF2F302C,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,
    0xFF0C400B,0xFF382F38,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF0C400B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,
    0xFF0C400B,0xFF0C400B,0xFF3B363B,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF0C400B,
    0xFF0C400B,0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,
    0xFF0C400B,0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3D4F12,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,
    0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3D4F12,0xFF3B363B,0xFF382F38,0xFF382F38,0xFF382F38,0xFF0C400B,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,
    0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3D4F12,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,
    0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF0A0A0A,
    0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF2F302C,0xFF3B363B,0xFF3B363B,
    0xFF3B363B,0xFF382F38,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF0C400B,0xFF3B363B,0xFF3B363B,0xFF3B363B,
    0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF382F38,0xFF382F38,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF3B363B,
    0xFF3D4F12,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF2F302C,0xFF2F302C,0xFF3B363B,0xFF3B363B,0xFF3B363B,0xFF382F38,0xFF0A0A0A,0xFF0A0A0A,0xFF0A0A0A,0xFF3B363B,0xFF3B363B,0xFF2F302C
  },
  { // Oak planks
    0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFBC9862,0xFFBC9862,0xFFB4905A,0xFF9F844D,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF9F844D,
    0xFFBC9862,0xFFBC9862,0xFFB4905A,0xFF735E39,0xFFB4905A,0xFFB4905A,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFB4905A,0xFF9F844D,0xFFB4905A,0xFFBC9862,0xFF9F844D,
    0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFB4905A,0xFFB4905A,0xFF735E39,0xFFBC9862,0xFFBC9862,0xFFB4905A,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF9F844D,
    0xFF695433,0xFF695433,0xFF695433,0xFF735E39,0xFF735E39,0xFF735E39,0xFF735E39,0xFF735E39,0xFF735E39,0xFF735E39,0xFF695433,0xFF695433,0xFF695433,0xFF695433,0xFF735E39,0xFF735E39,
    0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF735E39,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFBC9862,
    0xFF9F844D,0xFF9F844D,0xFF735E39,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFBC9862,0xFF9F844D,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,
    0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFB4905A,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF735E39,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,
    0xFF735E39,0xFF735E39,0xFF695433,0xFF695433,0xFF695433,0xFF695433,0xFF695433,0xFF735E39,0xFF735E39,0xFF735E39,0xFF695433,0xFF695433,0xFF695433,0xFF695433,0xFF735E39,0xFF735E39,
    0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF735E39,0xFF9F844D,0xFFBC9862,0xFFBC9862,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF9F844D,
    0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFBC9862,0xFFB4905A,0xFFB4905A,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF735E39,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,
    0xFFBC9862,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFFB4905A,0xFF9F844D,
    0xFF735E39,0xFF695433,0xFF735E39,0xFF735E39,0xFF695433,0xFF695433,0xFF695433,0xFF735E39,0xFF735E39,0xFF735E39,0xFF735E39,0xFF735E39,0xFF695433,0xFF695433,0xFF695433,0xFF735E39,
    0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF735E39,0xFF9F844D,0xFF9F844D,0xFFB4905A,0xFFBC9862,0xFFBC9862,0xFFBC9862,
    0xFFB4905A,0xFF9F844D,0xFF735E39,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFFBC9862,0xFFB4905A,0xFFB4905A,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFB4905A,
    0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFF9F844D,0xFF9F844D,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFFBC9862,0xFFBC9862,0xFF9F844D,0xFFB4905A,0xFFBC9862,0xFFBC9862,0xFFBC9862,0xFFBC9862,
    0xFF735E39,0xFF735E39,0xFF735E39,0xFF7C623E,0xFF695433,0xFF695433,0xFF7C623E,0xFF695433,0xFF735E39,0xFF735E39,0xFF735E39,0xFF695433,0xFF4C3D26,0xFF695433,0xFF695433,0xFF735E39
  },
  {
    0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF8C7149,0xFF8C7149,0xFF866B43,0xFF766239,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF766239,
    0xFF8C7149,0xFF8C7149,0xFF866B43,0xFF55462A,0xFF866B43,0xFF866B43,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF866B43,0xFF766239,0xFF866B43,0xFF8C7149,0xFF766239,
    0xFF8C7149,0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF866B43,0xFF866B43,0xFF55462A,0xFF8C7149,0xFF8C7149,0xFF866B43,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF766239,
    0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF55462A,0xFF55462A,0xFF55462A,0xFF55462A,0xFF55462A,0xFF55462A,0xFF55462A,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF55462A,0xFF55462A,
    0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF8C7149,0xFF766239,0xFF766239,0xFF55462A,0xFF866B43,0xFF866B43,0xFF866B43,0xFF8C7149,
    0xFF766239,0xFF766239,0xFF55462A,0xFF866B43,0xFF866B43,0xFF866B43,0xFF8C7149,0xFF766239,0xFF866B43,0xFF866B43,0xFF866B43,0xFF866B43,0xFF766239,0xFF766239,0xFF766239,0xFF766239,
    0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF866B43,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,0xFF55462A,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,
    0xFF55462A,0xFF55462A,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF55462A,0xFF55462A,0xFF55462A,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF55462A,0xFF55462A,
    0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF55462A,0xFF766239,0xFF8C7149,0xFF8C7149,0xFF866B43,0xFF866B43,0xFF866B43,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF766239,
    0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF8C7149,0xFF866B43,0xFF866B43,0xFF766239,0xFF766239,0xFF766239,0xFF55462A,0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,
    0xFF8C7149,0xFF866B43,0xFF866B43,0xFF866B43,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF866B43,0xFF866B43,0xFF866B43,0xFF866B43,0xFF866B43,0xFF766239,
    0xFF55462A,0xFF4E3E26,0xFF55462A,0xFF55462A,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF55462A,0xFF55462A,0xFF55462A,0xFF55462A,0xFF55462A,0xFF4E3E26,0xFF4E3E26,0xFF4E3E26,0xFF55462A,
    0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF8C7149,0xFF766239,0xFF766239,0xFF55462A,0xFF766239,0xFF766239,0xFF866B43,0xFF8C7149,0xFF8C7149,0xFF8C7149,
    0xFF866B43,0xFF766239,0xFF55462A,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF8C7149,0xFF866B43,0xFF866B43,0xFF766239,0xFF766239,0xFF766239,0xFF766239,0xFF866B43,
    0xFF8C7149,0xFF8C7149,0xFF766239,0xFF766239,0xFF766239,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF8C7149,0xFF8C7149,0xFF766239,0xFF866B43,0xFF8C7149,0xFF8C7149,0xFF8C7149,0xFF8C7149,
    0xFF55462A,0xFF55462A,0xFF55462A,0xFF5C492E,0xFF4E3E26,0xFF4E3E26,0xFF5C492E,0xFF4E3E26,0xFF55462A,0xFF55462A,0xFF55462A,0xFF4E3E26,0xFF382D1C,0xFF4E3E26,0xFF4E3E26,0xFF55462A
  },
  { // Cobblestone
    0xEBC7CAC8,0xF5C5CAC6,0xF5C2C6C3,0xF5C0C4C1,0xF5878887,0xF57B7C7B,0xF5494B49,0xF5404240,0xF54A4D4A,0xF5BABEBB,0xF5C6CAC7,0xF5BFC2C0,0xF5838583,0xF54B4D4B,0xF5B3B7B4,0xF58B8D8B,
    0xF7BDC2BE,0xFFC2C6C3,0xFF959896,0xFF7C7D7C,0xFF191919,0xFF4D4F4D,0xFFB5B9B6,0xFFB0B4B1,0xFF4C4E4C,0xFF7D7E7D,0xFF8E908F,0xFF717272,0xFF595959,0xFF161616,0xFF7B7C7B,0xFF8B8C8B,
    0xF5494D4A,0xFF7D7F7D,0xFF777877,0xFF171817,0xFF3D3F3D,0xFFADB1AE,0xFFC6CAC7,0xFF8E908E,0xFF0C0C0C,0xFF0E0E0E,0xFF6F6F6F,0xFF4D4D4D,0xFF131313,0xFF3A3C3A,0xFF505250,0xFF808180,
    0xF5434543,0xFF1A1A1A,0xFF0E0E0E,0xFF3C3E3C,0xFFB0B4B1,0xFF969796,0xFF8B8C8B,0xFF757675,0xFF0E0E0E,0xFF353635,0xFF1C1D1C,0xFF131313,0xFF3B3D3B,0xFFA6A8A6,0xFFA8AAA8,0xFF4C4E4C,
    0xF5BCBFBC,0xFFA9ADAA,0xFF424442,0xFF484A48,0xFFB7BBB8,0xFF929392,0xFF7D7E7D,0xFF151615,0xFF454745,0xFFB3B7B4,0xFFB3B7B4,0xFFA5A8A5,0xFF535553,0xFF868786,0xFF808180,0xFF3D3F3D,
    0xF5C2C6C3,0xFF969897,0xFF7F807F,0xFF4A4C4A,0xFF7B7C7B,0xFF7F807F,0xFF717271,0xFF151615,0xFFA4A8A5,0xFFC6CAC7,0xFF979897,0xFF898989,0xFF848584,0xFF7F807F,0xFF1A1A1A,0xFF070707,
    0xF58A8B8A,0xFF898A89,0xFF7C7D7C,0xFF454745,0xFF474947,0xFF1C1C1C,0xFF111111,0xFF353735,0xFF505250,0xFF808280,0xFF7E7F7E,0xFF858685,0xFF898A89,0xFF808180,0xFF737373,0xFF444644,
    0xF5868786,0xFF777877,0xFF161716,0xFF474947,0xFFB3B7B4,0xFFADB0AE,0xFF414241,0xFF464846,0xFF484A48,0xFF424442,0xFF1A1A1A,0xFF767776,0xFF767676,0xFF171817,0xFF848584,0xFFBEC2BF,
    0xF5828382,0xFF181918,0xFF3F413F,0xFFB3B7B4,0xFFC4C8C5,0xFF878A88,0xFF545654,0xFFACAFAC,0xFFBBBFBC,0xFF808180,0xFF3D3F3D,0xFF121312,0xFF0F0F0F,0xFF171717,0xFFAEB2AF,0xFFC7CBC8,
    0xF57C7D7C,0xFF4F5250,0xFFAEB2AF,0xFFC6CAC7,0xFF848685,0xFF1F1F1F,0xFFA4A8A4,0xFF9A9C9A,0xFFBABDBB,0xFF8F918F,0xFF7E7F7E,0xFF363837,0xFF3B3E3C,0xFFA2A6A3,0xFF939493,0xFF8C8D8C,
    0xF5464846,0xFFA5A9A6,0xFF939593,0xFF8E908F,0xFF7C7C7C,0xFF161616,0xFFADB0AD,0xFF8E908E,0xFF8A8B8A,0xFF8B8C8B,0xFF747474,0xFF151515,0xFF434543,0xFF545654,0xFF7B7C7B,0xFF7C7D7C,
    0xF5474947,0xFF555755,0xFF555555,0xFF777777,0xFF707170,0xFF0E0F0E,0xFF787978,0xFF8B8C8B,0xFF7D7E7D,0xFF727372,0xFF202020,0xFF474947,0xFFA7AAA8,0xFF4F514F,0xFF3E403E,0xFF131313,
    0xF5BEC2BF,0xFFAEB1AF,0xFF191A1A,0xFF161616,0xFF181919,0xFF111111,0xFF444644,0xFF737473,0xFF161616,0xFF202020,0xFFA9ADAA,0xFFB9BCBA,0xFF919391,0xFF7D7E7D,0xFF494B49,0xFF343534,
    0xF5C4C8C5,0xFFA5A5A5,0xFF484948,0xFFA7ABA8,0xFFB8BBB9,0xFFB0B3B0,0xFF4C4E4C,0xFF101110,0xFF373937,0xFFA3A6A4,0xFFC4C8C5,0xFF949594,0xFF868786,0xFF8B8C8B,0xFF707170,0xFF0A0A0A,
    0xF58A8B8A,0xFF828382,0xFF545654,0xFFB1B5B2,0xFF8F908F,0xFF868786,0xFF757675,0xFF3F403F,0xFF474947,0xFF575957,0xFF818381,0xFF818281,0xFF7F807F,0xFF767776,0xFF171817,0xFF030303,
    0xF5838483,0xFF878887,0xFF818281,0xFF494A49,0xFF090909,0xFF040404,0xFF080808,0xFF474947,0xFFB3B6B4,0xFF848584,0xFF474947,0xFF3E403E,0xFF090A09,0xFF070707,0xFF373937,0xFF404240
  },
  {
    0xEB9C9E9C,0xF5969997,0xF5949794,0xF5929593,0xF56A6B6A,0xF5616261,0xF53D3F3D,0xF5373837,0xF53E413E,0xF58F928F,0xF5979997,0xF5929492,0xF5686968,0xF53F413F,0xF58A8D8A,0xF56D6E6D,
    0xF78F9490,0xFF909391,0xFF6F716F,0xFF5C5D5C,0xFF131313,0xFF393B39,0xFF868987,0xFF838683,0xFF383A38,0xFF5D5E5D,0xFF696B6A,0xFF545555,0xFF424242,0xFF101010,0xFF5B5C5B,0xFF676867,
    0xF53D413E,0xFF5D5E5D,0xFF585958,0xFF111211,0xFF2D2F2D,0xFF808381,0xFF939694,0xFF696B69,0xFF090909,0xFF0A0A0A,0xFF525252,0xFF393939,0xFF0E0E0E,0xFF2B2D2B,0xFF3B3D3B,0xFF5F605F,
    0xF5393B39,0xFF131313,0xFF0A0A0A,0xFF2D2E2D,0xFF838683,0xFF6F706F,0xFF676867,0xFF575857,0xFF0A0A0A,0xFF272827,0xFF151615,0xFF0E0E0E,0xFF2C2D2C,0xFF7B7D7B,0xFF7D7E7D,0xFF383A38,
    0xF5909290,0xFF7E807E,0xFF313231,0xFF353735,0xFF888B89,0xFF6C6D6C,0xFF5D5E5D,0xFF101010,0xFF333533,0xFF858886,0xFF858886,0xFF7B7D7B,0xFF3E3F3E,0xFF636463,0xFF5F605F,0xFF2D2F2D,
    0xF5949794,0xFF6F7170,0xFF5E5F5E,0xFF373837,0xFF5B5C5B,0xFF5E5F5E,0xFF545554,0xFF101010,0xFF7A7D7B,0xFF939694,0xFF707170,0xFF666666,0xFF626362,0xFF5E5F5E,0xFF131313,0xFF050505,
    0xF56D6D6D,0xFF666666,0xFF5C5D5C,0xFF333533,0xFF353635,0xFF151515,0xFF0D0D0D,0xFF272927,0xFF3B3D3B,0xFF5F615F,0xFF5E5E5E,0xFF636363,0xFF666666,0xFF5F605F,0xFF555555,0xFF323432,
    0xF56A6A6A,0xFF585958,0xFF101110,0xFF353635,0xFF858886,0xFF808381,0xFF303130,0xFF343534,0xFF353735,0xFF313231,0xFF131313,0xFF585858,0xFF585858,0xFF111211,0xFF626362,0xFF8D908E,
    0xF5676867,0xFF121312,0xFF2F302F,0xFF858886,0xFF929592,0xFF646665,0xFF3E403E,0xFF808280,0xFF8B8E8C,0xFF5F605F,0xFF2D2F2D,0xFF0D0E0D,0xFF0B0B0B,0xFF111111,0xFF818482,0xFF949795,
    0xF5626362,0xFF3B3D3B,0xFF818482,0xFF939694,0xFF626363,0xFF171717,0xFF7A7D7A,0xFF727472,0xFF8A8C8B,0xFF6A6C6A,0xFF5E5E5E,0xFF282A29,0xFF2C2E2D,0xFF787B79,0xFF6D6E6D,0xFF686968,
    0xF53B3D3B,0xFF7B7E7B,0xFF6D6F6D,0xFF696B6A,0xFF5C5C5C,0xFF101010,0xFF808380,0xFF696B69,0xFF666766,0xFF676867,0xFF565656,0xFF101010,0xFF323332,0xFF3E403E,0xFF5B5C5B,0xFF5C5D5C,
    0xF53C3D3C,0xFF3F413F,0xFF3F3F3F,0xFF585858,0xFF535453,0xFF0A0B0A,0xFF595A59,0xFF676867,0xFF5D5E5D,0xFF555555,0xFF181818,0xFF353635,0xFF7C7E7D,0xFF3B3C3B,0xFF2E302E,0xFF0E0E0E,
    0xF5929492,0xFF818382,0xFF131313,0xFF101010,0xFF121313,0xFF0D0D0D,0xFF323432,0xFF555655,0xFF101010,0xFF181818,0xFF7E807E,0xFF898C8A,0xFF6C6D6C,0xFF5D5E5D,0xFF363836,0xFF272727,
    0xF5959896,0xFF7B7B7B,0xFF353635,0xFF7C7F7D,0xFF898B89,0xFF838583,0xFF383A38,0xFF0C0D0C,0xFF292A29,0xFF797B7A,0xFF929592,0xFF6E6F6E,0xFF636463,0xFF676867,0xFF535453,0xFF070707,
    0xF56D6D6D,0xFF616161,0xFF3E403E,0xFF838684,0xFF6A6B6A,0xFF636463,0xFF575857,0xFF2F302F,0xFF353635,0xFF414241,0xFF606160,0xFF606160,0xFF5E5F5E,0xFF585858,0xFF111211,0xFF020202,
    0xF5686868,0xFF646564,0xFF606160,0xFF363736,0xFF070707,0xFF030303,0xFF060606,0xFF353635,0xFF858786,0xFF626362,0xFF353635,0xFF2E302E,0xFF070707,0xFF050505,0xFF292A29,0xFF303130
  }
};

#endif /* INC_TEXTURES_H_ */
