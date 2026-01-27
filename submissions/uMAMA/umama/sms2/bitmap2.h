//=======================================================================================//
// File             : sms/bitmap.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 19/AUG/2025
// Description      : Implements bitmap functionality required for SMS
//=======================================================================================//

#ifndef __BITMAP2_H__
#define __BITMAP2_H__

#include <bitset>
#include <stdint.h>
#include <string>
#define BITMAP2_MAX_SIZE 64

typedef std::bitset<BITMAP2_MAX_SIZE> Bitmap2;

class Bitmap2Helper
{
public:
  static uint64_t value(Bitmap2 bmp, uint32_t size = BITMAP2_MAX_SIZE);
  static std::string to_string(Bitmap2 bmp, uint32_t size = BITMAP2_MAX_SIZE);
  static uint32_t count_bits_set(Bitmap2 bmp, uint32_t size = BITMAP2_MAX_SIZE);
  static uint32_t count_bits_same(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size = BITMAP2_MAX_SIZE);
  static uint32_t count_bits_diff(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size = BITMAP2_MAX_SIZE);
  static Bitmap2 rotate_left(Bitmap2 bmp, uint32_t amount, uint32_t size = BITMAP2_MAX_SIZE);
  static Bitmap2 rotate_right(Bitmap2 bmp, uint32_t amount, uint32_t size = BITMAP2_MAX_SIZE);
  static Bitmap2 compress(Bitmap2 bmp, uint32_t granularity, uint32_t size = BITMAP2_MAX_SIZE);
  static Bitmap2 decompress(Bitmap2 bmp, uint32_t granularity, uint32_t size = BITMAP2_MAX_SIZE);
  static Bitmap2 bitwise_or(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size = BITMAP2_MAX_SIZE);
  static Bitmap2 bitwise_and(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size = BITMAP2_MAX_SIZE);
};

#endif /* __BITMAP2_H__ */
