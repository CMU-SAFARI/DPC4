//=======================================================================================//
// File             : sms/bitmap.cc
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 19/AUG/2025
// Description      : Implements bitmap functionality required for SMS
//=======================================================================================//

#include "bitmap2.h"

#include <assert.h>
#include <sstream>

std::string Bitmap2Helper::to_string(Bitmap2 bmp, uint32_t size)
{
  // return bmp.to_string<char,std::string::traits_type,std::string::allocator_type>();
  std::stringstream ss;
  for (int32_t bit = size - 1; bit >= 0; --bit) {
    ss << bmp[bit];
  }
  return ss.str();
}

uint32_t Bitmap2Helper::count_bits_set(Bitmap2 bmp, uint32_t size)
{
  // return static_cast<uint32_t>(bmp.count());
  uint32_t count = 0;
  for (uint32_t index = 0; index < size; ++index) {
    if (bmp[index])
      count++;
  }
  return count;
}

uint32_t Bitmap2Helper::count_bits_same(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size)
{
  uint32_t count_same = 0;
  for (uint32_t index = 0; index < size; ++index) {
    if (bmp1[index] && bmp1[index] == bmp2[index]) {
      count_same++;
    }
  }
  return count_same;
}

uint32_t Bitmap2Helper::count_bits_diff(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size)
{
  uint32_t count_diff = 0;
  for (uint32_t index = 0; index < size; ++index) {
    if (bmp1[index] && !bmp2[index]) {
      count_diff++;
    }
  }
  return count_diff;
}

uint64_t Bitmap2Helper::value(Bitmap2 bmp, uint32_t size) { return bmp.to_ullong(); }

Bitmap2 Bitmap2Helper::rotate_left(Bitmap2 bmp, uint32_t amount, uint32_t size)
{
  Bitmap2 result;
  for (uint32_t index = 0; index < (size - amount); ++index) {
    result[index + amount] = bmp[index];
  }
  for (uint32_t index = 0; index < amount; ++index) {
    result[index] = bmp[index + size - amount];
  }
  return result;
}

Bitmap2 Bitmap2Helper::rotate_right(Bitmap2 bmp, uint32_t amount, uint32_t size)
{
  Bitmap2 result;
  for (uint32_t index = 0; index < size - amount; ++index) {
    result[index] = bmp[index + amount];
  }
  for (uint32_t index = 0; index < amount; ++index) {
    result[size - amount + index] = bmp[index];
  }
  return result;
}

Bitmap2 Bitmap2Helper::compress(Bitmap2 bmp, uint32_t granularity, uint32_t size)
{
  assert(size % granularity == 0);
  uint32_t index = 0;
  Bitmap2 result;
  uint32_t ptr = 0;

  while (index < size) {
    bool res = false;
    uint32_t gran = 0;
    for (gran = 0; gran < granularity; ++gran) {
      assert(index + gran < size);
      res = res | bmp[index + gran];
    }
    result[ptr] = res;
    ptr++;
    index = index + gran;
  }
  return result;
}

Bitmap2 Bitmap2Helper::decompress(Bitmap2 bmp, uint32_t granularity, uint32_t size)
{
  Bitmap2 result;
  result.reset();
  assert(size * granularity <= BITMAP2_MAX_SIZE);
  for (uint32_t index = 0; index < size; ++index) {
    if (bmp[index]) {
      uint32_t ptr = index * granularity;
      for (uint32_t count = 0; count < granularity; ++count) {
        result[ptr + count] = true;
      }
    }
  }
  return result;
}

Bitmap2 Bitmap2Helper::bitwise_or(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size)
{
  Bitmap2 result;
  for (uint32_t index = 0; index < size; ++index) {
    if (bmp1[index] || bmp2[index]) {
      result[index] = true;
    }
  }
  return result;
}

Bitmap2 Bitmap2Helper::bitwise_and(Bitmap2 bmp1, Bitmap2 bmp2, uint32_t size)
{
  Bitmap2 result;
  for (uint32_t index = 0; index < size; ++index) {
    if (bmp1[index] && bmp2[index]) {
      result[index] = true;
    }
  }
  return result;
}
