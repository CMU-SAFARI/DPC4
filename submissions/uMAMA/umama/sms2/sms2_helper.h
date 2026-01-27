//=======================================================================================//
// File             : sms/sms_helper.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 19/AUG/2025
// Description      : Defines auxiliary structures to implement
//                    Spatial Memory Streaming prefetcher, ISCA'06
//=======================================================================================//

#ifndef __SMS2_HELPER_H__
#define __SMS2_HELPER_H__

#include <stdint.h>

#include "bitmap2.h"

class FTEntry2
{
public:
  uint64_t page;
  uint64_t pc;
  uint32_t trigger_offset;

public:
  void reset()
  {
    page = 0xdeadbeef;
    pc = 0xdeadbeef;
    trigger_offset = 0;
  }
  FTEntry2() { reset(); }
  ~FTEntry2() {}
};

class ATEntry2
{
public:
  uint64_t page;
  uint64_t pc;
  uint32_t trigger_offset;
  Bitmap2 pattern;
  uint32_t age;

public:
  void reset()
  {
    page = pc = 0xdeadbeef;
    trigger_offset = 0;
    pattern.reset();
    age = 0;
  }
  ATEntry2() { reset(); }
  ~ATEntry2() {}
};

class PHTEntry2
{
public:
  uint64_t signature;
  Bitmap2 pattern;
  uint32_t age;

public:
  void reset()
  {
    signature = 0xdeadbeef;
    pattern.reset();
    age = 0;
  }
  PHTEntry2() { reset(); }
  ~PHTEntry2() {}
};

#endif /* __SMS_HELPER_H__ */
