/*
 *    Copyright 2015, Xiaomi Corporation All rights reserved.
 *
 *    sensor halleventlistener native library
 *
 */

#ifndef ANDROID_INPUT_EVENT_READER_H
#define ANDROID_INPUT_EVENT_READER_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <linux/input.h>

/*****************************************************************************/
class InputEventCircularReader {
  struct input_event *const mBuffer;
  struct input_event *const mBufferEnd;
  struct input_event *mHead;
  struct input_event *mCurr;
  ssize_t mFreeSpace;

public:
   InputEventCircularReader(size_t numEvents);
  ~InputEventCircularReader();
  ssize_t fill(int fd);
  ssize_t readEvent(input_event const **events);
  void next();
};

/*****************************************************************************/

#endif // ANDROID_INPUT_EVENT_READER_H
