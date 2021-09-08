/*
 *    Copyright 2021, Xiaomi Corporation All rights reserved.
 *
 *    touch test library
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include "TouchSensorHandler.h"

int main(int argc, char *argv[])
{
  int ret = 3;
  int ret_count, count = 1;
  bool isrunning = true;
  struct input_event *touch_event;
  TouchSensorHandler *mTouchSensorHandler = new TouchSensorHandler();
  if (mTouchSensorHandler == nullptr) {
    printf("%s: create TouchSensorHandler failed", __func__);
    return -1;
  }

  touch_event =
      (struct input_event *)malloc(sizeof(struct input_event) * count);
  //printf("%s: toucheventlistener count : %d\n", __func__, count);
  while (isrunning) {
    ret_count = mTouchSensorHandler->pollTouchEvents(touch_event, count);
    if (ret_count > 0) {
      if ((touch_event->type == EV_KEY)) {
        switch (touch_event->code - GESTURE_XM_ADDR) {
        case LPWG_SINGLETAP_DETECTED:
          printf("touch_test Single tap\n");
          break;
        case LPWG_DOUBLETAP_DETECTED:
          printf("touch_test Double tap\n");
          break;
        case LPWG_TOUCHANDHOLD_DETECTED:
          printf("touch_test handhold\n");
          break;
        case LPWG_SWIPE_DETECTED:
          printf("touch_test Swipe\n");
          break;
        default:
          ret = touch_event->value;
          printf("event ret : %d\n", ret);
        }
        isrunning = false;
      }
    }
  }
  free(touch_event);
  return 0;
}
