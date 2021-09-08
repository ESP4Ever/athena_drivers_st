/*
 * rotation_vector.h
 *
 *  Created on: Dec 8, 2020
 *      Author: sangweilin@xiaomi.com
 */

#ifndef INC_ROTATION_VECTOR_H_
#define INC_ROTATION_VECTOR_H_

#define NUM_ELEMS      (3)
#define _PI (3.14159265358979323846264338327950f)
#define RAD2DEG (57.2957795130823208767981548141050f)
#define DEG2RAD (0.017453292f)
#define GRAVITY (9.80665f)
#define ALGO_ARRAY_SIZE (38)
#define ALGO_SAMPLE_HZ (25)
#define PSD_STDEV (0.024f)      //noise 0.014 dps/rtHz * sqrt(3)

typedef struct rot_v_algo_struct {
  int posture_start;
  int acc_correct;
  int mag_correct;
  float state[4];
  float state_1[4];
  float variance_1[16];
  float A_1[16];
  float acc[3];
  float gyro[3];
  float mag[3];
} rot_v_algo_struct;

#endif /* INC_ROTATION_VECTOR_H_ */
