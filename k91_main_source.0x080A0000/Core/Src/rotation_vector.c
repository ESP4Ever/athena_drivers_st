#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "sensor_device.h"
#include "rotation_vector.h"

static sensor_device **sensor_ptr = NULL;
static platform_prams *rot_v_init_parms;
static rot_v_algo_struct posture_loc;
static float acc_correct_var[3] = { 1.0f, 1.0f, 1.0f };

static uint8_t sensor_activate_count;
static bool rot_v_activated = false;

static float rot_v_quaternion[4] = { 1.0, 0.0, 0.0, 0.0 };
static float temp_roll, temp_pitch, temp_yaw;
void rot_v_state_init()
{

  float variance_1_temp[16] = { 1, 0, 0, 0,     // initial variance = I matrix
    0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  posture_loc.posture_start = 5;

  posture_loc.acc_correct = 0;
  posture_loc.mag_correct = 0;
  for (int i = 0; i < 4; i++) {
    posture_loc.state[i] = 0;
    posture_loc.state_1[i] = 0;
  }

  for (int i = 0; i < 16; i++)
    posture_loc.variance_1[i] = variance_1_temp[i];

  for (int i = 0; i < 16; i++)
    posture_loc.A_1[i] = 0;

  for (int i = 0; i < 3; i++) {
    posture_loc.acc[i] = 0;
    posture_loc.gyro[i] = 0;
    posture_loc.mag[i] = 0;
  }

  printf("rot_v_state_init done !!!\r\n");
}

static void matrix_multiply(float *A, float *B, float *C, int AR, int AC,
                            int BC)
{                               // AR = row of matrix A, AC = column of matrix A, BC = column of matrix B
  float a_temp = 0, b_temp = 0, c_temp = 0;
  int i = 0, j = 0, k = 0, l = 0;

  for (i = 0; i < AR; i++) {
    for (j = 0; j < BC; j++) {
      l = i * BC + j;
      C[l] = 0;
      for (k = 0; k < AC; k++) {
        a_temp = A[i * AC + k];
        b_temp = B[k * BC + j];
        c_temp = a_temp * b_temp;
        C[l] += c_temp;
      }
    }
  }
}

// matrix A * transpose of matrix B = matrix C
static void matrix_multiply_trans(float *A, float *B, float *C, int AR, int AC,
                                  int BR)
{                               // AR = row of matrix A, AC = column of matrix A, BC = row of matrix B
  float a_temp = 0, b_temp = 0, c_temp = 0;
  int i = 0, j = 0, k = 0, l = 0;

  for (i = 0; i < AR; i++) {
    for (j = 0; j < BR; j++) {
      l = i * BR + j;
      C[l] = 0;
      for (k = 0; k < AC; k++) {
        a_temp = A[i * AC + k];
        b_temp = B[j * AC + k];
        c_temp = a_temp * b_temp;
        C[l] += c_temp;
      }
    }
  }
}

// matrix A * matrix B * transpose of matrix A = matrix C
static void ABAT(float *A, float *B, float *C, int AR, int AC)
{                               // AR = row of matrix A, AC = column of matrix A
  float Temp[16] = { 0 };
  matrix_multiply(A, B, Temp, AR, AC, AC);
  matrix_multiply_trans(Temp, A, C, AR, AC, AR);
}

// inverse of matrix A = matrix AI
static void matrix_inverse(float *A, float *AI)
{
  float det = 0, invdet = 0;

  det = A[0] * (A[4] * A[8] - A[7] * A[5])
      - A[1] * (A[3] * A[8] - A[5] * A[6])
      + A[2] * (A[3] * A[7] - A[4] * A[6]);

  if (det >= 0 && det <= 0)
    return;

  invdet = 1.0f / det;

  AI[0] = (A[4] * A[8] - A[7] * A[5]) * invdet;
  AI[1] = -(A[1] * A[8] - A[2] * A[7]) * invdet;
  AI[2] = (A[1] * A[5] - A[2] * A[4]) * invdet;
  AI[3] = -(A[3] * A[8] - A[5] * A[6]) * invdet;
  AI[4] = (A[0] * A[8] - A[2] * A[6]) * invdet;
  AI[5] = -(A[0] * A[5] - A[3] * A[2]) * invdet;
  AI[6] = (A[3] * A[7] - A[6] * A[4]) * invdet;
  AI[7] = -(A[0] * A[7] - A[6] * A[1]) * invdet;
  AI[8] = (A[0] * A[4] - A[3] * A[1]) * invdet;
}

// calculate total force of matirx elements
static float matrix_length(float *matrix, int size)
{
  float sum = 0, norm = 0, temp = 0;
  int i = 0;

  for (i = 0; i < size; i++) {
    temp = matrix[i] * matrix[i];
    sum += temp;
  }
  norm = sqrtf(sum);

  return norm;
}

static void matrix_normalize(float *matrix, int size)
{
  float sum = 0, norm = 0, temp = 0;
  int i = 0;

  for (i = 0; i < size; i++) {
    temp = matrix[i] * matrix[i];
    sum += temp;
  }

  norm = sqrtf(sum);
  if (norm >= 0 && norm <= 0)   // prevent divide by 0
    norm = 0.0000000001f;
  for (i = 0; i < size; i++)
    matrix[i] /= norm;
}

static void remap_coordinates(float *mapped, float *ori, float *coord)
{
  float sum = 0, temp = 0;
  int i = 0, j = 0;

  for (i = 0; i < 3; i++) {
    sum = 0;
    for (j = 0; j < 3; j++) {
      temp = ori[j] * coord[i * 3 + j];
      sum += temp;
    }
    mapped[i] = sum;
  }
}

static void imu_Acc_LowPassFilter(float *raw, float *filtered)
{
  //25Hz sample rate with 0.1Hz/0.5Hz cutoff freq.
  //Denominator
  //const double a0 = 1;
  const double a1 = -0.88135386020153494;
  //Numerator
  const double b0 = 0.059323069899232528;
  const double b1 = 0.059323069899232528;

  static double x_1[3] = { 0 };
  static double y_1[3] = { 0 };

  double filter_double[3] = { 0 };

  for (int i = 0; i < 3; i++) {
    filter_double[i] = b0 * (double)raw[i] + b1 * x_1[i] - a1 * y_1[i];

    x_1[i] = (double)raw[i];
    y_1[i] = filter_double[i];

    filtered[i] = (float)filter_double[i];
  }
}

static void imu_Velocity_HighPassFilter(float *raw, float *filtered)
{
  //25Hz sample rate with 0.5Hz/1Hz cutoff freq.
  //Denominator
  //const double a0 = 1;
  const double a1 = -0.88188277091996325;
  //Numerator
  const double b0 = 0.94094138545998163;
  const double b1 = -0.94094138545998163;

  static double x_1[3] = { 0 };
  static double y_1[3] = { 0 };

  double filter_double[3] = { 0 };

  for (int i = 0; i < 3; i++) {
    filter_double[i] = b0 * (double)raw[i] + b1 * x_1[i] - a1 * y_1[i];

    x_1[i] = (double)raw[i];
    y_1[i] = filter_double[i];

    filtered[i] = (float)filter_double[i];
  }
}

static int preprocess(float *input_data, int data_type)
{
  int i = 0;
  static float gyro_strength = -1;
  float ANDROID_AXES[] = { 1, 0, 0,     // Android axis: roll:90~-90, pitch:180~-180
    0, -1, 0, 0, 0, -1
  };
  if (data_type == SENSOR_TYPE_ACCELEROMETER
      || data_type == SENSOR_TYPE_MAGNETIC_FIELD) {
    if (input_data[0] >= 0 && input_data[0] <= 0 && input_data[1] >= 0
        && input_data[1] <= 0 && input_data[2] >= 0 && input_data[2] <= 0) {
      printf("input_data are all zeros, return!\r\n");
      return -1;
    }
  }
  // transform to Android axis
  if (data_type == SENSOR_TYPE_ACCELEROMETER) {
    remap_coordinates(posture_loc.acc, input_data, ANDROID_AXES);
    for (i = 0; i < 3; i++)
      posture_loc.acc[i] /= -9.80665f;
    float acc_strength = matrix_length(posture_loc.acc, 3);
    posture_loc.acc_correct = 0;
    float acc_g_lpf_out[3] = { 0 };
    imu_Acc_LowPassFilter(posture_loc.acc, acc_g_lpf_out);
    //swl
    //if (gyro_strength > 0 && gyro_strength < 1.75f) { don't use gyro to restrict acc processing
    if (acc_strength < 1.15f && acc_strength > 0.85f) {

      float acc_uncert_bias = fabs(acc_strength - 1.0f);
      float total_uncertain[3] = { 0 };

      for (int i = 0; i < 3; i++) {
//        total_uncertain[i] = fabs(posture_loc.acc[i] - acc_g_lpf_out[i])        //swl,use out?
//            + acc_uncert_bias;
//        total_uncertain[i] = fabs(posture_loc.acc[i] - acc_g_lpf_out[i]);


        if (total_uncertain[i] < 0.001f)        //min = 0.01 m/s2
          total_uncertain[i] = 0.001f;

        acc_correct_var[i] = total_uncertain[i] * total_uncertain[i];   //swl
      }
      posture_loc.acc_correct = 1;
    }
    //}
  } else if (data_type == SENSOR_TYPE_GYROSCOPE) {
    remap_coordinates(posture_loc.gyro, input_data, ANDROID_AXES);
    gyro_strength = matrix_length(posture_loc.gyro, 3);
  } else if (data_type == SENSOR_TYPE_MAGNETIC_FIELD) {
    remap_coordinates(posture_loc.mag, input_data, ANDROID_AXES);
    posture_loc.mag_correct = 0;
  }
  return 0;
}

static void setting_state(float *acc, float *mag, float *state_1)
{                               // state_1: previous state
  float init_theta = 0, init_phi = 0, init_psi = 0;
  float cos_pitch = 0, sin_pitch = 0, cos_roll = 0, sin_roll = 0;
  float cp2 = 0, sp2 = 0, cr2 = 0, sr2 = 0, cy2 = 0, sy2 = 0;

  // ===== use acc to calculate roll and pitch =====//
  init_theta = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
  init_phi = atan2f(acc[1], acc[2]);

  if (acc[2] < 0) {
    if (init_phi < 0)
      init_phi = -_PI - init_phi;
    else
      init_phi = _PI - init_phi;
  }

  cos_pitch = cosf(init_theta);
  sin_pitch = sinf(init_theta);
  cos_roll = cosf(init_phi);
  sin_roll = sinf(init_phi);
  // ===== use acc to calculate roll and pitch =====//

  // ===== use mag to calculate yaw =====//
  init_psi = -atan2f(mag[0] * cos_pitch + mag[1] * sin_pitch * sin_roll
                     + mag[2] * sin_pitch * cos_roll,
                     mag[2] * sin_roll - mag[1] * cos_roll);
  // ===== use mag to calculate yaw =====//

  // ===== convert euler angle to quaternion =====//
  cp2 = cosf(init_theta / 2.0f);
  sp2 = sinf(init_theta / 2.0f);
  cr2 = cosf(init_phi / 2.0f);
  sr2 = sinf(init_phi / 2.0f);
  cy2 = cosf(init_psi / 2.0f);
  sy2 = sinf(init_psi / 2.0f);

  state_1[0] = cy2 * cp2 * cr2 + sy2 * sp2 * sr2;
  state_1[1] = cy2 * cp2 * sr2 - sy2 * sp2 * cr2;
  state_1[2] = cy2 * sp2 * cr2 + sy2 * cp2 * sr2;
  state_1[3] = sy2 * cp2 * cr2 - cy2 * sp2 * sr2;
  // ===== convert euler angle to quaternion =====//

  matrix_normalize(state_1, 4); // data of quaternion should be in 1~-1
}

static void propagation(float *gyro, float deltaTime, float *state_p,
                        float *var_p, float *state_1, float *var_1, float *A_1)
{                               // state_p: predict state, var_p: predict variance, state_1: previous state, var_1: previous variance, A_1:previous matrix A, correct_level: correct level setting
  int i = 0;
  float gyro_half[3] = { 0 };
  float state_gain[4] = { 0 };
  float sf_err = 0;
  float int_err = 0;
  float A[16] = { 0 };
  float A_mid[16] = { 0 };
  float A_tmp[16] = { 0 };
  float G_mat[16] = { 0 };
  float W[12] = { 0 };
  float R[9] = { 0 };
  float GVGT[16] = { 0 };
  float WRWT[16] = { 0 };

  gyro_half[0] = gyro[0] / 2.0f;
  gyro_half[1] = gyro[1] / 2.0f;
  gyro_half[2] = gyro[2] / 2.0f;

  A[0] = 0;
  A[1] = -gyro_half[0];
  A[2] = -gyro_half[1];
  A[3] = -gyro_half[2];
  A[4] = gyro_half[0];
  A[5] = 0;
  A[6] = gyro_half[2];
  A[7] = -gyro_half[1];
  A[8] = gyro_half[1];
  A[9] = -gyro_half[2];
  A[10] = 0;
  A[11] = gyro_half[0];
  A[12] = gyro_half[2];
  A[13] = gyro_half[1];
  A[14] = -gyro_half[0];
  A[15] = 0;

  for (i = 0; i < 16; i++)
    A_mid[i] = (A_1[i] + A[i]) * deltaTime / 2.0f;
  for (i = 0; i < 16; i++)
    A_tmp[i] = A_1[i] * deltaTime / 2.0f;
  A_tmp[0]++;                   // Add I matrix
  A_tmp[5]++;
  A_tmp[10]++;
  A_tmp[15]++;

  matrix_multiply(A_mid, A_tmp, G_mat, 4, 4, 4);
  G_mat[0]++;                   // Add I matrix
  G_mat[5]++;
  G_mat[10]++;
  G_mat[15]++;
  matrix_multiply(G_mat, state_1, state_p, 4, 4, 1);

  matrix_normalize(state_p, 4);
  for (i = 0; i < 16; i++)
    A_1[i] = A[i];

  for (i = 0; i < 4; i++)
    state_gain[i] = state_1[i] * deltaTime;

  W[0] = -state_gain[1];
  W[1] = -state_gain[2];
  W[2] = -state_gain[3];
  W[3] = state_gain[0];
  W[4] = -state_gain[3];
  W[5] = state_gain[2];
  W[6] = state_gain[3];
  W[7] = state_gain[0];
  W[8] = -state_gain[1];
  W[9] = -state_gain[2];
  W[10] = state_gain[1];
  W[11] = state_gain[0];

  // ===== correct level: default: 2% of 2000 dps & 0,07dps RMS noise===== //
  sf_err = 0.000001f;           //(0.02*w*0.05)^2
  int_err = 0.00000002f;        //w^6 * 0.05^6

  float w0_squre = gyro[0] * gyro[0];
  float w1_squre = gyro[1] * gyro[1];
  float w2_squre = gyro[2] * gyro[2];

  R[0] = (w0_squre * sf_err) + (w0_squre * w0_squre * w0_squre * int_err);
  R[4] = (w1_squre * sf_err) + (w1_squre * w1_squre * w1_squre * int_err);
  R[8] = (w2_squre * sf_err) + (w2_squre * w2_squre * w2_squre * int_err);

  ABAT(G_mat, var_1, GVGT, 4, 4);
  ABAT(W, R, WRWT, 4, 3);
  for (i = 0; i < 16; i++)
    var_p[i] = GVGT[i] + WRWT[i];
  for (i = 0; i < 4; i++)
    state_1[i] = state_p[i];
}

// Correction of EKF: use acc to correct pose
static void acc_update(float *acc_in, float *state_p, float *variance_p)
{                               // state_p: predict state, var_p: predict variance, correct_level: correct level setting
  int i = 0;
  float variance[16] = { 0 };
  float acc[3] = { 0 };
  float Zt_Zp[3] = { 0 };
  //float error = 0;
  float H[12] = { 0 };
  float HVHT[9] = { 0 };
  float VHT[12] = { 0 };
  float phi[9] = { 0 };
  float inv_phi[9] = { 0 };
  float K[12] = { 0 };
  float KE[4] = { 0 };
  float KH[16] = { 0 };

  for (i = 0; i < 3; i++)
    acc[i] = acc_in[i];

  matrix_normalize(acc, 3);

  // this should be handling critical value, need or not? swl
//      if (acc[1] < 0.15f && acc[1] > -0.15f && acc[2] < 0.15f && acc[2] > -0.15f)
//              acc[1] *= (acc[2] * acc[2]);

  H[0] = -2 * state_p[2];
  H[1] = 2 * state_p[3];
  H[2] = -2 * state_p[0];
  H[3] = 2 * state_p[1];
  H[4] = 2 * state_p[1];
  H[5] = 2 * state_p[0];
  H[6] = 2 * state_p[3];
  H[7] = 2 * state_p[2];
  H[8] = 2 * state_p[0];
  H[9] = -2 * state_p[1];
  H[10] = -2 * state_p[2];
  H[11] = 2 * state_p[3];

  ABAT(H, variance_p, HVHT, 3, 4);
  for (i = 0; i < 9; i++)
    phi[i] = HVHT[i];

  // ===== correct level: default: 4% of 9.8 m/s^2 ===== //
#if 0
  error = 0.0001f;              //0.1 m/s2
  phi[0] += error;
  phi[4] += error;
  phi[8] += error;
#endif
  phi[0] += acc_correct_var[0]; //swl
  phi[4] += acc_correct_var[1];
  phi[8] += acc_correct_var[2];

  matrix_inverse(phi, inv_phi);

  // error = measurement data (acc) - predict data by gyro
  Zt_Zp[0] = acc[0] - 2 * (state_p[1] * state_p[3] - state_p[0] * state_p[2]);
  Zt_Zp[1] = acc[1] - 2 * (state_p[2] * state_p[3] + state_p[0] * state_p[1]);
  Zt_Zp[2] = acc[2]
      - (state_p[0] * state_p[0] - state_p[1] * state_p[1]
         - state_p[2] * state_p[2] + state_p[3] * state_p[3]);
  matrix_multiply_trans(variance_p, H, VHT, 4, 4, 3);
  matrix_multiply(VHT, inv_phi, K, 4, 3, 3);    // K = gain value
  matrix_multiply(K, Zt_Zp, KE, 4, 3, 1);       // KE = gain * error

  for (i = 0; i < 4; i++)
    state_p[i] += KE[i];

  matrix_multiply(K, H, KH, 4, 3, 4);
  KH[0] = 1 - KH[0];
  KH[1] = 0 - KH[1];
  KH[2] = 0 - KH[2];
  KH[3] = 0 - KH[3];
  KH[4] = 0 - KH[4];
  KH[5] = 1 - KH[5];
  KH[6] = 0 - KH[6];
  KH[7] = 0 - KH[7];
  KH[8] = 0 - KH[8];
  KH[9] = 0 - KH[9];
  KH[10] = 1 - KH[10];
  KH[11] = 0 - KH[11];
  KH[12] = 0 - KH[12];
  KH[13] = 0 - KH[13];
  KH[14] = 0 - KH[14];
  KH[15] = 1 - KH[15];

  matrix_multiply(KH, variance_p, variance, 4, 4, 4);

  for (i = 0; i < 16; i++)
    variance_p[i] = variance[i];
}

static void posture_update(float dt, int data_type)
{
  int i = 0;
//      static float state_p[4] = { 0 };
  static float variance_p[16] =
      { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  float singular_test_p = 0;

  if (posture_loc.posture_start > 0) {  // calculate initial pose when start
    posture_loc.posture_start--;
    matrix_normalize(posture_loc.acc, 3);

    setting_state(posture_loc.acc, posture_loc.mag, posture_loc.state_1);
    for (i = 0; i < 4; i++)
      posture_loc.state[i] = posture_loc.state_1[i];
    matrix_normalize(posture_loc.state, 4);
  } else {
    if (data_type == SENSOR_TYPE_GYROSCOPE) {
      propagation(posture_loc.gyro, dt, posture_loc.state, variance_p,  // may be need update state_p to posture_loc.state
                  posture_loc.state_1, posture_loc.variance_1, posture_loc.A_1);
    } else if (data_type == SENSOR_TYPE_ACCELEROMETER) {
      if (posture_loc.acc_correct)      // acc correct if static, may be don't need this condition
        acc_update(posture_loc.acc, posture_loc.state, variance_p);     // may be need update state_p to posture_loc.state
    } else if (data_type == SENSOR_TYPE_MAGNETIC_FIELD) {
      posture_loc.mag_correct = 0;      // disable mag correction
      if (posture_loc.mag_correct)      // mag correct if static and no interference
      {
        singular_test_p =
            posture_loc.state[1] * posture_loc.state[3] -
            posture_loc.state[0] * posture_loc.state[2];
        //if (singular_test_p < 0.4999f && singular_test_p > -0.4999f)    // this should be handling critical value
        //mag_update(posture_loc.mag, state_p, variance_p, posture_loc.correct_level);
      }                         // save state and variance to previous
    }
    for (i = 0; i < 4; i++) {
      posture_loc.state_1[i] = posture_loc.state[i];
    }
    for (i = 0; i < 16; i++)
      posture_loc.variance_1[i] = variance_p[i];
    matrix_normalize(posture_loc.state, 4);     // data of rotation vector should be in 1~-1
  }
}

static int posture(float *input_data, float *rot_vec, float dt_s, int data_type)
{
  int ret = 0;

  if (dt_s <= 0 || dt_s > 1.0f) {
    printf("time diff is not right!!!\r\n");
    return -1;                  // dt error
  }
  ret = preprocess(input_data, data_type);
  if (ret < 0)
    return ret;

  posture_update(dt_s, data_type);

  // get rotation vector
  if (posture_loc.state[0] > 0) {
    rot_vec[0] = posture_loc.state[1];
    rot_vec[1] = -posture_loc.state[2];
    rot_vec[2] = -posture_loc.state[3];
    rot_vec[3] = posture_loc.state[0];
  } else {
    rot_vec[0] = -posture_loc.state[1];
    rot_vec[1] = posture_loc.state[2];
    rot_vec[2] = posture_loc.state[3];
    rot_vec[3] = -posture_loc.state[0];
  }

  return 0;
}

static void get_rot_mat(float *rot_vec, float *rot_mat)
{                               // rotation vector to rotation matrix
  int i = 0;

  float state[4] = { 0 };

  state[0] = rot_vec[3];
  if (state[0] > 0) {
    state[1] = rot_vec[0];
    state[2] = -rot_vec[1];
    state[3] = -rot_vec[2];
  } else {
    state[1] = -rot_vec[0];
    state[2] = rot_vec[1];
    state[3] = rot_vec[2];
  }

  rot_mat[0] = (state[0] * state[0] + state[1] * state[1]
                - state[2] * state[2] - state[3] * state[3]);   // cos(theta)cos(psi)
  rot_mat[1] = 2 * (state[1] * state[2] - state[0] * state[3]);
  rot_mat[2] = 2 * (state[1] * state[3] + state[0] * state[2]);
  rot_mat[3] = 2 * (state[1] * state[2] + state[0] * state[3]); //  cos(theta)sin(psi)
  rot_mat[4] = (state[0] * state[0] - state[1] * state[1]
                + state[2] * state[2] - state[3] * state[3]);
  rot_mat[5] = 2 * (state[2] * state[3] - state[0] * state[1]);
  rot_mat[6] = 2 * (state[1] * state[3] - state[0] * state[2]); // -sin(theta)
  rot_mat[7] = 2 * (state[0] * state[1] + state[2] * state[3]); //  cos(theta)sin(phi)
  rot_mat[8] = (state[0] * state[0] - state[1] * state[1]
                - state[2] * state[2] + state[3] * state[3]);   // cos(theta)cos(phi)

  // data of rotation matrix should be in 1~-1
  for (i = 0; i < 9; i++) {
    if (rot_mat[i] > 1.0f)
      rot_mat[i] = 1.0f;
    else if (rot_mat[i] < -1.0f)
      rot_mat[i] = -1.0f;
  }
}

static void get_orien(float *rot_mat, float *orien)
{
  // rotation matrix to euler angle
#ifdef __REPLAY_MODE2__
  orien[0] = atan2f(rot_mat[3], rot_mat[0]) * RAD2DEG;
  orien[1] = asinf(-rot_mat[6]) * RAD2DEG;
  orien[2] = atan2f(rot_mat[7], rot_mat[8]) * RAD2DEG;
#else
  orien[0] = atan2f(rot_mat[3], rot_mat[0]) * RAD2DEG;
  if (orien[0] < 0)             // Android definition: yaw: 0~360
    orien[0] += 360.0f;
  //---Android roll definition is opposite---//
  orien[1] = -atan2f(rot_mat[7], rot_mat[8]) * RAD2DEG;
  orien[2] = asinf(-rot_mat[6]) * RAD2DEG;
#endif
  //printf("imu_orient %.1f %.1f %.1f\n", orien[0], orien[1], orien[2]);
}

static void get_linear_acc(float *rot_mat, float *la)
{
  int i = 0;
  // already transfer to Android Axis (XYZ)=(x, -y, -z)
  la[0] = (rot_mat[6] - posture_loc.acc[0]) * GRAVITY;
  la[1] = (posture_loc.acc[1] - rot_mat[7]) * GRAVITY;
  la[2] = (posture_loc.acc[2] - rot_mat[8]) * GRAVITY;
#if 0
  for (i = 0; i < 3; i++) {
    if (posture_loc.static_time > 0.05f)        // calculate bias of linear acc when static
      posture_var.la_bias[i] = 0.9f * posture_var.la_bias[i] + 0.1f * la[i];
    la[i] = (la[i] - posture_var.la_bias[i]) * 9.80665f;        // minus bias  to prevent linear acc not 0 when static
  }
#endif
}

static void get_velocity(float *rot_mat, float DT)
{
  float acc_complement[3];
  float Gyro_p, Gyro_q, Gyro_r = 0;
  int i = 0;

  float A[9] = { 0 };
  float B[3] = { 0 };
  static float vel_state[3] = { 0 };
  static float vel_state_1[3] = { 0 };
  acc_complement[0] = (posture_loc.acc[0] - rot_mat[6]) * (-1) * (GRAVITY);
  acc_complement[1] = (posture_loc.acc[1] - rot_mat[7]) * (-1) * (GRAVITY);
  acc_complement[2] = (posture_loc.acc[2] - rot_mat[8]) * (-1) * (GRAVITY);

  Gyro_p = posture_loc.gyro[0];
  Gyro_q = posture_loc.gyro[1];
  Gyro_r = posture_loc.gyro[2];
  //***********************predict***************************************
  A[0] = 1;
  A[1] = (Gyro_r) * DT;
  A[2] = (-Gyro_q) * DT;
  A[3] = (-Gyro_r) * DT;
  A[4] = 1;
  A[5] = (Gyro_p) * DT;
  A[6] = (Gyro_q) * DT;
  A[7] = (-Gyro_p) * DT;
  A[8] = 1;
  //************************
  B[0] = acc_complement[0] * DT;        //*GRAVITY;
  B[1] = acc_complement[1] * DT;        //*GRAVITY;
  B[2] = acc_complement[2] * DT;        //*GRAVITY;
  //************************
  float vel_state_p[3] = { 0.0 };
  matrix_multiply(A, vel_state_1, vel_state_p, 3, 3, 1);
  for (i = 0; i < 3; i++)
    vel_state_p[i] = vel_state_p[i] + B[i];
  //************************predict**************************************
  for (i = 0; i < 3; i++) {
    vel_state[i] = vel_state_p[i];
    vel_state_1[i] = vel_state[i];      //update (state)t-1
  }
}

static void get_rms(float *in, float *out_mean, float *out_rms)
{
  static float gbuf[3][ALGO_ARRAY_SIZE] = { {0.0f} };
  float sum[3] = { 0 };
  float mean_rms[3] = { 0 };
  float sum_rms[3] = { 0 };
  float tmp_window[3][ALGO_ARRAY_SIZE] = { {0.0f} };

  for (int i = 0; i < 3; i++) {

    memcpy(tmp_window, gbuf, sizeof(gbuf));
    memcpy(&gbuf[i][1], &tmp_window[i][0],
           (ALGO_ARRAY_SIZE - 1) * sizeof(float));
    gbuf[i][0] = in[i];
    // TODO: simple moving average
    for (int k = 0; k < ALGO_ARRAY_SIZE; k++) {
      sum[i] += gbuf[i][k];
    }

    mean_rms[i] = sum[i] / ALGO_ARRAY_SIZE;

    for (int m = 0; m < ALGO_ARRAY_SIZE; m++) {
      sum_rms[i] += ((gbuf[i][m] - mean_rms[i])
                     * (gbuf[i][m] - mean_rms[i]));
    }
    out_rms[i] = sqrtf(sum_rms[i] / ALGO_ARRAY_SIZE);
    out_mean[i] = mean_rms[i];
  }
}

static int is_omega_stream_stable(float *w)
{
  int ret = 0;                  //non-stable
  int dynamics = 0;

  for (int i = 0; i < (int)(ALGO_SAMPLE_HZ / 2); i++) { // 0.5sec
    if (w[i] > 0.0045f)         // 0.26 dps
      dynamics = 1;
  }

  if (dynamics == 0)
    ret = 1;

  return ret;
}

void accLowPassFilter(float *raw, float *filtered)
{
  //25Hz sample rate with 1Hz/2Hz cutoff freq.
  //Denominator
  //const double a0 = 1;
  const double a1 = -0.59062581160898153;
  //Numerator
  const double b0 = 0.20468709419550918;
  const double b1 = 0.20468709419550918;

  static double x_1[3] = { 0 };
  static double y_1[3] = { 0 };

  double filter_double[3] = { 0 };

  for (int i = 0; i < 3; i++) {
    filter_double[i] = b0 * (double)raw[i] + b1 * x_1[i] - a1 * y_1[i];

    x_1[i] = (double)raw[i];
    y_1[i] = filter_double[i];

    filtered[i] = (float)filter_double[i];
  }
}

static void gyroLowPassFilter(float *raw, float *filtered)
{
  //25Hz sample rate with 1Hz/3Hz cutoff freq.
  //Denominator
  //const double a0 = 1;
  const double a1 = -0.43177323588356381;
  //Numerator
  const double b0 = 0.28411338205821812;
  const double b1 = 0.28411338205821812;

  static double x_1[3] = { 0 };
  static double y_1[3] = { 0 };

  double filter_double[3] = { 0 };

  for (int i = 0; i < 3; i++) {
    filter_double[i] = b0 * (double)raw[i] + b1 * x_1[i] - a1 * y_1[i];

    x_1[i] = (double)raw[i];
    y_1[i] = filter_double[i];

    filtered[i] = (float)filter_double[i];
  }
}

static int rot_v_gyro_autocal(float *in, float *bias, float *calibrated)
{
  float g_lpf_out_dps[3] = { 0.0f };
  float threeHz_gyro_mean[3] = { 0.0f }, threeHz_gyro_rms[3] = {
  0.0f};
  int calibration_success = 0;
  static unsigned int gyro_static_counter = 0;

  float in_dps[3] = { in[0] * RAD2DEG, in[1] * RAD2DEG, in[2] * RAD2DEG };

  gyroLowPassFilter(in_dps, g_lpf_out_dps);

  get_rms(g_lpf_out_dps, threeHz_gyro_mean, threeHz_gyro_rms);

  if (threeHz_gyro_rms[0] < PSD_STDEV &&        //dps-rms @ 0.014*sqrt(3)
      threeHz_gyro_rms[1] < PSD_STDEV && threeHz_gyro_rms[2] < PSD_STDEV && fabs(threeHz_gyro_rms[0]) < 20 &&   // 20dps absolute
      fabs(threeHz_gyro_rms[1]) < 20 && fabs(threeHz_gyro_rms[2]) < 20) {

    gyro_static_counter++;

    if (gyro_static_counter > ALGO_SAMPLE_HZ) { // update bias

      calibration_success = 1;

      for (int i = 0; i < 3; i++)
        bias[i] = threeHz_gyro_mean[i] * DEG2RAD;
    }
  } else {
    gyro_static_counter = 0;
  }

  for (int i = 0; i < 3; i++) {

    if (bias[0] != 0 && bias[1] != 0 && bias[2] != 0)
      calibrated[i] = in[i] - bias[i];
    else
      calibrated[i] = in[i];
  }

  return calibration_success;
}

static void rot_v_fusion_acc(unsigned long long timestamp, float *acc)
{
  float a_lpf_out[3] = { 0.0f };
  float a_magnitude, pitch, roll, yaw, momentum;

  static unsigned long long timestamp_previous = 0;
  float rot_vec[4] = { 0.0f };
  float rot_mat[9] = { 0.0f };

  float dt_s = (float)(timestamp - timestamp_previous) / osKernelGetTickFreq(); //timestamp unit is us

  if (dt_s > 1)                 //need refine, 100ms duration is wrong
    printf("@@@ACC_TIME_ERROR %f ms\r\n", dt_s);

  //printf("acc_data: %d %f %f %f\r\n", timestamp, acc[0], acc[1], acc[2]);
  posture(acc, rot_vec, dt_s, (int)SENSOR_TYPE_ACCELEROMETER);
  get_rot_mat(rot_vec, rot_mat);
  timestamp_previous = timestamp;

  pitch = asinf(rot_mat[7]) * RAD2DEG;
  roll = asinf(-rot_mat[6]) * RAD2DEG;
  yaw = asinf(rot_mat[8]) * RAD2DEG;
  temp_roll = roll;
  temp_pitch = pitch;
  temp_yaw = yaw;
  //printf("eular after acc: %d %.1f %.1f %.1f\r\n", timestamp, yaw, pitch, roll);

  accLowPassFilter(acc, a_lpf_out);

  a_magnitude = sqrtf(a_lpf_out[0] * a_lpf_out[0] + a_lpf_out[1] * a_lpf_out[1]
                      + a_lpf_out[2] * a_lpf_out[2]);
  momentum = (a_magnitude - GRAVITY);
  momentum *= momentum;

  rot_v_quaternion[0] = rot_vec[0];
  rot_v_quaternion[1] = rot_vec[1];
  rot_v_quaternion[2] = rot_vec[2];
  rot_v_quaternion[3] = rot_vec[3];
}

static void rot_v_fusion_gyro(unsigned long long timestamp, float *gyro)
{
  float a_lpf_out[3] = { 0.0f };
  float gyro_calibrated[3] = { 0.0f };
  static float gyro_bias[3] = { 0.0f };

  float a_magnitude, pitch, roll, yaw, momentum;

  static unsigned long long timestamp_previous = 0;
  float rot_vec[4] = { 0.0f };
  float rot_mat[9] = { 0.0f };

  float dt_s = (float)(timestamp - timestamp_previous) / osKernelGetTickFreq(); //timestamp_ unit is ms

  if (dt_s > 1)                 //need refine, 1s duration is too big
    printf("@@@GYRO_TIME_ERROR %f ms\r\n", dt_s);

  //printf("gyro_data: %d %f %f %f\r\n", timestamp, gyro[0], gyro[1], gyro[2]);

  rot_v_gyro_autocal(gyro, gyro_bias, gyro_calibrated);
  posture(gyro_calibrated, rot_vec, dt_s, (int)SENSOR_TYPE_GYROSCOPE);
  get_rot_mat(rot_vec, rot_mat);
  timestamp_previous = timestamp;

  pitch = asinf(rot_mat[7]) * RAD2DEG;
  roll = asinf(-rot_mat[6]) * RAD2DEG;
  yaw = asinf(rot_mat[8]) * RAD2DEG;
  temp_roll = roll;
  temp_pitch = pitch;
  temp_yaw = yaw;
  //printf("eular_after_gyro: %llu %.1f %.1f %.1f\r\n", timestamp, yaw, pitch, roll);

  float gyro_momentum = matrix_length(gyro_calibrated, 3);
  //-----------------------------Preprocess----------------------------------//
  rot_v_quaternion[0] = rot_vec[0];
  rot_v_quaternion[1] = rot_vec[1];
  rot_v_quaternion[2] = rot_vec[2];
  rot_v_quaternion[3] = rot_vec[3];
}

int rotv_init(void *para, void *para2)
{
  rot_v_init_parms = (platform_prams *) para;
  sensor_ptr = (sensor_device **) para2;
  rot_v_state_init();
  if ((sensor_ptr[SENSOR_TYPE_ACCELEROMETER] != NULL)
//      && (sensor_ptr[SENSOR_TYPE_MAGNETIC_FIELD] != NULL)
      && (sensor_ptr[SENSOR_TYPE_GYROSCOPE] != NULL)) {
    if ((sensor_ptr[SENSOR_TYPE_ACCELEROMETER]->init_completed)
//          && (sensor_ptr[SENSOR_TYPE_MAGNETIC_FIELD]->init_completed)
        && (sensor_ptr[SENSOR_TYPE_GYROSCOPE]->init_completed)) {
      printf("all depending sensor enabled\r\n");
      return SENSOR_SUCCESS;
    } else {
      printf("depending sensor missing, init will exit with fail!\r\n");
    }
  }

  return SENSOR_FAILED;
}

int rotv_init_complete(void *para)
{
  return SENSOR_SUCCESS;
}

int rotv_enable()
{
  sensor_message_event_t sensor_msg_event;
  osStatus_t res;
  if (sensor_activate_count == 0) {
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type = SENSOR_ACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_ACCELEROMETER;
    osMessageQueuePut(rot_v_init_parms->SensorMessageQHandle, &sensor_msg_event,
                      0, 0);
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type = SENSOR_ACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_GYROSCOPE;
    osMessageQueuePut(rot_v_init_parms->SensorMessageQHandle, &sensor_msg_event,
                      0, 0);
/*		sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
		sensor_msg_event.message_event_t.config_event.config_type = SENSOR_ACTIVATE;
		sensor_msg_event.message_event_t.config_event.sensor_type = SENSOR_TYPE_MAGNETIC_FIELD;
		osMessageQueuePut(rot_v_init_parms->SensorMessageQHandle, &sensor_msg_event, 0, 0);*/
    rot_v_activated = true;
  }
  sensor_activate_count++;
}

int rotv_disable()
{
  sensor_message_event_t sensor_msg_event;
  osStatus_t res;
  sensor_activate_count--;
  if (sensor_activate_count == 0) {
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type =
        SENSOR_DEACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_ACCELEROMETER;
    osMessageQueuePut(rot_v_init_parms->SensorMessageQHandle, &sensor_msg_event,
                      0, 0);
    sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
    sensor_msg_event.message_event_t.config_event.config_type =
        SENSOR_DEACTIVATE;
    sensor_msg_event.message_event_t.config_event.sensor_type =
        SENSOR_TYPE_GYROSCOPE;
    osMessageQueuePut(rot_v_init_parms->SensorMessageQHandle, &sensor_msg_event,
                      0, 0);
/*		sensor_msg_event.message_event_type = SENSOR_CONFIG_MESSAGE;
		sensor_msg_event.message_event_t.config_event.config_type = SENSOR_DEACTIVATE;
		sensor_msg_event.message_event_t.config_event.sensor_type = SENSOR_TYPE_MAGNETIC_FIELD;
		osMessageQueuePut(rot_v_init_parms->SensorMessageQHandle, &sensor_msg_event, 0, 0);*/
    rot_v_activated = false;
  }
}

int rotv_activate(bool activate)
{
  if (activate) {
    rotv_enable();
  } else {
    rotv_disable();
  }
  return SENSOR_SUCCESS;
}

int rotv_publish_sensor_data(void *para)
{
  /* Read samples in polling mode (no int) */
  sensors_event_t sensor_data = { 0 };
  //printf("rotv_publish_sensor_data: %d \r\n", light_activated);
  if (rot_v_activated) {
    //send related messages to SensorDataQ
    sensor_data.sensor_type = SENSOR_TYPE_ROTATION_VECTOR;
    sensor_data.accuracy = 3;
    sensor_data.timestamp = sensor_get_timestamp();
    sensor_data.sensor_data_t.vec.data[0] = rot_v_quaternion[0];
    sensor_data.sensor_data_t.vec.data[1] = rot_v_quaternion[1];
    sensor_data.sensor_data_t.vec.data[2] = rot_v_quaternion[2];
    sensor_data.sensor_data_t.vec.data[3] = rot_v_quaternion[3];
    //sensor_data.sensor_data_t.vec.data[0] = temp_roll;
    //sensor_data.sensor_data_t.vec.data[1] = temp_pitch;
    //sensor_data.sensor_data_t.vec.data[2] = temp_yaw;
    //sensor_data.sensor_data_t.vec.data[3] = 0;

    printf("rot_v quaternion %f %f %f %f\r\n", rot_v_quaternion[0],
           rot_v_quaternion[1], rot_v_quaternion[2], rot_v_quaternion[3]);

    osMessageQueuePut(rot_v_init_parms->SensorDataQHandle, &sensor_data, 0, 0);
  }
  return SENSOR_SUCCESS;
}

int rotv_config(uint8_t config_type, void *para)
{
  sensors_event_t *sensor_data;
  sensor_data = (sensors_event_t *) para;
  float raw_data[3] = { 0 };
  if (config_type == SENSOR_CONFIG_DATA) {
    if (sensor_data->sensor_type == SENSOR_TYPE_ACCELEROMETER) {
      raw_data[0] = sensor_data->sensor_data_t.vec.acceleration.v[0];
      raw_data[1] = sensor_data->sensor_data_t.vec.acceleration.v[1];
      raw_data[2] = sensor_data->sensor_data_t.vec.acceleration.v[2];

      rot_v_fusion_acc(sensor_data->timestamp, raw_data);

    } else if (sensor_data->sensor_type == SENSOR_TYPE_GYROSCOPE) {
      raw_data[0] = sensor_data->sensor_data_t.vec.gyro.v[0];
      raw_data[1] = sensor_data->sensor_data_t.vec.gyro.v[1];
      raw_data[2] = sensor_data->sensor_data_t.vec.gyro.v[2];

      rot_v_fusion_gyro(sensor_data->timestamp, raw_data);

    } else if (sensor_data->sensor_type == SENSOR_TYPE_MAGNETIC_FIELD) {
      raw_data[0] = sensor_data->sensor_data_t.vec.magnetic.v[0];
      raw_data[1] = sensor_data->sensor_data_t.vec.magnetic.v[1];
      raw_data[2] = sensor_data->sensor_data_t.vec.magnetic.v[2];
    }
  }
  return SENSOR_SUCCESS;
}

int rotv_publish_config_resp(void *para)
{
  return SENSOR_SUCCESS;
}
