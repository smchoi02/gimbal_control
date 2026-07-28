#pragma once

#include <math.h>

class MahonyAhrs {
 public:
  void reset() {
    q_[0] = 1.0f;
    q_[1] = q_[2] = q_[3] = 0.0f;
    integral_[0] = integral_[1] = integral_[2] = 0.0f;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz, float dt) {
    const float magNormSq = mx * mx + my * my + mz * mz;
    if (magNormSq < 1.0e-9f) {
      updateImu(gx, gy, gz, ax, ay, az, dt);
      return;
    }

    const float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq < 1.0e-9f || dt <= 0.0f) return;
    const float invAcc = 1.0f / sqrtf(accNormSq);
    ax *= invAcc;
    ay *= invAcc;
    az *= invAcc;
    const float invMag = 1.0f / sqrtf(magNormSq);
    mx *= invMag;
    my *= invMag;
    mz *= invMag;

    const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    const float hx =
        2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) +
                my * (q1 * q2 - q0 * q3) +
                mz * (q1 * q3 + q0 * q2));
    const float hy =
        2.0f * (mx * (q1 * q2 + q0 * q3) +
                my * (0.5f - q1 * q1 - q3 * q3) +
                mz * (q2 * q3 - q0 * q1));
    const float bx = sqrtf(hx * hx + hy * hy);
    const float bz =
        2.0f * (mx * (q1 * q3 - q0 * q2) +
                my * (q2 * q3 + q0 * q1) +
                mz * (0.5f - q1 * q1 - q2 * q2));

    const float vx = q1 * q3 - q0 * q2;
    const float vy = q0 * q1 + q2 * q3;
    const float vz = q0 * q0 - 0.5f + q3 * q3;
    const float wx = bx * (0.5f - q2 * q2 - q3 * q3) +
                     bz * (q1 * q3 - q0 * q2);
    const float wy = bx * (q1 * q2 - q0 * q3) +
                     bz * (q0 * q1 + q2 * q3);
    const float wz = bx * (q0 * q2 + q1 * q3) +
                     bz * (0.5f - q1 * q1 - q2 * q2);

    applyFeedback(gx, gy, gz,
                  ay * vz - az * vy + my * wz - mz * wy,
                  az * vx - ax * vz + mz * wx - mx * wz,
                  ax * vy - ay * vx + mx * wy - my * wx, dt);
  }

  const float* quaternion() const { return q_; }

 private:
  static constexpr float TWO_KP = 2.0f;
  static constexpr float TWO_KI = 0.02f;

  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float integral_[3] = {0.0f, 0.0f, 0.0f};

  void updateImu(float gx, float gy, float gz, float ax, float ay, float az,
                 float dt) {
    const float normSq = ax * ax + ay * ay + az * az;
    if (normSq < 1.0e-9f || dt <= 0.0f) return;
    const float invNorm = 1.0f / sqrtf(normSq);
    ax *= invNorm;
    ay *= invNorm;
    az *= invNorm;

    const float vx = q_[1] * q_[3] - q_[0] * q_[2];
    const float vy = q_[0] * q_[1] + q_[2] * q_[3];
    const float vz = q_[0] * q_[0] - 0.5f + q_[3] * q_[3];
    applyFeedback(gx, gy, gz, ay * vz - az * vy,
                  az * vx - ax * vz, ax * vy - ay * vx, dt);
  }

  void applyFeedback(float gx, float gy, float gz, float ex, float ey,
                     float ez, float dt) {
    integral_[0] += TWO_KI * ex * dt;
    integral_[1] += TWO_KI * ey * dt;
    integral_[2] += TWO_KI * ez * dt;
    gx += integral_[0] + TWO_KP * ex;
    gy += integral_[1] + TWO_KP * ey;
    gz += integral_[2] + TWO_KP * ez;

    const float halfDt = 0.5f * dt;
    const float qa = q_[0], qb = q_[1], qc = q_[2];
    q_[0] += (-qb * gx - qc * gy - q_[3] * gz) * halfDt;
    q_[1] += (qa * gx + qc * gz - q_[3] * gy) * halfDt;
    q_[2] += (qa * gy - qb * gz + q_[3] * gx) * halfDt;
    q_[3] += (qa * gz + qb * gy - qc * gx) * halfDt;

    const float invNorm =
        1.0f / sqrtf(q_[0] * q_[0] + q_[1] * q_[1] +
                     q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] *= invNorm;
    q_[1] *= invNorm;
    q_[2] *= invNorm;
    q_[3] *= invNorm;
  }
};

