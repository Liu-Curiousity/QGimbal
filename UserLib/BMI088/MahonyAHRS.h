#ifndef __MAHONYAHRS_H
#define __MAHONYAHRS_H

#include <array>
#include <cmath>

class MahonyAHRS {
public:
    explicit MahonyAHRS(float sampleFreq) : sampleFreq(sampleFreq) {}

    float q[4]{1, 0, 0, 0};

    void update(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q[1] * q[3] - q[0] * q[2];
            halfvy = q[0] * q[1] + q[2] * q[3];
            halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute and apply integral feedback if enabled
            if (twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                gx += integralFBx; // apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            } else {
                integralFBx = 0.0f; // prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        float hx, hy, bx, bz;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            update(gx, gy, gz, ax, ay, az);
            return;
        }

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q[0] * q[0];
            q0q1 = q[0] * q[1];
            q0q2 = q[0] * q[2];
            q0q3 = q[0] * q[3];
            q1q1 = q[1] * q[1];
            q1q2 = q[1] * q[2];
            q1q3 = q[1] * q[3];
            q2q2 = q[2] * q[2];
            q2q3 = q[2] * q[3];
            q3q3 = q[3] * q[3];

            // Reference direction of Earth's magnetic field
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute and apply integral feedback if enabled
            if (twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                gx += integralFBx; // apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            } else {
                integralFBx = 0.0f; // prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }

private:
    float sampleFreq;                       // sample frequency in Hz
    constexpr static float twoKp{2 * 0.5f}; // 2 * proportional gain (Kp)
    constexpr static float twoKi{2 * 0.0f}; // 2 * integral gain (Ki)

    volatile float integralFBx{0}, integralFBy{0}, integralFBz{0}; // integral error terms scaled by Ki

    // Fast inverse square-root
    static float invSqrt(const float x) {
        const float halfx = 0.5f * x;
        float y = x;
        long i = *reinterpret_cast<long *>(&y);
        i = 0x5f3759df - (i >> 1);
        y = *reinterpret_cast<float *>(&i);
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
};

#endif
