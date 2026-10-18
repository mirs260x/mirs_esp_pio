#include "Mirs2605Ekf.hpp"
#include <math.h>

namespace {

float clampDt(float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }
    if (dt > 0.1f) {
        return 0.1f;
    }
    return dt;
}

}  // namespace

Mirs2605Ekf::Mirs2605Ekf() {
    reset();
}

void Mirs2605Ekf::reset() {
    for (int i = 0; i < N; ++i) {
        x_[i] = 0.0f;
        for (int j = 0; j < N; ++j) {
            p_[i][j] = 0.0f;
        }
    }
    // 起動直後は姿勢・バイアスが不定のため大きめに持つ
    p_[IX][IX] = 1e-2f;
    p_[IY][IY] = 1e-2f;
    p_[ITH][ITH] = 1e-2f;
    p_[IV][IV] = 1e-2f;
    p_[IBG][IBG] = 1e-4f;
}

void Mirs2605Ekf::setProcessNoise(float q_theta, float q_v, float q_bg) {
    if (q_theta > 0.0f) {
        q_theta_ = q_theta;
    }
    if (q_v > 0.0f) {
        q_v_ = q_v;
    }
    if (q_bg > 0.0f) {
        q_bg_ = q_bg;
    }
}

void Mirs2605Ekf::setMeasurementNoise(float r_vel, float r_yaw_rate, float r_mag) {
    if (r_vel > 0.0f) {
        r_vel_ = r_vel;
    }
    if (r_yaw_rate > 0.0f) {
        r_yaw_rate_ = r_yaw_rate;
    }
    if (r_mag > 0.0f) {
        r_mag_ = r_mag;
    }
}

void Mirs2605Ekf::setInitialCovariance(float p_xy, float p_theta, float p_v, float p_bg) {
    p_[IX][IX] = p_xy;
    p_[IY][IY] = p_xy;
    p_[ITH][ITH] = p_theta;
    p_[IV][IV] = p_v;
    p_[IBG][IBG] = p_bg;
}

float Mirs2605Ekf::cov(int i, int j) const {
    if (i < 0 || i >= N || j < 0 || j >= N) {
        return 0.0f;
    }
    return p_[i][j];
}

void Mirs2605Ekf::predict(float gz_rad_s, float dt_sec) {
    const float dt = clampDt(dt_sec);
    if (dt <= 0.0f) {
        return;
    }
    const float theta = x_[ITH];
    const float v = x_[IV];
    const float w = gz_rad_s - x_[IBG];
    const float c = cosf(theta);
    const float s = sinf(theta);

    // --- 状態伝搬 ---
    x_[IX] += v * c * dt;
    x_[IY] += v * s * dt;
    x_[ITH] = normalizeAngle(theta + w * dt);
    // v, bg はランダムウォーク（予測では維持）

    // --- 共分散伝搬 P = F P F^T + Q ---
    // F の非自明成分のみ展開する（5x5全乗算はESP32でも軽いが自明項を省く）
    const float f02 = -v * s * dt;
    const float f03 = c * dt;
    const float f12 = v * c * dt;
    const float f13 = s * dt;
    const float f24 = -dt;

    // FP = F * P
    float fp[N][N];
    for (int j = 0; j < N; ++j) {
        fp[0][j] = p_[0][j] + f02 * p_[2][j] + f03 * p_[3][j];
        fp[1][j] = p_[1][j] + f12 * p_[2][j] + f13 * p_[3][j];
        fp[2][j] = p_[2][j] + f24 * p_[4][j];
        fp[3][j] = p_[3][j];
        fp[4][j] = p_[4][j];
    }
    // P = FP * F^T
    float np[N][N];
    for (int i = 0; i < N; ++i) {
        np[i][0] = fp[i][0] + fp[i][2] * f02 + fp[i][3] * f03;
        np[i][1] = fp[i][1] + fp[i][2] * f12 + fp[i][3] * f13;
        np[i][2] = fp[i][2] + fp[i][4] * f24;
        np[i][3] = fp[i][3];
        np[i][4] = fp[i][4];
    }
    // 対称化＋Q加算（数値誤差の蓄積を抑える）
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            p_[i][j] = 0.5f * (np[i][j] + np[j][i]);
        }
    }
    p_[ITH][ITH] += q_theta_ * dt;
    p_[IV][IV] += q_v_ * dt;
    p_[IBG][IBG] += q_bg_ * dt;
}

void Mirs2605Ekf::scalarUpdate(const float h[N], float residual, float r) {
    if (!(r > 0.0f)) {
        return;
    }
    // S = H P H^T + R
    float hp[N] = {0.0f};
    for (int j = 0; j < N; ++j) {
        float acc = 0.0f;
        for (int k = 0; k < N; ++k) {
            acc += h[k] * p_[k][j];
        }
        hp[j] = acc;
    }
    float s = r;
    for (int k = 0; k < N; ++k) {
        s += hp[k] * h[k];
    }
    if (!(s > 1e-12f)) {
        return;
    }
    // K = P H^T / S
    float k[N];
    for (int i = 0; i < N; ++i) {
        float acc = 0.0f;
        for (int kk = 0; kk < N; ++kk) {
            acc += p_[i][kk] * h[kk];
        }
        k[i] = acc / s;
    }
    for (int i = 0; i < N; ++i) {
        x_[i] += k[i] * residual;
    }
    x_[ITH] = normalizeAngle(x_[ITH]);
    // P = (I - K H) P（Joseph形の簡易版。逐次スカラ更新では十分）
    float kh[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            kh[i][j] = k[i] * hp[j];
        }
    }
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            p_[i][j] -= kh[i][j];
        }
        // 対角が負に潰れないよう下限を保つ
        if (p_[i][i] < 1e-12f) {
            p_[i][i] = 1e-12f;
        }
    }
}

void Mirs2605Ekf::updateVel(float v_enc, float r_override) {
    const float h[N] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    scalarUpdate(h, v_enc - x_[IV], r_override > 0.0f ? r_override : r_vel_);
}

void Mirs2605Ekf::updateYawRate(float w_enc, float gz_rad_s, float r_override) {
    // z = gz - w_enc を bg の観測とする
    const float h[N] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    scalarUpdate(h, (gz_rad_s - w_enc) - x_[IBG], r_override > 0.0f ? r_override : r_yaw_rate_);
}

void Mirs2605Ekf::updateYawRateGated(float w_enc, float gz_rad_s, float v_enc,
                                     float v_gate, float r_override) {
    if (!(v_gate > 0.0f)) {
        return;
    }
    float av = v_enc;
    if (av < 0.0f) {
        av = -av;
    }
    if (av > v_gate) {
        return;
    }
    updateYawRate(w_enc, gz_rad_s, r_override);
}

void Mirs2605Ekf::updateMagTheta(float yaw_mag_rad, float r_override) {
    const float h[N] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    scalarUpdate(h, angleResidual(yaw_mag_rad, x_[ITH]), r_override > 0.0f ? r_override : r_mag_);
}

void Mirs2605Ekf::updateZeroVel(float r_override) {
    const float h[N] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    scalarUpdate(h, 0.0f - x_[IV], r_override > 0.0f ? r_override : r_zupt_);
}

float Mirs2605Ekf::normalizeAngle(float a) {
    return atan2f(sinf(a), cosf(a));
}

float Mirs2605Ekf::angleResidual(float meas, float pred) {
    return normalizeAngle(meas - pred);
}

float Mirs2605Ekf::magYawTiltCompensated(float mx, float my, float mz,
                                        float ax, float ay, float az) {
    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (!(norm > 1e-6f)) {
        return atan2f(-my, mx);
    }
    // 正規化加速度から roll/pitch を求め、地磁気を水平面に戻す
    const float axn = ax / norm;
    const float ayn = ay / norm;
    const float azn = az / norm;
    const float roll = atan2f(ayn, azn);
    const float pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));
    const float cr = cosf(roll);
    const float sr = sinf(roll);
    const float cp = cosf(pitch);
    const float sp = sinf(pitch);
    const float mxh = mx * cp + mz * sp;
    const float myh = mx * sr * sp + my * cr - mz * sr * cp;
    return atan2f(-myh, mxh);
}

bool Mirs2605Ekf::isStationary(float ax, float ay, float az,
                               float gz_rad_s, float v_enc,
                               float acc_tol, float gz_tol, float v_tol) {
    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    constexpr float kGravity = 9.80665f;
    if (fabsf(norm - kGravity) > acc_tol) {
        return false;
    }
    if (fabsf(gz_rad_s) > gz_tol) {
        return false;
    }
    if (fabsf(v_enc) > v_tol) {
        return false;
    }
    return true;
}
