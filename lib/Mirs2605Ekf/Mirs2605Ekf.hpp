/**
 * @file Mirs2605Ekf.hpp
 * @brief 差動二輪用・ESP32向け軽量EKF（計算層・ヒープ不使用）
 *
 * 状態: [x, y, theta, v, bg]（位置・姿勢・前進速度・ジャイロバイアス）。
 * 角速度 w 自体は状態に持たず、入力 gz から (gz - bg) として求める。
 * 5x5 共分散1個だけ保持し、更新は全てスカラ逐次更新のため
 * 行列逆行列・動的確保・Eigen は不要。15ms制御周期で完結する。
 *
 * 融合:
 * - predict(gz, dt): ジャイロで姿勢・位置を伝搬
 * - updateVel(v_enc): エンコーダ前進速度で v を補正
 * - updateYawRate(w_enc, gz): エンコーダ角速度で bg を補正
 * - updateMagTheta(yaw_mag): 地磁気ヨーで theta を補正
 * - updateZeroVel(): 停止検出時の v=0 拘束（加速度計由来のZUPT用）
 *
 * 座標の約束: x前・y左・theta反時計回り（ROS REP-103準拠）。
 * BMX055の軸とロボット軸が一致していることが前提。
 * 不一致・マウント回転・磁気 declination は呼び出し側で補正すること。
 */
#pragma once

#include <stdint.h>

class Mirs2605Ekf {
public:
    enum StateIdx { IX = 0, IY = 1, ITH = 2, IV = 3, IBG = 4, N = 5 };

    Mirs2605Ekf();

    void reset();

    /** @brief プロセスノイズ対角を設定（既定値は室内低速向け）。
     *  @param q_theta 姿勢伝搬ノイズ [(rad^2/s)]
     *  @param q_v 速度ランダムウォーク [(m/s)^2/s]
     *  @param q_bg バイアスランダムウォーク [(rad/s)^2/s]
     */
    void setProcessNoise(float q_theta, float q_v, float q_bg);

    /** @brief 観測ノイズ（分散）を設定。
     *  @param r_vel エンコーダ速度 [(m/s)^2]
     *  @param r_yaw_rate エンコーダ角速度 [(rad/s)^2]
     *  @param r_mag 地磁気ヨー [rad^2]
     */
    void setMeasurementNoise(float r_vel, float r_yaw_rate, float r_mag);

    /** @brief 初期共分散の対角を設定（起動直後の不確かさ）。 */
    void setInitialCovariance(float p_xy, float p_theta, float p_v, float p_bg);

    /**
     * @brief 予測ステップ。ジャイロで theta/x/y を伝搬する。
     * @param gz_rad_s ジャイロz [rad/s]（ロボットz軸と同符号のこと）
     * @param dt_sec 周期 [s]（<=0では何もしない。>0.1は0.1にクランプ）
     */
    void predict(float gz_rad_s, float dt_sec);

    /** @brief エンコーダ前進速度で v を補正する。 */
    void updateVel(float v_enc, float r_override = -1.0f);

    /**
     * @brief エンコーダ角速度でジャイロバイアスを補正する。
     * @details 真の角速度を w_enc とみなし、z = gz - w_enc を
     *  bg の観測として扱う（H = [0,0,0,0,1]）。
     */
    void updateYawRate(float w_enc, float gz_rad_s, float r_override = -1.0f);

    /**
     * @brief 低速ゲート付きバイアス更新。
     * @details |v_enc| が v_gate 超では更新しない。走行中のエンコーダ
     *  角速度ノイズが bg 推定に混入するのを防ぐ（bgは停止付近で可観測）。
     *  制御周期内の使用は本関数を使うこと。
     */
    void updateYawRateGated(float w_enc, float gz_rad_s, float v_enc,
                            float v_gate = 0.05f, float r_override = -1.0f);

    /** @brief 地磁気ヨーで theta を補正する（屋内磁気外乱時は呼ばないこと）。 */
    void updateMagTheta(float yaw_mag_rad, float r_override = -1.0f);

    /** @brief 停止中拘束。v=0 の観測として扱う（ZUPT）。 */
    void updateZeroVel(float r_override = -1.0f);

    // --- 状態取得 ---
    float x() const { return x_[IX]; }
    float y() const { return x_[IY]; }
    float theta() const { return x_[ITH]; }
    float v() const { return x_[IV]; }
    float gyroBias() const { return x_[IBG]; }
    float cov(int i, int j) const;

    // --- 静的ヘルパ（Arduino非依存・単体テスト可） ---
    static float normalizeAngle(float a);
    static float angleResidual(float meas, float pred);

    /**
     * @brief 加速度でチルト補正した地磁気ヨーを求める。
     * @return ヨー [rad]（-PI..PI）。加速度ノルム異常時は mag の生ヨーを返す。
     */
    static float magYawTiltCompensated(float mx, float my, float mz,
                                       float ax, float ay, float az);

    /** @brief 停止判定。加速度ノルムがg付近かつ角速度・車輪速度が微小。 */
    static bool isStationary(float ax, float ay, float az,
                             float gz_rad_s, float v_enc,
                             float acc_tol = 0.6f,
                             float gz_tol = 0.03f,
                             float v_tol = 0.02f);

private:
    float x_[N];
    float p_[N][N];
    float q_theta_ = 1e-5f;
    float q_v_ = 1e-3f;
    float q_bg_ = 1e-8f;
    float r_vel_ = 4e-4f;       // sigma 0.02 m/s
    float r_yaw_rate_ = 1e-4f;  // sigma 0.01 rad/s
    float r_mag_ = 9e-2f;       // sigma 0.3 rad（屋内想定で大きめ）
    float r_zupt_ = 1e-6f;

    void scalarUpdate(const float h[N], float residual, float r);
};
