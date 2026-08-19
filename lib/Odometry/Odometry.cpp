#include "Odometry.hpp"
#include "hardware_config.hpp"

void Odometry::begin(uint8_t enc_l_a, uint8_t enc_l_b, uint8_t enc_r_a, uint8_t enc_r_b) {
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    _enc_l.attachFullQuad(enc_l_a, enc_l_b);
    _enc_r.attachFullQuad(enc_r_a, enc_r_b);

    reset();
}

void Odometry::reset() {
    _enc_l.clearCount();
    _enc_r.clearCount();
    _last_count_l = 0;
    _last_count_r = 0;
    x = y = theta = 0.0f;
    v_linear = v_angular = 0.0f;
}

void Odometry::update(float dt) {
    using namespace CugoParams;

    int64_t count_l = _enc_l.getCount();
    int64_t count_r = _enc_r.getCount();

    int64_t d_count_l = count_l - _last_count_l;
    int64_t d_count_r = count_r - _last_count_r;
    _last_count_l = count_l;
    _last_count_r = count_r;

    // カウント差分 -> 回転角[rad] -> 移動距離[m]
    float d_rot_l = (2.0f * PI * (float)d_count_l) / (ENCODER_RESOLUTION * REDUCTION_RATIO);
    float d_rot_r = (2.0f * PI * (float)d_count_r) / (ENCODER_RESOLUTION * REDUCTION_RATIO);

    float d_dist_l = d_rot_l * WHEEL_RADIUS_L;
    float d_dist_r = d_rot_r * WHEEL_RADIUS_R;

    // 差動二輪(クローラの仮想タイヤ)モデルによる積分
    float d_dist  = (d_dist_l + d_dist_r) / 2.0f;
    float d_theta = (d_dist_r - d_dist_l) / TREAD;

    theta += d_theta;
    x += d_dist * cosf(theta);
    y += d_dist * sinf(theta);

    if (dt > 0.0f) {
        v_linear  = d_dist / dt;
        v_angular = d_theta / dt;
    }
}
