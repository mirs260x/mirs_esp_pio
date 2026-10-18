#include "ImuPlugin.hpp"
#include "BMX055.hpp"

ImuPlugin::ImuPlugin(BMX055 &imu, int sda_pin, int scl_pin)
    : imu_(imu)
    , sda_pin_(sda_pin)
    , scl_pin_(scl_pin)
{
}

bool ImuPlugin::begin() {
    return imu_.begin(sda_pin_, scl_pin_);
}

void ImuPlugin::update(SharedSensor &snapshot) {
    BMX055Data data{};
    if (!imu_.update(data)) {
        snapshot.imu_ok = false;
        snapshot.mag_ok = false;
        snapshot.ax = snapshot.ay = snapshot.az = 0.0f;
        snapshot.gx = snapshot.gy = snapshot.gz = 0.0f;
        snapshot.mx = snapshot.my = snapshot.mz = 0.0f;
        return;
    }
    snapshot.imu_ok = imu_.isAccelOk() || imu_.isGyroOk();
    snapshot.mag_ok = imu_.isMagOk();
    snapshot.ax = data.ax;
    snapshot.ay = data.ay;
    snapshot.az = data.az;
    snapshot.gx = data.gx;
    snapshot.gy = data.gy;
    snapshot.gz = data.gz;
    snapshot.mx = data.mx;
    snapshot.my = data.my;
    snapshot.mz = data.mz;
}
