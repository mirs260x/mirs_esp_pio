/**
 * @file BMX055.hpp
 * @brief BMX055 (Accelerometer + Gyroscope + Magnetometer) 9-Axis IMU Driver for ESP32
 *        I2C backend: ESP-IDF i2c_master (non-blocking timeout, no Wire dependency)
 *
 * References & Credits:
 * - Bosch Sensortec BMX055 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmx055-ds000.pdf
 * - Akizuki Denshi AE-BMX055 Module & Sample Code: https://akizukidenshi.com/catalog/g/gK-13010/
 *
 * License: MIT License
 */

#pragma once

#include <Arduino.h>
#include "driver/i2c_master.h"

/**
 * @brief BMX055 default I2C addresses
 *        Akizuki module: JP1/JP2/JP3 Open  = 0x19 / 0x69 / 0x13
 *                        JP1/JP2/JP3 Closed = 0x18 / 0x68 / 0x10
 */
namespace BMX055_ADDR {
    constexpr uint8_t ACCEL_DEFAULT = 0x19;
    constexpr uint8_t GYRO_DEFAULT  = 0x69;
    constexpr uint8_t MAG_DEFAULT   = 0x13;

    constexpr uint8_t ACCEL_ALT     = 0x18;
    constexpr uint8_t GYRO_ALT      = 0x68;
    constexpr uint8_t MAG_ALT       = 0x10;
}

struct BMX055Data {
    // Accelerometer [m/s^2]
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    // Gyroscope [rad/s]
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    // Gyroscope [deg/s]
    float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
    // Magnetometer [uT]
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
};

struct BMX055RawData {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    int16_t mx = 0, my = 0, mz = 0;
};

class BMX055 {
public:
    BMX055(uint8_t addr_acc  = BMX055_ADDR::ACCEL_DEFAULT,
           uint8_t addr_gyro = BMX055_ADDR::GYRO_DEFAULT,
           uint8_t addr_mag  = BMX055_ADDR::MAG_DEFAULT);

    /**
     * @brief I2Cバスを初期化してセンサーレジスタを設定する。
     * @param sda_pin  SDA GPIO番号
     * @param scl_pin  SCL GPIO番号
     * @param port     I2Cポート番号 (I2C_NUM_0 or I2C_NUM_1)
     * @param freq_hz  I2Cクロック周波数 [Hz] (デフォルト 400kHz)
     * @param timeout_ms 1トランザクションあたりのタイムアウト [ms] (デフォルト 10ms)
     * @return true: Accel/Gyro 少なくとも一方の初期化成功
     */
    bool begin(int sda_pin, int scl_pin,
               i2c_port_num_t port = I2C_NUM_0,
               uint32_t freq_hz    = 400000,
               int timeout_ms      = 10);

    /**
     * @brief 9軸データを一括読み取りして data に格納する。
     * @return true: 少なくとも1センサーの読み取り成功
     */
    bool update(BMX055Data &data);

    bool readAccel(float &ax, float &ay, float &az);
    bool readGyro(float &gx, float &gy, float &gz, bool in_rad = true);
    bool readMag(float &mx, float &my, float &mz);
    bool readRaw(BMX055RawData &raw);

    const BMX055Data&    getData()    const { return _data; }
    const BMX055RawData& getRawData() const { return _raw;  }

    bool isInitialized() const { return _initialized; }
    bool isAccelOk()     const { return _acc_ok;  }
    bool isGyroOk()      const { return _gyro_ok; }
    bool isMagOk()       const { return _mag_ok;  }

private:
    uint8_t _addr_acc;
    uint8_t _addr_gyro;
    uint8_t _addr_mag;

    bool _initialized = false;
    bool _acc_ok  = false;
    bool _gyro_ok = false;
    bool _mag_ok  = false;

    int _timeout_ms = 10;

    // ESP-IDF i2c_master ハンドル
    i2c_master_bus_handle_t  _bus_handle  = nullptr;
    i2c_master_dev_handle_t  _dev_acc     = nullptr;
    i2c_master_dev_handle_t  _dev_gyro    = nullptr;
    i2c_master_dev_handle_t  _dev_mag     = nullptr;

    BMX055Data    _data;
    BMX055RawData _raw;

    // デバイスハンドルを取得 (候補アドレスを順に試す)
    bool probeAndAdd(i2c_master_dev_handle_t &handle,
                     uint8_t &addr,
                     const uint8_t candidates[],
                     size_t count);

    bool addDevice(i2c_master_dev_handle_t &handle, uint8_t addr);

    bool initAccel();
    bool initGyro();
    bool initMag();

    // --- 低レベルI2O操作 ---
    bool writeRegister(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value);
    bool readRegisters(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);
};
