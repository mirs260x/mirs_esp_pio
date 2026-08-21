/**
 * @file BMX055.cpp
 * @brief BMX055 9-Axis IMU Driver Implementation
 *        I2C backend: ESP-IDF i2c_master (non-blocking timeout, no Wire dependency)
 *
 * References & Credits:
 * - Bosch Sensortec BMX055 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmx055-ds000.pdf
 * - Akizuki Denshi AE-BMX055 Module & Sample Code: https://akizukidenshi.com/catalog/g/gK-13010/
 * - ControlEverything BMX055 Arduino Library: https://github.com/ControlEverythingCommunity/BMX055
 *
 * License: MIT License
 */

#include "BMX055.hpp"

// ============================================================
// Accelerometer Registers
// ============================================================
#define REG_ACC_CHIP_ID    0x00
#define REG_ACC_DATA_X_LSB 0x02
#define REG_ACC_PMU_RANGE  0x0F
#define REG_ACC_PMU_BW     0x10
#define REG_ACC_PMU_LPW    0x11

// ============================================================
// Gyroscope Registers
// ============================================================
#define REG_GYRO_CHIP_ID    0x00
#define REG_GYRO_RATE_X_LSB 0x02
#define REG_GYRO_RANGE      0x0F
#define REG_GYRO_BW         0x10
#define REG_GYRO_LPM1       0x11

// ============================================================
// Magnetometer Registers
// ============================================================
#define REG_MAG_CHIP_ID   0x40
#define REG_MAG_DATA_X_LSB 0x42
#define REG_MAG_PWR_CNTL1  0x4B
#define REG_MAG_PWR_CNTL2  0x4C
#define REG_MAG_INT_EN2    0x4E
#define REG_MAG_REP_XY     0x51
#define REG_MAG_REP_Z      0x52

// ============================================================
// スケール定数
// ============================================================
static constexpr float GRAVITY           = 9.80665f;
static constexpr float ACCEL_SCALE_2G    = (2.0f / 2048.0f) * GRAVITY; // 1 LSB → m/s^2
static constexpr float GYRO_SCALE_2000DPS = 2000.0f / 32768.0f;        // 1 LSB → deg/s
static constexpr float MAG_SCALE_UT      = 0.3f;                       // 1 LSB → uT

// ============================================================
// コンストラクタ
// ============================================================
BMX055::BMX055(uint8_t addr_acc, uint8_t addr_gyro, uint8_t addr_mag)
    : _addr_acc(addr_acc), _addr_gyro(addr_gyro), _addr_mag(addr_mag) {}

// ============================================================
// begin: バス初期化 → 各センサー初期化
// ============================================================
bool BMX055::begin(int sda_pin, int scl_pin,
                   i2c_port_num_t port,
                   uint32_t freq_hz,
                   int timeout_ms)
{
    _timeout_ms = timeout_ms;

    // --- I2Cバス設定 ---
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port      = port;
    bus_cfg.sda_io_num    = (gpio_num_t)sda_pin;
    bus_cfg.scl_io_num    = (gpio_num_t)scl_pin;
    bus_cfg.clk_source    = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &_bus_handle);
    if (err != ESP_OK) {
        return false;
    }

    // freq_hz をメンバーに持たせず、各デバイス追加時に渡すため一時保存
    // (addDevice 内で使うためにクラス変数に仮置きする)
    // ここでは bus_cfg には freq が入らないので dev_cfg 側で設定する。
    // addDevice がまだ freq を知る必要があるため、begin() ローカルで
    // ラムダの代わりにヘルパー関数を使う。

    // --- 各センサーを追加・初期化 ---
    // Accel
    {
        const uint8_t cands[] = { _addr_acc,
                                  BMX055_ADDR::ACCEL_DEFAULT,
                                  BMX055_ADDR::ACCEL_ALT };
        i2c_device_config_t dev_cfg = {};
        dev_cfg.scl_speed_hz = freq_hz;
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;

        for (auto addr : cands) {
            dev_cfg.device_address = addr;
            if (i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_acc) == ESP_OK) {
                // ACK確認: 空のトランザクションで応答チェック
                uint8_t dummy;
                if (i2c_master_receive(_dev_acc, &dummy, 1,
                                       _timeout_ms) == ESP_OK) {
                    _addr_acc = addr;
                    break;
                }
                i2c_master_bus_rm_device(_dev_acc);
                _dev_acc = nullptr;
            }
        }
        if (_dev_acc) {
            _acc_ok = initAccel();
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Gyro
    {
        const uint8_t cands[] = { _addr_gyro,
                                  BMX055_ADDR::GYRO_DEFAULT,
                                  BMX055_ADDR::GYRO_ALT };
        i2c_device_config_t dev_cfg = {};
        dev_cfg.scl_speed_hz = freq_hz;
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;

        for (auto addr : cands) {
            dev_cfg.device_address = addr;
            if (i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_gyro) == ESP_OK) {
                uint8_t dummy;
                if (i2c_master_receive(_dev_gyro, &dummy, 1,
                                       _timeout_ms) == ESP_OK) {
                    _addr_gyro = addr;
                    break;
                }
                i2c_master_bus_rm_device(_dev_gyro);
                _dev_gyro = nullptr;
            }
        }
        if (_dev_gyro) {
            _gyro_ok = initGyro();
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Mag: suspend モードから起こしてから probe する
    {
        const uint8_t cands[] = { _addr_mag,
                                  BMX055_ADDR::MAG_DEFAULT,
                                  BMX055_ADDR::MAG_ALT,
                                  0x11, 0x12 };
        i2c_device_config_t dev_cfg = {};
        dev_cfg.scl_speed_hz = freq_hz;
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;

        // Step1: suspend 解除コマンドをブロードキャスト
        for (auto addr : cands) {
            dev_cfg.device_address = addr;
            i2c_master_dev_handle_t tmp = nullptr;
            if (i2c_master_bus_add_device(_bus_handle, &dev_cfg, &tmp) == ESP_OK) {
                uint8_t wake_cmd[2] = { REG_MAG_PWR_CNTL1, 0x01 };
                i2c_master_transmit(tmp, wake_cmd, 2, _timeout_ms);
                i2c_master_bus_rm_device(tmp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 起動待ち

        // Step2: ACK が返るアドレスを探す
        for (auto addr : cands) {
            dev_cfg.device_address = addr;
            if (i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_mag) == ESP_OK) {
                uint8_t dummy;
                if (i2c_master_receive(_dev_mag, &dummy, 1,
                                       _timeout_ms) == ESP_OK) {
                    _addr_mag = addr;
                    break;
                }
                i2c_master_bus_rm_device(_dev_mag);
                _dev_mag = nullptr;
            }
        }
        if (_dev_mag) {
            _mag_ok = initMag();
        }
    }

    _initialized = _acc_ok || _gyro_ok;
    return _initialized;
}

// ============================================================
// センサー初期化
// ============================================================
bool BMX055::initAccel() {
    // range +/-2g
    if (!writeRegister(_dev_acc, REG_ACC_PMU_RANGE, 0x03)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // bandwidth 125Hz
    if (!writeRegister(_dev_acc, REG_ACC_PMU_BW, 0x0C)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // normal power mode
    if (!writeRegister(_dev_acc, REG_ACC_PMU_LPW, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

bool BMX055::initGyro() {
    // range +/-2000 deg/s
    if (!writeRegister(_dev_gyro, REG_GYRO_RANGE, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // bandwidth 1000Hz ODR
    if (!writeRegister(_dev_gyro, REG_GYRO_BW, 0x02)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // normal power mode
    if (!writeRegister(_dev_gyro, REG_GYRO_LPM1, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

bool BMX055::initMag() {
    // soft reset → power on
    writeRegister(_dev_mag, REG_MAG_PWR_CNTL1, 0x83);
    vTaskDelay(pdMS_TO_TICKS(50));

    // active mode
    if (!writeRegister(_dev_mag, REG_MAG_PWR_CNTL1, 0x01)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // normal mode, ODR 10Hz
    if (!writeRegister(_dev_mag, REG_MAG_PWR_CNTL2, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // enable axes
    if (!writeRegister(_dev_mag, REG_MAG_INT_EN2, 0x84)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // repetitions XY = 9 (regular preset)
    if (!writeRegister(_dev_mag, REG_MAG_REP_XY, 0x04)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    // repetitions Z = 15 (regular preset)
    if (!writeRegister(_dev_mag, REG_MAG_REP_Z, 0x0E)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

// ============================================================
// データ読み取り
// ============================================================
bool BMX055::readAccel(float &ax, float &ay, float &az) {
    if (!_acc_ok || !_dev_acc) return false;
    uint8_t data[6];
    if (!readRegisters(_dev_acc, REG_ACC_DATA_X_LSB, data, 6)) return false;

    int16_t raw_x = ((int16_t)data[1] << 4) | (data[0] >> 4);
    if (raw_x > 2047) raw_x -= 4096;
    int16_t raw_y = ((int16_t)data[3] << 4) | (data[2] >> 4);
    if (raw_y > 2047) raw_y -= 4096;
    int16_t raw_z = ((int16_t)data[5] << 4) | (data[4] >> 4);
    if (raw_z > 2047) raw_z -= 4096;

    _raw.ax = raw_x; _raw.ay = raw_y; _raw.az = raw_z;

    ax = _data.ax = (float)raw_x * ACCEL_SCALE_2G;
    ay = _data.ay = (float)raw_y * ACCEL_SCALE_2G;
    az = _data.az = (float)raw_z * ACCEL_SCALE_2G;
    return true;
}

bool BMX055::readGyro(float &gx, float &gy, float &gz, bool in_rad) {
    if (!_gyro_ok || !_dev_gyro) return false;
    uint8_t data[6];
    if (!readRegisters(_dev_gyro, REG_GYRO_RATE_X_LSB, data, 6)) return false;

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    _raw.gx = raw_x; _raw.gy = raw_y; _raw.gz = raw_z;

    _data.gx_dps = (float)raw_x * GYRO_SCALE_2000DPS;
    _data.gy_dps = (float)raw_y * GYRO_SCALE_2000DPS;
    _data.gz_dps = (float)raw_z * GYRO_SCALE_2000DPS;
    _data.gx = _data.gx_dps * DEG_TO_RAD;
    _data.gy = _data.gy_dps * DEG_TO_RAD;
    _data.gz = _data.gz_dps * DEG_TO_RAD;

    if (in_rad) { gx = _data.gx; gy = _data.gy; gz = _data.gz; }
    else        { gx = _data.gx_dps; gy = _data.gy_dps; gz = _data.gz_dps; }
    return true;
}

bool BMX055::readMag(float &mx, float &my, float &mz) {
    if (!_mag_ok || !_dev_mag) return false;
    uint8_t data[6];
    if (!readRegisters(_dev_mag, REG_MAG_DATA_X_LSB, data, 6)) return false;

    int16_t raw_x = ((int16_t)data[1] << 8) | data[0]; raw_x >>= 3;
    int16_t raw_y = ((int16_t)data[3] << 8) | data[2]; raw_y >>= 3;
    int16_t raw_z = ((int16_t)data[5] << 8) | data[4]; raw_z >>= 1;

    _raw.mx = raw_x; _raw.my = raw_y; _raw.mz = raw_z;

    mx = _data.mx = (float)raw_x * MAG_SCALE_UT;
    my = _data.my = (float)raw_y * MAG_SCALE_UT;
    mz = _data.mz = (float)raw_z * MAG_SCALE_UT;
    return true;
}

bool BMX055::update(BMX055Data &data) {
    bool ok_a = readAccel(data.ax, data.ay, data.az);
    bool ok_g = readGyro(data.gx, data.gy, data.gz);
    bool ok_m = readMag(data.mx, data.my, data.mz);

    data.gx_dps = _data.gx_dps;
    data.gy_dps = _data.gy_dps;
    data.gz_dps = _data.gz_dps;

    return ok_a || ok_g || ok_m;
}

bool BMX055::readRaw(BMX055RawData &raw) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);
    readMag(mx, my, mz);
    raw = _raw;
    return true;
}

// ============================================================
// 低レベルI2C操作 (esp-idf i2c_master)
// ============================================================

/**
 * @brief レジスタに1バイト書き込む
 *        i2c_master_transmit でアドレス+データを1トランザクションで送信する。
 */
bool BMX055::writeRegister(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dev, buf, 2, _timeout_ms) == ESP_OK;
}

/**
 * @brief 連続レジスタを len バイト読み取る
 *        i2c_master_transmit_receive でレジスタアドレス書き込み→リード
 *        を1トランザクションで行う（リピートスタート）。
 *        Wire の endTransmission + requestFrom とは異なり、
 *        途中でタイムアウトしても他のタスクをブロックしない。
 */
bool BMX055::readRegisters(i2c_master_dev_handle_t dev, uint8_t reg,
                            uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len,
                                       _timeout_ms) == ESP_OK;
}
