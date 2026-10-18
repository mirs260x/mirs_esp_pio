#pragma once
#include "OdometryCalculator.hpp"

/**
 * @brief 後方互換性エイリアス。
 * @details DiffDriveの責務（左右エンコーダ反転補正・距離変換等）は
 *          OdometryCalculatorに完全統合されました。
 */
using DiffDrive = OdometryCalculator;
