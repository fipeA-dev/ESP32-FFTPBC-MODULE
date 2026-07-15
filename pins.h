/**
 * @file    pins.h
 * @brief   Mapeamento centralizado de todos os pinos físicos do hardware.
 *
 * Este arquivo é a ÚNICA fonte de verdade sobre a pinagem da PCB.
 * Qualquer alteração de hardware (revisão de placa) deve ser refletida
 * somente aqui — nenhum outro arquivo deve conter números de pino "soltos".
 *
 * Hardware alvo: ESP32 DevKit V1 (30 pinos, ESP-WROOM-32)
 */

#pragma once

#include <cstdint>

namespace Pins {

// ---------------------------------------------------------------------------
// Barramento I2C de Hardware 0 — MPU1 (0x68) e MPU2 (0x69)
// ---------------------------------------------------------------------------
constexpr uint8_t I2C0_SDA = 21;
constexpr uint8_t I2C0_SCL = 22;

// ---------------------------------------------------------------------------
// Barramento I2C de Hardware 1 — MPU3 (0x68) e MPU4 (0x69)
// ---------------------------------------------------------------------------
constexpr uint8_t I2C1_SDA = 32;
constexpr uint8_t I2C1_SCL = 33;

// ---------------------------------------------------------------------------
// Barramento I2C por SOFTWARE — exclusivo do display LCD (PCF8574)
// Fisicamente isolado dos barramentos de hardware acima.
// ---------------------------------------------------------------------------
constexpr uint8_t LCD_SDA = 25;
constexpr uint8_t LCD_SCL = 26;

// ---------------------------------------------------------------------------
// LEDs
// ---------------------------------------------------------------------------
// LED Power é puramente de hardware (alimentação), não requer GPIO.
constexpr uint8_t LED_STATUS = 4;   // Através de resistor de 330 ohm para GND

// ---------------------------------------------------------------------------
// Botões (todos com INPUT_PULLUP — nível LOW = pressionado)
// ---------------------------------------------------------------------------
constexpr uint8_t BTN_START = 23;
constexpr uint8_t BTN_PLUS  = 14;
constexpr uint8_t BTN_MINUS = 27;

} // namespace Pins
