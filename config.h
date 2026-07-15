/**
 * @file    config.h
 * @brief   Parâmetros de configuração do sistema (endereços, tempos, escalas).
 *
 * Separado de pins.h propositalmente: pins.h descreve "onde" (fiação física),
 * config.h descreve "como" (parâmetros lógicos/comportamentais).
 */

#pragma once

#include <cstdint>

namespace Config {

// ---------------------------------------------------------------------------
// Endereços I2C dos sensores (definidos pelo nível lógico do pino AD0)
// ---------------------------------------------------------------------------
constexpr uint8_t MPU1_ADDR = 0x68; // AD0 -> GND   (Barramento 0)
constexpr uint8_t MPU2_ADDR = 0x69; // AD0 -> 3V3   (Barramento 0)
constexpr uint8_t MPU3_ADDR = 0x68; // AD0 -> GND   (Barramento 1)
constexpr uint8_t MPU4_ADDR = 0x69; // AD0 -> 3V3   (Barramento 1)

// Endereço do PCF8574 mais comum em módulos LCD I2C (ajustar se necessário;
// backpacks PCF8574A usam tipicamente 0x3F).
constexpr uint8_t LCD_I2C_ADDR = 0x27;

// ---------------------------------------------------------------------------
// Velocidades dos barramentos
// ---------------------------------------------------------------------------
constexpr uint32_t I2C_HARDWARE_CLOCK_HZ = 400000UL; // 400 kHz (Fast Mode)
constexpr uint32_t I2C_SOFTWARE_CLOCK_HZ = 100000UL; // 100 kHz (bit-bang, mais conservador)

// ---------------------------------------------------------------------------
// Tempo de aquisição configurável pelo usuário (botões +/-)
// ---------------------------------------------------------------------------
constexpr uint16_t ACQ_TIME_MIN_S     = 20;
constexpr uint16_t ACQ_TIME_MAX_S     = 120;
constexpr uint16_t ACQ_TIME_STEP_S    = 10;
constexpr uint16_t ACQ_TIME_DEFAULT_S = 20;

// ---------------------------------------------------------------------------
// Taxa de amostragem por sensor.
//
// IMPORTANTE (limite de RAM): o ESP32-WROOM-32 desta placa não possui PSRAM.
// Os buffers de aquisição residem inteiramente em SRAM (~320 KB totais).
// Com 4 sensores, 120 s de aquisição e amostras de ~40 bytes, a taxa aqui
// definida foi escolhida para caber com folga na memória disponível.
//
// TODO (extensão futura): para taxas mais altas (necessárias em FFT de
// vibração de precisão), substituir o armazenamento em RAM por streaming
// direto para cartão SD (ver comentário em mpu.h / main.cpp) sem necessidade
// de alterar a arquitetura geral do firmware.
// ---------------------------------------------------------------------------
constexpr uint16_t SAMPLE_RATE_HZ    = 20;
constexpr uint32_t SAMPLE_PERIOD_MS  = 1000UL / SAMPLE_RATE_HZ;

// ---------------------------------------------------------------------------
// Debounce de botões
// ---------------------------------------------------------------------------
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

// ---------------------------------------------------------------------------
// Atualização de display (evita escrita excessiva no barramento software)
// ---------------------------------------------------------------------------
constexpr uint32_t LCD_UPDATE_INTERVAL_MS = 250;

// ---------------------------------------------------------------------------
// Piscar do LED de status (usado em estados de espera/erro)
// ---------------------------------------------------------------------------
constexpr uint32_t LED_BLINK_READY_MS = 1000; // pisca lento em modo pronto
constexpr uint32_t LED_BLINK_ERROR_MS = 200;  // pisca rápido em erro

// ---------------------------------------------------------------------------
// Quantidade de sensores no sistema
// ---------------------------------------------------------------------------
constexpr uint8_t NUM_MPUS = 4;

// ---------------------------------------------------------------------------
// Escalas de fundo de escala (full-scale) do MPU9250
// ---------------------------------------------------------------------------
// Acelerômetro: 0x00=±2g  0x08=±4g  0x10=±8g  0x18=±16g
constexpr uint8_t ACCEL_FS_SEL = 0x08; // ±4g (adequado para vibração industrial)
// Giroscópio: 0x00=±250dps 0x08=±500dps 0x10=±1000dps 0x18=±2000dps
constexpr uint8_t GYRO_FS_SEL = 0x08;  // ±500 dps

} // namespace Config
