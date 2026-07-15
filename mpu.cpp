/**
 * @file mpu.cpp
 * @brief Implementação do driver MPU9250 (ver mpu.h para documentação de projeto).
 */

#include "mpu.h"
#include "config.h"

namespace {

// ---------------------------------------------------------------------------
// Mapa de registradores do MPU6500/MPU9250 (núcleo accel+giro)
// ---------------------------------------------------------------------------
constexpr uint8_t REG_SELF_TEST_X_GYRO = 0x00;
constexpr uint8_t REG_CONFIG           = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG      = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG     = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2    = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG      = 0x37;
constexpr uint8_t REG_ACCEL_XOUT_H     = 0x3B;
constexpr uint8_t REG_TEMP_OUT_H       = 0x41;
constexpr uint8_t REG_GYRO_XOUT_H      = 0x43;
constexpr uint8_t REG_EXT_SENS_DATA_00 = 0x49;
constexpr uint8_t REG_I2C_MST_CTRL     = 0x24;
constexpr uint8_t REG_I2C_SLV0_ADDR    = 0x25;
constexpr uint8_t REG_I2C_SLV0_REG     = 0x26;
constexpr uint8_t REG_I2C_SLV0_CTRL    = 0x27;
constexpr uint8_t REG_USER_CTRL        = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1       = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2       = 0x6C;
constexpr uint8_t REG_WHO_AM_I         = 0x75;

// Valores de WHO_AM_I aceitos (variam conforme o silício MPU6500/MPU9250/MPU9255).
constexpr uint8_t WHO_AM_I_VALID[] = {0x71, 0x73, 0x70, 0x68, 0x98};

// ---------------------------------------------------------------------------
// Mapa de registradores do magnetômetro AK8963 (interno ao módulo GY-9250)
// ---------------------------------------------------------------------------
constexpr uint8_t AK8963_ADDR   = 0x0C;
constexpr uint8_t AK8963_WIA    = 0x00; // Device ID -> esperado 0x48
constexpr uint8_t AK8963_HXL    = 0x03;
constexpr uint8_t AK8963_ST2    = 0x09;
constexpr uint8_t AK8963_CNTL1  = 0x0A;
constexpr uint8_t AK8963_ASAX   = 0x10;

constexpr uint8_t AK8963_MODE_POWER_DOWN   = 0x00;
constexpr uint8_t AK8963_MODE_FUSE_ROM     = 0x0F;
constexpr uint8_t AK8963_MODE_CONT2_16BIT  = 0x16; // 100Hz, 16-bit

// Sensibilidade do magnetômetro: faixa fixa de ±4912 uT em 16 bits.
constexpr float MAG_UT_PER_LSB = 4912.0f / 32760.0f;

// Bit de overflow no ST2 do AK8963 (dado inválido se setado).
constexpr uint8_t AK8963_ST2_HOFL = 0x08;

bool isWhoAmIValid(uint8_t value) {
    for (uint8_t expected : WHO_AM_I_VALID) {
        if (value == expected) return true;
    }
    return false;
}

/// Converte o código FS_SEL do acelerômetro em g por LSB (resolução de 16 bits).
float accelScaleFromFsSel(uint8_t fsSel) {
    switch (fsSel) {
        case 0x00: return 2.0f  / 32768.0f;
        case 0x08: return 4.0f  / 32768.0f;
        case 0x10: return 8.0f  / 32768.0f;
        case 0x18: return 16.0f / 32768.0f;
        default:   return 4.0f  / 32768.0f;
    }
}

/// Converte o código FS_SEL do giroscópio em dps por LSB (resolução de 16 bits).
float gyroScaleFromFsSel(uint8_t fsSel) {
    switch (fsSel) {
        case 0x00: return 250.0f  / 32768.0f;
        case 0x08: return 500.0f  / 32768.0f;
        case 0x10: return 1000.0f / 32768.0f;
        case 0x18: return 2000.0f / 32768.0f;
        default:   return 500.0f  / 32768.0f;
    }
}

int16_t combineBytes(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

} // namespace

// =============================================================================
// API pública
// =============================================================================

void MPU9250::attach(TwoWire &wire, uint8_t address, MpuId id) {
    wire_    = &wire;
    address_ = address;
    id_      = id;
}

void MPU9250::waitMs(uint32_t ms) {
    // Espera ativa baseada em millis() — evita a chamada delay(), conforme
    // exigido pela arquitetura do firmware. Usada apenas durante init do chip
    // (não faz parte do laço principal de aquisição).
    const uint32_t start = millis();
    while (static_cast<uint32_t>(millis() - start) < ms) {
        // busy-wait curto e determinístico (poucos milissegundos)
    }
}

bool MPU9250::testConnection() {
    uint8_t whoAmI = 0;
    if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1)) return false;
    return isWhoAmIValid(whoAmI);
}

bool MPU9250::begin() {
    if (wire_ == nullptr) return false;

    if (!initAccelGyro())    return false;
    if (!initMagnetometer()) return false;

    return true;
}

bool MPU9250::readSample(MpuSample &outSample) {
    if (wire_ == nullptr) return false;

    // --- Bloco 1: Accel(6) + Temp(2) + Gyro(6) = 14 bytes contíguos ---
    uint8_t raw[14] = {0};
    if (!readRegisters(REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

    const int16_t rawAx = combineBytes(raw[0],  raw[1]);
    const int16_t rawAy = combineBytes(raw[2],  raw[3]);
    const int16_t rawAz = combineBytes(raw[4],  raw[5]);
    const int16_t rawT  = combineBytes(raw[6],  raw[7]);
    const int16_t rawGx = combineBytes(raw[8],  raw[9]);
    const int16_t rawGy = combineBytes(raw[10], raw[11]);
    const int16_t rawGz = combineBytes(raw[12], raw[13]);

    // --- Bloco 2: dados do magnetômetro, coletados automaticamente pelo
    //     I2C Master interno do MPU e espelhados em EXT_SENS_DATA_00..06 ---
    uint8_t rawMag[7] = {0};
    if (!readRegisters(REG_EXT_SENS_DATA_00, rawMag, sizeof(rawMag))) return false;

    const int16_t rawMx = combineBytes(rawMag[1], rawMag[0]); // AK8963 é little-endian
    const int16_t rawMy = combineBytes(rawMag[3], rawMag[2]);
    const int16_t rawMz = combineBytes(rawMag[5], rawMag[4]);
    const uint8_t st2   = rawMag[6];
    const bool magOverflow = (st2 & AK8963_ST2_HOFL) != 0;

    outSample.timestampMs = millis();

    outSample.accelX = rawAx * accelScale_;
    outSample.accelY = rawAy * accelScale_;
    outSample.accelZ = rawAz * accelScale_;

    outSample.gyroX = rawGx * gyroScale_;
    outSample.gyroY = rawGy * gyroScale_;
    outSample.gyroZ = rawGz * gyroScale_;

    // Fórmula oficial do datasheet MPU-9250: T(°C) = raw / 333.87 + 21
    outSample.temperatureC = (rawT / 333.87f) + 21.0f;

    if (!magOverflow) {
        outSample.magX = rawMx * MAG_UT_PER_LSB * magAdjX_;
        outSample.magY = rawMy * MAG_UT_PER_LSB * magAdjY_;
        outSample.magZ = rawMz * MAG_UT_PER_LSB * magAdjZ_;
    }
    // Em overflow, mantém o último valor válido em outSample (comportamento
    // conservador: preferível a publicar uma leitura conhecida como inválida).

    return true;
}

// =============================================================================
// Comunicação de baixo nível
// =============================================================================

bool MPU9250::writeRegister(uint8_t reg, uint8_t value) {
    return writeRegisterAt(address_, reg, value);
}

bool MPU9250::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    return readRegistersAt(address_, reg, buffer, length);
}

bool MPU9250::writeRegisterAt(uint8_t devAddress, uint8_t reg, uint8_t value) {
    wire_->beginTransmission(devAddress);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool MPU9250::readRegistersAt(uint8_t devAddress, uint8_t reg, uint8_t *buffer, uint8_t length) {
    wire_->beginTransmission(devAddress);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false; // repeated start

    const uint8_t received = wire_->requestFrom(devAddress, length);
    if (received != length) return false;

    for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = wire_->read();
    }
    return true;
}

// =============================================================================
// Inicialização interna
// =============================================================================

bool MPU9250::initAccelGyro() {
    // Reset completo do dispositivo
    if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
    waitMs(100);

    // Sai do reset, seleciona clock PLL (recomendado pelo fabricante)
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
    // Habilita todos os eixos de accel/giro
    if (!writeRegister(REG_PWR_MGMT_2, 0x00)) return false;
    waitMs(10);

    if (!testConnection()) return false;

    // Filtro passa-baixa digital (DLPF) moderado — reduz ruído mantendo
    // banda útil para análise de vibração de baixa/média frequência.
    if (!writeRegister(REG_CONFIG, 0x03)) return false;

    if (!writeRegister(REG_GYRO_CONFIG, Config::GYRO_FS_SEL))   return false;
    if (!writeRegister(REG_ACCEL_CONFIG, Config::ACCEL_FS_SEL)) return false;
    if (!writeRegister(REG_ACCEL_CONFIG2, 0x03))                return false;

    accelScale_ = accelScaleFromFsSel(Config::ACCEL_FS_SEL);
    gyroScale_  = gyroScaleFromFsSel(Config::GYRO_FS_SEL);

    return true;
}

bool MPU9250::initMagnetometer() {
    // --- Etapa 1: habilita bypass TEMPORARIAMENTE para configurar o AK8963
    //     diretamente. Importante: cada sensor é inicializado de forma
    //     estritamente sequencial pelo firmware (ver setupMPUs() em main.cpp),
    //     portanto não há risco de colisão de endereço mesmo que outros
    //     AK8963 do mesmo barramento compartilhem 0x0C. ---
    if (!writeRegister(REG_USER_CTRL, 0x00))   return false; // master I2C desabilitado
    if (!writeRegister(REG_INT_PIN_CFG, 0x02)) return false; // bypass habilitado
    waitMs(10);

    uint8_t whoAmIMag = 0;
    if (!readRegistersAt(AK8963_ADDR, AK8963_WIA, &whoAmIMag, 1)) return false;
    if (whoAmIMag != 0x48) return false;

    // Power-down antes de trocar de modo (exigência do datasheet do AK8963)
    if (!writeRegisterAt(AK8963_ADDR, AK8963_CNTL1, AK8963_MODE_POWER_DOWN)) return false;
    waitMs(10);

    // Acesso à Fuse ROM para ler os fatores de ajuste de sensibilidade (ASA)
    if (!writeRegisterAt(AK8963_ADDR, AK8963_CNTL1, AK8963_MODE_FUSE_ROM)) return false;
    waitMs(10);

    uint8_t asa[3] = {0};
    if (!readRegistersAt(AK8963_ADDR, AK8963_ASAX, asa, 3)) return false;

    // Fórmula do datasheet: adj = ((ASA - 128) * 0.5 / 128) + 1
    magAdjX_ = ((asa[0] - 128) * 0.5f / 128.0f) + 1.0f;
    magAdjY_ = ((asa[1] - 128) * 0.5f / 128.0f) + 1.0f;
    magAdjZ_ = ((asa[2] - 128) * 0.5f / 128.0f) + 1.0f;

    if (!writeRegisterAt(AK8963_ADDR, AK8963_CNTL1, AK8963_MODE_POWER_DOWN)) return false;
    waitMs(10);

    // Modo de medição contínua 2 (100 Hz), saída de 16 bits
    if (!writeRegisterAt(AK8963_ADDR, AK8963_CNTL1, AK8963_MODE_CONT2_16BIT)) return false;
    waitMs(10);

    // --- Etapa 2: desabilita bypass e configura o I2C Master interno do
    //     MPU para ler automaticamente o AK8963 a cada ciclo de amostragem,
    //     publicando o resultado em EXT_SENS_DATA_00..06. A partir daqui o
    //     AK8963 fica isolado do barramento externo (sem risco de colisão). ---
    if (!writeRegister(REG_INT_PIN_CFG, 0x00)) return false; // bypass desabilitado
    if (!writeRegister(REG_USER_CTRL, 0x20))   return false; // I2C_MST_EN = 1
    if (!writeRegister(REG_I2C_MST_CTRL, 0x0D)) return false; // 400 kHz para o master interno

    if (!writeRegister(REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80)) return false; // leitura
    if (!writeRegister(REG_I2C_SLV0_REG, AK8963_HXL))          return false; // início: HXL
    if (!writeRegister(REG_I2C_SLV0_CTRL, 0x87))                return false; // habilita, 7 bytes

    waitMs(10);
    return true;
}
