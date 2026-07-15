/**
 * @file    mpu.h
 * @brief   Driver enxuto e autocontido para o sensor MPU9250 (MPU6500 + AK8963).
 *
 * Motivação de projeto:
 *  Este firmware precisa operar QUATRO sensores em DOIS barramentos I2C de
 *  hardware distintos (Wire e Wire1). A maioria das bibliotecas MPU9250/
 *  MPU6050 disponíveis publicamente assume um único objeto global `Wire`,
 *  o que as torna incompatíveis com esta arquitetura multi-barramento.
 *
 *  Por isso foi implementado um driver de registradores mínimo, porém
 *  completo, que aceita qualquer instância de TwoWire (Wire ou Wire1) no
 *  momento da inicialização — sem dependências externas além do core Arduino
 *  e da própria classe TwoWire.
 *
 * O magnetômetro AK8963 (interno ao módulo GY-9250) é acessado através do
 * I2C Master auxiliar embutido no MPU9250 (não pelo modo bypass permanente),
 * o que evita colisão de endereço entre os dois AK8963 de um mesmo barramento
 * (ambos respondem no endereço fixo 0x0C quando expostos ao barramento
 * externo). O bypass é usado apenas de forma transitória, durante a
 * inicialização, sensor a sensor, nunca de forma simultânea.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

/// Identificador lógico de cada sensor no sistema (usado em logs, buffers e mensagens de erro).
enum class MpuId : uint8_t {
    MPU1 = 0,
    MPU2 = 1,
    MPU3 = 2,
    MPU4 = 3
};

/// Amostra completa de um MPU9250: aceleração, giroscópio, magnetômetro e temperatura.
struct MpuSample {
    uint32_t timestampMs   = 0;

    float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f; ///< [g]
    float gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f; ///< [graus/s]
    float magX   = 0.0f, magY   = 0.0f, magZ   = 0.0f; ///< [microTesla]
    float temperatureC = 0.0f;                          ///< [Celsius]
};

/**
 * @class MPU9250
 * @brief Encapsula toda a comunicação de baixo nível com um sensor MPU9250.
 *
 * Cada instância representa fisicamente UM módulo GY-9250 em um endereço e
 * barramento específicos. O firmware principal instancia quatro objetos
 * desta classe (um por sensor).
 */
class MPU9250 {
public:
    /// Associa o driver a um barramento I2C, endereço e identificador lógico.
    /// Não realiza nenhuma comunicação — apenas guarda a configuração.
    void attach(TwoWire &wire, uint8_t address, MpuId id);

    /// Inicializa o sensor: reset, wake-up, configuração de escalas e do
    /// magnetômetro auxiliar. Retorna true somente se todas as etapas forem
    /// confirmadas por leitura de registrador.
    bool begin();

    /// Testa a presença/resposta do dispositivo no barramento (WHO_AM_I).
    bool testConnection();

    /// Realiza a leitura de uma amostra completa. Retorna false em falha de
    /// comunicação (o conteúdo de outSample não deve ser considerado válido).
    bool readSample(MpuSample &outSample);

    MpuId   id()      const { return id_; }
    uint8_t address()  const { return address_; }

private:
    TwoWire *wire_    = nullptr;
    uint8_t  address_ = 0;
    MpuId    id_      = MpuId::MPU1;

    // Fatores de conversão LSB -> unidade física, calculados em begin()
    // a partir das constantes Config::ACCEL_FS_SEL / GYRO_FS_SEL.
    float accelScale_ = 1.0f; ///< g por LSB
    float gyroScale_  = 1.0f; ///< dps por LSB

    // Fatores de ajuste de sensibilidade do magnetômetro (lidos da Fuse ROM
    // do AK8963 durante a inicialização — específicos de cada chip).
    float magAdjX_ = 1.0f, magAdjY_ = 1.0f, magAdjZ_ = 1.0f;

    // --- Acesso a registradores no endereço principal do MPU (address_) ---
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);

    // --- Acesso direto a um endereço arbitrário (usado só durante bypass,
    //     para configurar o AK8963 antes de habilitar o I2C Master interno) ---
    bool writeRegisterAt(uint8_t devAddress, uint8_t reg, uint8_t value);
    bool readRegistersAt(uint8_t devAddress, uint8_t reg, uint8_t *buffer, uint8_t length);

    bool initAccelGyro();
    bool initMagnetometer();

    /// Espera ativa (sem usar delay()) baseada em millis(), usada apenas
    /// durante a sequência de inicialização/reset do chip.
    static void waitMs(uint32_t ms);
};
