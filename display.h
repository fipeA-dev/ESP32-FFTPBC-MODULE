/**
 * @file    display.h
 * @brief   Driver de LCD 16x2 (HD44780) via expansor PCF8574, sobre um
 *          barramento I2C implementado inteiramente por SOFTWARE.
 *
 * Motivação de projeto:
 *  A biblioteca "LiquidCrystal_I2C" clássica (Frank de Brabander) trabalha
 *  exclusivamente com o objeto global `Wire`, não aceitando uma instância
 *  alternativa de barramento — o que a torna incompatível com o requisito
 *  de um barramento de software isolado para o LCD. Este arquivo substitui
 *  aquela biblioteca por um driver próprio e enxuto, construído sobre a
 *  biblioteca "SoftwareWire" (compatível com a interface de TwoWire, porém
 *  bit-banged em quaisquer pinos digitais), implementando diretamente o
 *  protocolo HD44780 em modo 4 bits através do PCF8574.
 *
 * Mapeamento de bits do PCF8574 (padrão dos módulos "LCM1602 I2C backpack"):
 *   P0 = RS   P1 = RW   P2 = EN   P3 = Backlight   P4..P7 = D4..D7
 */

#pragma once

#include <Arduino.h>
#include <SoftwareWire.h>

class LcdDisplay {
public:
    /// @param sdaPin,sclPin Pinos digitais usados pelo barramento por software.
    /// @param i2cAddress    Endereço do expansor PCF8574 (ver Config::LCD_I2C_ADDR).
    LcdDisplay(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddress);

    /// Inicializa o barramento por software e a sequência de boot do HD44780.
    /// Retorna false se o dispositivo não responder no barramento (ACK ausente).
    bool begin();

    /// Limpa todo o conteúdo do display.
    void clear();

    /// Posiciona o cursor (coluna 0-15, linha 0-1).
    void setCursor(uint8_t col, uint8_t row);

    /// Escreve uma string a partir da posição atual do cursor.
    void print(const String &text);

    /// Atalho: posiciona o cursor e escreve uma string, preenchendo o
    /// restante da linha com espaços (evita "sujeira" de escritas anteriores).
    void printLine(uint8_t row, const String &text);

    /// Liga/desliga o backlight.
    void setBacklight(bool on);

private:
    SoftwareWire wire_;
    uint8_t address_;
    uint8_t backlightMask_;
    bool initialized_ = false;

    bool expanderWrite(uint8_t data);
    bool pulseEnable(uint8_t data);
    bool writeNibble(uint8_t nibble, bool isData);
    bool writeByteToLcd(uint8_t value, bool isData);
    bool command(uint8_t value);
    bool writeData(uint8_t value);

    /// Espera ativa baseada em micros()/millis() — necessária para respeitar
    /// os tempos mínimos de pulso do protocolo HD44780 (na ordem de
    /// microssegundos/milissegundos). Não é usada na lógica de estados do
    /// firmware, apenas internamente na temporização de baixo nível do
    /// protocolo do display, de forma equivalente ao uso em qualquer driver
    /// de LCD (a própria HD44780 exige esses tempos mínimos por hardware).
    static void waitMicros(uint32_t us);
    static void waitMs(uint32_t ms);
};
