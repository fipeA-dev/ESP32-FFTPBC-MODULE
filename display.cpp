/**
 * @file display.cpp
 * @brief Implementação do driver de LCD 16x2 via PCF8574 (ver display.h).
 */

#include "display.h"
#include "config.h"

namespace {

// Bits do expansor PCF8574 (mapeamento padrão dos backpacks LCM1602/YwRobot)
constexpr uint8_t BIT_RS = 0x01;
constexpr uint8_t BIT_RW = 0x02;
constexpr uint8_t BIT_EN = 0x04;
constexpr uint8_t BIT_BACKLIGHT = 0x08;
constexpr uint8_t NIBBLE_SHIFT = 4; // D4..D7 ocupam os 4 bits mais significativos

// Comandos HD44780
constexpr uint8_t CMD_CLEAR_DISPLAY   = 0x01;
constexpr uint8_t CMD_ENTRY_MODE_SET  = 0x06; // incrementa cursor, sem shift
constexpr uint8_t CMD_DISPLAY_CONTROL = 0x0C; // display ON, cursor OFF, blink OFF
constexpr uint8_t CMD_FUNCTION_SET_4BIT_2LINE = 0x28;
constexpr uint8_t CMD_SET_DDRAM_ADDR  = 0x80;

// Endereço inicial de cada linha (LCD 16x2 padrão)
constexpr uint8_t ROW_OFFSETS[2] = {0x00, 0x40};

} // namespace

LcdDisplay::LcdDisplay(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddress)
    : wire_(sdaPin, sclPin),
      address_(i2cAddress),
      backlightMask_(BIT_BACKLIGHT) {}

void LcdDisplay::waitMicros(uint32_t us) {
    const uint32_t start = micros();
    while (static_cast<uint32_t>(micros() - start) < us) {
        // busy-wait curto, necessário para o protocolo HD44780
    }
}

void LcdDisplay::waitMs(uint32_t ms) {
    const uint32_t start = millis();
    while (static_cast<uint32_t>(millis() - start) < ms) {
        // busy-wait curto, necessário para o boot do controlador HD44780
    }
}

bool LcdDisplay::expanderWrite(uint8_t data) {
    wire_.beginTransmission(address_);
    wire_.write(data | backlightMask_);
    return wire_.endTransmission() == 0;
}

bool LcdDisplay::pulseEnable(uint8_t data) {
    if (!expanderWrite(data | BIT_EN))  return false;
    waitMicros(1);   // largura mínima do pulso EN (>450ns)
    if (!expanderWrite(data & ~BIT_EN)) return false;
    waitMicros(50);  // tempo mínimo de recuperação do comando
    return true;
}

bool LcdDisplay::writeNibble(uint8_t nibble, bool isData) {
    uint8_t data = (nibble << NIBBLE_SHIFT) & 0xF0;
    if (isData) data |= BIT_RS;
    return pulseEnable(data);
}

bool LcdDisplay::writeByteToLcd(uint8_t value, bool isData) {
    const bool highOk = writeNibble((value >> 4) & 0x0F, isData);
    const bool lowOk  = writeNibble(value & 0x0F, isData);
    return highOk && lowOk;
}

bool LcdDisplay::command(uint8_t value) { return writeByteToLcd(value, false); }
bool LcdDisplay::writeData(uint8_t value) { return writeByteToLcd(value, true); }

bool LcdDisplay::begin() {
    wire_.begin();
    wire_.setClock(Config::I2C_SOFTWARE_CLOCK_HZ);

    // Verifica presença do PCF8574 no barramento antes de iniciar a
    // sequência do HD44780 (evita travar em busy-wait caso o LCD não exista).
    wire_.beginTransmission(address_);
    if (wire_.endTransmission() != 0) {
        initialized_ = false;
        return false;
    }

    waitMs(50); // tempo de estabilização de alimentação do HD44780

    // Sequência de inicialização "força 4 bits" recomendada pelo datasheet
    // (necessária mesmo partindo de um estado desconhecido do controlador).
    writeNibble(0x03, false); waitMs(5);
    writeNibble(0x03, false); waitMicros(150);
    writeNibble(0x03, false); waitMicros(150);
    writeNibble(0x02, false); waitMicros(150); // agora sim, modo 4 bits

    command(CMD_FUNCTION_SET_4BIT_2LINE); waitMicros(50);
    command(CMD_DISPLAY_CONTROL);         waitMicros(50);
    command(CMD_CLEAR_DISPLAY);           waitMs(2);
    command(CMD_ENTRY_MODE_SET);          waitMicros(50);

    initialized_ = true;
    return true;
}

void LcdDisplay::clear() {
    if (!initialized_) return;
    command(CMD_CLEAR_DISPLAY);
    waitMs(2); // CLEAR exige tempo extra de execução no controlador
}

void LcdDisplay::setCursor(uint8_t col, uint8_t row) {
    if (!initialized_ || row > 1) return;
    command(CMD_SET_DDRAM_ADDR | (col + ROW_OFFSETS[row]));
}

void LcdDisplay::print(const String &text) {
    if (!initialized_) return;
    for (size_t i = 0; i < text.length(); ++i) {
        writeData(static_cast<uint8_t>(text[i]));
    }
}

void LcdDisplay::printLine(uint8_t row, const String &text) {
    if (!initialized_ || row > 1) return;

    // Preenche até 16 colunas para sobrescrever completamente a linha
    // anterior, evitando caracteres residuais de mensagens mais longas.
    String padded = text;
    while (padded.length() < 16) padded += ' ';
    if (padded.length() > 16) padded = padded.substring(0, 16);

    setCursor(0, row);
    print(padded);
}

void LcdDisplay::setBacklight(bool on) {
    backlightMask_ = on ? BIT_BACKLIGHT : 0x00;
    expanderWrite(0x00); // reenvia o byte de controle com a nova máscara
}
