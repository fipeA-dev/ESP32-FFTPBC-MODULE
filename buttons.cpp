/**
 * @file buttons.cpp
 * @brief Implementação do debounce não bloqueante de botões (ver buttons.h).
 */

#include "buttons.h"

Button::Button(uint8_t pin, uint32_t debounceMs)
    : pin_(pin),
      debounceMs_(debounceMs),
      lastReading_(false),
      stableState_(false),
      lastChangeMs_(0),
      edgeReported_(true) {}

void Button::begin() {
    pinMode(pin_, INPUT_PULLUP);
    // Leitura inicial para evitar falso evento de borda logo no boot.
    lastReading_  = (digitalRead(pin_) == LOW);
    stableState_  = lastReading_;
    lastChangeMs_ = millis();
    edgeReported_ = true;
}

bool Button::wasPressed() {
    const bool reading = (digitalRead(pin_) == LOW); // ativo em nível baixo (pull-up)

    if (reading != lastReading_) {
        // Nível mudou: reinicia a contagem da janela de debounce.
        lastReading_  = reading;
        lastChangeMs_ = millis();
    }

    if (static_cast<uint32_t>(millis() - lastChangeMs_) >= debounceMs_) {
        // Sinal estável há tempo suficiente: aceita como novo estado real.
        if (stableState_ != reading) {
            stableState_  = reading;
            edgeReported_ = false; // uma nova transição estável ocorreu

            if (stableState_) {
                // Borda de pressionamento detectada e ainda não reportada.
                edgeReported_ = true;
                return true;
            }
        }
    }

    return false;
}
