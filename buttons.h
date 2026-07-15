/**
 * @file    buttons.h
 * @brief   Botão digital com debounce não bloqueante (baseado em millis()).
 *
 * Cada botão físico (START, +, -) é representado por uma instância desta
 * classe. A lógica de debounce é encapsulada aqui, mantendo main.cpp livre
 * de temporizadores soltos ou variáveis globais de estado por botão.
 */

#pragma once

#include <Arduino.h>

class Button {
public:
    /// @param pin        Pino digital do botão (deve usar fiação para GND).
    /// @param debounceMs Janela de estabilização do sinal, em milissegundos.
    explicit Button(uint8_t pin, uint32_t debounceMs);

    /// Configura o pino como INPUT_PULLUP. Deve ser chamado em setupGPIO().
    void begin();

    /// Deve ser chamado a cada iteração de loop(). Atualiza o estado interno
    /// de debounce. Retorna 'true' exatamente uma vez por evento de borda de
    /// descida (pressionamento), nunca em nível contínuo.
    bool wasPressed();

    /// Estado estável atual (true = pressionado), já filtrado pelo debounce.
    bool isPressed() const { return stableState_; }

private:
    uint8_t  pin_;
    uint32_t debounceMs_;

    bool     lastReading_;
    bool     stableState_;
    uint32_t lastChangeMs_;
    bool     edgeReported_;
};
