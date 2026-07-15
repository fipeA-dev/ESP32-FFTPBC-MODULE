/**
 * @file    main.cpp
 * @brief   Firmware de aquisição simultânea de vibrações — ESP32 + 4x MPU9250.
 *
 * Arquitetura geral:
 *   - pins.h      -> mapeamento físico de pinos (única fonte de verdade)
 *   - config.h    -> parâmetros de comportamento (endereços, tempos, escalas)
 *   - mpu.h/.cpp  -> driver do sensor MPU9250, independente de barramento
 *   - display.h/.cpp -> driver do LCD 16x2 via PCF8574 em I2C por software
 *   - buttons.h/.cpp -> debounce não bloqueante de botões
 *   - main.cpp    -> máquina de estados e laço principal (este arquivo)
 *
 * Máquina de estados do sistema:
 *
 *   INITIALIZING -> (sucesso) -> READY
 *   INITIALIZING -> (falha)   -> ERROR_STATE
 *   READY        -> (START)   -> ACQUIRING
 *   ACQUIRING    -> (tempo esgotado) -> FINISHED
 *   FINISHED     -> (START)   -> READY
 *
 * Nenhuma chamada a delay() é usada na lógica de estados — toda temporização
 * do laço principal é feita via millis(), conforme exigido pela especificação.
 *
 * Pontos de extensão futura (sem necessidade de reescrever a arquitetura):
 *   - FFT:          processar os vetores em AcquisitionBuffers após saveBuffers().
 *   - Cartão SD:     substituir/complementar saveBuffers() por escrita em arquivo.
 *   - Wi-Fi/OTA/Web: podem ser inicializados em setup() e atualizados em loop()
 *                    de forma independente, pois o restante do firmware não usa
 *                    delay() nem bloqueia o laço principal.
 *   - Bluetooth:     mesmo princípio acima (loop cooperativo, não bloqueante).
 */

#include <Arduino.h>
#include <Wire.h>
#include <vector>

#include "pins.h"
#include "config.h"
#include "mpu.h"
#include "display.h"
#include "buttons.h"

// =============================================================================
// Tipos do domínio da aplicação
// =============================================================================

/// Estados possíveis do sistema.
enum class SystemState : uint8_t {
    INITIALIZING,
    READY,
    ACQUIRING,
    FINISHED,
    ERROR_STATE
};

/// Identifica qual dispositivo falhou na checagem inicial (para LCD/Serial).
enum class DeviceError : uint8_t {
    NONE,
    MPU1,
    MPU2,
    MPU3,
    MPU4,
    LCD
};

/// Buffers de aquisição independentes por sensor.
/// Uso de std::vector com reserve() dimensionado ao tempo de aquisição
/// escolhido pelo usuário, evitando alocação fixa desnecessariamente grande.
struct AcquisitionBuffers {
    std::vector<MpuSample> mpu1;
    std::vector<MpuSample> mpu2;
    std::vector<MpuSample> mpu3;
    std::vector<MpuSample> mpu4;

    void reserveFor(uint16_t seconds) {
        const size_t n = static_cast<size_t>(seconds) * Config::SAMPLE_RATE_HZ + 4;
        mpu1.reserve(n);
        mpu2.reserve(n);
        mpu3.reserve(n);
        mpu4.reserve(n);
    }

    void clearAll() {
        mpu1.clear();
        mpu2.clear();
        mpu3.clear();
        mpu4.clear();
    }
};

/**
 * @struct SystemContext
 * @brief  Agrega TODO o estado mutável do firmware em uma única estrutura.
 *
 * Em vez de espalhar variáveis globais soltas pelo arquivo (o que a
 * especificação pede para evitar), o estado do sistema é centralizado aqui.
 * Uma única instância global (g_ctx) referencia este contexto — um padrão
 * comum e aceito em firmware embarcado, pois simplifica a assinatura das
 * funções e deixa explícito tudo que é compartilhado entre elas.
 */
struct SystemContext {
    // --- Periféricos ---
    LcdDisplay lcd{Pins::LCD_SDA, Pins::LCD_SCL, Config::LCD_I2C_ADDR};
    MPU9250 mpus[Config::NUM_MPUS];

    Button btnStart{Pins::BTN_START, Config::BUTTON_DEBOUNCE_MS};
    Button btnPlus{Pins::BTN_PLUS, Config::BUTTON_DEBOUNCE_MS};
    Button btnMinus{Pins::BTN_MINUS, Config::BUTTON_DEBOUNCE_MS};

    // --- Estado da máquina de estados ---
    SystemState state = SystemState::INITIALIZING;
    DeviceError lastError = DeviceError::NONE;

    // --- Parâmetros de aquisição ---
    uint16_t acquisitionTimeS = Config::ACQ_TIME_DEFAULT_S;

    // --- Temporização da aquisição em andamento ---
    uint32_t acquisitionStartMs = 0;
    uint32_t lastSampleMs = 0;

    // --- Temporização de atualização de LCD / LED ---
    uint32_t lastLcdUpdateMs = 0;
    uint32_t lastLedToggleMs = 0;
    bool ledState = false;

    // --- Buffers de dados ---
    AcquisitionBuffers buffers;

    // --- Cache para redesenho parcial do LCD (evita escrita redundante) ---
    SystemState lastRenderedState = SystemState::INITIALIZING;
    int32_t lastRenderedSecondsLeft = -1;
    uint16_t lastRenderedAcqTime = 0;
};

static SystemContext g_ctx;

// =============================================================================
// Protótipos das funções de arquitetura (conforme especificação)
// =============================================================================

void setupGPIO();
void setupLCD();
void setupI2C();
void setupMPUs();
bool checkDevices();
bool readMPU(MPU9250 &mpu, MpuSample &sample);
void readAllSensors();
void startAcquisition();
void updateLCD();
void readButtons();
void blinkStatus();
void saveBuffers();

namespace {
    void enterErrorState(DeviceError error, const char *serialMessage);
    const char *deviceErrorToLcdText(DeviceError error);
}

// =============================================================================
// setup() / loop()
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=== Firmware de Aquisicao de Vibracoes - Boot ==="));

    setupGPIO();
    setupI2C();
    setupLCD();
    setupMPUs();

    if (checkDevices()) {
        g_ctx.state = SystemState::READY;
        Serial.println(F("[BOOT] Todos os dispositivos OK. Sistema pronto."));
    }
    // Em caso de falha, checkDevices() já deixou o sistema em ERROR_STATE
    // e exibiu a mensagem apropriada no LCD/Serial.
}

void loop() {
    readButtons();
    blinkStatus();

    switch (g_ctx.state) {
        case SystemState::READY:
            if (g_ctx.btnStart.wasPressed()) {
                startAcquisition();
            }
            break;

        case SystemState::ACQUIRING: {
            readAllSensors();

            const uint32_t elapsedMs = millis() - g_ctx.acquisitionStartMs;
            if (elapsedMs >= static_cast<uint32_t>(g_ctx.acquisitionTimeS) * 1000UL) {
                digitalWrite(Pins::LED_STATUS, LOW);
                g_ctx.ledState = false;
                saveBuffers();
                g_ctx.state = SystemState::FINISHED;
                Serial.println(F("[ACQ] Aquisicao concluida."));
            }
            break;
        }

        case SystemState::FINISHED:
            if (g_ctx.btnStart.wasPressed()) {
                g_ctx.state = SystemState::READY;
            }
            break;

        case SystemState::ERROR_STATE:
            // Permite nova tentativa de inicialização pressionando START.
            if (g_ctx.btnStart.wasPressed()) {
                Serial.println(F("[ERROR] Nova tentativa de inicializacao solicitada."));
                setupMPUs();
                if (checkDevices()) {
                    g_ctx.state = SystemState::READY;
                }
            }
            break;

        case SystemState::INITIALIZING:
        default:
            // Não deveria persistir aqui após setup(); nada a fazer.
            break;
    }

    updateLCD();
}

// =============================================================================
// Implementação das funções de arquitetura
// =============================================================================

/// Configura todos os pinos digitais (LED, botões) que não são de barramento I2C.
void setupGPIO() {
    pinMode(Pins::LED_STATUS, OUTPUT);
    digitalWrite(Pins::LED_STATUS, LOW);

    g_ctx.btnStart.begin();
    g_ctx.btnPlus.begin();
    g_ctx.btnMinus.begin();
}

/// Inicializa os dois barramentos I2C de hardware (Wire e Wire1).
void setupI2C() {
    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire.setClock(Config::I2C_HARDWARE_CLOCK_HZ);

    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);
    Wire1.setClock(Config::I2C_HARDWARE_CLOCK_HZ);
}

/// Inicializa o LCD no barramento I2C por software (independente dos MPUs).
void setupLCD() {
    if (!g_ctx.lcd.begin()) {
        enterErrorState(DeviceError::LCD, "Falha ao inicializar o LCD (I2C por software).");
        return;
    }
    g_ctx.lcd.clear();
}

/// Associa e inicializa (begin) os quatro sensores MPU9250 em seus respectivos barramentos.
void setupMPUs() {
    g_ctx.mpus[0].attach(Wire,  Config::MPU1_ADDR, MpuId::MPU1);
    g_ctx.mpus[1].attach(Wire,  Config::MPU2_ADDR, MpuId::MPU2);
    g_ctx.mpus[2].attach(Wire1, Config::MPU3_ADDR, MpuId::MPU3);
    g_ctx.mpus[3].attach(Wire1, Config::MPU4_ADDR, MpuId::MPU4);

    for (auto &mpu : g_ctx.mpus) {
        mpu.begin(); // falhas são tratadas de forma centralizada em checkDevices()
    }
}

/// Verifica se todos os dispositivos (4 MPUs + LCD) responderam corretamente.
/// Em caso de falha, já direciona o sistema para ERROR_STATE com a mensagem adequada.
bool checkDevices() {
    // O LCD já foi validado em setupLCD(); se chegou aqui em erro, já está tratado.
    if (g_ctx.state == SystemState::ERROR_STATE) return false;

    struct Check { MPU9250 *mpu; DeviceError error; const char *label; };
    Check checks[Config::NUM_MPUS] = {
        {&g_ctx.mpus[0], DeviceError::MPU1, "MPU1 (0x68, Bus0)"},
        {&g_ctx.mpus[1], DeviceError::MPU2, "MPU2 (0x69, Bus0)"},
        {&g_ctx.mpus[2], DeviceError::MPU3, "MPU3 (0x68, Bus1)"},
        {&g_ctx.mpus[3], DeviceError::MPU4, "MPU4 (0x69, Bus1)"},
    };

    for (auto &check : checks) {
        if (!check.mpu->testConnection()) {
            String msg = String("Falha de comunicacao: ") + check.label;
            enterErrorState(check.error, msg.c_str());
            return false;
        }
    }

    return true;
}

/// Realiza a leitura de um único sensor, encapsulando o tratamento de erro.
bool readMPU(MPU9250 &mpu, MpuSample &sample) {
    return mpu.readSample(sample);
}

/// Lê os quatro sensores simultaneamente (sequencialmente no barramento, mas
/// dentro da mesma janela temporal) e acrescenta as amostras aos buffers.
void readAllSensors() {
    const uint32_t now = millis();
    if (now - g_ctx.lastSampleMs < Config::SAMPLE_PERIOD_MS) {
        return; // ainda não é hora da próxima amostra (não bloqueante)
    }
    g_ctx.lastSampleMs = now;

    MpuSample sample;

    if (readMPU(g_ctx.mpus[0], sample)) g_ctx.buffers.mpu1.push_back(sample);
    if (readMPU(g_ctx.mpus[1], sample)) g_ctx.buffers.mpu2.push_back(sample);
    if (readMPU(g_ctx.mpus[2], sample)) g_ctx.buffers.mpu3.push_back(sample);
    if (readMPU(g_ctx.mpus[3], sample)) g_ctx.buffers.mpu4.push_back(sample);
}

/// Inicia uma nova aquisição: zera buffers, dimensiona memória e liga o LED.
void startAcquisition() {
    g_ctx.buffers.clearAll();
    g_ctx.buffers.reserveFor(g_ctx.acquisitionTimeS);

    g_ctx.acquisitionStartMs = millis();
    g_ctx.lastSampleMs = 0; // força leitura já no primeiro loop

    digitalWrite(Pins::LED_STATUS, HIGH);
    g_ctx.ledState = true;

    g_ctx.state = SystemState::ACQUIRING;

    Serial.print(F("[ACQ] Iniciando aquisicao por "));
    Serial.print(g_ctx.acquisitionTimeS);
    Serial.println(F(" segundos."));
}

/// Atualiza o LCD conforme o estado atual, evitando redesenhos desnecessários.
void updateLCD() {
    const uint32_t now = millis();
    if (now - g_ctx.lastLcdUpdateMs < Config::LCD_UPDATE_INTERVAL_MS) return;
    g_ctx.lastLcdUpdateMs = now;

    switch (g_ctx.state) {
        case SystemState::READY: {
            if (g_ctx.lastRenderedState != g_ctx.state) {
                g_ctx.lcd.printLine(0, "Sistema FFT");
            }
            if (g_ctx.lastRenderedState != g_ctx.state ||
                g_ctx.lastRenderedAcqTime != g_ctx.acquisitionTimeS) {
                String line1 = "Tempo:" + String(g_ctx.acquisitionTimeS) + "s Ready";
                g_ctx.lcd.printLine(1, line1);
                g_ctx.lastRenderedAcqTime = g_ctx.acquisitionTimeS;
            }
            break;
        }

        case SystemState::ACQUIRING: {
            if (g_ctx.lastRenderedState != g_ctx.state) {
                g_ctx.lcd.printLine(0, "Coletando...");
            }
            const uint32_t elapsedS = (millis() - g_ctx.acquisitionStartMs) / 1000UL;
            const int32_t secondsLeft = static_cast<int32_t>(g_ctx.acquisitionTimeS) - static_cast<int32_t>(elapsedS);
            const int32_t clamped = secondsLeft < 0 ? 0 : secondsLeft;

            if (g_ctx.lastRenderedSecondsLeft != clamped) {
                g_ctx.lcd.printLine(1, "Restante:" + String(clamped) + "s");
                g_ctx.lastRenderedSecondsLeft = clamped;
            }
            break;
        }

        case SystemState::FINISHED: {
            if (g_ctx.lastRenderedState != g_ctx.state) {
                g_ctx.lcd.printLine(0, "Aquisicao");
                g_ctx.lcd.printLine(1, "concluida!");
            }
            break;
        }

        case SystemState::ERROR_STATE: {
            if (g_ctx.lastRenderedState != g_ctx.state) {
                g_ctx.lcd.printLine(0, "ERRO HARDWARE");
                g_ctx.lcd.printLine(1, deviceErrorToLcdText(g_ctx.lastError));
            }
            break;
        }

        case SystemState::INITIALIZING:
        default:
            break;
    }

    g_ctx.lastRenderedState = g_ctx.state;
}

/// Lê os botões e aplica a lógica de incremento/decremento do tempo de aquisição.
/// A leitura de START é tratada diretamente na máquina de estados em loop().
void readButtons() {
    // Botões +/- só têm efeito fora de uma aquisição em andamento.
    if (g_ctx.state != SystemState::READY) return;

    if (g_ctx.btnPlus.wasPressed()) {
        if (g_ctx.acquisitionTimeS + Config::ACQ_TIME_STEP_S <= Config::ACQ_TIME_MAX_S) {
            g_ctx.acquisitionTimeS += Config::ACQ_TIME_STEP_S;
        } else {
            g_ctx.acquisitionTimeS = Config::ACQ_TIME_MAX_S;
        }
    }

    if (g_ctx.btnMinus.wasPressed()) {
        if (g_ctx.acquisitionTimeS >= Config::ACQ_TIME_MIN_S + Config::ACQ_TIME_STEP_S) {
            g_ctx.acquisitionTimeS -= Config::ACQ_TIME_STEP_S;
        } else {
            g_ctx.acquisitionTimeS = Config::ACQ_TIME_MIN_S;
        }
    }
}

/// Controla o padrão de piscar do LED de status fora da aquisição:
///  - READY:       pisca lento (heartbeat de "sistema vivo e pronto").
///  - ERROR_STATE: pisca rápido (alerta de falha).
///  - ACQUIRING:   LED permanece SÓLIDO (controlado diretamente em
///                 startAcquisition() / na transição para FINISHED),
///                 esta função não o afeta nesse estado.
void blinkStatus() {
    if (g_ctx.state == SystemState::ACQUIRING) return; // LED fixo aceso

    uint32_t interval = 0;
    switch (g_ctx.state) {
        case SystemState::READY:       interval = Config::LED_BLINK_READY_MS; break;
        case SystemState::ERROR_STATE: interval = Config::LED_BLINK_ERROR_MS; break;
        default:                       digitalWrite(Pins::LED_STATUS, LOW); return;
    }

    const uint32_t now = millis();
    if (now - g_ctx.lastLedToggleMs >= interval) {
        g_ctx.lastLedToggleMs = now;
        g_ctx.ledState = !g_ctx.ledState;
        digitalWrite(Pins::LED_STATUS, g_ctx.ledState ? HIGH : LOW);
    }
}

/// Consolida/persiste os buffers ao final da aquisição.
///
/// TODO (extensão futura): esta função é o ponto único de integração para
/// gravação em cartão SD — basta iterar os quatro vetores de
/// g_ctx.buffers e escrever em arquivo (ex.: CSV por sensor), sem alterar
/// nenhuma outra parte do firmware. Também é o ponto natural para disparar
/// processamento FFT ou publicação via Wi-Fi/Bluetooth.
void saveBuffers() {
    Serial.println(F("[SAVE] Resumo da aquisicao:"));
    Serial.print(F("  MPU1: ")); Serial.print(g_ctx.buffers.mpu1.size()); Serial.println(F(" amostras"));
    Serial.print(F("  MPU2: ")); Serial.print(g_ctx.buffers.mpu2.size()); Serial.println(F(" amostras"));
    Serial.print(F("  MPU3: ")); Serial.print(g_ctx.buffers.mpu3.size()); Serial.println(F(" amostras"));
    Serial.print(F("  MPU4: ")); Serial.print(g_ctx.buffers.mpu4.size()); Serial.println(F(" amostras"));

    // Exemplo de acesso aos dados para depuração/validação (última amostra de cada sensor):
    if (!g_ctx.buffers.mpu1.empty()) {
        const MpuSample &s = g_ctx.buffers.mpu1.back();
        Serial.printf("  Ultima MPU1 -> Ax:%.3f Ay:%.3f Az:%.3f Temp:%.1fC\n",
                      s.accelX, s.accelY, s.accelZ, s.temperatureC);
    }
}

// =============================================================================
// Auxiliares internos (linkage interno, não expostos fora deste arquivo)
// =============================================================================

namespace {

void enterErrorState(DeviceError error, const char *serialMessage) {
    g_ctx.state = SystemState::ERROR_STATE;
    g_ctx.lastError = error;

    Serial.print(F("[ERROR] "));
    Serial.println(serialMessage);

    // Tenta exibir no LCD mesmo que o próprio LCD não seja a causa da falha;
    // se o LCD for a origem do erro, a chamada simplesmente não terá efeito
    // (o driver verifica initialized_ internamente).
    g_ctx.lcd.printLine(0, "ERRO HARDWARE");
    g_ctx.lcd.printLine(1, deviceErrorToLcdText(error));
}

const char *deviceErrorToLcdText(DeviceError error) {
    switch (error) {
        case DeviceError::MPU1: return "Erro MPU1";
        case DeviceError::MPU2: return "Erro MPU2";
        case DeviceError::MPU3: return "Erro MPU3";
        case DeviceError::MPU4: return "Erro MPU4";
        case DeviceError::LCD:  return "Erro LCD";
        case DeviceError::NONE:
        default:                return "";
    }
}

} // namespace
