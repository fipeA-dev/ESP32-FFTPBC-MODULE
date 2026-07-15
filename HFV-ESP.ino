/**
 * ============================================================================
 * Vibration Board V1 - Hardware Validation Firmware
 * ============================================================================
 * Plataforma: ESP32 DevKit V1
 * Framework: Arduino (C++17)
 * Descrição: Firmware de teste e validação de hardware industrial.
 * NÃO possui código bloqueante. NÃO utiliza bibliotecas externas.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// CONSTANTES E CONFIGURAÇÕES DE PINOS
// ============================================================================

// Pinos I2C Hardware 0 (MPU 1 e 2)
constexpr uint8_t I2C0_SDA_PIN = 21;
constexpr uint8_t I2C0_SCL_PIN = 22;
constexpr uint32_t I2C0_FREQ   = 400000;

// Pinos I2C Hardware 1 (MPU 3 e 4)
constexpr uint8_t I2C1_SDA_PIN = 32;
constexpr uint8_t I2C1_SCL_PIN = 33;
constexpr uint32_t I2C1_FREQ   = 400000;

// Pinos Software I2C (LCD)
constexpr uint8_t SOFT_I2C_SDA_PIN = 25;
constexpr uint8_t SOFT_I2C_SCL_PIN = 26;
constexpr uint8_t LCD_I2C_ADDR     = 0x27; 

// Pinos Botões (Ativos em LOW)
constexpr uint8_t BTN1_PIN = 13;
constexpr uint8_t BTN2_PIN = 14;
constexpr uint8_t BTN3_PIN = 27;

// Pino LED
constexpr uint8_t LED_PIN  = 2;

// Endereços I2C MPU9250 e AK8963
constexpr uint8_t MPU_ADDR_LOW  = 0x68;
constexpr uint8_t MPU_ADDR_HIGH = 0x69;
constexpr uint8_t AK8963_ADDR   = 0x0C;

// ============================================================================
// ESTRUTURAS GERAIS E ESTATÍSTICAS
// ============================================================================

struct SystemStats {
    uint32_t readCount;
    uint32_t mpuFailures[4];
    uint32_t i2cTimeouts;
    uint32_t commErrors;
    
    void reset() {
        readCount = 0;
        i2cTimeouts = 0;
        commErrors = 0;
        for (int i = 0; i < 4; i++) mpuFailures[i] = 0;
    }
};

SystemStats sysStats;

// ============================================================================
// FUNÇÕES UTILITÁRIAS
// ============================================================================

void printBanner() {
    Serial.println(F("========================================"));
    Serial.println(F("          Vibration Board V1            "));
    Serial.println(F("         Hardware Validation            "));
    Serial.println(F("========================================"));
}

void printMemory() {
    Serial.println(F("\n--- Diagnostico de Memoria ---"));
    Serial.printf("Heap Livre: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Maior Bloco Livre: %u bytes\n", ESP.getMaxAllocHeap());
    Serial.println(F("------------------------------"));
}

// ============================================================================
// DRIVER SOFTWARE I2C (Bit-Banging) - Sem bibliotecas externas
// ============================================================================

class SoftI2C {
private:
    uint8_t sdaPin;
    uint8_t sclPin;

    void sdaHigh() { pinMode(sdaPin, INPUT_PULLUP); }
    void sdaLow()  { pinMode(sdaPin, OUTPUT); digitalWrite(sdaPin, LOW); }
    void sclHigh() { 
        pinMode(sclPin, INPUT_PULLUP); 
        // Clock stretching
        uint32_t timeout = 1000;
        while(digitalRead(sclPin) == LOW && timeout > 0) { delayMicroseconds(1); timeout--; }
        if(timeout == 0) sysStats.i2cTimeouts++;
    }
    void sclLow()  { pinMode(sclPin, OUTPUT); digitalWrite(sclPin, LOW); }
    void i2cDelay() { delayMicroseconds(4); }

public:
    SoftI2C(uint8_t sda, uint8_t scl) : sdaPin(sda), sclPin(scl) {}

    void begin() {
        sdaHigh();
        sclHigh();
    }

    void start() {
        sdaLow();
        i2cDelay();
        sclLow();
        i2cDelay();
    }

    void stop() {
        sdaLow();
        i2cDelay();
        sclHigh();
        i2cDelay();
        sdaHigh();
        i2cDelay();
    }

    bool write(uint8_t data) {
        for (int i = 0; i < 8; i++) {
            if ((data & 0x80) != 0) sdaHigh();
            else sdaLow();
            data <<= 1;
            i2cDelay();
            sclHigh();
            i2cDelay();
            sclLow();
        }
        sdaHigh(); // Release for ACK
        i2cDelay();
        sclHigh();
        i2cDelay();
        bool ack = (digitalRead(sdaPin) == LOW);
        sclLow();
        if (!ack) sysStats.commErrors++;
        return ack;
    }

    uint8_t read(bool ack) {
        uint8_t data = 0;
        sdaHigh();
        for (int i = 0; i < 8; i++) {
            data <<= 1;
            i2cDelay();
            sclHigh();
            i2cDelay();
            if (digitalRead(sdaPin)) data |= 1;
            sclLow();
        }
        if (ack) sdaLow();
        else sdaHigh();
        i2cDelay();
        sclHigh();
        i2cDelay();
        sclLow();
        sdaHigh();
        return data;
    }
};

SoftI2C lcdWire(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN);

// ============================================================================
// DRIVER LCD 16x2 I2C (Implementação Própria via SoftI2C)
// ============================================================================

class LCD16x2 {
private:
    uint8_t address;
    uint8_t backlightVal;
    
    void expanderWrite(uint8_t _data) {
        lcdWire.start();
        lcdWire.write(address << 1);
        lcdWire.write(_data | backlightVal);
        lcdWire.stop();
    }
    
    void pulseEnable(uint8_t _data) {
        expanderWrite(_data | 0x04); // EN HIGH
        delayMicroseconds(1);
        expanderWrite(_data & ~0x04); // EN LOW
        delayMicroseconds(50);
    }
    
    void write4bits(uint8_t value) {
        expanderWrite(value);
        pulseEnable(value);
    }
    
    void send(uint8_t value, uint8_t mode) {
        uint8_t highnib = value & 0xF0;
        uint8_t lownib  = (value << 4) & 0xF0;
        write4bits(highnib | mode);
        write4bits(lownib | mode);
    }

public:
    LCD16x2(uint8_t addr) : address(addr), backlightVal(0x08) {}

    void begin() {
        lcdWire.begin();
        delay(50);
        expanderWrite(backlightVal);
        delay(1000);
        write4bits(0x03 << 4); delayMicroseconds(4500);
        write4bits(0x03 << 4); delayMicroseconds(4500);
        write4bits(0x03 << 4); delayMicroseconds(150);
        write4bits(0x02 << 4); // 4-bit mode
        command(0x28); // Function set
        command(0x0C); // Display ON, Cursor OFF, Blink OFF
        command(0x06); // Entry mode set
        clear();
    }

    void command(uint8_t value) { send(value, 0x00); }
    void write(uint8_t value)   { send(value, 0x01); }

    void clear() {
        command(0x01);
        delayMicroseconds(2000);
    }

    void setCursor(uint8_t col, uint8_t row) {
        int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
        if (row > 1) row = 1;
        command(0x80 | (col + row_offsets[row]));
    }

    void print(const char* str) {
        while (*str) write(*str++);
    }
    
    void backlight() { backlightVal = 0x08; expanderWrite(0); }
    void noBacklight() { backlightVal = 0x00; expanderWrite(0); }
    
    void createChar(uint8_t location, uint8_t charmap[]) {
        location &= 0x7;
        command(0x40 | (location << 3));
        for (int i=0; i<8; i++) write(charmap[i]);
    }
};

LCD16x2 lcd(LCD_I2C_ADDR);

// ============================================================================
// DRIVER MPU9250 + AK8963 (Implementação Própria)
// ============================================================================

struct MPUData {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float temp;
};

class MPU9250 {
private:
    TwoWire* wireBus;
    uint8_t mpuAddr;
    uint8_t id;
    bool active;
    
    float accRes, gyroRes, magRes;
    float magCalibration[3];

    void writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
        wireBus->beginTransmission(addr);
        wireBus->write(reg);
        wireBus->write(data);
        uint8_t error = wireBus->endTransmission();
        if (error) sysStats.commErrors++;
    }

    uint8_t readRegister(uint8_t addr, uint8_t reg) {
        wireBus->beginTransmission(addr);
        wireBus->write(reg);
        if(wireBus->endTransmission(false)) sysStats.commErrors++;
        wireBus->requestFrom((int)addr, (int)1);
        if (wireBus->available()) return wireBus->read();
        sysStats.i2cTimeouts++;
        return 0;
    }

    void readRegisters(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* dest) {
        wireBus->beginTransmission(addr);
        wireBus->write(reg);
        if(wireBus->endTransmission(false)) sysStats.commErrors++;
        uint8_t bytes = wireBus->requestFrom((int)addr, (int)count);
        if (bytes == count) {
            for (int i = 0; i < count; i++) dest[i] = wireBus->read();
        } else {
            sysStats.i2cTimeouts++;
        }
    }

public:
    MPU9250(uint8_t index) : wireBus(nullptr), mpuAddr(0), id(index), active(false), 
                             accRes(8.0/32768.0), gyroRes(1000.0/32768.0), magRes(10.0*4912.0/32760.0) {
        magCalibration[0] = magCalibration[1] = magCalibration[2] = 1.0f;
    }

    bool begin(TwoWire* bus, uint8_t address) {
        wireBus = bus;
        mpuAddr = address;
        
        // Verifica WHO_AM_I MPU
        uint8_t whoami = readRegister(mpuAddr, 0x75);
        if (whoami != 0x71 && whoami != 0x73 && whoami != 0x70) {
            active = false;
            return false;
        }

        // Wake up
        writeRegister(mpuAddr, 0x6B, 0x00); delay(10);
        // Configuração Auto Clock
        writeRegister(mpuAddr, 0x6B, 0x01); delay(10);
        // Config ACCEL (±8g)
        writeRegister(mpuAddr, 0x1C, 0x10);
        // Config GYRO (±1000dps)
        writeRegister(mpuAddr, 0x1B, 0x10);
        // Habilita I2C Bypass para o Magnetômetro
        writeRegister(mpuAddr, 0x37, 0x02); delay(10);
        
        // Configura AK8963
        uint8_t magWhoAmI = readRegister(AK8963_ADDR, 0x00);
        if (magWhoAmI == 0x48) {
            // Power down mag
            writeRegister(AK8963_ADDR, 0x0A, 0x00); delay(10);
            // Fuse ROM mode
            writeRegister(AK8963_ADDR, 0x0A, 0x0F); delay(10);
            uint8_t rawCal[3];
            readRegisters(AK8963_ADDR, 0x10, 3, rawCal);
            magCalibration[0] = (float)(rawCal[0] - 128)/256.0f + 1.0f;
            magCalibration[1] = (float)(rawCal[1] - 128)/256.0f + 1.0f;
            magCalibration[2] = (float)(rawCal[2] - 128)/256.0f + 1.0f;
            // Power down mag
            writeRegister(AK8963_ADDR, 0x0A, 0x00); delay(10);
            // Modo contínuo 2 (100Hz), 16-bit
            writeRegister(AK8963_ADDR, 0x0A, 0x16); delay(10);
        }
        
        active = true;
        return true;
    }

    bool readAll(MPUData& data) {
        if (!active) return false;
        
        uint8_t rawData[14];
        readRegisters(mpuAddr, 0x3B, 14, rawData);
        
        int16_t ax = (rawData[0] << 8) | rawData[1];
        int16_t ay = (rawData[2] << 8) | rawData[3];
        int16_t az = (rawData[4] << 8) | rawData[5];
        int16_t temp = (rawData[6] << 8) | rawData[7];
        int16_t gx = (rawData[8] << 8) | rawData[9];
        int16_t gy = (rawData[10] << 8) | rawData[11];
        int16_t gz = (rawData[12] << 8) | rawData[13];

        data.accX = (float)ax * accRes;
        data.accY = (float)ay * accRes;
        data.accZ = (float)az * accRes;
        data.temp = ((float)temp - 21.0f) / 333.87f + 21.0f;
        data.gyroX = (float)gx * gyroRes;
        data.gyroY = (float)gy * gyroRes;
        data.gyroZ = (float)gz * gyroRes;

        // Leitura do Magnetômetro
        uint8_t st1 = readRegister(AK8963_ADDR, 0x02);
        if (st1 & 0x01) { 
            uint8_t magRaw[7];
            readRegisters(AK8963_ADDR, 0x03, 7, magRaw);
            uint8_t c = magRaw[6]; 
            if (!(c & 0x08)) { // Verifica overflow
                int16_t mx = (magRaw[1] << 8) | magRaw[0];
                int16_t my = (magRaw[3] << 8) | magRaw[2];
                int16_t mz = (magRaw[5] << 8) | magRaw[4];
                data.magX = (float)mx * magRes * magCalibration[0];
                data.magY = (float)my * magRes * magCalibration[1];
                data.magZ = (float)mz * magRes * magCalibration[2];
            }
        }
        return true;
    }

    bool isActive() const { return active; }
    uint8_t getAddress() const { return mpuAddr; }
};

MPU9250 mpu[4] = { MPU9250(0), MPU9250(1), MPU9250(2), MPU9250(3) };

// ============================================================================
// DRIVER BOTÕES (Debounce por Software)
// ============================================================================

class Button {
private:
    uint8_t pin;
    uint32_t pressTime;
    uint32_t lastDebounceTime;
    bool state;
    bool lastState;
    bool processed;
    
    static constexpr uint32_t DEBOUNCE_DELAY = 50;
    static constexpr uint32_t LONG_PRESS_DELAY = 1000;
    static constexpr uint32_t REPEAT_DELAY = 250;

public:
    Button(uint8_t p) : pin(p), pressTime(0), lastDebounceTime(0), 
                        state(HIGH), lastState(HIGH), processed(false) {}

    void begin() { pinMode(pin, INPUT_PULLUP); }

    void update() {
        bool reading = digitalRead(pin);
        if (reading != lastState) lastDebounceTime = millis();
        
        if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
            if (reading != state) {
                state = reading;
                if (state == LOW) { // Pressed
                    pressTime = millis();
                    processed = false;
                } else { // Released
                    if (!processed && (millis() - pressTime < LONG_PRESS_DELAY)) {
                        Serial.printf("Botao GPIO %d: Clique detectado.\n", pin);
                    }
                }
            } else if (state == LOW) { // Holding
                if (!processed && (millis() - pressTime > LONG_PRESS_DELAY)) {
                    Serial.printf("Botao GPIO %d: Clique LONGO detectado.\n", pin);
                    processed = true;
                    pressTime = millis(); // Reseta para repetição
                } else if (processed && (millis() - pressTime > REPEAT_DELAY)) {
                    Serial.printf("Botao GPIO %d: Repeticao...\n", pin);
                    pressTime = millis();
                }
            }
        }
        lastState = reading;
    }
};

Button btn1(BTN1_PIN);
Button btn2(BTN2_PIN);
Button btn3(BTN3_PIN);

// ============================================================================
// DRIVER LED (Não bloqueante)
// ============================================================================

enum class LedMode { OFF, ON, BLINK_SLOW, BLINK_FAST, FADE };

class StatusLED {
private:
    uint8_t pin;
    LedMode mode;
    uint32_t lastUpdate;
    bool ledState;
    int fadeValue;
    int fadeDirection;

public:
    StatusLED(uint8_t p) : pin(p), mode(LedMode::OFF), lastUpdate(0), ledState(false), fadeValue(0), fadeDirection(5) {}

    void begin() {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    void setMode(LedMode newMode) {
        mode = newMode;
        if(mode == LedMode::OFF) { ledState = false; digitalWrite(pin, LOW); }
        if(mode == LedMode::ON)  { ledState = true; digitalWrite(pin, HIGH); }
        fadeValue = 0;
        fadeDirection = 5;
    }

    void update() {
        uint32_t now = millis();
        if (mode == LedMode::BLINK_SLOW) {
            if (now - lastUpdate >= 500) {
                lastUpdate = now;
                ledState = !ledState;
                digitalWrite(pin, ledState ? HIGH : LOW);
            }
        } else if (mode == LedMode::BLINK_FAST) {
            if (now - lastUpdate >= 100) {
                lastUpdate = now;
                ledState = !ledState;
                digitalWrite(pin, ledState ? HIGH : LOW);
            }
        } else if (mode == LedMode::FADE) {
            if (now - lastUpdate >= 30) {
                lastUpdate = now;
                fadeValue += fadeDirection;
                if (fadeValue <= 0 || fadeValue >= 255) fadeDirection = -fadeDirection;
                // Usando analogWrite (suportado na maioria dos cores ESP32 Arduino modernos)
                analogWrite(pin, fadeValue);
            }
        }
    }
};

StatusLED statusLed(LED_PIN);

// ============================================================================
// FUNÇÕES DE SISTEMA E TESTES
// ============================================================================

void scanBus(const char* busName, SoftI2C* softBus, TwoWire* hardBus) {
    Serial.printf("\nIniciando scanner em: %s\n", busName);
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        bool ack = false;
        if (softBus != nullptr) {
            softBus->start();
            ack = softBus->write(i << 1);
            softBus->stop();
        } else if (hardBus != nullptr) {
            hardBus->beginTransmission(i);
            ack = (hardBus->endTransmission() == 0);
        }
        if (ack) {
            Serial.printf("Dispositivo encontrado no endereco 0x%02X\n", i);
            count++;
        }
    }
    if (count == 0) Serial.println(F("Nenhum dispositivo encontrado."));
    else Serial.printf("Total de dispositivos: %d\n", count);
}

void testI2CScanner() {
    scanBus("Hardware I2C 0 (MPU 1 e 2)", nullptr, &Wire);
    scanBus("Hardware I2C 1 (MPU 3 e 4)", nullptr, &Wire1);
    scanBus("Software I2C (LCD)", &lcdWire, nullptr);
    Serial.println(F("\nValidacao do Barramento concluida -> STATUS: OK"));
}

void testMPU() {
    Serial.println(F("\n--- Teste Individual de Leitura MPU9250 ---"));
    MPUData data;
    for (int i = 0; i < 4; i++) {
        if (mpu[i].isActive()) {
            if (mpu[i].readAll(data)) {
                Serial.printf("MPU %d (0x%02X) -> OK\n", i+1, mpu[i].getAddress());
                Serial.printf("  Accel (g): X=%.2f Y=%.2f Z=%.2f\n", data.accX, data.accY, data.accZ);
                Serial.printf("  Gyro (dps): X=%.2f Y=%.2f Z=%.2f\n", data.gyroX, data.gyroY, data.gyroZ);
                Serial.printf("  Mag (uT): X=%.2f Y=%.2f Z=%.2f\n", data.magX, data.magY, data.magZ);
                Serial.printf("  Temp (C): %.2f\n", data.temp);
            } else {
                Serial.printf("MPU %d -> ERRO NA LEITURA\n", i+1);
            }
        } else {
            Serial.printf("MPU %d -> INATIVO/DESCONECTADO\n", i+1);
        }
    }
}

void testLCD() {
    Serial.println(F("\n--- Teste do LCD ---"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hardware Valido!");
    lcd.setCursor(0, 1);
    lcd.print("ESP32 Test OK");
    Serial.println(F("Texto enviado ao LCD. Verifique visualmente."));
    delay(2000);
    
    lcd.noBacklight();
    Serial.println(F("Backlight OFF."));
    delay(1000);
    lcd.backlight();
    Serial.println(F("Backlight ON."));
    
    // Teste de Caractere Customizado (Bateria cheia)
    uint8_t bat[8] = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
    lcd.createChar(0, bat);
    lcd.setCursor(15, 0);
    lcd.write(0);
    Serial.println(F("Caractere customizado enviado."));
}

void testLED() {
    Serial.println(F("\n--- Teste de LED ---"));
    Serial.println(F("LED ON")); statusLed.setMode(LedMode::ON); delay(1000);
    Serial.println(F("LED OFF")); statusLed.setMode(LedMode::OFF); delay(1000);
    Serial.println(F("LED BLINK LENTO (Deixando rodar por 3s)")); statusLed.setMode(LedMode::BLINK_SLOW);
    uint32_t t = millis(); while(millis() - t < 3000) statusLed.update();
    Serial.println(F("LED BLINK RAPIDO (Deixando rodar por 3s)")); statusLed.setMode(LedMode::BLINK_FAST);
    t = millis(); while(millis() - t < 3000) statusLed.update();
    Serial.println(F("LED FADE PWM (Deixando rodar por 4s)")); statusLed.setMode(LedMode::FADE);
    t = millis(); while(millis() - t < 4000) statusLed.update();
    statusLed.setMode(LedMode::OFF);
    Serial.println(F("Teste de LED concluido."));
}

void testButtons() {
    Serial.println(F("\n--- Teste de Botoes ---"));
    Serial.println(F("Pressione os botoes. Aguardando 10 segundos..."));
    uint32_t start = millis();
    while (millis() - start < 10000) {
        btn1.update();
        btn2.update();
        btn3.update();
        statusLed.update();
    }
    Serial.println(F("Teste de Botoes encerrado."));
}

void runBenchmark() {
    Serial.println(F("\n--- Benchmark de Leitura MPU9250 ---"));
    Serial.println(F("Executando 1000 leituras de todos os sensores..."));
    
    MPUData dummy;
    uint32_t minTime = 0xFFFFFFFF;
    uint32_t maxTime = 0;
    uint64_t totalTime = 0;
    uint32_t errors = 0;

    for (int i = 0; i < 1000; i++) {
        uint32_t tStart = micros();
        bool success = true;
        for (int m = 0; m < 4; m++) {
            if (mpu[m].isActive()) {
                if (!mpu[m].readAll(dummy)) success = false;
            }
        }
        uint32_t tEnd = micros();
        uint32_t diff = tEnd - tStart;
        
        if (!success) errors++;
        
        if (diff < minTime) minTime = diff;
        if (diff > maxTime) maxTime = diff;
        totalTime += diff;
    }

    Serial.printf("Tempo Minimo para 4 MPUs: %u us\n", minTime);
    Serial.printf("Tempo Maximo para 4 MPUs: %u us\n", maxTime);
    Serial.printf("Tempo Medio: %u us\n", (uint32_t)(totalTime / 1000));
    Serial.printf("Jitter (Max-Min): %u us\n", maxTime - minTime);
    Serial.printf("Falhas de Leitura no Benchmark: %u\n", errors);
}

void showStatistics() {
    Serial.println(F("\n--- Estatisticas Globais do Sistema ---"));
    Serial.printf("Leituras Completas Solicitadas: %u\n", sysStats.readCount);
    for (int i = 0; i < 4; i++) {
        Serial.printf("Falhas MPU %d: %u\n", i+1, sysStats.mpuFailures[i]);
    }
    Serial.printf("Timeouts I2C: %u\n", sysStats.i2cTimeouts);
    Serial.printf("Erros de Comunicacao / NACK: %u\n", sysStats.commErrors);
    printMemory();
}

void runStressTest() {
    Serial.println(F("\n--- Stress Test ---"));
    Serial.println(F("Opcoes de tempo: [A] 1 Min | [B] 5 Min | [C] 30 Min"));
    while (!Serial.available()) {
        btn1.update(); btn2.update(); btn3.update(); statusLed.update();
    }
    char op = toupper(Serial.read());
    uint32_t durationStr = 0;
    if (op == 'A') durationStr = 1;
    else if (op == 'B') durationStr = 5;
    else if (op == 'C') durationStr = 30;
    else { Serial.println(F("Opcao invalida.")); return; }

    Serial.printf("Iniciando Stress Test por %u minuto(s)...\n", durationStr);
    uint32_t startMs = millis();
    uint32_t durationMs = durationStr * 60000;
    
    MPUData data;
    uint32_t loops = 0;
    uint32_t lastPrint = millis();

    statusLed.setMode(LedMode::BLINK_FAST);

    while ((millis() - startMs) < durationMs) {
        sysStats.readCount++;
        for (int i = 0; i < 4; i++) {
            if (mpu[i].isActive()) {
                if (!mpu[i].readAll(data)) {
                    sysStats.mpuFailures[i]++;
                    Serial.printf("! Erro MPU %d detectado em %u ms\n", i+1, millis());
                }
            }
        }
        loops++;
        
        if (millis() - lastPrint > 5000) { // A cada 5s, relata status
            Serial.printf("Stress Test rodando... Tempo restante: %u s | Ciclos: %u\n", 
                          (durationMs - (millis() - startMs)) / 1000, loops);
            lastPrint = millis();
        }
        statusLed.update();
        btn1.update(); btn2.update(); btn3.update();
    }
    statusLed.setMode(LedMode::OFF);
    Serial.println(F("Stress Test Concluido. Verifique as Estatisticas (Menu 7)."));
}

// ============================================================================
// MENU SERIAL
// ============================================================================

void printMenu() {
    Serial.println(F("\n=============================="));
    Serial.println(F(" 1 - Scanner I2C"));
    Serial.println(F(" 2 - Testar MPU9250"));
    Serial.println(F(" 3 - Testar LCD"));
    Serial.println(F(" 4 - Testar LED"));
    Serial.println(F(" 5 - Testar Botoes (10 seg)"));
    Serial.println(F(" 6 - Benchmark"));
    Serial.println(F(" 7 - Estatisticas"));
    Serial.println(F(" 8 - Informacoes de Memoria"));
    Serial.println(F(" 9 - Stress Test"));
    Serial.println(F(" 0 - Reiniciar Estatisticas"));
    Serial.println(F("=============================="));
    Serial.print(F("Selecione uma opcao: "));
}

void processMenu() {
    if (Serial.available()) {
        char cmd = Serial.read();
        // Limpa buffer
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case '1': testI2CScanner(); break;
            case '2': testMPU(); break;
            case '3': testLCD(); break;
            case '4': testLED(); break;
            case '5': testButtons(); break;
            case '6': runBenchmark(); break;
            case '7': showStatistics(); break;
            case '8': printMemory(); break;
            case '9': runStressTest(); break;
            case '0': sysStats.reset(); Serial.println(F("\nEstatisticas resetadas.")); break;
            default: if (cmd != '\r' && cmd != '\n') Serial.println(F("\nComando Invalido.")); break;
        }
        if (cmd >= '0' && cmd <= '9') printMenu();
    }
}

// ============================================================================
// SETUP E LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    printBanner();

    // Init Pinos Gerais
    btn1.begin(); btn2.begin(); btn3.begin();
    statusLed.begin();
    sysStats.reset();

    // Init Barramentos Hardware I2C
    Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, I2C0_FREQ);
    Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, I2C1_FREQ);
    
    // Init Barramento Software e LCD
    lcd.begin();
    lcd.setCursor(0,0);
    lcd.print("Iniciando sist..");
    
    Serial.println(F("\n--- Inicializacao dos Sensores MPU9250 ---"));
    
    // Tenta iniciar os 4 MPUs nas suas respectivas redes e endereços
    if (mpu[0].begin(&Wire, MPU_ADDR_LOW))   Serial.println(F("MPU 1 (I2C 0, 0x68): OK")); else Serial.println(F("MPU 1: FALHA"));
    if (mpu[1].begin(&Wire, MPU_ADDR_HIGH))  Serial.println(F("MPU 2 (I2C 0, 0x69): OK")); else Serial.println(F("MPU 2: FALHA"));
    if (mpu[2].begin(&Wire1, MPU_ADDR_LOW))  Serial.println(F("MPU 3 (I2C 1, 0x68): OK")); else Serial.println(F("MPU 3: FALHA"));
    if (mpu[3].begin(&Wire1, MPU_ADDR_HIGH)) Serial.println(F("MPU 4 (I2C 1, 0x69): OK")); else Serial.println(F("MPU 4: FALHA"));

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Validacao V1");
    lcd.setCursor(0,1);
    lcd.print("Pronto.");
    
    printMenu();
}

void loop() {
    // Mantém estado de drivers em background
    btn1.update();
    btn2.update();
    btn3.update();
    statusLed.update();
    
    // Gerencia menu
    processMenu();
}