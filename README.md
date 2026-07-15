# Firmware de Aquisição de Vibrações — ESP32 + 4x MPU9250

Firmware para aquisição simultânea de dados de 4 sensores MPU9250
em uma PCB baseada em ESP32 DevKit V1, preparado para posterior processamento
FFT.

## Como compilar

Projeto PlatformIO. Com a extensão do VS Code ou a CLI instalada:

```bash
pio run                 # compila
pio run -t upload       # grava no ESP32
pio device monitor       # abre o Serial Monitor (115200 baud)
```

## Estrutura do projeto

```
platformio.ini      -> configuração de build e dependências
include/pins.h       -> ÚNICA fonte de verdade sobre pinagem física
include/config.h     -> parâmetros de comportamento (endereços, tempos, escalas)
include/mpu.h        -> driver do MPU9250 (independente de barramento)
include/display.h    -> driver do LCD 16x2 via PCF8574 em I2C por software
include/buttons.h     -> classe Button com debounce não bloqueante
src/mpu.cpp
src/display.cpp
src/buttons.cpp
src/main.cpp          -> máquina de estados e laço principal
```

## Barramentos I2C

| Barramento          | Pinos          | Dispositivos          |
|---------------------|----------------|------------------------|
| Hardware `Wire`     | SDA 21 / SCL 22 | MPU1 (0x68), MPU2 (0x69) |
| Hardware `Wire1`    | SDA 32 / SCL 33 | MPU3 (0x68), MPU4 (0x69) |
| Software (bit-bang) | SDA 25 / SCL 26 | LCD 16x2 (PCF8574)     |

O LCD usa um barramento **totalmente isolado por software** (biblioteca
`SoftwareWire`)
## Máquina de estados

```
INITIALIZING -> (todos os dispositivos OK) -> READY
INITIALIZING -> (qualquer falha)           -> ERROR_STATE
READY        -> (botão START)              -> ACQUIRING
ACQUIRING    -> (tempo configurado esgotado) -> FINISHED
FINISHED     -> (botão START)              -> READY
ERROR_STATE  -> (botão START, nova tentativa) -> READY | ERROR_STATE
```

Nenhuma parte da lógica de estados usa `delay()`; toda a temporização é
feita comparando `millis()`. As únicas esperas ativas baseadas em
`micros()/millis()` existem exclusivamente na temporização de baixo nível
dos protocolos de hardware (reset do MPU, pulsos do HD44780), como em
qualquer driver de sensor/display — não bloqueiam a máquina de estados.

## Buffers de aquisição

Cada sensor possui seu próprio `std::vector<MpuSample>` em
`AcquisitionBuffers` (dentro de `main.cpp`), dimensionado (`reserve`) no
momento em que a aquisição começa, de acordo com o tempo configurado e a
taxa de amostragem (`Config::SAMPLE_RATE_HZ`, padrão 20 Hz).

> **Nota de engenharia:** o ESP32-WROOM-32 desta placa não possui PSRAM.
> A taxa de amostragem em `config.h` foi escolhida para caber com folga na
> SRAM disponível (~320 KB) mesmo na duração máxima de aquisição (120 s x 4
> sensores). Para taxas mais altas — recomendáveis para FFT de vibração de
> precisão — a extensão natural é fazer *streaming* direto para cartão SD
> em `saveBuffers()`/`readAllSensors()`, sem alterar o restante da
> arquitetura (ver seção abaixo).

## Extensões futuras (arquitetura já preparada)

- **FFT**: processar os `std::vector<MpuSample>` de `AcquisitionBuffers`
  logo após `saveBuffers()` — os dados já estão organizados por sensor e
  ordenados no tempo (`timestampMs`).
- **Cartão SD**: substituir/complementar o corpo de `saveBuffers()` por
  gravação em arquivo (ex.: um CSV por sensor); é o único ponto de
  integração necessário.
- **Wi-Fi / OTA / Interface Web**: podem ser inicializados em `setup()` e
  atualizados de forma cooperativa em `loop()`, pois nenhuma rotina do
  firmware bloqueia com `delay()`.
- **Bluetooth**: mesmo princípio — laço principal não bloqueante permite
  adicionar um serviço BLE sem reestruturar o código existente.

## Wiring resumido

| Sinal        | Pino ESP32 |
|--------------|------------|
| MPU1/2 SDA   | GPIO21     |
| MPU1/2 SCL   | GPIO22     |
| MPU3/4 SDA   | GPIO32     |
| MPU3/4 SCL   | GPIO33     |
| LCD SDA (SW) | GPIO25     |
| LCD SCL (SW) | GPIO26     |
| LED Status   | GPIO4 (via resistor 330 Ω para GND) |
| Botão START  | GPIO23 (para GND, INPUT_PULLUP) |
| Botão +      | GPIO14 (para GND, INPUT_PULLUP) |
| Botão -      | GPIO27 (para GND, INPUT_PULLUP) |

MPU1: AD0→GND (0x68) · MPU2: AD0→3V3 (0x69) · MPU3: AD0→GND (0x68) · MPU4: AD0→3V3 (0x69)


<img src="https://github.com/fipeA-dev/ESP32-FFTPBC-MODULE/blob/main/pcb1.jpg?raw=true" alt="PCB FRONT" width="500">

<img src="https://github.com/fipeA-dev/ESP32-FFTPBC-MODULE/blob/main/pcb2.jpg?raw=true" alt="PCB BACK" width="500">

Os pinos dos MPUs foram removidos, devido a utilização deles em longas distâncias, a imagem mostra somente ponto de referencia daonde irá as outras plaquinhas bases de sensores descritas abaixo

<img src="https://github.com/fipeA-dev/ESP32-FFTPBC-MODULE/blob/main/pcb3.jpg?raw=true" alt="PCB BACK" width="500">
<img src="https://github.com/fipeA-dev/ESP32-FFTPBC-MODULE/blob/main/pcb3.jpg?raw=true" alt="PCB BACK" width="500">
