#include <Wire.h>
#include <Servo.h>

/* ================== CONFIG I2C ================== */
#define I2C_ADDRESS 0x08   // <-- Cambia para cada esclavo: 0x08/0x09/0x0A/0x0B/0x0C

/* ================== Pines ================== */
const uint8_t FLOW_PIN   = 2;   // YF-S201 salida a D2 (INT0)
const uint8_t BTN_PIN    = 4;   // Botón relé (INPUT_PULLUP, activo LOW)
const uint8_t RELAY_PIN  = 5;   // Relé

// Ultrasónico (HC-SR04)
const uint8_t US_TRIG_PIN = 12; // TRIG (ajustado como pediste)
const uint8_t US_ECHO_PIN = 11; // ECHO (ajustado como pediste)

// Servo + botón de control
const uint8_t SERVO_PIN      = 3;  // Señal del servo
const uint8_t SERVO_BTN_PIN  = 9;  // Botón para alternar 0°/90°

/* ================== Flujo / Medición ================== */
volatile unsigned long pulseCount = 0;  // incrementado en ISR
unsigned long flowTickMs = 0;

volatile uint16_t lastFlowLminX100 = 0; // L/min * 100 (2 decimales)
volatile uint16_t lastHzX10        = 0; // Hz * 10     (1 decimal)

/* ================== Ultrasónico ================== */
volatile uint16_t lastUltraCm   = 0;    // centímetros (0..65535)
unsigned long lastUltraSampleMs = 0;
const unsigned long ULTRA_EVERY_MS = 120; // refresco rápido

/* ================== Relé: enclavamiento / Botón ================== */
int relayState = LOW;                     // arranca apagado
int btnLast = HIGH;
unsigned long btnLastChange = 0;
const unsigned long DEBOUNCE_MS = 30;

/* ================== Servo: estado y antirrebote ================== */
Servo servoMotor;
bool servoAt90 = false;            // false=0°, true=90°
int  servoBtnLast = HIGH;
int  servoBtnReported = HIGH;
unsigned long servoBtnLastChange = 0;

/* ================== Prototipos ================== */
void onRequestHandler();
void onReceiveHandler(int n);

void flowISR();
void updateFlowOncePerSecond();

void setRelay(int state);
void handleRelayButton();

void updateUltrasonicFast();
uint16_t measureUltraCm();

void handleServoButton();
void setServoAngle(uint8_t deg);

/* ================== Setup ================== */
void setup() {
  Serial.begin(9600);

  // Caudalímetro
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  // Relé + botón
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(LOW);

  // Ultrasónico
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  // Servo + botón
  pinMode(SERVO_BTN_PIN, INPUT_PULLUP);
  servoMotor.attach(SERVO_PIN);
  setServoAngle(0);       // arranca en 0°

  // I2C esclavo
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onRequestHandler); // enviar 8 bytes: status,0,flow,hz,ultra
  Wire.onReceive(onReceiveHandler); // recibir 0/1 (OFF/ON)

  flowTickMs = millis();
  lastUltraSampleMs = millis();

  Serial.print(F("Slave I2C listo en 0x"));
  Serial.println(I2C_ADDRESS, HEX);
}

/* ================== Loop ================== */
void loop() {
  handleRelayButton();        // enclavamiento local del relé
  handleServoButton();        // alterna 0°/90° con el botón en D9
  updateFlowOncePerSecond();  // calcula caudal/Hz cada 1 s
  updateUltrasonicFast();     // muestrea UCM ~cada 120 ms
}

/* ================== Botón RELÉ ================== */
void handleRelayButton() {
  int btnNow = digitalRead(BTN_PIN);
  if (btnNow != btnLast) {
    btnLastChange = millis();
    btnLast = btnNow;
  }
  if ((millis() - btnLastChange) > DEBOUNCE_MS) {
    static int btnReported = HIGH;
    if (btnNow != btnReported) {
      if (btnReported == HIGH && btnNow == LOW) { // PRESIÓN
        setRelay(relayState ? LOW : HIGH);
        Serial.print(F("Relay -> "));
        Serial.println(relayState ? F("ON") : F("OFF"));
      }
      btnReported = btnNow;
    }
  }
}

/* ================== Botón SERVO (toggle 0°/90°) ================== */
void handleServoButton() {
  int now = digitalRead(SERVO_BTN_PIN);
  if (now != servoBtnLast) {
    servoBtnLastChange = millis();
    servoBtnLast = now;
  }
  if ((millis() - servoBtnLastChange) > DEBOUNCE_MS) {
    if (now != servoBtnReported) {
      if (servoBtnReported == HIGH && now == LOW) { // PRESIÓN
        servoAt90 = !servoAt90;
        setServoAngle(servoAt90 ? 90 : 0);
        Serial.print(F("Servo -> "));
        Serial.println(servoAt90 ? F("90°") : F("0°"));
      }
      servoBtnReported = now;
    }
  }
}

void setServoAngle(uint8_t deg) {
  if (deg > 180) deg = 180;
  servoMotor.write(deg);
  // opcional: pequeño delay para que llegue (depende del servo)
  delay(10);
}

/* ================== Medición de flujo ================== */
void updateFlowOncePerSecond() {
  unsigned long now = millis();
  if ((now - flowTickMs) >= 1000) {
    noInterrupts();
    unsigned long pulses = pulseCount; // cuentas en 1 s
    pulseCount = 0;
    interrupts();

    flowTickMs = now;

    // YF-S201: f(Hz) = 7.5 * Q(L/min)  => Q = f / 7.5
    float    flowLmin = pulses / 7.5f;
    uint32_t flowX100 = (uint32_t)(flowLmin * 100.0f + 0.5f);
    if (flowX100 > 65535UL) flowX100 = 65535UL;

    uint32_t hz10 = pulses * 10UL;   // Hz con 1 decimal
    if (hz10 > 65535UL) hz10 = 65535UL;

    noInterrupts();
    lastFlowLminX100 = (uint16_t)flowX100;
    lastHzX10        = (uint16_t)hz10;
    interrupts();

    // Debug opcional
    Serial.print(F("Flujo: "));
    Serial.print(lastFlowLminX100 / 100);
    Serial.print('.');
    uint8_t d2 = lastFlowLminX100 % 100; if (d2 < 10) Serial.print('0'); Serial.print(d2);
    Serial.print(F(" L/min  |  Hz: "));
    Serial.print(lastHzX10 / 10);
    Serial.print('.');
    Serial.println(lastHzX10 % 10);
  }
}

void flowISR() { pulseCount++; }

/* ================== Ultrasónico ================== */
void updateUltrasonicFast() {
  unsigned long now = millis();
  if (now - lastUltraSampleMs >= ULTRA_EVERY_MS) {
    lastUltraSampleMs = now;

    uint16_t cm = measureUltraCm();
    if (cm > 500) cm = 500; // recorte de rango útil para estabilidad

    noInterrupts();
    lastUltraCm = cm;
    interrupts();

    // Debug opcional:
    // Serial.print(F("Ultra: ")); Serial.print(cm); Serial.println(F(" cm"));
  }
}

uint16_t measureUltraCm() {
  // Pulso de disparo
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  // Lectura del eco con timeout ~20 ms (≈3.4 m)
  unsigned long dur = pulseIn(US_ECHO_PIN, HIGH, 20000UL);
  if (dur == 0) {
    // sin eco: conserva la última medición válida
    return lastUltraCm;
  }

  // Conversión a cm (≈58 µs por cm)
  unsigned long cm = dur / 58UL;
  if (cm > 65535UL) cm = 65535UL;
  return (uint16_t)cm;
}

/* ================== I2C ================== */
/*
  Reporte 8 bytes (orden fijo):
  [0] status (bit0=relé ON)
  [1] 0 (reservado)
  [2] flowLminX100 low
  [3] flowLminX100 high
  [4] hz_x10 low
  [5] hz_x10 high
  [6] ultraCm low
  [7] ultraCm high
*/
void onRequestHandler() {
  // Snapshots atómicos
  uint16_t flowSnap, hzSnap, ultraSnap;
  noInterrupts();
  flowSnap  = lastFlowLminX100;
  hzSnap    = lastHzX10;
  ultraSnap = lastUltraCm;
  interrupts();

  uint8_t status = 0;
  if (relayState) status |= 0x01;

  Wire.write(status);                     // [0]
  Wire.write((uint8_t)0);                 // [1]
  Wire.write((uint8_t)(flowSnap & 0xFF)); // [2]
  Wire.write((uint8_t)(flowSnap >> 8));   // [3]
  Wire.write((uint8_t)(hzSnap & 0xFF));   // [4]
  Wire.write((uint8_t)(hzSnap >> 8));     // [5]
  Wire.write((uint8_t)(ultraSnap & 0xFF));// [6]
  Wire.write((uint8_t)(ultraSnap >> 8));  // [7]
}

// Recibe 1 byte: 0=OFF, 1=ON → controla SOLO el relé
void onReceiveHandler(int n) {
  if (n <= 0) return;
  int cmd = Wire.read();
  if (cmd == 0 || cmd == 1) setRelay(cmd ? HIGH : LOW);

  // Descarta bytes extra si los hubiera
  while (Wire.available()) (void)Wire.read();
}

/* ================== Utilidades ================== */
void setRelay(int state) {
  relayState = (state ? HIGH : LOW);
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
}