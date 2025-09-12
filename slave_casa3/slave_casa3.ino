#include <Wire.h>
#include <Servo.h>
#include <util/atomic.h>

// ================== CONFIG I2C ==================
#define I2C_ADDRESS 0x0A

// ================== Pines ==================
const uint8_t FLOW_PIN   = 2;
const uint8_t BTN_PIN    = 4;
const uint8_t RELAY_PIN  = 5;
const uint8_t TRIG_PIN   = 12;
const uint8_t ECHO_PIN   = 11;
const uint8_t SERVO_PIN  = 3;
const uint8_t SERVO_BTN_PIN = 9;

// ================== Flujo / Medición ==================
volatile unsigned long pulseCount = 0;
unsigned long oldTime = 0;
uint16_t lastFlowLminX100 = 0;
uint16_t lastHzX10 = 0;

// ================== Enclavamiento / Botones ==================
int relayState = LOW;
int btnLast = HIGH;
unsigned long btnLastChange = 0;

Servo servoMotor;
bool servoAt90 = false;
int servoBtnLast = HIGH;
unsigned long servoBtnLastChange = 0;
const unsigned long DEBOUNCE_MS = 50;

// ================== Ultrasonido ==================
bool objetoCerca = false;
const float UMBRAL_CM = 6.0;
volatile uint16_t lastUltraCmES = 0xFFFF;
unsigned long lastUltraAt = 0;
const unsigned long ULTRA_EVERY_MS = 1000;

// ================== Prototipos ==================
void onRequestHandler();
void onReceiveHandler(int n);
void flowISR();
void updateFlowOncePerSecond();
void setRelay(int state);
bool detectarObjeto();
void handleServoButton();
void setServoAngle(uint8_t deg);
uint16_t medirUltraCm();

void setup() {
  // Flujo
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  // Botón + Relé
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(LOW);

  // Ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Servo
  pinMode(SERVO_BTN_PIN, INPUT_PULLUP);
  servoMotor.attach(SERVO_PIN);
  setServoAngle(0);

  // I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onRequestHandler);
  Wire.onReceive(onReceiveHandler);

  oldTime = millis();
}

void loop() {
  // 1) Seguridad del relé basada en la última medición disponible
  objetoCerca = detectarObjeto();
  
  if (objetoCerca) {
    if (relayState != LOW) {
      setRelay(LOW);
    }
  } else {
    // 2) Botón RELÉ con enclavamiento (solo si NO hay objeto cerca)
    int btnNow = digitalRead(BTN_PIN);
    if (btnNow != btnLast) {
      btnLastChange = millis();
      btnLast = btnNow;
    }
    if ((millis() - btnLastChange) > DEBOUNCE_MS) {
      static int btnReported = HIGH;
      if (btnNow != btnReported) {
        if (btnReported == HIGH && btnNow == LOW) {
          setRelay(relayState ? LOW : HIGH);
        }
        btnReported = btnNow;
      }
    }
  }

  // 3) Botón SERVO
  handleServoButton();

  // 4) Medición flujo cada 1s
  updateFlowOncePerSecond();

  // 5) Refrescar el valor para el reporte (cada ~1s)
  if (millis() - lastUltraAt >= ULTRA_EVERY_MS) {
    lastUltraAt = millis();
    uint16_t tmp = medirUltraCm();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
      lastUltraCmES = tmp; 
    }
  }
}

// ================== Medición de flujo ==================
void updateFlowOncePerSecond() {
  if ((millis() - oldTime) >= 1000) {
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    oldTime = millis();

    float flowLmin = pulses / 7.5f;
    uint32_t flowX100 = (uint32_t)(flowLmin * 100.0f + 0.5f);
    if (flowX100 > 65535UL) flowX100 = 65535UL;
    lastFlowLminX100 = (uint16_t)flowX100;

    uint32_t hz10 = pulses * 10UL;
    if (hz10 > 65535UL) hz10 = 65535UL;
    lastHzX10 = (uint16_t)hz10;
  }
}

void flowISR() { pulseCount++; }

// ================== I2C ==================
void onRequestHandler() {
  uint8_t status = 0;
  if (relayState) status |= 0x01;

  uint8_t response[8] = {
    status, 
    0,
    (uint8_t)(lastFlowLminX100 & 0xFF),
    (uint8_t)(lastFlowLminX100 >> 8),
    (uint8_t)(lastHzX10 & 0xFF),
    (uint8_t)(lastHzX10 >> 8),
    (uint8_t)(lastUltraCmES & 0xFF),
    (uint8_t)(lastUltraCmES >> 8)
  };
  
  Wire.write(response, 8);
}

void onReceiveHandler(int n) {
  if (n <= 0) return;
  int cmd = Wire.read();
  
  if (!detectarObjeto()) {
    if (cmd == 0 || cmd == 1) setRelay(cmd ? HIGH : LOW);
  } else {
    setRelay(LOW);
  }
  
  while (Wire.available()) (void)Wire.read();
}

void setRelay(int state) {
  relayState = (state ? HIGH : LOW);
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
}

// ================== SERVO ==================
void handleServoButton() {
  int now = digitalRead(SERVO_BTN_PIN);
  if (now != servoBtnLast) {
    servoBtnLastChange = millis();
    servoBtnLast = now;
  }
  if ((millis() - servoBtnLastChange) > DEBOUNCE_MS) {
    static int servoBtnReported = HIGH;
    if (now != servoBtnReported) {
      if (servoBtnReported == HIGH && now == LOW) {
        servoAt90 = !servoAt90;
        setServoAngle(servoAt90 ? 90 : 0);
      }
      servoBtnReported = now;
    }
  }
}

void setServoAngle(uint8_t deg) {
  if (deg > 180) deg = 180;
  servoMotor.write(deg);
}

// ================== HC-SR04 ==================
bool detectarObjeto() {
  uint16_t cm;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { cm = lastUltraCmES; }
  if (cm == 0xFFFF) return false;
  return (cm > 0 && cm <= (uint16_t)UMBRAL_CM);
}

uint16_t medirUltraCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 60000UL);
  if (dur == 0) return 0xFFFF;

  float cm = (float)dur * 0.034f * 0.5f;
  if (cm < 2.0f || cm > 400.0f) return 0xFFFF;
  return (uint16_t)(cm + 0.5f);
}