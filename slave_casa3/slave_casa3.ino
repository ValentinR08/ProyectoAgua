#include <Wire.h>
#include <Servo.h>

// ================== CONFIG I2C ==================
#define I2C_ADDRESS 0x0A   // <-- CAMBIA para cada esclavo (0x08/0x09/0x0A/0x0B)

// ================== Pines ==================
const uint8_t FLOW_PIN   = 2;   // YF-S201 salida a D2 (INT0)
const uint8_t BTN_PIN    = 4;   // Botón RELÉ (INPUT_PULLUP, activo LOW)
const uint8_t RELAY_PIN  = 5;   // Relé (activo HIGH o según tu módulo)

// Ultrasonido
const uint8_t TRIG_PIN   = 12;  // TRIG
const uint8_t ECHO_PIN   = 11;  // ECHO

// Servo
const uint8_t SERVO_PIN      = 3;  // Señal del servo
const uint8_t SERVO_BTN_PIN  = 9;  // Botón SERVO (INPUT_PULLUP, activo LOW)

// ================== Flujo / Medición ==================
volatile unsigned long pulseCount = 0;  // incrementado en ISR
unsigned long oldTime = 0;

uint16_t lastFlowLminX100 = 0;  // L/min * 100
uint16_t lastHzX10        = 0;  // Hz * 10

// ================== Enclavamiento / Botones ==================
int relayState = LOW;           // arranca apagado
int btnLast = HIGH;
unsigned long btnLastChange = 0;

Servo servoMotor;
bool servoAt90 = false;         // false=0°, true=90°
int servoBtnLast = HIGH;
unsigned long servoBtnLastChange = 0;

// Antirrebote
const unsigned long DEBOUNCE_MS = 50;  // como en tu ejemplo original (50ms)

// ================== Ultrasonido ==================
bool objetoCerca = false;       // true si distancia válida ≤ 3 cm
const float UMBRAL_CM = 3.0;    // umbral para “objeto cerca”

// ================== Prototipos ==================
void onRequestHandler();
void onReceiveHandler(int n);
void flowISR();
void updateFlowOncePerSecond();
void setRelay(int state);

bool detectarObjeto();  // HC‑SR04

void handleServoButton();
void setServoAngle(uint8_t deg);

void setup() {
  Serial.begin(9600);

  // Flujo
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  // Botón + Relé
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(LOW);  // arranca apagado

  // Ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Servo
  pinMode(SERVO_BTN_PIN, INPUT_PULLUP);
  servoMotor.attach(SERVO_PIN);
  setServoAngle(0);   // inicia en 0°

  // I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onRequestHandler);   // enviar reporte (6 bytes)
  Wire.onReceive(onReceiveHandler);   // recibir 0/1 (OFF/ON)

  oldTime = millis();

  Serial.print(F("Esclavo listo 0x"));
  Serial.println(I2C_ADDRESS, HEX);
}

void loop() {
  // 1) Leer ultrasónico y aplicar lógica de seguridad del relé
  objetoCerca = detectarObjeto();

  if (objetoCerca) {
    // Objeto a ≤ 3 cm: forzar relé OFF e ignorar pulsador del relé
    if (relayState != LOW) {
      setRelay(LOW);
      Serial.println(F("[Ultra] Objeto cerca: RELAY OFF"));
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
        if (btnReported == HIGH && btnNow == LOW) { // PRESIÓN
          setRelay(relayState ? LOW : HIGH);
        }
        btnReported = btnNow;
      }
    }
  }

  // 3) Botón SERVO (toggle 0°/90°) — independiente del ultrasónico
  handleServoButton();

  // 4) Medición flujo cada 1s
  updateFlowOncePerSecond();
}

// ================== Medición de flujo ==================
void updateFlowOncePerSecond() {
  if ((millis() - oldTime) >= 1000) {
    noInterrupts();
    unsigned long pulses = pulseCount; // cps
    pulseCount = 0;
    interrupts();

    oldTime = millis();

    // YF-S201: f(Hz) = 7.5 * Q(L/min)  => Q = f / 7.5
    float flowLmin = pulses / 7.5f;                      // L/min
    uint32_t flowX100 = (uint32_t)(flowLmin * 100.0f + 0.5f);
    if (flowX100 > 65535UL) flowX100 = 65535UL;
    lastFlowLminX100 = (uint16_t)flowX100;

    // Frecuencia * 10 (para 1 decimal)
    uint32_t hz10 = pulses * 10UL;
    if (hz10 > 65535UL) hz10 = 65535UL;
    lastHzX10 = (uint16_t)hz10;

    // Debug opcional:
    // Serial.print(F("Flujo: "));
    // Serial.print(lastFlowLminX100 / 100);
    // Serial.print('.');
    // uint8_t d2 = lastFlowLminX100 % 100; if (d2 < 10) Serial.print('0');
    // Serial.print(d2);
    // Serial.print(F(" L/min  |  Hz: "));
    // Serial.print(lastHzX10 / 10);
    // Serial.print('.');
    // Serial.println(lastHzX10 % 10);
  }
}

void flowISR() {
  pulseCount++;
}

// ================== I2C ==================
// Reporte fijo 6 bytes: [status, 0, flowLminX100_L, flowLminX100_H, hz_x10_L, hz_x10_H]
void onRequestHandler() {
  uint8_t status = 0;
  if (relayState) status |= 0x01;

  Wire.write(status);
  Wire.write((uint8_t)0); // reservado
  Wire.write((uint8_t)(lastFlowLminX100 & 0xFF));
  Wire.write((uint8_t)(lastFlowLminX100 >> 8));
  Wire.write((uint8_t)(lastHzX10 & 0xFF));
  Wire.write((uint8_t)(lastHzX10 >> 8));
}

// Recibe 1 byte: 0=OFF, 1=ON
void onReceiveHandler(int n) {
  if (n <= 0) return;
  int cmd = Wire.read();
  if (cmd == 0 || cmd == 1) {
    // Si el maestro manda ON pero hay objeto cerca, el loop lo forzará a OFF
    setRelay(cmd ? HIGH : LOW);
  }
  // descartar bytes extra si los hubiera
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
        // Toggle 0°/90°
        servoAt90 = !servoAt90;
        setServoAngle(servoAt90 ? 90 : 0);
        // Debug opcional:
        // Serial.print(F("Servo -> ")); Serial.println(servoAt90 ? F("90") : F("0"));
      }
      servoBtnReported = now;
    }
  }
}

void setServoAngle(uint8_t deg) {
  if (deg > 180) deg = 180;
  servoMotor.write(deg);
  // sin delays largos para no interferir con I2C
}

// ================== HC-SR04 ==================
bool detectarObjeto() {
  // Pulso de disparo
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Medición con timeout (evita colgarse si no hay eco)
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL); // máx. 30ms ~ 5m
  if (dur == 0) return false; // sin eco, asumimos lejos

  // Distancia en cm  (aprox: 58us por cm, o 0.034 cm/us ida/vuelta -> /2)
  float distancia = (float)dur * 0.034f * 0.5f;

  // Debug opcional:
  // Serial.print("Dist: "); Serial.print(distancia); Serial.println(" cm");

  return (distancia > 0.0f && distancia <= UMBRAL_CM);
}
