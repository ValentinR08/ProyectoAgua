#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ================== Direcciones I2C ================== */
const uint8_t DEV_ADDRS[]  = { 0x0C, 0x08, 0x09, 0x0A, 0x0B };
const char*   DEV_NAMES[]  = { "Entrada", "Casa 1", "Casa 2", "Casa 3", "Deposito" };
const uint8_t NUM_DEVICES  = sizeof(DEV_ADDRS) / sizeof(DEV_ADDRS[0]);

/* ================== LCD (20x4) ================== */
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Cambia a 0x3F si tu backpack lo requiere
static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;

/* ================== Joystick ================== */
const uint8_t Y_AXIS_PIN = A1;    // eje Y
const uint8_t BTN_PIN    = 6;     // botón (a GND), usar INPUT_PULLUP

// Umbrales eje Y (ajústalos si tu joystick da otros rangos)
const int Y_LOW_THRESH   = 350;
const int Y_HIGH_THRESH  = 700;
const int Y_CENTER_MIN   = 450;
const int Y_CENTER_MAX   = 600;

/* ================== Estado UI ================== */
bool inMenu     = true;
int  menuIndex  = 0;

bool btnPrev    = HIGH;
unsigned long btnDownAt = 0;
const unsigned long LONG_PRESS_MS = 1000;

/* ================== Poll I2C ================== */
unsigned long lastPollMs = 0;
const unsigned long POLL_EVERY_MS = 300;   // 300 ms = suave
const uint8_t REPORT_SIZE = 8;

/* ================== Último reporte ================== */
bool     haveReport   = false;
uint8_t  lastStatus   = 0;       // bit0 = relé
uint16_t lastFlowX100 = 0;       // L/min * 100
uint16_t lastHzX10    = 0;       // Hz * 10
uint16_t lastUltraCm  = 0;       // cm
bool     lastHasUltra = false;

/* ================== Autorepeat navegación ================== */
enum NavState { NAV_CENTER, NAV_UP, NAV_DOWN };
NavState navState = NAV_CENTER;
unsigned long navSince = 0, navLastStep = 0;
const unsigned long NAV_INITIAL_DELAY   = 350;
const unsigned long NAV_REPEAT_INTERVAL = 200;

/* ================== Utiles ================== */
static inline int median3(int a, int b, int c) {
  if (a > b) { int t=a; a=b; b=t; }
  if (b > c) { int t=b; b=c; c=t; }
  if (a > b) { int t=a; a=b; b=t; }
  return b;
}

int readYAxisStable() {
  int a = analogRead(Y_AXIS_PIN);
  delayMicroseconds(150);
  int b = analogRead(Y_AXIS_PIN);
  delayMicroseconds(150);
  int c = analogRead(Y_AXIS_PIN);
  return median3(a,b,c);
}

uint8_t currentAddr()    { return DEV_ADDRS[menuIndex]; }
const char* currentName(){ return DEV_NAMES[menuIndex]; }

/* =============== Dibujo seguro al LCD (sin String) =============== */
char lineCache[LCD_ROWS][LCD_COLS + 1]; // cache por fila

void lcdPrintLine(uint8_t row, const char* src) {
  if (row >= LCD_ROWS) return;
  char buf[LCD_COLS+1];
  uint8_t i=0; 
  for (; i < LCD_COLS && src[i]; ++i) buf[i] = src[i];
  for (; i < LCD_COLS; ++i) buf[i] = ' ';
  buf[LCD_COLS] = '\0';
  if (strncmp(buf, lineCache[row], LCD_COLS) != 0) {
    memcpy(lineCache[row], buf, LCD_COLS+1);
    lcd.setCursor(0, row);
    lcd.print(buf);
  }
}

void lcdClearCache() {
  for (uint8_t r=0;r<LCD_ROWS;++r) {
    for (uint8_t c=0;c<LCD_COLS;++c) lineCache[r][c] = '\0';
    lineCache[r][LCD_COLS] = '\0';
  }
  lcd.clear();
}

/* ================== I2C robusto ================== */
// Devuelve true si pudo leer 8 bytes. Si lee 6 bytes, deja hasUltra=false.
bool readReport(uint8_t addr,
                uint8_t &status,
                uint16_t &flowX100,
                uint16_t &hzX10,
                uint16_t &ultraCm,
                bool &hasUltra)
{
  hasUltra = false;

  // Pequeña separación para que el backpack del LCD no “acapare” el bus
  delayMicroseconds(200);

  // Intento #1: 8 bytes
  {
    Wire.requestFrom((uint8_t)addr, (uint8_t)8, (uint8_t)true);
    uint8_t got = Wire.available();
    if (got == 8) {
      uint8_t b0 = Wire.read(); (void)Wire.read();
      uint8_t fL = Wire.read(), fH = Wire.read();
      uint8_t hL = Wire.read(), hH = Wire.read();
      uint8_t uL = Wire.read(), uH = Wire.read();
      status   = b0;
      flowX100 = (uint16_t)fL | ((uint16_t)fH << 8);
      hzX10    = (uint16_t)hL | ((uint16_t)hH << 8);
      ultraCm  = (uint16_t)uL | ((uint16_t)uH << 8);
      hasUltra = true;
      // Limpieza por si quedaron bytes (no debería)
      while (Wire.available()) (void)Wire.read();
      return true;
    } else {
      // Vaciar lo que haya
      while (Wire.available()) (void)Wire.read();
    }
  }

  // Intento #2: 6 bytes
  {
    Wire.requestFrom((uint8_t)addr, (uint8_t)6, (uint8_t)true);
    uint8_t got = Wire.available();
    if (got == 6) {
      uint8_t b0 = Wire.read(); (void)Wire.read();
      uint8_t fL = Wire.read(), fH = Wire.read();
      uint8_t hL = Wire.read(), hH = Wire.read();
      status   = b0;
      flowX100 = (uint16_t)fL | ((uint16_t)fH << 8);
      hzX10    = (uint16_t)hL | ((uint16_t)hH << 8);
      ultraCm  = 0;
      hasUltra = false;
      while (Wire.available()) (void)Wire.read();
      return true;
    } else {
      while (Wire.available()) (void)Wire.read();
    }
  }

  return false;
}

bool sendRelay(uint8_t addr, bool on) {
  // Separación para no colisionar con el LCD en el bus
  delayMicroseconds(200);

  Wire.beginTransmission(addr);
  Wire.write(on ? 1 : 0);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) {
    Serial.print(F("I2C endTransmission err=")); Serial.print(err);
    Serial.print(F(" addr=0x")); Serial.println(addr, HEX);
    return false;
  }
  return true;
}

/* ================== UI ================== */
void showMenu() {
  lcdClearCache();
  lcdPrintLine(0, "Menu:");
  for (uint8_t i=0; i<3; ++i) {
    uint8_t idx = (menuIndex + i) % NUM_DEVICES;
    char l[21];
    snprintf(l, sizeof(l), "%c%-19s", (i==0 ? '>' : ' '), DEV_NAMES[idx]);
    lcdPrintLine(i+1, l);
  }
}

void showProjectScreen() {
  lcdClearCache();
  char l0[21];
  snprintf(l0, sizeof(l0), "%-12s (0x%02X)", currentName(), currentAddr());
  lcdPrintLine(0, l0);

  if (haveReport) {
    char l1[21];
    if (lastHasUltra) {
      snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u U:%u",
        (lastStatus & 0x01) ? "ON " : "OFF",
        (unsigned)(lastHzX10/10), (unsigned)(lastHzX10%10),
        (unsigned)lastUltraCm);
    } else {
      snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u       ",
        (lastStatus & 0x01) ? "ON " : "OFF",
        (unsigned)(lastHzX10/10), (unsigned)(lastHzX10%10));
    }
    lcdPrintLine(1, l1);

    char l2[21];
    snprintf(l2, sizeof(l2), "F:%2u.%02u L/m        ",
      (unsigned)(lastFlowX100/100), (unsigned)(lastFlowX100%100));
    lcdPrintLine(2, l2);
  } else {
    lcdPrintLine(1, "R:-- Hz:--.- U:---");
    lcdPrintLine(2, "F:--.-- L/m       ");
  }

  lcdPrintLine(3, "Corto:ON/OFF Largo:Menu");
}

/* ================== Setup / Loop ================== */
void setup() {
  // I2C
  Wire.begin();            // UNO como maestro
  Wire.setClock(100000);   // 100kHz: seguro con LCD backpack
  Wire.setTimeout(100);    // 100ms de timeout de bus

  // LCD
  lcd.init();
  lcd.backlight();
  lcdClearCache();

  // Entradas
  pinMode(BTN_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  delay(80);

  showMenu();
}

void loop() {
  int y = readYAxisStable();

  // Botón corto/largo
  bool btnNow = digitalRead(BTN_PIN);
  if (btnPrev == HIGH && btnNow == LOW) { btnDownAt = millis(); }
  if (btnPrev == LOW && btnNow == HIGH) {
    unsigned long held = millis() - btnDownAt;
    if (inMenu) {
      if (held >= 5) { // cualquier clic entra
        inMenu = false;
        haveReport = false;
        lastHasUltra = false;
        showProjectScreen();
        // Hacer un primer intento inmediato
        uint8_t st; uint16_t fx, hz, ucm; bool hu;
        if (readReport(currentAddr(), st, fx, hz, ucm, hu)) {
          haveReport   = true;
          lastStatus   = st;
          lastFlowX100 = fx;
          lastHzX10    = hz;
          lastHasUltra = hu;
          if (hu) lastUltraCm = ucm;
        } else {
          haveReport   = false;
          lastHasUltra = false;
        }
        showProjectScreen();
        lastPollMs = millis();
      }
    } else {
      if (held >= LONG_PRESS_MS) {
        // LARGO: volver al menú (apagando el relé por seguridad)
        sendRelay(currentAddr(), false);
        inMenu = true;
        showMenu();
      } else {
        // CORTO: toggle relé
        bool desiredOn = haveReport ? ((lastStatus & 0x01) == 0) : true;
        if (sendRelay(currentAddr(), desiredOn)) {
          // Actualizar estado local por “optimismo”
          if (desiredOn) lastStatus |= 0x01; else lastStatus &= ~0x01;
          haveReport = true; // ya sabemos algo
          showProjectScreen();
        } else {
          lcdPrintLine(2, "I2C error al enviar ");
        }
      }
    }
  }
  btnPrev = btnNow;

  // Navegación con autorepeat
  if (inMenu) {
    NavState ns = NAV_CENTER;
    if (y < Y_LOW_THRESH) ns = NAV_UP;
    else if (y > Y_HIGH_THRESH) ns = NAV_DOWN;

    unsigned long now = millis();
    if (ns != navState) {
      navState = ns; navSince = now; navLastStep = 0;
      if (navState == NAV_UP || navState == NAV_DOWN) {
        int dir = (navState == NAV_UP) ? +1 : -1; // invierte si lo prefieres
        menuIndex = (menuIndex + (dir > 0 ? 1 : -1) + NUM_DEVICES) % NUM_DEVICES;
        showMenu(); navLastStep = now;
      }
    } else if (navState == NAV_UP || navState == NAV_DOWN) {
      if ((now - navSince) >= NAV_INITIAL_DELAY && (now - navLastStep) >= NAV_REPEAT_INTERVAL) {
        int dir = (navState == NAV_UP) ? +1 : -1;
        menuIndex = (menuIndex + (dir > 0 ? 1 : -1) + NUM_DEVICES) % NUM_DEVICES;
        showMenu(); navLastStep = now;
      }
    }
  } else {
    // En pantalla de proyecto: poll del reporte
    unsigned long now = millis();
    if (now - lastPollMs >= POLL_EVERY_MS) {
      lastPollMs = now;

      uint8_t st; uint16_t fx, hz, ucm; bool hu;
      if (readReport(currentAddr(), st, fx, hz, ucm, hu)) {
        haveReport   = true;
        lastStatus   = st;
        lastFlowX100 = fx;
        lastHzX10    = hz;
        if (hu) lastUltraCm = ucm;
        lastHasUltra = hu;

        // Redibujar filas variables
        char l0[21];
        snprintf(l0, sizeof(l0), "%-12s (0x%02X)", currentName(), currentAddr());
        lcdPrintLine(0, l0);

        char l1[21];
        if (hu) {
          snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u U:%u",
            (st & 0x01) ? "ON " : "OFF",
            (unsigned)(hz/10), (unsigned)(hz%10),
            (unsigned)lastUltraCm);
        } else {
          snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u       ",
            (st & 0x01) ? "ON " : "OFF",
            (unsigned)(hz/10), (unsigned)(hz%10));
        }
        lcdPrintLine(1, l1);

        char l2[21];
        snprintf(l2, sizeof(l2), "F:%2u.%02u L/m        ",
          (unsigned)(fx/100), (unsigned)(fx%100));
        lcdPrintLine(2, l2);

        lcdPrintLine(3, "Corto:ON/OFF Largo:Menu");
      } else {
        haveReport   = false;
        lastHasUltra = false;
        lcdPrintLine(1, "Sin respuesta I2C   ");
        lcdPrintLine(2, "                    ");
        lcdPrintLine(3, "Largo:Menu          ");
        Serial.print(F("No responde 0x")); Serial.println(currentAddr(), HEX);
      }
    }
  }

  delay(8); // pequeña pausa antipolución del bus
}
