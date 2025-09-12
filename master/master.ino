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

// Umbrales eje Y
const int Y_LOW_THRESH   = 350;
const int Y_HIGH_THRESH  = 700;
const int Y_CENTER_MIN   = 450;
const int Y_CENTER_MAX   = 600;

/* ================== Estado UI ================== */
bool inMenu     = true;
int  menuIndex  = 0;

bool btnPrev    = HIGH;
unsigned long btnDownAt = 0;
const unsigned long LONG_PRESS_MS_PROJECT = 1000; // 1s: volver a menú desde proyecto
const unsigned long LONG_PRESS_MS_MONITOR = 2000; // 2s: modo TODOS (solo desde menú)
const unsigned long DEBOUNCE_MS           = 50;

/* ================== Poll I2C ================== */
unsigned long lastPollMs = 0;
const unsigned long POLL_EVERY_MS = 300;

/* ================== Último reporte (proyecto) ================== */
bool     haveReport   = false;
uint8_t  lastStatus   = 0;       // bit0 = relé
uint16_t lastFlowX100 = 0;       // L/min * 100
uint16_t lastHzX10    = 0;       // Hz * 10
uint16_t lastUltraCm  = 0;       // cm
bool     lastHasUltra = false;
unsigned long lastUltraOkMs = 0;

/* ================== Autorepeat navegación ================== */
enum NavState { NAV_CENTER, NAV_UP, NAV_DOWN };
NavState navState = NAV_CENTER;
unsigned long navSince = 0, navLastStep = 0;
const unsigned long NAV_INITIAL_DELAY   = 350;
const unsigned long NAV_REPEAT_INTERVAL = 200;

/* ================== MODO MONITOREO TODOS ================== */
bool monitorAll = false;
int  monitorPage = 0;                   // 3 dispositivos por página
unsigned long monitorLastPollMs = 0;
unsigned long monitorLastPageMs = 0;
const unsigned long MONITOR_POLL_MS = 400;
const unsigned long MONITOR_PAGE_MS = 2000;

bool     monHaveReport[NUM_DEVICES];
uint8_t  monStatus[NUM_DEVICES];
uint16_t monFlowX100[NUM_DEVICES];
uint16_t monHzX10[NUM_DEVICES];

// Ultrasonido por dispositivo (para seguridad en monitoreo)
bool     monHasUltra[NUM_DEVICES];
uint16_t monUltraCm[NUM_DEVICES];
const uint16_t ULTRA_THRESHOLD_CM = 6;          // umbral de seguridad (cm)
const unsigned long ULTRA_GRACE_MS = 1500;      // tiempo mínimo continuo para cortar relé
unsigned long ultraDangerSince[NUM_DEVICES];    // timestamp de inicio de condición peligrosa

/* =============== Suavizado (EMA) para caudal mostrado =============== */
uint16_t emaFlowX100[NUM_DEVICES];  // un EMA por dispositivo

// EMA con α = 0.3 → (0.7 * prev + 0.3 * now)
static inline uint16_t emaUpdate(uint16_t prev, uint16_t now) {
  // cálculo entero: (7*prev + 3*now)/10 con redondeo
  uint32_t v = (uint32_t)prev * 7 + (uint32_t)now * 3;
  return (uint16_t)((v + 5) / 10);
}

/* =============== Dibujo seguro al LCD (sin String) =============== */
char lineCache[LCD_ROWS][LCD_COLS + 1];

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

/* ================== I2C ================== */
bool readReport(uint8_t addr,
                uint8_t &status,
                uint16_t &flowX100,
                uint16_t &hzX10,
                uint16_t &ultraCm,
                bool &hasUltra)
{
  hasUltra = false;
  ultraCm = 0;
  delayMicroseconds(400);

  // 8 bytes
  {
    Wire.requestFrom((uint8_t)addr, (uint8_t)8);
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
      hasUltra = (ultraCm != 0xFFFF);
      if (!hasUltra) ultraCm = 0;
      while (Wire.available()) (void)Wire.read();
      return true;
    } else {
      while (Wire.available()) (void)Wire.read();
    }
  }

  // 6 bytes
  {
    Wire.requestFrom((uint8_t)addr, (uint8_t)6);
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
  delayMicroseconds(400);
  Wire.beginTransmission(addr);
  Wire.write(on ? 1 : 0);
  uint8_t err = Wire.endTransmission(true);
  return (err == 0);
}

/* ================== Helpers TODOS ================== */
void setAllRelays(bool on) {
  for (uint8_t i=0; i<NUM_DEVICES; ++i) {
    (void)sendRelay(DEV_ADDRS[i], on);
    delay(5);
  }
}

void showMonitorPage(uint8_t page) {
  // Muestra SOLO L/min (suavizado) por dispositivo
  uint8_t start = page * 3;
  uint8_t end   = start + 3;
  if (end > NUM_DEVICES) end = NUM_DEVICES;

  lcdClearCache();

  uint8_t totalPages = (NUM_DEVICES + 3 - 1) / 3;
  char h[21];
  snprintf(h, sizeof(h), "Monitoreo Pg%u/%u", (unsigned)(page+1), (unsigned)totalPages);
  lcdPrintLine(0, h);

  uint8_t row = 1;
  for (uint8_t i = start; i < end && row < LCD_ROWS; ++i, ++row) {
    char l[21];
    if (monHaveReport[i]) {
      // Solo nombre + L/min (EMA)
      snprintf(l, sizeof(l), "%-8s %2u.%02u L/m",
        DEV_NAMES[i],
        (unsigned)(emaFlowX100[i]/100), (unsigned)(emaFlowX100[i]%100));
    } else {
      snprintf(l, sizeof(l), "%-8s --.-- L/m", DEV_NAMES[i]);
    }
    lcdPrintLine(row, l);
  }
  for (; row < LCD_ROWS; ++row) lcdPrintLine(row, "");
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
      snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u CM:%u",
        (lastStatus & 0x01) ? "ON " : "OFF",
        (unsigned)(lastHzX10/10), (unsigned)(lastHzX10%10),
        (unsigned)lastUltraCm);
    } else {
      snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u CM:---",
        (lastStatus & 0x01) ? "ON " : "OFF",
        (unsigned)(lastHzX10/10), (unsigned)(lastHzX10%10));
    }
    lcdPrintLine(1, l1);

    // Mostrar F: con EMA por dispositivo
    emaFlowX100[menuIndex] = emaUpdate(emaFlowX100[menuIndex], lastFlowX100);
    char l2[21];
    snprintf(l2, sizeof(l2), "F:%2u.%02u L/m        ",
      (unsigned)(emaFlowX100[menuIndex]/100), (unsigned)(emaFlowX100[menuIndex]%100));
    lcdPrintLine(2, l2);
  } else {
    lcdPrintLine(1, "R:-- Hz:--.- CM:---");
    lcdPrintLine(2, "F:--.-- L/m       ");
  }

  lcdPrintLine(3, "Corto:ON/OFF Largo:Menu");
}

/* ================== Setup / Loop ================== */
void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(200);

  lcd.init();
  lcd.backlight();

  for (uint8_t r=0; r<LCD_ROWS; ++r) {
    for (uint8_t c=0; c<LCD_COLS; ++c) lineCache[r][c] = '\0';
    lineCache[r][LCD_COLS] = '\0';
  }
  lcdClearCache();

  pinMode(BTN_PIN, INPUT_PULLUP);

  for (uint8_t i=0; i<NUM_DEVICES; ++i) {
    monHaveReport[i] = false;
    monStatus[i] = 0;
    monFlowX100[i] = 0;
    monHzX10[i] = 0;
    monHasUltra[i] = false;
    monUltraCm[i] = 0xFFFF;
    emaFlowX100[i] = 0;
    ultraDangerSince[i] = 0;
  }

  showMenu();
}

void loop() {
  static unsigned long lastDebounceTime = 0;
  static bool consumeRelease = false;   // consumimos el release tras salir de Monitoreo

  int y = readYAxisStable();

  bool btnNow = digitalRead(BTN_PIN);
  unsigned long currentTime = millis();

  if (btnPrev == HIGH && btnNow == LOW) {
    btnDownAt = currentTime;
    lastDebounceTime = currentTime;
  }

  bool releasedWithDebounce = (btnPrev == LOW && btnNow == HIGH && (currentTime - lastDebounceTime) > DEBOUNCE_MS);
  unsigned long held = currentTime - btnDownAt;

  // ====== 2s: entrar a Monitoreo (SOLO DESDE MENÚ) ======
  if (!monitorAll && inMenu && (btnPrev == LOW) && (held >= LONG_PRESS_MS_MONITOR)) {
    monitorAll = true;
    inMenu = false;
    setAllRelays(true);
    monitorPage = 0;
    monitorLastPollMs = 0;
    monitorLastPageMs = currentTime;
    showMonitorPage(monitorPage);
    // consumeRelease se marca al salir
  }

  // ====== Toque corto: salir de Monitoreo ======
  if (monitorAll && releasedWithDebounce && held < LONG_PRESS_MS_MONITOR) {
    setAllRelays(false);
    monitorAll = false;
    inMenu = true;
    showMenu();
    consumeRelease = true; // Consumir este release para que no active nada más
  }

  if (!monitorAll) {
    // ====== Modo normal ======
    if (releasedWithDebounce && !consumeRelease) {
      if (inMenu) {
        if (held >= 50 && held < LONG_PRESS_MS_MONITOR) {
          inMenu = false;
          haveReport   = false;
          lastHasUltra = false;
          lastUltraCm  = 0;
          navState = NAV_CENTER;
          showProjectScreen();
          lastPollMs = currentTime;
        }
      } else {
        // Dentro de proyecto NUNCA entrar a monitoreo (ya está bloqueado arriba)
        if (held >= LONG_PRESS_MS_PROJECT && held < LONG_PRESS_MS_MONITOR) {
          // 1s largo: volver a menú
          sendRelay(currentAddr(), false);
          inMenu = true;
          navState = NAV_CENTER;
          showMenu();
        } else if (held < LONG_PRESS_MS_PROJECT) {
          // Toque corto: toggle del relé del dispositivo actual
          bool desiredOn = haveReport ? ((lastStatus & 0x01) == 0) : true;
          if (sendRelay(currentAddr(), desiredOn)) {
            if (desiredOn) lastStatus |= 0x01; else lastStatus &= ~0x01;
            haveReport = true;
            showProjectScreen();
          } else {
            lcdPrintLine(2, "I2C error al enviar ");
          }
        }
      }
    }

    // Reset del “consumidor”
    if (consumeRelease && btnNow == LOW) consumeRelease = false;

    // Navegación con autorepeat
    if (inMenu) {
      NavState ns = NAV_CENTER;
      if (y < Y_LOW_THRESH) ns = NAV_UP;
      else if (y > Y_HIGH_THRESH) ns = NAV_DOWN;

      unsigned long now = millis();
      if (ns != navState) {
        navState = ns; navSince = now; navLastStep = 0;
        if (navState == NAV_UP || navState == NAV_DOWN) {
          int dir = (navState == NAV_UP) ? +1 : -1;
          menuIndex = (menuIndex + dir + NUM_DEVICES) % NUM_DEVICES;
          showMenu(); navLastStep = now;
        }
      } else if (navState == NAV_UP || navState == NAV_DOWN) {
        if ((now - navSince) >= NAV_INITIAL_DELAY && (now - navLastStep) >= NAV_REPEAT_INTERVAL) {
          int dir = (navState == NAV_UP) ? +1 : -1;
          menuIndex = (menuIndex + dir + NUM_DEVICES) % NUM_DEVICES;
          showMenu(); navLastStep = now;
        }
      }
    } else {
      // Poll en pantalla de proyecto
      unsigned long now = millis();
      if (now - lastPollMs >= POLL_EVERY_MS) {
        lastPollMs = now;

        uint8_t st; uint16_t fx, hz, ucm; bool hu;
        if (readReport(currentAddr(), st, fx, hz, ucm, hu)) {
          haveReport   = true;
          lastStatus   = st;
          lastFlowX100 = fx;
          lastHzX10    = hz;

          unsigned long nowMs = millis();
          if (hu) {
            lastUltraCm   = ucm;
            lastHasUltra  = true;
            lastUltraOkMs = nowMs;
          } else {
            if (nowMs - lastUltraOkMs > 5000UL) lastHasUltra = false;
          }

          // Actualiza EMA para el dispositivo en foco
          emaFlowX100[menuIndex] = emaUpdate(emaFlowX100[menuIndex], lastFlowX100);

          char l1[21];
          if (lastHasUltra) {
            snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u CM:%u",
              (st & 0x01) ? "ON " : "OFF",
              (unsigned)(hz/10), (unsigned)(hz%10),
              (unsigned)lastUltraCm);
          } else {
            snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u CM:---",
              (st & 0x01) ? "ON " : "OFF",
              (unsigned)(hz/10), (unsigned)(hz%10));
          }
          lcdPrintLine(1, l1);

          char l2[21];
          snprintf(l2, sizeof(l2), "F:%2u.%02u L/m        ",
            (unsigned)(emaFlowX100[menuIndex]/100), (unsigned)(emaFlowX100[menuIndex]%100));
          lcdPrintLine(2, l2);

        } else {
          haveReport   = false;
          lastHasUltra = false;
          lastUltraCm  = 0;
          lcdPrintLine(1, "Sin respuesta I2C   ");
          lcdPrintLine(2, "                    ");
        }
      }
    }

  } else {
    // ====== MODO MONITOREO TODOS ======
    unsigned long now = millis();

    if (now - monitorLastPollMs >= MONITOR_POLL_MS) {
      monitorLastPollMs = now;

      for (uint8_t i=0; i<NUM_DEVICES; ++i) {
        uint8_t  st; 
        uint16_t fx, hz, ucm; 
        bool     hu;

        if (readReport(DEV_ADDRS[i], st, fx, hz, ucm, hu)) {
          monHaveReport[i] = true;
          monStatus[i]     = st;
          monFlowX100[i]   = fx;
          monHzX10[i]      = hz;
          monHasUltra[i]   = hu;
          monUltraCm[i]    = hu ? ucm : 0xFFFF;

          // Actualiza EMA para mostrarse en monitoreo
          emaFlowX100[i] = emaUpdate(emaFlowX100[i], monFlowX100[i]);

          // ===== Seguridad con GRACE: objeto ≤ umbral por ULTRA_GRACE_MS =====
          bool danger = (hu && ucm > 0 && ucm <= ULTRA_THRESHOLD_CM);
          if (danger) {
            if (ultraDangerSince[i] == 0) ultraDangerSince[i] = now;
            // Si el relé está ON y la condición persiste más que el grace → apagar
            if ((st & 0x01) && (now - ultraDangerSince[i] >= ULTRA_GRACE_MS)) {
              (void)sendRelay(DEV_ADDRS[i], false);
              monStatus[i] &= ~0x01; // refleja de inmediato
            }
          } else {
            ultraDangerSince[i] = 0;
          }

        } else {
          monHaveReport[i] = false;
          monHasUltra[i]   = false;
          monUltraCm[i]    = 0xFFFF;
          // Reinicia temporizador de peligro si no hay datos
          ultraDangerSince[i] = 0;
        }

        delay(2); // pequeña separación en el bus
      }

      showMonitorPage(monitorPage);
    }

    if (now - monitorLastPageMs >= MONITOR_PAGE_MS) {
      monitorLastPageMs = now;
      uint8_t totalPages = (NUM_DEVICES + 3 - 1) / 3;
      monitorPage = (monitorPage + 1) % totalPages;
      showMonitorPage(monitorPage);
    }
  }

  btnPrev = btnNow;
  delay(8);
}
