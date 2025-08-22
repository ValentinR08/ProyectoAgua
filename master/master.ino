#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================== Direcciones I2C de esclavos ==================
const uint8_t DEV_ADDRS[]  = { 0x0C, 0x08, 0x09, 0x0A, 0x0B };
const char*   DEV_NAMES[]  = { "Entrada", "Casa 1", "Casa 2", "Casa 3", "Deposito" };
const int NUM_DEVICES = sizeof(DEV_ADDRS) / sizeof(DEV_ADDRS[0]);

// ================== LCD (20x4) ==================
LiquidCrystal_I2C lcd(0x27, 20, 4);   // cambia 0x27 → 0x3F si tu módulo lo requiere
static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;

// ================== Joystick ==================
const int yAxisPin  = A1;   // eje Y del joystick
const int buttonPin = 6;    // botón del joystick (a GND) con INPUT_PULLUP

// Umbrales/histeresis del eje Y
const int Y_LOW_THRESH   = 350;
const int Y_HIGH_THRESH  = 700;
const int Y_CENTER_MIN   = 450;
const int Y_CENTER_MAX   = 600;

// ================== Estado UI ==================
bool inMenu = true;
int  menuIndex = 0;

// Botón corto/largo
bool btnPrev = HIGH;
unsigned long btnDownAt = 0;
const unsigned long LONG_PRESS_MS = 1200;

// ================== Lectura periódica esclavo ==================
unsigned long lastReadAt = 0;
const unsigned long READ_EVERY_MS = 500;

// Último estado leído del esclavo seleccionado
bool     haveLastReport = false;
uint8_t  lastStatus     = 0;
uint16_t lastFlowX100   = 0;
uint16_t lastHzX10      = 0;
uint16_t lastUltraCm    = 0;
bool     lastHasUltra   = false;   // indica si el esclavo envió ultrasonido en el último reporte

// Cache de líneas LCD
String lineCache[LCD_ROWS] = {"", "", "", ""};

// ================== Navegación con autorepeat ==================
enum NavState { NAV_CENTER, NAV_UP, NAV_DOWN };
NavState navState = NAV_CENTER;
unsigned long navLastStepAt = 0;
unsigned long navStateSince = 0;
const unsigned long NAV_INITIAL_DELAY   = 350;
const unsigned long NAV_REPEAT_INTERVAL = 220;

// ================== Prototipos ==================
void showMenu();
void showProjectScreen();
void enterProject(int index);
void exitProject(bool forceOff);
bool readSlaveReportFlexible(uint8_t addr,
                             uint8_t &status,
                             uint16_t &flowX100,
                             uint16_t &hzX10,
                             uint16_t &ultraCm,
                             bool &hasUltra);
bool sendRelay(uint8_t addr, uint8_t onOff);
void refreshFromSlaveNow();

// ================== Util ==================
static inline int median3(int a, int b, int c) {
  if (a > b) { int t=a; a=b; b=t; }
  if (b > c) { int t=b; b=c; c=t; }
  if (a > b) { int t=a; a=b; b=t; }
  return b;
}

int readYAxisStable() {
  int a = analogRead(yAxisPin);
  delayMicroseconds(200);
  int b = analogRead(yAxisPin);
  delayMicroseconds(200);
  int c = analogRead(yAxisPin);
  return median3(a,b,c);
}

void printLineFixed(uint8_t row, const char* cstr) {
  char buf[21];
  uint8_t i = 0;
  for (; i < LCD_COLS && cstr[i] != '\0'; ++i) buf[i] = cstr[i];
  for (; i < LCD_COLS; ++i) buf[i] = ' ';
  buf[LCD_COLS] = '\0';

  String s(buf);
  if (row < LCD_ROWS && s != lineCache[row]) {
    lineCache[row] = s;
    lcd.setCursor(0, row);
    lcd.print(s);
  }
}

void clearCacheAndLCD() {
  for (uint8_t r = 0; r < LCD_ROWS; ++r) lineCache[r] = "";
  lcd.clear();
}

uint8_t currentAddr()    { return DEV_ADDRS[menuIndex]; }
const char* currentName(){ return DEV_NAMES[menuIndex]; }

// ================== Setup ==================
void setup() {
  lcd.init();
  lcd.backlight();
  clearCacheAndLCD();
  showMenu();

  pinMode(buttonPin, INPUT_PULLUP);

  Wire.begin();           // UNO como maestro
  Wire.setClock(100000);  // 100 kHz (sube a 400k si todo lo soporta)
  Wire.setTimeout(20);    // 20 ms timeout I2C

  Serial.begin(9600);
  delay(50);
}

// ================== Loop ==================
void loop() {
  int yValue = readYAxisStable();

  // Botón corto/largo
  bool btnNow = digitalRead(buttonPin);
  if (btnPrev == HIGH && btnNow == LOW) { btnDownAt = millis(); }
  if (btnPrev == LOW && btnNow == HIGH) {
    unsigned long held = millis() - btnDownAt;
    if (inMenu) {
      if (held >= 10) enterProject(menuIndex);
    } else {
      if (held >= LONG_PRESS_MS) {
        exitProject(true); // apaga y regresa al menú
      } else {
        // Toggle relé del esclavo actual
        uint8_t addr = currentAddr();
        uint8_t desired = haveLastReport ? ((lastStatus & 0x01) ? 0 : 1) : 1;
        if (!sendRelay(addr, desired)) {
          printLineFixed(2, "I2C error al enviar ");
        }
      }
    }
  }
  btnPrev = btnNow;

  // Menú con autorepeat por eje Y
  if (inMenu) {
    unsigned long now = millis();
    NavState newState;
    if (yValue >= Y_CENTER_MIN && yValue <= Y_CENTER_MAX) newState = NAV_CENTER;
    else if (yValue < Y_LOW_THRESH)                       newState = NAV_UP;   // arriba
    else if (yValue > Y_HIGH_THRESH)                      newState = NAV_DOWN; // abajo
    else                                                  newState = navState;

    if (newState != navState) {
      navState = newState;
      navStateSince = now;
      navLastStepAt = 0;
      if (navState == NAV_UP || navState == NAV_DOWN) {
        int dir = (navState == NAV_UP) ? +1 : -1; // invertido para tu joystick
        menuIndex = (menuIndex + (dir > 0 ? 1 : -1) + NUM_DEVICES) % NUM_DEVICES;
        navLastStepAt = now;
        showMenu();
      }
    } else if (navState == NAV_UP || navState == NAV_DOWN) {
      if ((now - navStateSince) >= NAV_INITIAL_DELAY &&
          (now - navLastStepAt)  >= NAV_REPEAT_INTERVAL) {
        int dir = (navState == NAV_UP) ? +1 : -1;
        menuIndex = (menuIndex + (dir > 0 ? 1 : -1) + NUM_DEVICES) % NUM_DEVICES;
        navLastStepAt = now;
        showMenu();
      }
    }
  } else {
    // pantalla de proyecto: refrescar datos del esclavo
    if (millis() - lastReadAt >= READ_EVERY_MS) {
      lastReadAt = millis();

      uint8_t st; uint16_t fx100, hz10, ucm; bool hasU;
      if (readSlaveReportFlexible(currentAddr(), st, fx100, hz10, ucm, hasU)) {
        haveLastReport = true;
        lastStatus   = st;
        lastFlowX100 = fx100;
        lastHzX10    = hz10;
        if (hasU) lastUltraCm = ucm;
        lastHasUltra = hasU;   // guardar si hubo ultrasonido

        char l0[21];
        snprintf(l0, sizeof(l0), "%s (0x%02X)", currentName(), currentAddr());
        printLineFixed(0, l0);

        // Línea 1: R:ON/OFF  Hz:x.x  U:xxx (solo si hay U)
        if (hasU) {
          char l1[21];
          snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u U:%u",
                   (st & 0x01) ? "ON " : "OFF",
                   (unsigned)(hz10/10), (unsigned)(hz10%10),
                   (unsigned)lastUltraCm);
          printLineFixed(1, l1);
        } else {
          char l1[21];
          snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u       ",
                   (st & 0x01) ? "ON " : "OFF",
                   (unsigned)(hz10/10), (unsigned)(hz10%10));
          printLineFixed(1, l1);
        }

        // Línea 2: F:xx.xx L/m
        char l2[21];
        snprintf(l2, sizeof(l2), "F:%2u.%02u L/m",
                 (unsigned)(fx100/100), (unsigned)(fx100%100));
        printLineFixed(2, l2);

        printLineFixed(3, "Corto:ON/OFF Largo:Menu");
      } else {
        haveLastReport = false;
        lastHasUltra = false;
        printLineFixed(1, "Sin respuesta I2C   ");
        printLineFixed(2, "                    ");
        printLineFixed(3, "Largo:Menu          ");
        Serial.print(F("No responde addr 0x")); Serial.println(currentAddr(), HEX);
      }
    }
  }

  delay(10);
}

// ================== UI ==================
void showMenu() {
  clearCacheAndLCD();
  printLineFixed(0, "Menu:");
  for (int i = 0; i < 3; i++) {
    int idx = (menuIndex + i) % NUM_DEVICES;
    char l[21];
    snprintf(l, sizeof(l), "%c%s", (i==0 ? '>' : ' '), DEV_NAMES[idx]);
    printLineFixed(i+1, l);
  }
}

void showProjectScreen() {
  clearCacheAndLCD();

  // Encabezado con nombre + dirección
  char l0[21];
  snprintf(l0, sizeof(l0), "%s (0x%02X)", currentName(), currentAddr());
  printLineFixed(0, l0);

  // Si ya tenemos una lectura previa, la mostramos; si no, placeholders.
  if (haveLastReport) {
    // Línea 1: R:ON/OFF  Hz:x.x  [U:xxx si hay ultrasonido]
    if (lastHasUltra) {
      char l1[21];
      snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u U:%u",
               (lastStatus & 0x01) ? "ON " : "OFF",
               (unsigned)(lastHzX10/10), (unsigned)(lastHzX10%10),
               (unsigned)lastUltraCm);
      printLineFixed(1, l1);
    } else {
      char l1[21];
      snprintf(l1, sizeof(l1), "R:%s Hz:%u.%u       ",
               (lastStatus & 0x01) ? "ON " : "OFF",
               (unsigned)(lastHzX10/10), (unsigned)(lastHzX10%10));
      printLineFixed(1, l1);
    }

    // Línea 2: F:xx.xx L/m
    char l2[21];
    snprintf(l2, sizeof(l2), "F:%2u.%02u L/m       ",
             (unsigned)(lastFlowX100/100), (unsigned)(lastFlowX100%100));
    printLineFixed(2, l2);
  } else {
    // Aún no hay datos
    printLineFixed(1, "R:-- Hz:--.- U:---");
    printLineFixed(2, "F:--.-- L/m        ");
  }

  // Línea 3: ayuda de controles
  printLineFixed(3, "Corto:ON/OFF Largo:Menu");
}

void enterProject(int index) {
  (void)index;
  inMenu = false;
  haveLastReport = false;
  lastHasUltra = false;

  // Hacer una lectura inmediata para que la pantalla ya salga con datos reales
  refreshFromSlaveNow();

  // Ahora dibuja la pantalla (ya usará valores si existen)
  showProjectScreen();

  // Forzar próximo refresco periódico sin esperar todo el intervalo
  lastReadAt = millis();  // o 0 si prefieres que refresque de inmediato en loop
}

void exitProject(bool forceOff) {
  if (forceOff) sendRelay(currentAddr(), 0);
  inMenu = true; showMenu();
}

// ================== I2C helpers ==================
// Intenta 8 bytes (status,res,flowL,flowH,hzL,hzH,ultraL,ultraH)
// Si falla, intenta 6 bytes (status,res,flowL,flowH,hzL,hzH)
bool readSlaveReportFlexible(uint8_t addr,
                             uint8_t &status,
                             uint16_t &flowX100,
                             uint16_t &hzX10,
                             uint16_t &ultraCm,
                             bool &hasUltra) {
  hasUltra = false;

  // Intento 8 bytes
  {
    const uint8_t N = 8;
    uint8_t got = Wire.requestFrom((uint8_t)addr, (uint8_t)N);
    if (got == N && Wire.available() >= N) {
      uint8_t b0 = Wire.read();
      (void)Wire.read(); // reservado
      uint8_t fL = Wire.read(), fH = Wire.read();
      uint8_t hL = Wire.read(), hH = Wire.read();
      uint8_t uL = Wire.read(), uH = Wire.read();

      status   = b0;
      flowX100 = (uint16_t)fL | ((uint16_t)fH << 8);
      hzX10    = (uint16_t)hL | ((uint16_t)hH << 8);
      ultraCm  = (uint16_t)uL | ((uint16_t)uH << 8);
      hasUltra = true;
      return true;
    }
  }

  // Intento 6 bytes
  {
    const uint8_t N = 6;
    uint8_t got = Wire.requestFrom((uint8_t)addr, (uint8_t)N);
    if (got == N && Wire.available() >= N) {
      uint8_t b0 = Wire.read();
      (void)Wire.read(); // reservado
      uint8_t fL = Wire.read(), fH = Wire.read();
      uint8_t hL = Wire.read(), hH = Wire.read();

      status   = b0;
      flowX100 = (uint16_t)fL | ((uint16_t)fH << 8);
      hzX10    = (uint16_t)hL | ((uint16_t)hH << 8);
      ultraCm  = 0;
      hasUltra = false;
      return true;
    }
  }

  return false;
}

bool sendRelay(uint8_t addr, uint8_t onOff) {
  Wire.beginTransmission(addr);
  Wire.write(onOff ? 1 : 0);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print(F("I2C endTransmission error="));
    Serial.print(err);
    Serial.print(F(" to 0x")); Serial.println(addr, HEX);
    return false;
  }
  return true;
}

void refreshFromSlaveNow() {
  uint8_t st; uint16_t fx100, hz10, ucm; bool hasU;
  if (readSlaveReportFlexible(currentAddr(), st, fx100, hz10, ucm, hasU)) {
    haveLastReport = true;
    lastStatus   = st;
    lastFlowX100 = fx100;
    lastHzX10    = hz10;
    if (hasU) lastUltraCm = ucm;
    lastHasUltra = hasU;
  } else {
    haveLastReport = false;
    lastHasUltra = false;
  }
}
