# Codigo_Smart_Track_Gimbal
// Smart-Track Gimbal: CAM-OLED + MPU6050/servos.
// Modo Seguidor recibe por UDP: "errorX,..." y controla solamente yaw.
// Modo Gimbal conserva la lectura DMP y el mapeo de servos del proyecto MPU-Servos.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// -----------------------------------------------------------------------------
// Pines y periféricos compartidos
// -----------------------------------------------------------------------------
constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;
constexpr uint8_t PIN_MPU_INTERRUPT = 23;
constexpr uint8_t PIN_SERVO_YAW = 25;
constexpr uint8_t PIN_SERVO_PITCH = 26;
constexpr uint8_t PIN_SERVO_ROLL = 27;

constexpr uint8_t BTN_A = 33;       // Seguidor
constexpr uint8_t BTN_B = 32;       // Gimbal
constexpr uint8_t BTN_VOLVER = 14;
constexpr unsigned long DEBOUNCE_MS = 50;

constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 32;
constexpr int8_t OLED_RESET = -1;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Un único objeto para GPIO25: ambos modos controlan el mismo servo yaw,
// pero nunca lo hacen de forma simultánea.
Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

// -----------------------------------------------------------------------------
// Botones y estados
// -----------------------------------------------------------------------------
struct Boton {
  uint8_t pin;
  bool estadoEstable;
  bool estadoLeido;
  unsigned long ultimoCambio;
};

Boton btnA = {BTN_A, true, true, 0};
Boton btnB = {BTN_B, true, true, 0};
Boton btnVolver = {BTN_VOLVER, true, true, 0};

enum Estado { MENU, MODO_SEGUIDOR, MODO_GIMBAL };
Estado estadoActual = MENU;
bool refrescarPantalla = true;

bool leerBoton(Boton &btn) {
  const bool lectura = digitalRead(btn.pin);
  if (lectura != btn.estadoLeido) {
    btn.ultimoCambio = millis();
    btn.estadoLeido = lectura;
  }

  bool presionado = false;
  if (millis() - btn.ultimoCambio >= DEBOUNCE_MS) {
    if (!btn.estadoLeido && btn.estadoEstable) presionado = true;
    btn.estadoEstable = btn.estadoLeido;
  }
  return presionado;
}

// -----------------------------------------------------------------------------
// Modo Seguidor: CAM-OLED, con control PD de velocidad.
// -----------------------------------------------------------------------------
constexpr float ANGULO_MIN = 20.0f;
constexpr float ANGULO_MAX = 160.0f;
constexpr float ANGULO_CENTRO = 90.0f;
constexpr float ANGULO_INICIO = 90.0f;
constexpr float VELOCIDAD_CENTRADO = 15.0f;

// Ajustes del seguidor. La velocidad máxima depende del error: rápida lejos
// del centro, pero intencionalmente limitada al aproximarse para no pasarse.
constexpr float VELOCIDAD_MAX_LEJOS = 18.0f;
constexpr float KP = 20.0f;
constexpr float KD = 0.8f;
constexpr float ACEL_MAX = 35.0f;
constexpr float ZONA_MUERTA = 0.06f;
constexpr int SIGNO = -1;
constexpr float ERROR_MAX = 1.5f;
constexpr float ALPHA = 0.25f;
constexpr float ALPHA_DERIVADA = 0.15f;
constexpr unsigned long ARRANQUE_IGNORAR_MS = 2000;
constexpr unsigned long TIMEOUT_MS = 500;
constexpr int PAQUETES_CONFIRMACION = 3;
constexpr unsigned long PERIODO_MS = 20;

const char *AP_SSID = "PersonFollower";
const char *AP_PASSWORD = "12345678";
constexpr unsigned int UDP_PORT = 4210;

WiFiUDP udp;
char paquete[64];
float anguloActual = ANGULO_INICIO;
float velActual = 0.0f;
float errorFiltrado = 0.0f;
float errorAnterior = 0.0f;
float derivadaFiltrada = 0.0f;
bool filtroIniciado = false;
bool derivadaIniciada = false;
int paquetesSeguidos = 0;
unsigned long ultimoPaquete = 0;
unsigned long tiempoInicio = 0;
unsigned long ultimaActualizacion = 0;

int gradosAMicros(float grados) {
  return static_cast<int>(500.0f + grados * (2000.0f / 180.0f));
}

void iniciarSeguidor() {
  // El modo Gimbal pudo haber escrito yaw directamente. Tomamos esa posición
  // como inicio para que al cambiar de modo no haya un salto artificial.
  anguloActual = constrain(static_cast<float>(servoYaw.read()), ANGULO_MIN, ANGULO_MAX);
  filtroIniciado = false;
  derivadaIniciada = false;
  derivadaFiltrada = 0.0f;
  paquetesSeguidos = 0;
  velActual = 0.0f;
  tiempoInicio = millis();
  ultimaActualizacion = millis();
}

void leerUDP() {
  if (udp.parsePacket() <= 0) return;

  int len = 0;
  do {
    len = udp.read(paquete, sizeof(paquete) - 1);
  } while (udp.parsePacket() > 0);

  if (millis() - tiempoInicio < ARRANQUE_IGNORAR_MS || len <= 0) return;
  paquete[len] = '\0';

  char *fin;
  const float errX = strtof(paquete, &fin);
  if (fin == paquete || *fin != ',' || !isfinite(errX) || fabs(errX) > ERROR_MAX) return;

  const unsigned long ahora = millis();
  if (ahora - ultimoPaquete > TIMEOUT_MS) {
    filtroIniciado = false;
    derivadaIniciada = false;
    derivadaFiltrada = 0.0f;
    paquetesSeguidos = 0;
  }
  ultimoPaquete = ahora;

  if (!filtroIniciado) {
    errorFiltrado = errX;
    filtroIniciado = true;
  } else {
    errorFiltrado = ALPHA * errX + (1.0f - ALPHA) * errorFiltrado;
  }
  if (paquetesSeguidos < PAQUETES_CONFIRMACION) paquetesSeguidos++;
}

void actualizarServoSeguidor() {
  const unsigned long ahora = millis();
  if (ahora - ultimaActualizacion < PERIODO_MS) return;

  const float dt = (ahora - ultimaActualizacion) / 1000.0f;
  ultimaActualizacion = ahora;
  const bool activo = filtroIniciado && (ahora - ultimoPaquete <= TIMEOUT_MS) &&
                      (paquetesSeguidos >= PAQUETES_CONFIRMACION);

  float velDeseada = 0.0f;
  if (activo) {
    // La derivada mide si el error está creciendo o disminuyendo.
    if (!derivadaIniciada) {
      errorAnterior = errorFiltrado;
      derivadaFiltrada = 0.0f;
      derivadaIniciada = true;
    } else {
      const float derivada = (errorFiltrado - errorAnterior) / dt;
      derivadaFiltrada = ALPHA_DERIVADA * derivada +
                          (1.0f - ALPHA_DERIVADA) * derivadaFiltrada;
      errorAnterior = errorFiltrado;
    }

    const float errorAbsoluto = fabs(errorFiltrado);
    const float errorControl = errorAbsoluto > ZONA_MUERTA ? errorFiltrado : 0.0f;

    // Cuanto más cerca esté la persona del centro, menor es el techo de
    // velocidad. Esto evita el salto brusco que provocaba sobrepasos.
    const float progreso = constrain(
        (errorAbsoluto - ZONA_MUERTA) / (ERROR_MAX - ZONA_MUERTA), 0.0f, 1.0f);
    const float limiteVelocidad = 4.0f +
        (VELOCIDAD_MAX_LEJOS - 4.0f) * sqrtf(progreso);

    float ordenPD = KP * errorControl;
    // La derivada solo frena si el error ya disminuye. Nunca puede sumar una
    // aceleración por un salto de medición de la cámara.
    if (errorControl * derivadaFiltrada < 0.0f) {
      ordenPD += KD * derivadaFiltrada;
    }
    velDeseada = SIGNO * constrain(ordenPD, -limiteVelocidad, limiteVelocidad);
  } else {
    // No usar una derivada vieja después de perder la detección.
    derivadaIniciada = false;
    derivadaFiltrada = 0.0f;
  }

  const float maxDv = ACEL_MAX * dt;
  velActual += constrain(velDeseada - velActual, -maxDv, maxDv);
  anguloActual += velActual * dt;

  if (anguloActual <= ANGULO_MIN) {
    anguloActual = ANGULO_MIN;
    if (velActual < 0) velActual = 0;
  }
  if (anguloActual >= ANGULO_MAX) {
    anguloActual = ANGULO_MAX;
    if (velActual > 0) velActual = 0;
  }
  servoYaw.writeMicroseconds(gradosAMicros(anguloActual));
}

// -----------------------------------------------------------------------------
// Modo Gimbal: MPU-Servos
// -----------------------------------------------------------------------------
MPU6050 mpu;
volatile bool mpuInterrupt = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float correct;
int j = 0;

void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

void iniciarMPU() {
  mpu.initialize();
  pinMode(PIN_MPU_INTERRUPT, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);

  if (devStatus != 0) {
    Serial.print("DMP initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    return;
  }

  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INTERRUPT), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;
}

void iniciarGimbal() {
  // El offset de yaw se vuelve a tomar al entrar al modo Gimbal.
  correct = 0.0f;
  j = 0;
  mpu.resetFIFO();
  fifoCount = 0;
  mpuInterrupt = false;
}

void actualizarGimbal() {
  if (!dmpReady) return;

  // Equivale a la espera del original, pero sin bloquear el botón VOLVER.
  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (!(mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))) return;
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[2] = ypr[2] * 180 / M_PI;

  if (j <= 300) {
    correct = ypr[0];
    j++;
    return;
  }

  ypr[0] = ypr[0] - correct;
  const int servo0Value = map(ypr[0], -90, 90, 0, 180);
  const int servo1Value = map(ypr[1], -90, 90, 0, 180);
  const int servo2Value = map(ypr[2], -90, 90, 180, 0);

  servoYaw.write(servo0Value);
  servoPitch.write(servo1Value);
  servoRoll.write(servo2Value);
}

// -----------------------------------------------------------------------------
// Interfaz OLED y despacho de modos
// -----------------------------------------------------------------------------
void mostrarMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 0); display.print("SELECCIONAR MODO");
  display.setCursor(0, 12); display.print(" [A]  Seguidor");
  display.setCursor(0, 22); display.print(" [B]  Gimbal");
  display.display();
}

void mostrarModoSeguidor() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(12, 0); display.print("MODO SEGUIDOR");
  display.setCursor(4, 12); display.print("En funcionamiento");
  display.setCursor(0, 23); display.print("[VOLVER] para salir");
  display.display();
}

void mostrarModoGimbal() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(18, 0); display.print("MODO GIMBAL");
  display.setCursor(4, 12); display.print(dmpReady ? "En funcionamiento" : "MPU/DMP con error");
  display.setCursor(0, 23); display.print("[VOLVER] para salir");
  display.display();
}

void loopMenu() {
  if (leerBoton(btnA)) {
    iniciarSeguidor();
    estadoActual = MODO_SEGUIDOR;
    refrescarPantalla = true;
    return;
  }
  if (leerBoton(btnB)) {
    iniciarGimbal();
    estadoActual = MODO_GIMBAL;
    refrescarPantalla = true;
    return;
  }
  leerBoton(btnVolver);
}

void loopSeguidor() {
  if (leerBoton(btnVolver)) {
    velActual = 0.0f;
    filtroIniciado = false;
    paquetesSeguidos = 0;
    servoYaw.writeMicroseconds(gradosAMicros(anguloActual));
    estadoActual = MENU;
    refrescarPantalla = true;
    return;
  }
  leerBoton(btnA);
  leerBoton(btnB);
  leerUDP();
  actualizarServoSeguidor();
}

void loopGimbal() {
  if (leerBoton(btnVolver)) {
    estadoActual = MENU;
    refrescarPantalla = true;
    return;
  }
  leerBoton(btnA);
  leerBoton(btnB);
  actualizarGimbal();
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_VOLVER, INPUT_PULLUP);

  // Un solo bus I2C: OLED en 0x3C y MPU6050 en 0x68.
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[ERROR] OLED SSD1306 no encontrado");
    for (;;) delay(1);
  }
  display.clearDisplay();
  display.display();

  servoYaw.attach(PIN_SERVO_YAW, 500, 2500);
  servoPitch.attach(PIN_SERVO_PITCH, 500, 2500);
  servoRoll.attach(PIN_SERVO_ROLL, 500, 2500);
  servoYaw.writeMicroseconds(gradosAMicros(anguloActual));

  iniciarMPU();

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.print("AP creado. IP: ");
  Serial.println(WiFi.softAPIP());
  udp.begin(UDP_PORT);
  while (udp.parsePacket() > 0) udp.read(paquete, sizeof(paquete) - 1);
}

void loop() {
  if (refrescarPantalla) {
    switch (estadoActual) {
      case MENU: mostrarMenu(); break;
      case MODO_SEGUIDOR: mostrarModoSeguidor(); break;
      case MODO_GIMBAL: mostrarModoGimbal(); break;
    }
    refrescarPantalla = false;
  }

  switch (estadoActual) {
    case MENU: loopMenu(); break;
    case MODO_SEGUIDOR: loopSeguidor(); break;
    case MODO_GIMBAL: loopGimbal(); break;
  }
}
