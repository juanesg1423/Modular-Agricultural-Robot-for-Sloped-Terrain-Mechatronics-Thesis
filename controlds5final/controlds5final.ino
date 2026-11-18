#include <Arduino.h>
#include <ps5Controller.h>
#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_L3GD20_U.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>

#define CAN_TX GPIO_NUM_17
#define CAN_RX GPIO_NUM_16

// Ultrasonidos, según el esquemático
const uint8_t TRIG_US1 = 25;
const uint8_t ECHO_US1 = 26;
const uint8_t TRIG_US2 = 32;
const uint8_t ECHO_US2 = 33;

// Adafruit 10-DOF: se usan acelerómetro y giroscopio para roll/pitch.
// I2C: SDA GPIO21, SCL GPIO22. Direcciones: accel 0x19, gyro 0x6B.
const uint8_t IMU_SDA = 21;
const uint8_t IMU_SCL = 22;

// BLE directo en la ESP32 principal: la HMI recibe el mismo texto que Serial.
// DualSense sigue usando Bluetooth Classic; la HMI se conecta por BLE.
#define BLE_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *servidorBLE = nullptr;
BLECharacteristic *caracteristicaTXBLE = nullptr;
volatile bool hmiBLEConectada = false;

// Cola acotada: evita ejecutar notificaciones BLE dentro de Serial.print().
const size_t TAMANO_COLA_BLE = 2048;
uint8_t colaBLE[TAMANO_COLA_BLE];
size_t cabezaColaBLE = 0;
size_t colaColaBLE = 0;
uint32_t ultimoEnvioBLE = 0;

void guardarByteEnColaBLE(uint8_t dato)
{
  if (!hmiBLEConectada) return;

  size_t siguiente = (cabezaColaBLE + 1) % TAMANO_COLA_BLE;
  if (siguiente == colaColaBLE)
  {
    // Si se llena, descarta el byte más antiguo.
    colaColaBLE = (colaColaBLE + 1) % TAMANO_COLA_BLE;
  }

  colaBLE[cabezaColaBLE] = dato;
  cabezaColaBLE = siguiente;
}

class CallbacksServidorBLE : public BLEServerCallbacks
{
  void onConnect(BLEServer *servidor) override
  {
    hmiBLEConectada = true;
  }

  void onDisconnect(BLEServer *servidor) override
  {
    hmiBLEConectada = false;
    servidor->startAdvertising();
  }
};

HardwareSerial &serialUSB = Serial;

// Mantiene la salida USB y replica sus bytes a la HMI si está conectada.
class SerialConBLE : public Print
{
public:
  explicit SerialConBLE(HardwareSerial &usb) : usb_(usb) {}

  using Print::write;

  size_t write(uint8_t dato) override
  {
    size_t escritos = usb_.write(dato);
    guardarByteEnColaBLE(dato);
    return escritos;
  }

  size_t write(const uint8_t *datos, size_t cantidad) override
  {
    size_t escritos = usb_.write(datos, cantidad);
    for (size_t i = 0; i < cantidad; i++)
    {
      guardarByteEnColaBLE(datos[i]);
    }
    return escritos;
  }

private:
  HardwareSerial &usb_;
};

SerialConBLE serialConBLE(serialUSB);
#define Serial serialConBLE

void iniciarBLE()
{
  BLEDevice::init("Robot_Agricola_HMI");
  servidorBLE = BLEDevice::createServer();
  servidorBLE->setCallbacks(new CallbacksServidorBLE());

  BLEService *servicio = servidorBLE->createService(BLE_SERVICE_UUID);
  caracteristicaTXBLE = servicio->createCharacteristic(
      BLE_CHARACTERISTIC_TX,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  caracteristicaTXBLE->addDescriptor(new BLE2902());
  servicio->createCharacteristic(
      BLE_CHARACTERISTIC_RX,
      BLECharacteristic::PROPERTY_WRITE);

  servicio->start();

  BLEAdvertising *anuncio = BLEDevice::getAdvertising();
  anuncio->addServiceUUID(BLE_SERVICE_UUID);
  anuncio->start();
}

void actualizarSalidaBLE()
{
  if (!hmiBLEConectada || caracteristicaTXBLE == nullptr) return;
  if (millis() - ultimoEnvioBLE < 10) return;
  ultimoEnvioBLE = millis();

  uint8_t paquete[20];
  size_t cantidad = 0;
  while (colaColaBLE != cabezaColaBLE && cantidad < sizeof(paquete))
  {
    paquete[cantidad++] = colaBLE[colaColaBLE];
    colaColaBLE = (colaColaBLE + 1) % TAMANO_COLA_BLE;
  }

  if (cantidad > 0)
  {
    caracteristicaTXBLE->setValue(paquete, cantidad);
    caracteristicaTXBLE->notify();
  }
}

Adafruit_LSM303_Accel_Unified imuAccel(1001);
Adafruit_L3GD20_Unified imuGyro(1003);

bool accelDisponible = false;
bool gyroDisponible = false;
bool imuDisponible = false;
float inclinacionRoll = 0.0f;
float inclinacionPitch = 0.0f;
float rollGradosActual = 0.0f;
float pitchGradosActual = 0.0f;
bool angulosIMUValidos = false;
uint32_t ultimaLecturaIMU = 0;

// Calibración verificada del L3GD20: offsets medidos en reposo (°/s).
const float GYRO_OFFSET_X_DPS = 3.3021f;
const float GYRO_OFFSET_Y_DPS = -6.9752f;
const float GYRO_DEADZONE_DPS = 0.4f;
const uint8_t GYRO_I2C_ADDRESS = 0x6B;

// Orientación física del IMU: X lateral, Y longitudinal, Z vertical.
// Convención positiva por defecto: +X derecha, +Y frente, +Z arriba.
const int8_t SIGNO_IMU_X_DERECHA = +1;
const int8_t SIGNO_IMU_Y_FRENTE = +1;
const int8_t SIGNO_IMU_Z_ARRIBA = +1;

// Publica una lectura inicial y luego solo cambios sostenidos, no ruido puntual.
const uint8_t MUESTRAS_CONFIRMAR_CAMBIO_IMU = 3;
struct EstadoIMUReferencia
{
  float roll;
  float pitch;
};
EstadoIMUReferencia referenciaIMU = {};
bool referenciaIMUValida = false;
uint8_t muestrasCambioIMU = 0;

// Configuración de ultrasonidos
const uint16_t DISTANCIA_PARADA_CM = 45;
const uint32_t PERIODO_LECTURA_US_MS = 35;
const uint32_t TIMEOUT_ECHO_US = 25000;

long distanciaUS1 = 999;
long distanciaUS2 = 999;
bool leerSensor1Ahora = true;
bool sensoresListos = false;
uint8_t sensoresLeidosMask = 0;
uint32_t ultimoUltrasonico = 0;

// Posición física de los SteadyWin:
// lado izquierdo: 11 frontal, 12 trasero; lado derecho: 13 frontal, 14 trasero.
const uint8_t ID_STEADY_11 = 11;
const uint8_t ID_STEADY_12 = 12;
const uint8_t ID_STEADY_13 = 13;
const uint8_t ID_STEADY_14 = 14;

// Nodos CANopen de los motores hub
const uint8_t ID_FL = 1;
const uint8_t ID_RL = 2;
const uint8_t ID_FR = 3;
const uint8_t ID_RR = 4;

// Sentidos de los SteadyWin
const int8_t SIGNO_STEADY_11 = +1;
const int8_t SIGNO_STEADY_12 = -1;
const int8_t SIGNO_STEADY_13 = -1;
const int8_t SIGNO_STEADY_14 = +1;

// Cambia a -1 si algún hub gira al revés
const int8_t SIGN_FL = +1;
const int8_t SIGN_RL = +1;
const int8_t SIGN_FR = +1;
const int8_t SIGN_RR = +1;

// Velocidad de prueba de altura
const int16_t RPM_ALTURA = 2;
const int16_t RPM_CORRECCION_NIVELACION = 1;
const float UMBRAL_NIVELACION_GRADOS = 2.0f;
const float UMBRAL_LIBERAR_NIVELACION_GRADOS = 1.0f;

// Invertir si, después de validar la orientación física, la corrección va al revés.
const int8_t SIGNO_CORRECCION_ROLL = +1;
const int8_t SIGNO_CORRECCION_PITCH = +1;

// Límite de velocidad de los hubs
const int16_t RPM_HUB_MINIMO = 10;
const int16_t RPM_HUB_MAXIMO = 150;
const int16_t PASO_RPM_HUB = 10;
int16_t velocidadMaximaHub = 30;

// Zona muerta y habilitación R2
const int DEADZONE = 18;
const int DEADMAN_R2 = 30;

const uint32_t PERIODO_ENVIO_MS = 50;
const uint32_t ACEL_DEC = 33;
const uint32_t ESCALA_RPM_STEADY = 100;

// Posición reportada por GDZ468: 16384 cuentas equivalen a 360°.
// Los límites de recorrido se capturan con el mando en modo altura.
const int32_t CUENTAS_POR_VUELTA_GDZ = 16384;
const uint32_t PERIODO_LECTURA_POSICION_MS = 25;
const uint32_t POSICION_STALE_MS = 250;
const uint32_t TIMEOUT_POSICION_MS = 80;
const int32_t RECORRIDO_MINIMO_CALIBRACION_CUENTAS = 20;

int32_t posicionSteadyCounts[4] = {0, 0, 0, 0};
int32_t limiteInferiorSteadyCounts[4] = {0, 0, 0, 0};
int32_t limiteSuperiorSteadyCounts[4] = {0, 0, 0, 0};
int8_t direccionCuentaSubidaLogica[4] = {0, 0, 0, 0};
uint32_t ultimaPosicionSteadyMs[4] = {0, 0, 0, 0};
uint32_t contadorPosicionSteadyRx[4] = {0, 0, 0, 0};
bool posicionSteadyValida[4] = {false, false, false, false};
// 0 = sin calibrar; 1 = límite inferior guardado; 2 = recorrido completo.
uint8_t etapaLimitesRecorrido = 0;
uint32_t ultimaSolicitudPosicionMs = 0;
uint8_t siguienteSteadyAConsultar = 0;

enum ModoControl
{
  MODO_NINGUNO,
  MODO_ALTURA,
  MODO_DESPLAZAMIENTO
};

// Prototipo explícito para evitar el error de autogeneración del Arduino IDE.
void seleccionarModo(ModoControl nuevoModo);

ModoControl modoActual = MODO_NINGUNO;

bool sistemaArmado = false;
bool hubsConfigurados = false;
bool dualSenseConectadoAnterior = false;
bool nivelacionActivaAnterior = false;
int8_t direccionCorreccionRoll = 0;
int8_t direccionCorreccionPitch = 0;

uint32_t ultimoEnvioControl = 0;
uint32_t ultimaImpresionSteady = 0;

int16_t ultimoRPMAltura = 0;
int16_t ultimoRPMIzq = 0;
int16_t ultimoRPMDer = 0;
int16_t ultimoRPMSteady11 = 0;
int16_t ultimoRPMSteady12 = 0;
int16_t ultimoRPMSteady13 = 0;
int16_t ultimoRPMSteady14 = 0;


// ============================================================
// STEADYWIN
// ============================================================

bool enviarVelocidadSteady(uint8_t id, int32_t rpm)
{
  int32_t escalado = rpm * (int32_t)ESCALA_RPM_STEADY;
  uint32_t datos = (uint32_t)escalado;

  twai_message_t tx = {};
  tx.identifier = id;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 5;

  tx.data[0] = 0xC1;
  tx.data[1] = (uint8_t)(datos & 0xFF);
  tx.data[2] = (uint8_t)((datos >> 8) & 0xFF);
  tx.data[3] = (uint8_t)((datos >> 16) & 0xFF);
  tx.data[4] = (uint8_t)((datos >> 24) & 0xFF);

  esp_err_t resultado =
      twai_transmit(&tx, pdMS_TO_TICKS(10));

  if (resultado != ESP_OK)
  {
    Serial.print("Error TX SteadyWin, ID ");
    Serial.print(id);
    Serial.print(": ");
    Serial.println(esp_err_to_name(resultado));
    return false;
  }

  return true;
  if (resultado != ESP_OK)
{
  Serial.print("Error TX SteadyWin, ID ");
  Serial.print(id);
  Serial.print(": ");
  Serial.println(esp_err_to_name(resultado));

  imprimirEstadoTWAI();
  return false;
}
}


bool moverSteadyWinIndividual(
    int16_t rpm11,
    int16_t rpm12,
    int16_t rpm13,
    int16_t rpm14)
{
  bool ok = true;

  if (!enviarVelocidadSteady(
          ID_STEADY_11,
          rpm11 * SIGNO_STEADY_11))
  {
    ok = false;
  }

  if (!enviarVelocidadSteady(
          ID_STEADY_12,
          rpm12 * SIGNO_STEADY_12))
  {
    ok = false;
  }

  if (!enviarVelocidadSteady(
          ID_STEADY_13,
          rpm13 * SIGNO_STEADY_13))
  {
    ok = false;
  }

  if (!enviarVelocidadSteady(
          ID_STEADY_14,
          rpm14 * SIGNO_STEADY_14))
  {
    ok = false;
  }

  if (ok)
  {
    ultimoRPMSteady11 = rpm11;
    ultimoRPMSteady12 = rpm12;
    ultimoRPMSteady13 = rpm13;
    ultimoRPMSteady14 = rpm14;
  }

  return ok;
}


bool moverSteadyWin(int16_t rpmLogicos)
{
  return moverSteadyWinIndividual(
      rpmLogicos,
      rpmLogicos,
      rpmLogicos,
      rpmLogicos);
}


void detenerSteadyWin()
{
  moverSteadyWin(0);
  ultimoRPMAltura = 0;
  direccionCorreccionRoll = 0;
  direccionCorreccionPitch = 0;
  nivelacionActivaAnterior = false;
}


void limpiarCAN();


int8_t indiceSteadyPorID(uint32_t id)
{
  if (id == ID_STEADY_11) return 0;
  if (id == ID_STEADY_12) return 1;
  if (id == ID_STEADY_13) return 2;
  if (id == ID_STEADY_14) return 3;
  return -1;
}


bool guardarRespuestaPosicionSteady(const twai_message_t &rx)
{
  if (rx.data_length_code < 7 || rx.data[0] != 0xA3) return false;

  int8_t indice = indiceSteadyPorID(rx.identifier);
  if (indice < 0) return false;

  // A3 bytes [3..6]: posición absoluta multi-vuelta, int32 little-endian.
  uint32_t raw =
      ((uint32_t)rx.data[3])
    | ((uint32_t)rx.data[4] << 8)
    | ((uint32_t)rx.data[5] << 16)
    | ((uint32_t)rx.data[6] << 24);

  posicionSteadyCounts[indice] = (int32_t)raw;
  ultimaPosicionSteadyMs[indice] = millis();
  contadorPosicionSteadyRx[indice]++;
  posicionSteadyValida[indice] = true;
  return true;
}


bool solicitarPosicionSteady(uint8_t id)
{
  twai_message_t tx = {};
  tx.identifier = id;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 1;
  tx.data[0] = 0xA3;

  return twai_transmit(&tx, pdMS_TO_TICKS(10)) == ESP_OK;
}


bool solicitarYEsperarPosicionSteady(uint8_t id, uint32_t timeoutMs)
{
  int8_t indice = indiceSteadyPorID(id);
  if (indice < 0) return false;

  uint32_t antes = contadorPosicionSteadyRx[indice];
  if (!solicitarPosicionSteady(id)) return false;

  uint32_t inicio = millis();
  twai_message_t rx;

  while (millis() - inicio < timeoutMs)
  {
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK) continue;

    if (guardarRespuestaPosicionSteady(rx) &&
        rx.identifier == id &&
        contadorPosicionSteadyRx[indice] != antes)
    {
      return true;
    }
  }

  return false;
}


bool calibrarReferenciaNivelacionSteady()
{
  if (modoActual != MODO_ALTURA || sistemaArmado)
  {
    Serial.println("Calibración bloqueada: selecciona modo altura y desarma primero.");
    return false;
  }

  detenerSteadyWin();
  limpiarCAN();

  const uint8_t ids[4] = {
    ID_STEADY_11,
    ID_STEADY_12,
    ID_STEADY_13,
    ID_STEADY_14
  };

  if (etapaLimitesRecorrido != 1)
  {
    etapaLimitesRecorrido = 0;
    Serial.println("Leyendo límite INFERIOR del recorrido...");
  }
  else
  {
    Serial.println("Leyendo límite SUPERIOR del recorrido...");
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    if (!solicitarYEsperarPosicionSteady(ids[i], TIMEOUT_POSICION_MS))
    {
      Serial.print("Sin posición A3 del SteadyWin ID ");
      Serial.println(ids[i]);
      Serial.println("Límites sin completar; revisa dirección y CAN.");
      return false;
    }
  }

  if (etapaLimitesRecorrido != 1)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      limiteInferiorSteadyCounts[i] = posicionSteadyCounts[i];
    }

    etapaLimitesRecorrido = 1;
    Serial.println("Límite inferior guardado para los cuatro motores.");
    Serial.println("Arma, sube con R2 + stick arriba hasta la altura máxima segura.");
    Serial.println("Suelta R2, desarma con X y vuelve a pulsar flecha izquierda.");
    Serial.println("Mientras calibras, el movimiento hacia abajo queda bloqueado.");
    return true;
  }

  bool recorridoValido = true;
  int32_t nuevosLimitesInferiores[4];
  int32_t nuevosLimitesSuperiores[4];
  int8_t nuevasDireccionesSubida[4];

  for (uint8_t i = 0; i < 4; i++)
  {
    int32_t posicionBaja = limiteInferiorSteadyCounts[i];
    int32_t posicionAlta = posicionSteadyCounts[i];
    int32_t diferencia = posicionAlta - posicionBaja;

    if (labs(diferencia) < RECORRIDO_MINIMO_CALIBRACION_CUENTAS)
    {
      Serial.print("Recorrido insuficiente para el motor ID ");
      Serial.println(ids[i]);
      recorridoValido = false;
      continue;
    }

    nuevasDireccionesSubida[i] = (diferencia > 0) ? +1 : -1;
    nuevosLimitesInferiores[i] = min(posicionBaja, posicionAlta);
    nuevosLimitesSuperiores[i] = max(posicionBaja, posicionAlta);
  }

  if (!recorridoValido)
  {
    etapaLimitesRecorrido = 1;
    Serial.println("No se guardó el límite superior. Mueve más los motores y repite.");
    return false;
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    direccionCuentaSubidaLogica[i] = nuevasDireccionesSubida[i];
    limiteInferiorSteadyCounts[i] = nuevosLimitesInferiores[i];
    limiteSuperiorSteadyCounts[i] = nuevosLimitesSuperiores[i];
  }

  etapaLimitesRecorrido = 2;
  Serial.println("Recorrido personalizado guardado para los cuatro SteadyWin.");
  Serial.println("El límite aplica al movimiento manual y a la nivelación automática.");
  return true;
}


void actualizarConsultaPosicionesSteady()
{
  if (modoActual != MODO_ALTURA || etapaLimitesRecorrido == 0) return;
  if (millis() - ultimaSolicitudPosicionMs < PERIODO_LECTURA_POSICION_MS) return;
  ultimaSolicitudPosicionMs = millis();

  const uint8_t ids[4] = {
    ID_STEADY_11,
    ID_STEADY_12,
    ID_STEADY_13,
    ID_STEADY_14
  };

  solicitarPosicionSteady(ids[siguienteSteadyAConsultar]);
  siguienteSteadyAConsultar = (siguienteSteadyAConsultar + 1) % 4;
}


int16_t limitarMovimientoSteadyPorRecorrido(uint8_t indice, int16_t rpmLogico)
{
  if (rpmLogico == 0) return 0;
  if (indice >= 4 || etapaLimitesRecorrido == 0) return rpmLogico;

  if (!posicionSteadyValida[indice] ||
      millis() - ultimaPosicionSteadyMs[indice] > POSICION_STALE_MS)
  {
    return 0; // Sin posición fresca, no se permite mover fuera de límites conocidos.
  }

  if (etapaLimitesRecorrido == 1)
  {
    // Desde el límite inferior solo se permite subir durante el ajuste del máximo.
    return (rpmLogico > 0) ? rpmLogico : 0;
  }

  if (direccionCuentaSubidaLogica[indice] == 0) return 0;

  int32_t direccionMovimientoCounts =
      (rpmLogico > 0 ? +1 : -1) * direccionCuentaSubidaLogica[indice];
  int32_t posicion = posicionSteadyCounts[indice];

  if (direccionMovimientoCounts > 0 &&
      posicion >= limiteSuperiorSteadyCounts[indice]) return 0;
  if (direccionMovimientoCounts < 0 &&
      posicion <= limiteInferiorSteadyCounts[indice]) return 0;

  return rpmLogico;
}


void leerRespuestasSteadyWin()
{
  twai_message_t rx;

  while (twai_receive(&rx, 0) == ESP_OK)
  {
    guardarRespuestaPosicionSteady(rx);

    if (rx.data_length_code >= 8 && rx.data[0] == 0xAE)
    {
      uint16_t vbusRaw =
          (uint16_t)rx.data[1]
        | ((uint16_t)rx.data[2] << 8);

      uint16_t ibusRaw =
          (uint16_t)rx.data[3]
        | ((uint16_t)rx.data[4] << 8);

      Serial.print("SteadyWin ID ");
      Serial.print(rx.identifier);
      Serial.print(" | Bus: ");
      Serial.print(vbusRaw * 0.01f, 2);
      Serial.print(" V | Corriente: ");
      Serial.print(ibusRaw * 0.01f, 2);
      Serial.print(" A | Falla: 0x");
      Serial.println(rx.data[7], HEX);
    }

    if (rx.data_length_code >= 5 &&
        rx.data[0] == 0xC1 &&
        millis() - ultimaImpresionSteady >= 500)
    {
      uint32_t raw =
          ((uint32_t)rx.data[1])
        | ((uint32_t)rx.data[2] << 8)
        | ((uint32_t)rx.data[3] << 16)
        | ((uint32_t)rx.data[4] << 24);

      Serial.print("Respuesta C1 ID ");
      Serial.print(rx.identifier);
      Serial.print(" | velocidad: ");
      Serial.print((int32_t)raw / 100.0f, 2);
      Serial.println(" RPM");

      ultimaImpresionSteady = millis();
    }
  }
}


// ============================================================
// CANOPEN DE LOS HUBS
// ============================================================

void limpiarCAN()
{
  twai_message_t rx;

  while (twai_receive(&rx, 0) == ESP_OK)
  {
  }
}


bool esperarACK(uint8_t nodo, uint16_t index, uint8_t subindex)
{
  uint32_t inicio = millis();
  twai_message_t rx;

  while (millis() - inicio < 30)
  {
    if (twai_receive(&rx, pdMS_TO_TICKS(2)) == ESP_OK)
    {
      if (rx.identifier != (0x580 + nodo)) continue;
      if (rx.data_length_code < 8) continue;

      if (rx.data[1] != (index & 0xFF) ||
          rx.data[2] != ((index >> 8) & 0xFF) ||
          rx.data[3] != subindex)
      {
        continue;
      }

      if (rx.data[0] == 0x60)
      {
        return true;
      }

      if (rx.data[0] == 0x80)
      {
        uint32_t codigoError =
            ((uint32_t)rx.data[4])
          | ((uint32_t)rx.data[5] << 8)
          | ((uint32_t)rx.data[6] << 16)
          | ((uint32_t)rx.data[7] << 24);

        Serial.print("Error SDO nodo ");
        Serial.print(nodo);
        Serial.print(" | objeto 0x");
        Serial.print(index, HEX);
        Serial.print(":");
        Serial.print(subindex, HEX);
        Serial.print(" | código 0x");
        Serial.println(codigoError, HEX);

        return false;
      }
    }
  }

  Serial.print("Sin respuesta CANopen del nodo ");
  Serial.println(nodo);
  return false;
}


bool enviarSDO(
    uint8_t nodo,
    uint8_t comando,
    uint16_t index,
    uint8_t subindex,
    uint32_t valor)
{
  limpiarCAN();

  twai_message_t tx = {};
  tx.identifier = 0x600 + nodo;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 8;

  tx.data[0] = comando;
  tx.data[1] = index & 0xFF;
  tx.data[2] = (index >> 8) & 0xFF;
  tx.data[3] = subindex;
  tx.data[4] = valor & 0xFF;
  tx.data[5] = (valor >> 8) & 0xFF;
  tx.data[6] = (valor >> 16) & 0xFF;
  tx.data[7] = (valor >> 24) & 0xFF;

  esp_err_t resultado =
      twai_transmit(&tx, pdMS_TO_TICKS(20));

  if (resultado != ESP_OK)
  {
    Serial.print("Error TX CANopen nodo ");
    Serial.println(nodo);
    return false;
  }

  return esperarACK(nodo, index, subindex);
}


bool escribir8(
    uint8_t nodo,
    uint16_t index,
    uint8_t subindex,
    uint8_t valor)
{
  return enviarSDO(nodo, 0x2F, index, subindex, valor);
}


bool escribir16(
    uint8_t nodo,
    uint16_t index,
    uint8_t subindex,
    uint16_t valor)
{
  return enviarSDO(nodo, 0x2B, index, subindex, valor);
}


bool escribir32(
    uint8_t nodo,
    uint16_t index,
    uint8_t subindex,
    uint32_t valor)
{
  return enviarSDO(nodo, 0x23, index, subindex, valor);
}


bool enviarRPMHub(uint8_t nodo, int16_t rpm)
{
  rpm = constrain(
      rpm,
      -velocidadMaximaHub,
      velocidadMaximaHub
  );

  uint16_t valor = (uint16_t)rpm;

  return escribir16(nodo, 0x2FF0, 0x09, valor);
}


bool configurarHub(uint8_t nodo)
{
  Serial.print("Configurando hub nodo ");
  Serial.println(nodo);

  bool ok = true;

  ok &= escribir16(nodo, 0x6040, 0x00, 0x0086);
  delay(10);

  ok &= escribir8(nodo, 0x605A, 0x11, 0);
  delay(10);

  ok &= escribir8(nodo, 0x6060, 0x00, 3);
  delay(10);

  ok &= escribir32(nodo, 0x6083, 0x00, ACEL_DEC);
  delay(10);

  ok &= escribir32(nodo, 0x6084, 0x00, ACEL_DEC);
  delay(10);

  ok &= enviarRPMHub(nodo, 0);
  delay(10);

  ok &= escribir16(nodo, 0x6040, 0x00, 0x000F);

  Serial.print("Hub nodo ");
  Serial.println(ok ? "OK" : "ERROR");

  return ok;
}


bool configurarTodosLosHubs()
{
  Serial.println("Configurando los cuatro hubs...");

  bool ok1 = configurarHub(ID_FL);
  bool ok2 = configurarHub(ID_RL);
  bool ok3 = configurarHub(ID_FR);
  bool ok4 = configurarHub(ID_RR);

  hubsConfigurados = ok1 && ok2 && ok3 && ok4;

  if (hubsConfigurados)
  {
    Serial.println("Los cuatro hubs están configurados.");
  }
  else
  {
    Serial.println("ERROR: uno o más hubs no respondieron.");
  }

  return hubsConfigurados;
}


bool enviarVelocidadesHub(int16_t rpmIzq, int16_t rpmDer)
{
  bool ok = true;

  ok &= enviarRPMHub(ID_FL, rpmIzq * SIGN_FL);
  ok &= enviarRPMHub(ID_RL, rpmIzq * SIGN_RL);
  ok &= enviarRPMHub(ID_FR, rpmDer * SIGN_FR);
  ok &= enviarRPMHub(ID_RR, rpmDer * SIGN_RR);

  return ok;
}


void velocidadCeroHubs()
{
  if (!hubsConfigurados) return;

  enviarRPMHub(ID_FL, 0);
  enviarRPMHub(ID_RL, 0);
  enviarRPMHub(ID_FR, 0);
  enviarRPMHub(ID_RR, 0);

  ultimoRPMIzq = 0;
  ultimoRPMDer = 0;
}


void deshabilitarHubs()
{
  if (!hubsConfigurados) return;

  escribir16(ID_FL, 0x6040, 0x00, 0x0006);
  escribir16(ID_RL, 0x6040, 0x00, 0x0006);
  escribir16(ID_FR, 0x6040, 0x00, 0x0006);
  escribir16(ID_RR, 0x6040, 0x00, 0x0006);

  hubsConfigurados = false;
}


void emergenciaHubs()
{
  if (!hubsConfigurados) return;

  Serial.println("Parada de emergencia de los hubs.");

  velocidadCeroHubs();

  escribir8(ID_FL, 0x605A, 0x11, 1);
  escribir8(ID_RL, 0x605A, 0x11, 1);
  escribir8(ID_FR, 0x605A, 0x11, 1);
  escribir8(ID_RR, 0x605A, 0x11, 1);

  deshabilitarHubs();
}


// ============================================================
// IMU ADAFRUIT 10-DOF: LSM303DLHC + L3GD20 + BMP180
// ============================================================

bool superaUmbral(float actual, float referencia, float umbral)
{
  if (!isfinite(actual) || !isfinite(referencia)) return false;
  return fabsf(actual - referencia) >= umbral;
}


float corregirVelocidadGyro(float velocidadDps, float offsetDps)
{
  float corregida = velocidadDps - offsetDps;
  if (fabsf(corregida) <= GYRO_DEADZONE_DPS) return 0.0f;
  return corregida;
}


bool activarBDUGyro()
{
  // L3GD20 CTRL_REG4 (0x23): BDU=1 y FS=00 (±250 °/s).
  Wire.beginTransmission(GYRO_I2C_ADDRESS);
  Wire.write(0x23);
  Wire.write(0x80);
  return Wire.endTransmission() == 0;
}


bool iniciarIMU()
{
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000); // 100 kHz para compatibilidad estable con los tres sensores

  accelDisponible = imuAccel.begin(0x19, &Wire);
  gyroDisponible = imuGyro.begin(GYRO_RANGE_250DPS, &Wire);
  if (gyroDisponible)
  {
    if (activarBDUGyro())
      Serial.println("L3GD20 configurado a ±250 °/s con BDU activado.");
    else
      Serial.println("ADVERTENCIA: no se pudo confirmar BDU del L3GD20.");
  }
  if (accelDisponible)
  {
    imuAccel.setRange(LSM303_RANGE_8G);
    Serial.println("LSM303DLHC acelerómetro detectado en 0x19.");
  }
  else
  {
    Serial.println("ERROR: no se detectó acelerómetro LSM303DLHC en 0x19.");
  }

  if (gyroDisponible)
  {
    Serial.println("L3GD20 detectado por I2C (normalmente 0x6B).");
  }
  else
  {
    Serial.println("ERROR: no se detectó L3GD20. Verifica dirección/conexiones.");
  }

  // Roll/pitch se calculan con acelerómetro y giroscopio.
  imuDisponible = accelDisponible && gyroDisponible;
  return imuDisponible;
}


void actualizarIMU()
{
  if (millis() - ultimaLecturaIMU < 100) return;
  ultimaLecturaIMU = millis();
  angulosIMUValidos = false;

  sensors_event_t eventoAccel;
  sensors_event_t eventoGyro;

  bool accelOK = accelDisponible && imuAccel.getEvent(&eventoAccel);
  bool gyroOK = gyroDisponible && imuGyro.getEvent(&eventoGyro);

  static uint32_t microsAnterior = 0;
  static bool primeraLectura = true;
  uint32_t ahoraMicros = micros();
  float dt = 0.01f;

  if (microsAnterior != 0)
  {
    dt = (ahoraMicros - microsAnterior) / 1000000.0f;
    dt = constrain(dt, 0.001f, 0.1f);
  }
  microsAnterior = ahoraMicros;

  const float RAD_A_GRADOS = 180.0f / PI;
  // Con X lateral e Y longitudinal, el giro Y alimenta roll y el giro X pitch.
  float velocidadRollDps = gyroOK
      ? corregirVelocidadGyro(
            eventoGyro.gyro.y * RAD_A_GRADOS,
            GYRO_OFFSET_Y_DPS) * SIGNO_IMU_Y_FRENTE
      : 0.0f;
  float velocidadPitchDps = gyroOK
      ? corregirVelocidadGyro(
            eventoGyro.gyro.x * RAD_A_GRADOS,
            GYRO_OFFSET_X_DPS) * SIGNO_IMU_X_DERECHA
      : 0.0f;
  float velocidadRollRad = velocidadRollDps / RAD_A_GRADOS;
  float velocidadPitchRad = velocidadPitchDps / RAD_A_GRADOS;

  if (accelOK)
  {
    float accelDerecha =
        eventoAccel.acceleration.x * SIGNO_IMU_X_DERECHA;
    float accelFrente =
        eventoAccel.acceleration.y * SIGNO_IMU_Y_FRENTE;
    float accelArriba =
        eventoAccel.acceleration.z * SIGNO_IMU_Z_ARRIBA;

    // Roll positivo: lado derecho bajo. Pitch positivo: frente alto.
    float rollAccel = atan2f(
        accelDerecha,
        sqrtf(accelFrente * accelFrente + accelArriba * accelArriba));
    float pitchAccel = atan2f(
        accelFrente,
        sqrtf(accelDerecha * accelDerecha + accelArriba * accelArriba));

    if (primeraLectura)
    {
      inclinacionRoll = rollAccel;
      inclinacionPitch = pitchAccel;
      primeraLectura = false;
    }
    else if (gyroOK)
    {
      const float alpha = 0.98f;
      inclinacionRoll = alpha * (inclinacionRoll + velocidadRollRad * dt)
                      + (1.0f - alpha) * rollAccel;
      inclinacionPitch = alpha * (inclinacionPitch + velocidadPitchRad * dt)
                       + (1.0f - alpha) * pitchAccel;
    }
    else
    {
      inclinacionRoll = rollAccel;
      inclinacionPitch = pitchAccel;
    }

    float rollDeg = inclinacionRoll * RAD_A_GRADOS;
    float pitchDeg = inclinacionPitch * RAD_A_GRADOS;
    rollGradosActual = rollDeg;
    pitchGradosActual = pitchDeg;
    angulosIMUValidos = true;

    EstadoIMUReferencia actual = {
      rollDeg,
      pitchDeg
    };

    bool imprimirIMU = false;

    if (!referenciaIMUValida)
    {
      referenciaIMU = actual;
      referenciaIMUValida = true;
      imprimirIMU = true;
    }
    else
    {
      // Solo roll y pitch activan una nueva línea de telemetría.
      bool cambioSostenido = false;
      cambioSostenido |= superaUmbral(actual.roll, referenciaIMU.roll, 2.0f);
      cambioSostenido |= superaUmbral(actual.pitch, referenciaIMU.pitch, 2.0f);

      if (cambioSostenido)
      {
        if (muestrasCambioIMU < MUESTRAS_CONFIRMAR_CAMBIO_IMU)
        {
          muestrasCambioIMU++;
        }
      }
      else
      {
        muestrasCambioIMU = 0;
      }

      if (muestrasCambioIMU >= MUESTRAS_CONFIRMAR_CAMBIO_IMU)
      {
        referenciaIMU = actual;
        muestrasCambioIMU = 0;
        imprimirIMU = true;
      }
    }

    // Formato: IMU,roll_deg,pitch_deg
    if (imprimirIMU)
    {
      Serial.print("IMU,");
      Serial.print(rollDeg, 1); Serial.print(',');
      Serial.println(pitchDeg, 1);
    }
  }
}


// ============================================================
// ULTRASONIDOS HC-SR04
// ============================================================

long medirDistanciaCM(uint8_t pinTrig, uint8_t pinEcho)
{
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);

  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  unsigned long duracion =
      pulseIn(pinEcho, HIGH, TIMEOUT_ECHO_US);

  if (duracion == 0)
  {
    return 999;
  }

  return (long)(duracion / 58);
}


void actualizarUltrasonicos()
{
  if (modoActual != MODO_DESPLAZAMIENTO) return;

  if (millis() - ultimoUltrasonico < PERIODO_LECTURA_US_MS)
  {
    return;
  }

  ultimoUltrasonico = millis();

  if (leerSensor1Ahora)
  {
    distanciaUS1 = medirDistanciaCM(TRIG_US1, ECHO_US1);
    sensoresLeidosMask |= 0b01;
  }
  else
  {
    distanciaUS2 = medirDistanciaCM(TRIG_US2, ECHO_US2);
    sensoresLeidosMask |= 0b10;
  }

  leerSensor1Ahora = !leerSensor1Ahora;
  sensoresListos = (sensoresLeidosMask == 0b11);
}


bool obstaculoDetectado()
{
  return (distanciaUS1 <= DISTANCIA_PARADA_CM ||
          distanciaUS2 <= DISTANCIA_PARADA_CM);
}


void paradaEmergencia(const char *motivo)
{
  Serial.println();
  Serial.println("***** PARADA DE EMERGENCIA *****");
  Serial.print("Motivo: ");
  Serial.println(motivo);

  detenerSteadyWin();
  emergenciaHubs();

  sistemaArmado = false;
  modoActual = MODO_NINGUNO;

  Serial.println("Motores detenidos. Selecciona un modo y vuelve a armar.");
}


// ============================================================
// INICIALIZACIÓN CAN Y MODOS
// ============================================================

bool iniciarCAN()
{
  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(
          CAN_TX,
          CAN_RX,
          TWAI_MODE_NORMAL
      );

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filtro = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t resultado =
      twai_driver_install(&general, &timing, &filtro);

  if (resultado != ESP_OK)
  {
    Serial.print("Error instalando TWAI: ");
    Serial.println(esp_err_to_name(resultado));
    return false;
  }

  resultado = twai_start();

  if (resultado != ESP_OK)
  {
    Serial.print("Error iniciando TWAI: ");
    Serial.println(esp_err_to_name(resultado));
    return false;
  }

  return true;
}


void detenerTodo()
{
  detenerSteadyWin();
  velocidadCeroHubs();
}


void seleccionarModo(ModoControl nuevoModo)
{
  detenerTodo();
  sistemaArmado = false;
  modoActual = nuevoModo;

  if (nuevoModo == MODO_ALTURA)
  {
    Serial.println("Modo ALTURA seleccionado.");
    Serial.println("Configura el recorrido con dos capturas de flecha IZQUIERDA.");
    Serial.println("1) En el límite bajo, desarmado, pulsa flecha IZQUIERDA.");
    Serial.println("2) Arma y sube con R2 + stick; desarma en la altura máxima.");
    Serial.println("3) Pulsa flecha IZQUIERDA otra vez para guardar el límite alto.");
  }
  else if (nuevoModo == MODO_DESPLAZAMIENTO)
  {
    sensoresListos = false;
    sensoresLeidosMask = 0;
    distanciaUS1 = 999;
    distanciaUS2 = 999;
    leerSensor1Ahora = true;
    ultimoUltrasonico = 0;

    Serial.println("Modo DESPLAZAMIENTO seleccionado.");
    Serial.println("Pulsa X para configurar/armar los hubs.");
    Serial.println("Esperando lecturas de ambos ultrasonidos.");
  }
}


void armarSistema()
{
  if (modoActual == MODO_NINGUNO)
  {
    Serial.println("Selecciona primero modo altura (triángulo) o desplazamiento (cuadrado).");
    return;
  }

  if (modoActual == MODO_DESPLAZAMIENTO)
  {
    if (!configurarTodosLosHubs())
    {
      sistemaArmado = false;
      return;
    }
  }

  sistemaArmado = true;
  Serial.println("Sistema ARMADO. Mantén R2 para mover.");
}


void desarmarSistema()
{
  detenerTodo();
  sistemaArmado = false;
  Serial.println("Sistema DESARMADO.");
}


// ============================================================
// CONTROL DE ALTURA
// ============================================================

int16_t velocidadCorreccionAngulo(int8_t direccion)
{
  return (direccion != 0)
      ? RPM_CORRECCION_NIVELACION
      : 0;
}


int8_t actualizarDireccionCorreccion(float angulo, int8_t direccionAnterior)
{
  if (direccionAnterior > 0 && angulo > UMBRAL_LIBERAR_NIVELACION_GRADOS)
    return +1;

  if (direccionAnterior < 0 && angulo < -UMBRAL_LIBERAR_NIVELACION_GRADOS)
    return -1;

  if (angulo > UMBRAL_NIVELACION_GRADOS) return +1;
  if (angulo < -UMBRAL_NIVELACION_GRADOS) return -1;

  return 0;
}


void controlarAltura()
{
  if (millis() - ultimoEnvioControl < PERIODO_ENVIO_MS) return;
  ultimoEnvioControl = millis();

  int16_t rpmLogicos = 0;

  if (sistemaArmado && ps5.r2 >= DEADMAN_R2)
  {
    if (ps5.ly < -DEADZONE)
    {
      rpmLogicos = RPM_ALTURA;
    }
    else if (ps5.ly > DEADZONE)
    {
      rpmLogicos = -RPM_ALTURA;
    }
  }

  // Ejes del IMU: X lateral, Y longitudinal y Z vertical.
  // Con +X a la derecha, +Y al frente y +Z arriba:
  // roll positivo = lado derecho bajo; pitch positivo = frente alto.
  // Para nivelar, se eleva el lado o extremo que queda bajo, a 1 RPM.
  int16_t corr11 = 0;
  int16_t corr12 = 0;
  int16_t corr13 = 0;
  int16_t corr14 = 0;

  if (sistemaArmado && angulosIMUValidos && etapaLimitesRecorrido == 2)
  {
    float rollControl = rollGradosActual * SIGNO_CORRECCION_ROLL;
    float pitchControl = pitchGradosActual * SIGNO_CORRECCION_PITCH;

    direccionCorreccionRoll =
        actualizarDireccionCorreccion(rollControl, direccionCorreccionRoll);
    direccionCorreccionPitch =
        actualizarDireccionCorreccion(pitchControl, direccionCorreccionPitch);

    int16_t corrRoll = velocidadCorreccionAngulo(direccionCorreccionRoll);
    if (direccionCorreccionRoll > 0)
    {
      // Izquierda alta: subir el lado derecho (13 frontal, 14 trasero).
      corr13 += corrRoll;
      corr14 += corrRoll;
    }
    else if (direccionCorreccionRoll < 0)
    {
      // Derecha alta: subir el lado izquierdo (11 frontal, 12 trasero).
      corr11 += corrRoll;
      corr12 += corrRoll;
    }

    int16_t corrPitch = velocidadCorreccionAngulo(direccionCorreccionPitch);
    if (direccionCorreccionPitch > 0)
    {
      // Frente alto: subir la parte trasera (12 y 14).
      corr12 += corrPitch;
      corr14 += corrPitch;
    }
    else if (direccionCorreccionPitch < 0)
    {
      // Parte trasera alta: subir el frente (11 y 13).
      corr11 += corrPitch;
      corr13 += corrPitch;
    }
  }
  else
  {
    direccionCorreccionRoll = 0;
    direccionCorreccionPitch = 0;
  }

  // El recorrido capturado limita el comando neto: manual más nivelación.
  // Durante la captura del límite superior, la nivelación automática está apagada.
  int16_t rpm11 = limitarMovimientoSteadyPorRecorrido(
      0, constrain(rpmLogicos + corr11, -4, 4));
  int16_t rpm12 = limitarMovimientoSteadyPorRecorrido(
      1, constrain(rpmLogicos + corr12, -4, 4));
  int16_t rpm13 = limitarMovimientoSteadyPorRecorrido(
      2, constrain(rpmLogicos + corr13, -4, 4));
  int16_t rpm14 = limitarMovimientoSteadyPorRecorrido(
      3, constrain(rpmLogicos + corr14, -4, 4));

  bool hayComando = rpm11 || rpm12 || rpm13 || rpm14;
  bool motoresEstabanMoviendose =
      ultimoRPMSteady11 || ultimoRPMSteady12 ||
      ultimoRPMSteady13 || ultimoRPMSteady14;

  // En reposo envía cero una sola vez. Si hay corrección o movimiento manual,
  // refresca la referencia cada 50 ms.
  if (hayComando || motoresEstabanMoviendose)
  {
    if (!moverSteadyWinIndividual(rpm11, rpm12, rpm13, rpm14))
    {
      Serial.println("Error TX SteadyWin. Deteniendo y desarmando.");
      detenerSteadyWin();
      sistemaArmado = false;
      return;
    }
  }

  ultimoRPMAltura = rpmLogicos;

  bool nivelacionActiva =
      (corr11 || corr12 || corr13 || corr14) &&
      (rpm11 != rpmLogicos || rpm12 != rpmLogicos ||
       rpm13 != rpmLogicos || rpm14 != rpmLogicos);
  if (nivelacionActiva && !nivelacionActivaAnterior)
  {
    Serial.print("Nivelación lenta | Roll: ");
    Serial.print(rollGradosActual, 1);
    Serial.print("° | Pitch: ");
    Serial.print(pitchGradosActual, 1);
    Serial.println("°");
  }
  else if (!nivelacionActiva && nivelacionActivaAnterior)
  {
    Serial.println("Nivelación alcanzada; corrección detenida.");
  }
  nivelacionActivaAnterior = nivelacionActiva;
}


// ============================================================
// CONTROL DE DESPLAZAMIENTO
// ============================================================

int ejeCurvado(int valor)
{
  valor = constrain(valor, -127, 127);

  int magnitud = abs(valor);

  if (magnitud <= DEADZONE) return 0;

  int normalizado =
      (magnitud - DEADZONE) * 127
      / (127 - DEADZONE);

  int curvado = normalizado * normalizado / 127;

  return (valor < 0) ? -curvado : curvado;
}


void ajustarLimiteRPMHub()
{
  if (modoActual != MODO_DESPLAZAMIENTO) return;
  if (ps5.r2 >= DEADMAN_R2) return;
  if (abs((int)ps5.ly) > DEADZONE) return;

  if (ps5.up.pressed)
  {
    velocidadMaximaHub += PASO_RPM_HUB;

    if (velocidadMaximaHub > RPM_HUB_MAXIMO)
    {
      velocidadMaximaHub = RPM_HUB_MAXIMO;
    }

    Serial.print("Límite hubs: ");
    Serial.print(velocidadMaximaHub);
    Serial.println(" RPM");
  }

  if (ps5.down.pressed)
  {
    velocidadMaximaHub -= PASO_RPM_HUB;

    if (velocidadMaximaHub < RPM_HUB_MINIMO)
    {
      velocidadMaximaHub = RPM_HUB_MINIMO;
    }

    Serial.print("Límite hubs: ");
    Serial.print(velocidadMaximaHub);
    Serial.println(" RPM");
  }
}


void controlarDesplazamiento()
{
  if (!sensoresListos) return;

  if (millis() - ultimoEnvioControl < PERIODO_ENVIO_MS) return;
  ultimoEnvioControl = millis();

  if (!sistemaArmado || ps5.r2 < DEADMAN_R2)
  {
    if (ultimoRPMIzq != 0 || ultimoRPMDer != 0)
    {
      velocidadCeroHubs();
      Serial.println("R2 liberado: hubs a 0 RPM.");
    }
    return;
  }

  int acelerador = ejeCurvado(-ps5.ly);
  int giro = ejeCurvado(ps5.lx);

  int izquierda = acelerador + giro;
  int derecha = acelerador - giro;

  int maxValor = max(abs(izquierda), abs(derecha));

  if (maxValor > 127)
  {
    izquierda = izquierda * 127 / maxValor;
    derecha = derecha * 127 / maxValor;
  }

  int16_t rpmIzq =
      izquierda * velocidadMaximaHub / 127;

  int16_t rpmDer =
      derecha * velocidadMaximaHub / 127;

  rpmIzq = constrain(
      rpmIzq,
      -velocidadMaximaHub,
      velocidadMaximaHub
  );

  rpmDer = constrain(
      rpmDer,
      -velocidadMaximaHub,
      velocidadMaximaHub
  );

  if (!enviarVelocidadesHub(rpmIzq, rpmDer))
  {
    Serial.println("Error CANopen en los hubs. Deteniendo y desarmando.");

    velocidadCeroHubs();
    sistemaArmado = false;
    return;
  }

  if (rpmIzq != ultimoRPMIzq || rpmDer != ultimoRPMDer)
  {
    Serial.print("Hubs | Izquierda: ");
    Serial.print(rpmIzq);
    Serial.print(" RPM | Derecha: ");
    Serial.print(rpmDer);
    Serial.println(" RPM");
  }

  ultimoRPMIzq = rpmIzq;
  ultimoRPMDer = rpmDer;
}


// ============================================================
// SETUP Y LOOP
// ============================================================

void setup()
{
  serialUSB.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("DualSense + SteadyWin + hubs + ultrasonidos + Adafruit 10-DOF");
  Serial.println("CAN 1 Mbps | TX GPIO17 | RX GPIO16");
  Serial.println("Triángulo: modo altura");
  Serial.println("Cuadrado: modo desplazamiento");
  Serial.println("X: armar/desarmar");
  Serial.println("R2: mantener para mover");
  Serial.println("Círculo: parada de emergencia");
  Serial.println("Altura: flecha izquierda guarda límite bajo y luego límite alto.");
  Serial.println("Ultrasonidos: GPIO25/26 y GPIO32/33; parada a 45 cm");
  Serial.println("IMU Adafruit 10-DOF: SDA GPIO21, SCL GPIO22");
  Serial.println("HMI BLE directa: Robot_Agricola_HMI; no requiere ESP32-C3.");

  pinMode(TRIG_US1, OUTPUT);
  pinMode(ECHO_US1, INPUT);
  pinMode(TRIG_US2, OUTPUT);
  pinMode(ECHO_US2, INPUT);

  digitalWrite(TRIG_US1, LOW);
  digitalWrite(TRIG_US2, LOW);

  imuDisponible = iniciarIMU();

  if (!iniciarCAN())
  {
    Serial.println("ERROR fatal iniciando CAN.");

    while (true)
    {
      delay(1000);
    }
  }

  ps5.begin(30);
  iniciarBLE();
  Serial.println("CAN iniciado. Esperando DualSense y HMI BLE...");
}


void loop()
{
  leerRespuestasSteadyWin();
  actualizarIMU();
  actualizarSalidaBLE();

  bool conectado = ps5.isConnected();

  if (conectado && !dualSenseConectadoAnterior)
  {
    Serial.println("DualSense conectado.");
  }

  if (!conectado && dualSenseConectadoAnterior)
  {
    Serial.println("DualSense desconectado: parada.");

    detenerSteadyWin();
    emergenciaHubs();

    sistemaArmado = false;
    modoActual = MODO_NINGUNO;
  }

  dualSenseConectadoAnterior = conectado;

  if (!conectado)
  {
    delay(10);
    return;
  }

  // Selección de modo solo con R2 suelto.
  if (ps5.r2 < DEADMAN_R2)
  {
    if (ps5.triangle.pressed)
    {
      seleccionarModo(MODO_ALTURA);
    }
    else if (ps5.square.pressed)
    {
      seleccionarModo(MODO_DESPLAZAMIENTO);
    }
  }

  // Flecha izquierda captura un extremo del recorrido, siempre desarmado.
  if (modoActual == MODO_ALTURA &&
      !sistemaArmado &&
      ps5.r2 < DEADMAN_R2 &&
      ps5.left.pressed)
  {
    calibrarReferenciaNivelacionSteady();
  }

  // X arma o desarma el modo seleccionado.
  if (ps5.cross.pressed)
  {
    if (sistemaArmado)
    {
      desarmarSistema();
    }
    else
    {
      armarSistema();
    }
  }

  ajustarLimiteRPMHub();

  // El botón círculo y los ultrasonidos usan la misma parada.
  if (ps5.circle.pressed)
  {
    paradaEmergencia("botón círculo");
    delay(200);
    return;
  }

  actualizarUltrasonicos();
  actualizarConsultaPosicionesSteady();

  if (modoActual == MODO_DESPLAZAMIENTO &&
      sistemaArmado &&
      ps5.r2 >= DEADMAN_R2 &&
      sensoresListos &&
      obstaculoDetectado())
  {
    Serial.print("Obstáculo detectado. Sensor 1: ");
    Serial.print(distanciaUS1);
    Serial.print(" cm | Sensor 2: ");
    Serial.print(distanciaUS2);
    Serial.println(" cm");

    paradaEmergencia("obstáculo detectado por ultrasonido");
    delay(200);
    return;
  }

  if (modoActual == MODO_ALTURA)
  {
    controlarAltura();
  }
  else if (modoActual == MODO_DESPLAZAMIENTO)
  {
    controlarDesplazamiento();
  }

  delay(2);
}
void imprimirEstadoTWAI()
{
  twai_status_info_t estado = {};

  esp_err_t resultado = twai_get_status_info(&estado);

  if (resultado != ESP_OK)
  {
    Serial.print("No se pudo leer el estado TWAI: ");
    Serial.println(esp_err_to_name(resultado));
    return;
  }

  Serial.print("TWAI | estado numérico: ");
  Serial.print((int)estado.state);
  Serial.print(" | TEC: ");
  Serial.print(estado.tx_error_counter);
  Serial.print(" | REC: ");
  Serial.print(estado.rx_error_counter);
  Serial.print(" | errores de bus: ");
  Serial.println(estado.bus_error_count);
}
