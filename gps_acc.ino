#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <FirebaseESP32.h>
#include <WiFi.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Credenciales de WiFi
#define WIFI_SSID "fran"
#define WIFI_PASSWORD "12345678"

// Credenciales de Firebase
#define FIREBASE_HOST "gps-acc-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "kTLrvN5lKsVJNI4NcLZYsw6CkErRDVPTPBMoOz2c"

// Inicialización del objeto Firebase
FirebaseData firebaseData;

// Declaración de objetos FirebaseConfig y FirebaseAuth
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;

// Flag para indicar si un bache ha sido detectado
bool bacheDetectado = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  // Inicialización del sensor MPU6050
  if (!mpu.begin()) {
    Serial.println("No se pudo encontrar MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Inicialización de la conexión WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado a la red WiFi: ");
  Serial.println(WiFi.SSID());

  // Inicialización de Firebase
  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&firebaseConfig, &firebaseAuth);
}

void loop() {
  // Leer datos del MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Detectar cambios bruscos en la aceleración
  if (abs(a.acceleration.x) > 3 || abs(a.acceleration.y) > 3 || abs(a.acceleration.z) > 20) {
    if (!bacheDetectado) {
      Serial.println("Bache detectado");
      
      // Obtener datos GPS
      while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
      }

      // Crear un identificador único para cada evento
      String path = "/baches/" + String(millis());

      if (gps.location.isValid()) {
        Serial.print("Latitud: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitud: ");
        Serial.println(gps.location.lng(), 6);

        Firebase.setFloat(firebaseData, path + "/lat", gps.location.lat());
        Firebase.setFloat(firebaseData, path + "/lng", gps.location.lng());
      }

    // Obtener fecha y hora del GPS
    if (gps.date.isValid() && gps.time.isValid()) {
      char dateTime[30];
      sprintf(dateTime, "%02d-%02d-%02d %02d:%02d:%02d", 
              gps.date.year(), gps.date.month(), gps.date.day(),
              gps.time.hour(), gps.time.minute(), gps.time.second());
      Serial.print("Fecha y hora: ");
      Serial.println(dateTime);

      // Enviar fecha y hora a Firebase
      Firebase.setString(firebaseData, path + "/fecha_hora", dateTime);
    } else {
      Serial.println("Fecha y hora no disponibles");
    }
      // Enviar datos de aceleración a Firebase
      if (Firebase.setFloat(firebaseData, path + "/x", abs(a.acceleration.x)) &&
          Firebase.setFloat(firebaseData, path + "/y", abs(a.acceleration.y)) &&
          Firebase.setFloat(firebaseData, path + "/z", abs(a.acceleration.z))) {
        Serial.println("Datos enviados a Firebase correctamente");
      } else {
        Serial.println("Error al enviar datos a Firebase");
        Serial.println(firebaseData.errorReason());
      }

      // Marcar que se ha detectado un bache y enviar datos
      bacheDetectado = true;
    }
  } else {
    // Reiniciar el flag si no se detecta un bache
    bacheDetectado = false;
  }

  // Esperar un tiempo antes de la próxima lectura
  delay(500); // Ajusta el delay según la necesidad
}