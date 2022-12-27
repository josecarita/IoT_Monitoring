// I2Cdev y MPU6050 instaladas como librerías para el giroscopio.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// la librería Wire es requerida si se usa el implemento I2CDEV_ARDUINO_WIRE de I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// por defecto la dirección I2C del MPU6050 es 0x68
MPU6050 mpu;

// "OUTPUT_READABLE_YAWPITCHROLL" es un implemento que permite ver 
// los ángulos de movimiento de cada eje (yaw, pitch, roll)en grados 
// sexagesimales calculados previamente en la librería. 
// Notar que esto requiere cálculos de vectores de gravedad.
#define OUTPUT_READABLE_YAWPITCHROLL

// variable de estado y control del MPU6050
bool dmpReady = false;  // se setea en true cuando el DMP inicia 
uint8_t mpuIntStatus;   // mantiene el byte de estado de interrupción actual del MPU
uint8_t devStatus;      // retorna el estado después de la operación de cada dispositivo (0 = exitoso, !0 = errado)
uint16_t packetSize;    // tamaño de paquete de DMP esperado (por defecto es 42 bytes)
uint16_t fifoCount;     // cuenta de todos los bytes actuales en FIFO
uint8_t fifoBuffer[64]; // buffer de almacenamiento FIFO

// variables de orientación y movimiento
Quaternion q;           // [w, x, y, z]         contenedor de cuaternión
VectorInt16 aa;         // [x, y, z]            medidas del sensor de aceleración
VectorInt16 aaReal;     // [x, y, z]            medidas del sensor de aceleración gravity-free
VectorInt16 aaWorld;    // [x, y, z]            medidas del sensor de aceleración world-frame
VectorFloat gravity;    // [x, y, z]            vector de gravedad
float euler[3];         // [psi, theta, phi]    contenedor de ángulos de Euler
float ypr[3];           // [yaw, pitch, roll]   contenedor de yaw/pitch/roll y vector de gravedad

// estructura de paquete para InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



//rutina de detección de interrupción
volatile bool mpuInterrupt = false; // indica cuando el pin de interrupción del MPU cambió a estado HIGH
void dmpDataReady() {
    mpuInterrupt = true;
}


//Variables del sensor de temperatura
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();


//Variables del oxímetro
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) //identifica el tipo de microcontrolador de la placa
uint16_t irBuffer[100]; //datos de sensor LED infrarrojo
uint16_t redBuffer[100];  //datos de sensor LED rojo
#else
uint32_t irBuffer[100]; //datos de sensor LED infrarrojo
uint32_t redBuffer[100];  //datos de sensor LED rojo
#endif

int32_t bufferLength; //longitud de datos
int32_t spo2; //valor SPO2
int8_t validSPO2; //indicador de si el cálculo de SPO2 es válido
int32_t heartRate; //valor de ritmo cardiaco
int8_t validHeartRate; //indicador de si el cálculo de ritmo cardiaco es válido


//Variables para el alcoholímetro    
float valor_alcohol; //lectura del sensor
float porcentaje; //ecuación para transformar a g/L de sangre


//Variables del sensor de presión diferencial

const int numReadings = 100; // Cantidad de lecturas para sacar el promedio

int index             = 0;   // El indice de la lectura actual 
float readings[numReadings]; // Lecturas de la entrada analógica 
float total           = 0.0; // Total 
float presion         = 0.0; // Promedio 


int lectura = 0;    //variable de confimación de envío de datos


// ================================================================
// ===                         VOID SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(115200); // inicia la comunicación serial a 115200 bits por segundo

  // inicia oxímetro
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Usa el puerto I2C por defecto, 400kHz de velocidad
  {
    Serial.println(F("MAX30105 no fue encontrado. Chequear las conexiones."));
    while (1);
  }

  Serial.read();

  byte ledBrightness = 60; //Opciones: 0=Off to 255=50mA
  byte sampleAverage = 4; //Opciones: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Opciones: 1 = Solo Rojo, 2 = Rojo + Infrarrojo, 3 = Rojo + Infrarrojo + Verde
  byte sampleRate = 100; //Opciones: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Opciones: 69, 118, 215, 411
  int adcRange = 4096; //Opciones: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //configuración del sensor

///////////////////////////////////////////////////////////////////////////////////

  // se une al Bus I2C (la librería I2Cdev no lo hace automáticamente)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz reloj I2C 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // inicia dispositivo
    Serial.println(F("Iniciando dispositivos I2C..."));
    mpu.initialize();

    // verifica conexión
    Serial.println(F("Comprobando conexión de dispositivos..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 conexión exitosa") : F("MPU6050 conexión fallida"));

    // carga y configura el DMP
    Serial.println(F("Iniciando DMP..."));
    devStatus = mpu.dmpInitialize();

    // calibración del offset
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // se asegura de que funcione (si es asi retorna un 0)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Habilitando DMP..."));
        mpu.setDMPEnabled(true);

        // habilita Arduino detección de interrupción
        Serial.println(F("Habilitando detección de interrupción..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // setea DMP como listo así el main loop() sabrá que se puede usar
        Serial.println(F("DMP listo! esperando primera interrupción..."));
        dmpReady = true;

        // obtiene los datos del DMP para las futuras comparaciones
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = falla de carga de memoria inicial
        // 2 = falla de configuración de actualizaciones del DMP
        // (si esta iniciando, usualmente el código será 1)
        Serial.print(F("Falla en iniciación de DMP (código "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
//Inicia el Sensor de temperatura
  mlx.begin(); 

}

void loop()
{
  bufferLength = 100; // el buffer almacena 100 muestras 

  // toma muestras del MAX30102 continuamente.
  while (1)
  {
    //vierte los primeros 25 valores en la memoria y cambia de posicion las ultimas 75 hacia adelante
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
      ecg(); //lectura ecg por bucle
    }

    //toma las 25 muestras calculando el ritmo cardiaco
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //hay nuevos datos?
        particleSensor.check(); //revisa el sensor por nuevos datos
        ecg(); //lectura ecg por bucle



      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // Terminó con estas muestras asi que pasa a las siguientes

      //envía las muestras a la terminal
      /*Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);*/
    }

     //después de reunir 25 nuevas muestras recalcula el ritmo cardiaco y el SPO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
//////////////////////////////////////////////////
      giroscopio(); // ejecuta el código para el giroscopio
      ecg(); // ejecuta el código para el ECG
      alcohol(); // ejecuta el código para el alcoholímetro
      presionpulmonar(); // ejecuta el código para la presión pulmonar
      if(lectura=100){
      //giroscopio
      //Serial.print("Giroscopio:");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print(" ");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print(" ");
      Serial.print(ypr[2] * 180/M_PI);
      Serial.print(",");
      //temperatura
      //Serial.print("Temperatura:"); 
      Serial.print(mlx.readObjectTempC()); 
      //Serial.print("C");
      Serial.print(",");
      //oximetro
      //Serial.print(F("SPO2:"));
      Serial.print(spo2, DEC);
      Serial.print(",");
      //alcoholimetro
      //Serial.print("Alcohol:");
      Serial.print(porcentaje);      
      //Serial.print("g/L"); 
      Serial.print(",");
      //presion
      //Serial.print("Presión pulmonar:");
      Serial.print(presion); 
      //Serial.print("mmHg");
      Serial.print(",");
      Serial.println();
      lectura=0;
      }
    

    }
}

void giroscopio() {

    if (!dmpReady) return;

    // espera por una interrupción del MPU o un paquete extra disponible
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // Resetea la interrupción y obtiene el byte INT_STATUS
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if (mpuIntStatus & 0x02) {
        // espera por la longitud correcta de datos disponibles
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // lee un paquete FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // muestra ángulos de Euler
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif
        
    }
  lectura=lectura++; //variable de envío de datos aumenta en 1
}

void alcohol() {

  valor_alcohol=analogRead(A8);
  //Serial.println(valor_alcohol);       // Envia al Serial el valor leido del Sensor MQ3 
  porcentaje=(float(valor_alcohol)/1000);  //calcula el valor en gramos por litro de sangre
  
}

void presionpulmonar() {

total = total - readings[index]; 
  readings[index] = analogRead(A10);  // Lee el sensor
  total = total + readings[index]; // Añade la lectura al total
  index = index + 1; // Avanza a la proxima posicion del array 
  
  if (index >= numReadings) // Si esta en el final del array
     index = 0;   // vuelve al inicio
     
  // Calcula el promedio y aplica una ecuación para transformar los datos obtenidos a mmHg
  presion = 0.075*(total / numReadings) - 25.50; 
}
