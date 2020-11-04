/*                                                                                                                
  ,ad8888ba,                88           88              88b           d88                                        
 d8"'    `"8b               88           ""    ,d        888b         d888                                        
d8'        `8b              88                 88        88`8b       d8'88                                        
88          88  8b,dPPYba,  88,dPPYba,   88  MM88MMM     88 `8b     d8' 88   ,adPPYba,   8b       d8   ,adPPYba,  
88          88  88P'   "Y8  88P'    "8a  88    88        88  `8b   d8'  88  a8"     "8a  `8b     d8'  a8P_____88  
Y8,        ,8P  88          88       d8  88    88        88   `8b d8'   88  8b       d8   `8b   d8'   8PP"""""""  
 Y8a.    .a8P   88          88b,   ,a8"  88    88,       88    `888'    88  "8a,   ,a8"    `8b,d8'    "8b,   ,aa  
  `"Y8888Y"'    88          8Y"Ybbd8"'   88    "Y888     88     `8'     88   `"YbbdP"'       "8"       `"Ybbd8"'  
*/                                                                                                                
//********************************************************************************************************************************                                                                                                                  
/*
* Proyecto final para la clase de interconectividad de dispositivos
* Maestra: Dra. Diana Argarita Cordova Esparza
* 
* Integrantes del equipo:
* José Luis Peña Rivera
* Jaime Rodrigo González Rodríguez
* 
* PROGAMA DE CONTROL DEL PUENTE H POR MEDIO DE UN RECEPTRO NRF24L10
*/
//********************************************************************************************************************************                                                                                                                  
//=================================================== Configuracion Giroscopio ====================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//=================================================== Configuracion Transmisor ====================================================
#include <SPI.h>
#include <RF24.h>

const int velocidad = 50;
 
RF24 myRadio (7, 8); // in Mega can use> (48, 49); 
byte addresses[][6] = {"0"}; 

struct package
{
  int id = 0;
  /*
   * CLAVE DE CONTROL
   * 0 = NADA
   * 1 = AVANZAR
   * 2 = RETROCEDER
   * 3 = DERECHA
   * 4 = IZQUIERDA
  */
  int clave = 0;
  
};

typedef struct package Package;
Package data;
//=================================================== Configuracion LCD ====================================================
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//I2C pins declaration
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

//=====================================================PINES SWITCH==============================================
int accion1 = 3;
int accion2 = 6;

//=====================================================PINES JOYSTICKS==============================================
//Botones de control
int potenciometro = A0; //joyIzq
int potenciometro2 = A2; //joyDer

int brillo = 0;
int brillo2 = 0;

void setup() {

lcd.begin(16,2);//Defining 16 columns and 2 rows of lcd display
lcd.backlight();//To Power ON the back light

//=====================================================CONFIGURACION PINES SWITCH============================
  pinMode(accion1, INPUT_PULLUP); 
  pinMode(accion2, INPUT_PULLUP);

//===============================CONFIGURACION TRANSMISOR ===================================

  delay(velocidad);
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS ) ; 
  myRadio.openWritingPipe( addresses[0]);
  delay(velocidad);

//===============================CONFIGURACION GIROSCOPIO ===================================  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {

  //ALMACENAMOS EL ESTADO DEL SWITCH
  int state1 = digitalRead(accion1);
  int state2 = digitalRead(accion2);

  //INICIO DE COMUNICACION
  myRadio.write(&data, sizeof(data)); 

//CONTROL POR MEDIO DEL GIROSCOPIO
  if( state1 == LOW && state2 == HIGH){

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   Giroscopio   ");
    lcd.setCursor(0,1);
    lcd.print("  Ultra crikoso ");

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);

            //LECTURA DE YAW PITCH Y ROW DEL GIROSCOPIO
            if((ypr[1]* 180/M_PI) > 10 && (ypr[1]* 180/M_PI) > 10 && (ypr[2]* 180/M_PI) < -10){
              
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("   Giroscopio   ");
                  lcd.setCursor(0,1);
                  lcd.print("    Adelante    ");

                  Serial.print("\nAVANZAR -> clave: "); //Ok
                  Serial.print(data.clave);
                  Serial.print("\n");
                  data.clave = 1;

            }else if((ypr[0]* 180/M_PI) > 50 && (ypr[1]* 180/M_PI) < 20 && (ypr[2]* 180/M_PI) > 10){
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("   Giroscopio   ");
                  lcd.setCursor(0,1);
                  lcd.print("   Retroceder   ");
                  
                  Serial.print("\nRETROCEDER -> clave: "); //OK
                  Serial.print(data.clave);
                  Serial.print("\n");
                  data.clave = 2;

                  
            }else if((ypr[0]* 180/M_PI) < 0 && (ypr[1]* 180/M_PI) < 10 && (ypr[2]* 180/M_PI) < 10){
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("   Giroscopio   ");
                  lcd.setCursor(0,1);
                  lcd.print("     Derecha");
                  
                  Serial.print("\nDERECHA -> clave: "); //SOSO
                  Serial.print(data.clave);
                  Serial.print("\n");
                  data.clave = 3;
              
            }else if((ypr[0]* 180/M_PI) < 20 && (ypr[1]* 180/M_PI) <  20 && (ypr[1]* 180/M_PI) < 20){
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("   Giroscopio   ");
                  lcd.setCursor(0,1);
                  lcd.print("    Izquierda ");
                  
                  Serial.print("\nIZQUIERDA -> clave: "); //SOSO
                  Serial.print(data.clave);
                  Serial.print("\n");
                  data.clave = 4;
                  
            }else{

                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("   Giroscopio   ");
                lcd.setCursor(0,1);
                lcd.print("  Ultra crikoso ");
                
                Serial.print("\nNADA -> clave: ");
                Serial.print(data.clave);
                Serial.print("\n");
                data.clave = 0;
            }

          
        #endif        

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        
    }

    data.id = data.id + 1;
    delay(velocidad);

//CONTROL POR MEDIO DE LOS JOYSTICKS    
  }else if( state1 == HIGH && state2 == LOW){

        lcd.clear();
        lcd.print("    Joystick    ");
        lcd.setCursor(0,1);
        lcd.print("  Precision 99% ");
        
        brillo = analogRead(potenciometro);
        brillo = map(brillo,0,1024,0,255);
      
        brillo2 = analogRead(potenciometro2);
        brillo2 = map(brillo2,0,1024,0,255);
      
        //Avanzar
        if(brillo >= 230 && brillo2 >= 230){
          lcd.clear();
          lcd.print("    Joystick    ");
          lcd.setCursor(0,1);
          lcd.print("    Adelante    ");
          data.clave = 1;
      
        //Retroceder
        }else if(brillo <= 10 && brillo2 <= 10){
          lcd.clear();
          lcd.print("    Joystick    ");
          lcd.setCursor(0,1);
          lcd.print("   Retroceder   ");
          data.clave = 2;
      
        //Derecha
        }else if(brillo >= 230 && brillo2 <= 10){
          lcd.clear();
          lcd.print("    Joystick    ");
          lcd.setCursor(0,1);
          lcd.print("     Derecha");
          data.clave = 3;
      
        //Izquierda
        }else if(brillo <= 10 && brillo2 >= 230){
          lcd.clear();
          lcd.print("    Joystick    ");
          lcd.setCursor(0,1);
          lcd.print("    Izquierda ");
          data.clave = 4;
      
        //Nada y errores
        }else{
      
          data.clave = 0;
              
        }
        
        Serial.print("\nPackage:");
        Serial.println(data.id);
        Serial.print("\nClave: ");
        Serial.print(data.clave);
        
        data.id = data.id + 1;
      
        delay(velocidad);
    
  }
}
