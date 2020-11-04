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
#include <SPI.h>
#include <RF24.h>

//Led
int pin_led = 5;

// Motor A
int ENA = 10;
int IN1 = 7;
int IN2 = 4;

// Motor B
int ENB = 5;
int IN3 = 8;
int IN4 = 2;

//////////////////////////////
const int velocidad = 50;

RF24 myRadio (9, 10); // in Mega can use> (48, 49); 

byte addresses[][6] = {"0"}; 

struct package
{
  int id = 0;
  int clave= 0;
};

typedef struct package Package;
Package data;
////////////////////////////////
void Adelante ()
{
 //Direccion motor A
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 255); //Velocidad motor A
 //Direccion motor B
 digitalWrite (IN3, HIGH);
 digitalWrite (IN4, LOW);
 analogWrite (ENB, 255); //Velocidad motor B
}

void Atras ()
{
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, HIGH);
 analogWrite (ENA, 128); //Velocidad motor A
 //Direccion motor B
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, HIGH);
 analogWrite (ENB, 128); //Velocidad motor B
}
void Derecha ()
{
 //Direccion motor A
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 150); //Velocidad motor A
 //Direccion motor B
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, HIGH);
 analogWrite (ENB, 150); //Velocidad motor A
}
void Izquierda ()
{
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, HIGH);
 analogWrite (ENA, 150); //Velocidad motor A
 //Direccion motor B
 digitalWrite (IN3, HIGH);
 digitalWrite (IN4, LOW);
 analogWrite (ENB, 150); //Velocidad motor A
}

void Parar ()
{
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 0); //Velocidad motor A
 //Direccion motor B
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, LOW);
 analogWrite (ENB, 0); //Velocidad motor A
}

void setup ()
{
 //Led indicacion
 pinMode(pin_led, OUTPUT);
 
 // Declaramos todos los pines como salidas
 pinMode (ENA, OUTPUT);
 pinMode (ENB, OUTPUT);
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);
 ////////////////////////////////////////////
 Serial.begin(115200);  
  myRadio.begin(); 
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS ) ; 
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}

void loop() {

char clave;
if ( myRadio.available()) 
  {
    while (myRadio.available())
    {
      myRadio.read( &data, sizeof(data) );
    }

    digitalWrite(pin_led, HIGH);
    
    Serial.print("\nPackage:");
    Serial.println(data.id);
    Serial.print("\nClave: ");
    Serial.print(data.clave);
    
    switch (data.clave){
      case 1:   
      Serial.println("Avanzar\n");
      Adelante();
      //delay(1000);
      break; 
    
      case 2:
      Serial.println("Retroceder\n");
      Atras();
      //delay(1000);
      break;
    
      case 3:
      Serial.println("Derecha\n");
      Derecha();
      //delay(1000);
      break;
    
      case 4:
      Serial.println("Izquierda\n");
      Izquierda();
      //delay(1000);
      break;
    
      default:
      Serial.println("\nNADA\n");
      Parar();
      //delay(1000);
      break;
    }
  }
      Serial.println("\nERROR\n");
      digitalWrite(pin_led, LOW);
}
