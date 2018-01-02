#include <QTRSensors.h>
#include <SoftwareSerial.h>
SoftwareSerial bt(10, 11); // RX, TX

////////////////////////////PINES FISICOS USADOS EN EL ROBOT//////////////////////////////////////////////////////////
  //entradas
#define btn1      0
#define btn2      1
  //salidas
#define led1      13
#define led2      12
#define led3      2
#define led_on    9   //~
#define mi1       3
#define mi2       4
#define pwmi      6   //~
#define md1       8
#define md2       7
#define pwmd      5   //~
  //comunicacion
#define tx        11
#define rx        10
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NUM_SENSORS             8  // numero de sensores
#define NUM_SAMPLES_PER_SENSOR  1 // mustras tmadas por sensor
#define EMITTER_PIN             9   // pin que enciende a los sensores

QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2,1,0},NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int proporcional=0;
int derivativo=0;
int integral=0;
int salida_pwm=0;
int proporcional_pasado=0;
///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////CONFIGURACION DE ROBOT! ////////////////////////////////////////////////

int velocidad=100;   // cambia aqui velocidad de robot

//las constantes afectan el comportamiento del robot

float KP=0.01;  //constante proporcional, 
float KD=0.15;   //constante diferencial, aumenta aqui para que el robot no se tambalee mucho
float KI=0.0001;  //constante integral, noo muevas la integral mucho, mjor dejalo asi!

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


int position=0;
int boton1=7;
int boton2=7;

void setup()
{
    pinMode(led1,OUTPUT);  //pines usados como salida
    pinMode(led2,OUTPUT);
    pinMode(led3,OUTPUT);
    pinMode(led_on,OUTPUT);
    pinMode(mi1,OUTPUT);
    pinMode(mi2,OUTPUT);
    pinMode(pwmi,OUTPUT);
    pinMode(md1,OUTPUT);
    pinMode(md2,OUTPUT);
    pinMode(pwmd,OUTPUT);

digitalWrite(led1,HIGH);
digitalWrite(led2,HIGH);
digitalWrite(led3,HIGH);
delay(200);
digitalWrite(led1,LOW);
digitalWrite(led2,LOW);
digitalWrite(led3,LOW);
delay(200);

digitalWrite(led2, HIGH);    // indicador que muetra que se esta calibrando los sensores
  for (int i = 0; i < 200; i++)  // se calibra por 2 segundo el sensor
  {
    qtra.calibrate();       
  }
  
  digitalWrite(led2, LOW);     // 
 
   while(true) //espera a que sea precionado el boton del robot para empezar a correr en la pista
   {
      botones();   //funcion para verificar si se pulso boton
      if(boton2==0) 
      {
        delay(20);
        digitalWrite(led2,HIGH);
        digitalWrite(led3,HIGH);
        delay(80);
        digitalWrite(led2,LOW);
        digitalWrite(led3,LOW);
        delay(80);
        break;
       
      }
   }
  

}

void loop()
{ 
  pid(1,velocidad,KP,KI,KD);  //funcion para control de robot
  frenos_contorno(500);       //funcion para que no se salga de la linea robot
 delay(4);
}


void pid(int linea, int velocidad, float Kp, float Ki, float Kd)
{
   
   position = qtra.readLine(sensorValues,QTR_EMITTERS_ON, 0, 0, 500, 30);  
  proporcional = (position) - 3500;                     // punto medio es 3500, asi obtenemos el error
  integral=integral + proporcional_pasado;              //obteniendo integral
  derivativo = (proporcional - proporcional_pasado);    //obteniedo el derivativo
  int ITerm=integral*KI;
  if(ITerm>=255) ITerm=255;
  if(ITerm<=-255) ITerm=-255;
  
  salida_pwm =( proporcional * KP ) + ( derivativo * KD )+(ITerm);
  
  if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;
  
  if (salida_pwm < 0)  //voltaje entregado a motor cuando se mueve a la izquierda 
 {
    int der=velocidad-salida_pwm; //(+)
    int izq=velocidad+salida_pwm;  //(-)
    if(der>=255)der=255;
    if(izq<=0)izq=0;
    motores(izq, der);
 }
 if (salida_pwm >0)  //voltaje entregado a motor cuando se mueve a derecha
 {
  int der=velocidad-salida_pwm; //(-)
  int izq=velocidad+salida_pwm; //(+)
  
  if(izq>=255) izq=255;
  if(der<=0) der=0;
  motores(izq ,der );
 }

 proporcional_pasado = proporcional;  
}


void frenos_contorno(int flanco_comparacion)  
{
    if (position<=0) //si se salio por la parte derecha de la linea
    {
      
      while(true)
      { 
        digitalWrite(led2,HIGH);
        motores(-115,60);
        qtra.read(sensorValues); //lectura en bruto de sensor
        if ( sensorValues[0]<flanco_comparacion || sensorValues[1]<flanco_comparacion || sensorValues[2]<flanco_comparacion || sensorValues[3]<flanco_comparacion || sensorValues[4]<flanco_comparacion || sensorValues[5]<flanco_comparacion || sensorValues[6]<flanco_comparacion || sensorValues[7]<flanco_comparacion)
        {
          break;
        }
        
      }
    }

    if (position>=7000) //si se salio por la parte izquierda de la linea
    {
      while(true)
      {
        digitalWrite(led2,HIGH);
        motores(60,-115); 
        qtra.read(sensorValues);
        if (sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion|| sensorValues[5]<flanco_comparacion || sensorValues[4]<flanco_comparacion || sensorValues[3]<flanco_comparacion || sensorValues[2]<flanco_comparacion || sensorValues[1]<flanco_comparacion|| sensorValues[0]<flanco_comparacion)
        {
          break;
        }
      }
  }
  digitalWrite(led2,LOW);
}


void motores(int motor_izq, int motor_der)
{
  if ( motor_izq >= 0 )  
  {
    digitalWrite(mi1,LOW);
    digitalWrite(mi2,HIGH); 
    analogWrite(pwmi,motor_izq); 
  }
  else
  {
    digitalWrite(mi1,HIGH); 
    digitalWrite(mi2,LOW);
    motor_izq = motor_izq*(-1); 
    analogWrite(pwmi,motor_izq);
  }

  if ( motor_der >= 0 ) //motor derecho
  {
    digitalWrite(md1,LOW);
    digitalWrite(md2,HIGH);
    analogWrite(pwmd,motor_der);
  }
  else
  {
    digitalWrite(md1,HIGH);
    digitalWrite(md2,LOW);
    motor_der= motor_der*(-1);
    analogWrite(pwmd,motor_der);
  }
}

void botones()
{
    boton1=digitalRead(btn2);boton2=digitalRead(btn1); ///boton izquierdo, derecho
}



