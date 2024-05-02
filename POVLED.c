#include "Simple_MPU6050.h" //Libreria para el acelerometro
#define MPU6050_ADDRESS_AD0_LOW 0x68  // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())  // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
// printfloatx funcion para mostrar en monitor serie datos para evitar el uso se multiples print()

//Variables definidas para el serial

#define latchPin 10
#define ClockPin 9
#define DataPin1 11

//Acelerometro

Simple_MPU6050 mpu;  

// Palabras a proyectar

unsigned char buff[144]{
  0XFF, 0XFF, 0XFF,  //////////////////////
  0XF2, 0X49, 0X27,  //   ****       //
  0XF3, 0XFF, 0XE7,  //   *       *      //
  0XF3, 0XFF, 0XE7,  //   *        *     //
  0XF3, 0XFF, 0XE7,  //   *        *     //
  0XFE, 0X7F, 0X3F,  //   *       *      //
  0XFF, 0XC9, 0XFF,  //   ****       //
  0XFF, 0XFF, 0XFF,  //////////////////////
  0XFF, 0XFF, 0XFF,  //                  //
  0XE9, 0X7E, 0X97,  //    ****      //
  0XE9, 0X7E, 0X97,  //    ****      //
  0XE9, 0X24, 0X97,  //       **         //
  0XE9, 0X24, 0X97,  //       **         //
  0XE9, 0X7E, 0X97,  //    ****      //
  0XE9, 0X7E, 0X97,  //    ****      //
  0XFF, 0XFF, 0XFF,  //////////////////////
  0XFF, 0XFF, 0XFF,  //                  //
  0XE4, 0X93, 0XCF,  //    ****      //
  0XE7, 0XF3, 0XCF,  //    *             //
  0XE7, 0XF3, 0XCF,  //    *             //
  0XE7, 0XF3, 0XCF,  //    ****      //
  0XE7, 0XF3, 0XCF,  //           *      //
  0XE7, 0XF2, 0X4F,  //    ****      //
  0XFF, 0XFF, 0XFF,  //////////////////////
  0XFF, 0XFF, 0XFF,  // 				         //
  0XFF, 0XFF, 0XFF,  // 	*****    //
  0XFB, 0X6D, 0XB7,  // 	*	        *	   //
  0XFB, 0XFD, 0XFF,  // 	*	    	  *    //
  0XFB, 0XFD, 0XFF,  //   ****  	 //
  0XFB, 0XFD, 0XFF,  // 	*			         //
  0XFB, 0X6F, 0XFF,  // 	*			         //
  0XFF, 0XFF, 0XFF,  //////////////////////
  0XFF, 0XFF, 0XFF,  // 				         //
  0XED, 0XB6, 0XDF,  // 	***	     //
  0XEF, 0XFF, 0XFF,  // 	*	   	   *     //
  0XEF, 0XFF, 0XFF,  // 	*	   	   *     //
  0XEF, 0XF7, 0XFF,  // 	***	     //
  0XEF, 0XF6, 0XFF,  // 	*	  	  *      //
  0XED, 0XBF, 0XDF,  // 	*			  *      //
  0XFF, 0XFF, 0XFF,  //////////////////////
  0XFF, 0XFF, 0XFF,  // 				         //
  0XF6, 0XDB, 0X6F,  // 	****     //
  0XF7, 0XFF, 0XEF,  // 	*		     *     //
  0XF7, 0XFF, 0XEF,  // 	*		     *     //
  0XF7, 0XFF, 0XEF,  // 	*	       *	   //
  0XF7, 0XFF, 0XEF,  // 	*			   *     //
  0XF6, 0XDB, 0X6F,  // 	****     //
  0XFF, 0XFF, 0XFF   //////////////////////
};

// Variables globales 

byte byteToSend = 0;
byte byteToSend2 = 0;
byte byteToSend3 = 0;
int index = 0;
unsigned long p;

//Definiendo primera FSM

typedef enum { UP
} State;
State currentState = UP;

//Definiendo segunda FSM

typedef enum { STILL,
               MOVING
} Noboton;
Noboton Nobotonstate = STILL;

void setup() {
  
  //Control serial I2C
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  // activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
///////////////////////////////////////////////////////
  Serial.begin(115200);                                             // inicializacion de monitor serie a 115200 bps 
  Serial.println(F("Inicio:"));                                     // muestra texto estatico
#ifdef OFFSETS                                                      // si existen OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));                 // texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);  // inicializacion de sensor

#else  // si no existen OFFSETS
  Serial.println(F(" No se establecieron Offsets, haremos unos nuevos.\n"  // muestra texto estatico
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  while (Serial.available() && Serial.read())
    ;  // lectura de monitor serie
  while (!Serial.available())
    ;  // si no hay espera
  while (Serial.available() && Serial.read())
    ;                                                                       // lecyura de monitor serie
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();  // inicializacion de sensor
#endif
  mpu.on_FIFO(mostrar_valores);  // llamado a funcion mostrar_valores si memoria FIFO tiene valores

  // Definiendo pines para el serial
  
  pinMode(latchPin, OUTPUT);
  pinMode(ClockPin, OUTPUT);
  pinMode(DataPin1, OUTPUT);

  // Delay para FSM
  p = millis();
}


void loop() {
  juan(p, index);
}

/************************************************************************
*  void juan(unsigned long &p, int &index)
*
*  Function Return Type: void
*
*  Purpose: This function controls the lighting of each RGB LED with the 74HC545. 
   The Arduino shiftOut function is used to send the shutdown and color instructions.
*  Plan: It is not necessary
*
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  02/05/24  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #1
  *********************************************/

void juan(unsigned long &p, int &index) {
  switch (Nobotonstate) {
    case MOVING:
      switch (currentState) {
        case UP:
          //imprime una columna
          if (millis() - p > 100) {

            digitalWrite(latchPin, LOW);

            byteToSend = buff[index++];
            byteToSend2 = buff[index++];  // LEE Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7
            byteToSend3 = buff[index++];
            // CULO CABEZA
            shiftOut(DataPin1, ClockPin, LSBFIRST, byteToSend3);  // SHIFTER3
            shiftOut(DataPin1, ClockPin, LSBFIRST, byteToSend2);  // SHIFTER2
            shiftOut(DataPin1, ClockPin, LSBFIRST, byteToSend);   // SHIFTER1

            digitalWrite(latchPin, HIGH);

            p = millis();
            currentState = UP;
          } else if (index > 143) {
            index = 0;
            mpu.dmp_read_fifo();
            currentState = UP;
            Nobotonstate=STILL;
          }
          break;
      }
      break;
    case STILL:
      mpu.dmp_read_fifo();  // funcion que evalua si existen datos nuevos en el sensor y llama a funcion mostrar_valores si es el caso
      break;
  }
}

/************************************************************************
*  void mostrar_valores(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp)
*
*  Function Return Type: void
*
*  Purpose: This function monitors the accelerometer whenever data is available from the sensor. If the acceleration threshold is exceeded, the function is used to monitor the threshold and implement the juan() function.
*
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  02/05/24  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #2
  *********************************************/

void mostrar_valores(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  static float coso;
  uint8_t SpamDelay = 1000;                   // demora para escribir en monitor serie de 100 mseg
  Quaternion q;                              // variable necesaria para calculos posteriores
  VectorFloat gravity;                       // variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };                // array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };                // array para almacenar valores convertidos a grados de yaw, pitch, roll
  spamtimer(SpamDelay) {                     // si han transcurrido al menos 100 mseg entonces proceder
    mpu.GetQuaternion(&q, quat);             // funcion para obtener valor para calculo posterior
    mpu.GetGravity(&gravity, &q);            // funcion para obtener valor para calculo posterior
    mpu.GetYawPitchRoll(ypr, &q, &gravity);  // funcion obtiene valores de yaw, ptich, roll
    mpu.ConvertToDegrees(ypr, xyz);          // funcion convierte a grados sexagesimales
    Serial.printfloatx(F("ax"), accel[0], 5, 0, F(",   "));
    Serial.printfloatx(F("ay"), accel[1], 5, 0, F(",   "));
    Serial.printfloatx(F("az"), accel[2], 5, 0, F(",   "));
    Serial.println();  // salto de linea
    coso=abs(accel[0]);
    if (coso < 3000) {
      Nobotonstate = STILL;
    } else {
      Nobotonstate = MOVING;
    }
  }
}