// Robotarium-UCM - Agent firmware
// Copyright (C) 2022 UCM-237
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


// Esta version es para reducir el periodo de muestreo (Ts original = 0.1 s)


#include <MeanFilterLib.h>    //https://github.com/luisllamasbinaburo/Arduino-Meanfilter
#include <Adafruit_NeoPixel.h>
//#include <WiFiNINA.h>
//#include <utility/wifi_drv.h>

using namespace std;

// H-BRIDGE - Uncomment only one option
#define H_BRIDGE_RED 
//#define H_BRIDGE_BLACK  

// ARDUINO - Uncomment only one option
//#define ARDUINO_TYPE_EVERIS  
#define ARDUINO_TYPE_MKR  

// THE COMPANY - Uncomment only one option
// #define MERRY
// #define FRODO
// #define SAM
//#define ALEJANDRO_NANO
#define ALEJANDRO_MKR

// -----------------------------------------------------------------------------
// PIN layout
// -----------------------------------------------------------------------------
#define LED_RING        4  //  LED_RING
#define NUMPIXELS 16  // Numero de pixels
Adafruit_NeoPixel pixels(NUMPIXELS, LED_RING, NEO_GRB + NEO_KHZ400);

#ifdef ARDUINO_TYPE_MKR

  const int pin_left_encoder          = 0;
  const int pin_right_encoder         = 1;
  const int pin_LED                   = 6;
#endif

#ifdef ARDUINO_TYPE_EVERIS

  const int pin_left_encoder          = 2;
  const int pin_right_encoder         = 3;
#endif

#ifdef H_BRIDGE_BLACK
  const int pin_left_motor_direction  = 8;
  const int pin_left_motor_enable     = 6 ;
  const int pin_right_motor_direction = 9;
  const int pin_right_motor_enable    = 5;
#endif

#ifdef H_BRIDGE_RED
  const int pin_left_motor_dir_1   = 4;
  const int pin_left_motor_dir_2   = 5;
  const int pin_left_motor_enable  = 6;
  const int pin_right_motor_dir_1  = 7;
  const int pin_right_motor_dir_2  = 8;
  const int pin_right_motor_enable = 2;
#endif

// -----------------------------------------------------------------------------
// Motors
// -----------------------------------------------------------------------------
const int   LEFT_WHEEL = 0;
const int   RIGHT_WHEEL = 1;
const int   PIN_SHARP = 1;
const float R = 0.0325;       // Radio en m
const int   NT = 40;          // Numero de muescas del encoder

#define PI 3.14159265358979323846//...

#define FORWARD LOW
#define BACKWARD HIGH
#define MINPWM 100
#define MAXPWM 200
double LIMIT_PWM[2] = {200.0, 200.0};



void set_wheel_speed(int wheel, int direction, int speed) {
#ifdef H_BRIDGE_BLACK
  int pin_direction = (wheel == LEFT_WHEEL) ? pin_left_motor_direction : pin_right_motor_direction;
  int dir = (wheel == LEFT_WHEEL) ? direction : !direction; // La rueda izquierda esta al reves
  digitalWrite(pin_direction, dir);
#endif

#ifdef H_BRIDGE_RED
  int pin_dir_1  = (wheel == LEFT_WHEEL) ? pin_left_motor_dir_1 : pin_right_motor_dir_2;
  int pin_dir_2  = (wheel == LEFT_WHEEL) ? pin_left_motor_dir_2 : pin_right_motor_dir_1;
  digitalWrite(pin_dir_1, (direction == FORWARD && speed != 0) ? HIGH : LOW);
  digitalWrite(pin_dir_2, (direction == BACKWARD && speed != 0) ? HIGH : LOW);
#endif

  int pin_enable = (wheel == LEFT_WHEEL) ? pin_left_motor_enable : pin_right_motor_enable;
  analogWrite(pin_enable, speed);  

}

// -----------------------------------------------------------------------------
// Encoders
// -----------------------------------------------------------------------------
const int encoder_resolution_ppt = 20*2;
int filter_window_size = 10;
unsigned long encoder_count[2] = {0, 0};
MeanFilter<double> encoder_w_estimated[2] = {
  MeanFilter<double>(filter_window_size),
  MeanFilter<double>(filter_window_size)
};

void isr_left_encoder_count() {
  isr_encoder_count(LEFT_WHEEL);
}

void isr_right_encoder_count() {
  isr_encoder_count(RIGHT_WHEEL);
}

void isr_encoder_count(int pin) {
    encoder_count[pin] ++;
}

  

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  setup_motors();
  setup_encoders();
  setup_comms();
  setup_pixels();

  //delay(500);     // A la novena vez que te pilla la mano, ya no hace gracia :)
}

void setup_motors() {
  pinMode(pin_left_motor_enable, OUTPUT);
  pinMode(pin_right_motor_enable, OUTPUT);
  #ifdef H_BRIDGE_RED
    pinMode(pin_left_motor_dir_1, OUTPUT);
    pinMode(pin_left_motor_dir_2, OUTPUT);
    pinMode(pin_right_motor_dir_1, OUTPUT);
    pinMode(pin_right_motor_dir_2, OUTPUT);
  #endif
  #ifdef H_BRIDGE_BLACK
    pinMode(pin_left_motor_direction, OUTPUT);
    pinMode(pin_right_motor_direction, OUTPUT);
  #endif

  set_wheel_speed(LEFT_WHEEL, FORWARD, 0);
  set_wheel_speed(RIGHT_WHEEL, FORWARD, 0);
}

void setup_encoders() {

  pinMode(pin_left_encoder,  INPUT_PULLUP);
  pinMode(pin_right_encoder, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder),  isr_left_encoder_count,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder), isr_right_encoder_count, CHANGE);

}

void setup_comms() {
  Serial.begin(115200);
}

void setup_pixels() {
  pixels.begin();
}

// -----------------------------------------------------------------------------
// Control Loop
// -----------------------------------------------------------------------------
const unsigned long sampling_time_us = 25000; // 0.025 s = 25 ms
unsigned long last_time_us = 0;

void loop() {
  unsigned long current_time_us = micros();  
  unsigned long dt_us = current_time_us - last_time_us; 
  float d;
  if(dt_us >= sampling_time_us) {
    double w_left = encoder_w_estimated[LEFT_WHEEL].AddValue(encoder_count[LEFT_WHEEL]);
    double w_right = encoder_w_estimated[RIGHT_WHEEL].AddValue(encoder_count[RIGHT_WHEEL]);
    noInterrupts();
    encoder_count[LEFT_WHEEL] = encoder_count[RIGHT_WHEEL] = 0;
    interrupts();
    update_control(w_left, w_right, dt_us*1e-6);
    last_time_us = current_time_us;
  }
}

// -----------------------------------------------------------------------------
// PID Controller
// -----------------------------------------------------------------------------
#ifdef MERRY
double K_p[2] = {240, 180};
double K_i[2] = {75, 75};
double K_d[2]= {0.0, 0.0};
double a_ff[2] = {1.8, 1.85};
double b_ff[2] = {1.8/(MAXPWM - MINPWM), 1.8/(MAXPWM - MINPWM)};

#endif

#ifdef SAM
double K_p[2] = {240, 180};
double K_i[2] = {75, 75};
//double K_i[2] = {0, 0};
double K_d[2] = {0.0, 0.0};
double a_ff[2] = {MINPWM, MINPWM+50};
double b_ff[2] = {1.8/(MAXPWM - MINPWM), 1.8/(MAXPWM - MINPWM)};

#endif


#ifdef FRODO
double K_p[2] = {240, 180};
//double K_i[2] = {75, 75};
double K_i[2] = {0, 0};
double K_d[2] = {0.0, 0.0};
double a_ff[2] = {MINPWM, MINPWM+50};
double b_ff[2] = {1.8/(MAXPWM - MINPWM), 1.8/(MAXPWM - MINPWM)};
#endif


#ifdef ALEJANDRO_NANO
  double K_p[2] = {10, 10};
  double K_i[2] = {0, 0};
  double K_d[2] = {0.0, 0.0};
  double a_ff[2] = {-15, -30};
  double b_ff[2] = {2.35/(MAXPWM - MINPWM), 2.00/(MAXPWM - MINPWM)};
#endif

#ifdef ALEJANDRO_MKR
  double K_p[2] = {60, 60};
  double K_i[2] = {250, 250};
  double K_d[2] = {1.0, 1.0};
  double a_ff[2] = {-25, -140};
  double b_ff[2] = {1.83/(MAXPWM - MINPWM), 1.07/(MAXPWM - MINPWM)};
#endif


unsigned long previous_time_ms[2] = {0, 0};

double setpointW[2] = {0, 0}; // setpoint in { 0 U (1.8, 4) } 
double setpoint_left_motor = 0;
double setpoint_right_motor = 0;
double previous_error[2] = {0, 0};
double I_prev[2] = {0, 0};

int pid_right_motor(double w) {
  return pid(RIGHT_WHEEL, w);
}

int pid_left_motor(double w) {
  return pid(LEFT_WHEEL, w);
}

int pid(int motor, double w) {
  unsigned long current_time = millis();
  double elapsed_time = current_time - previous_time_ms[motor];
  double error = setpointW[motor] - w;
  double u0 = feedforward(motor);
  LIMIT_PWM[motor] = (1.05*u0 <= 200) ? 1.05*u0 : 200.0;  // Para evitar overshoot muy grande
  double P = K_p[motor]*error;
  double I = I_prev[motor] + K_i[motor]*error*elapsed_time*1e-3;
  double D = (elapsed_time == 0) ? 0 : K_d[motor]*(error - previous_error[motor]) / (elapsed_time*1e-3);
  //Serial.println(D);
  double u = u0 + P+I+D;
  //Serial.println(1.15*u0);
  double v = constrain(u, MINPWM, LIMIT_PWM[motor]);
  if((u - v)*error <= 0) {
    I_prev[motor] = I;
  }

  previous_error[motor] = error;
  previous_time_ms[motor] = current_time;
  return static_cast<int> (round(v));
}

int feedforward(int motor) {
  /*return (setpointW[motor] != 0.0) ? constrain(
    round((setpointW[motor] + a_ff[motor]) / b_ff[motor]), MINPWM, MAXPWM
  ) : 0;*/
  return (setpointW[motor] != 0.0) ? constrain(
    round(a_ff[motor]+(setpointW[motor] / b_ff[motor])), MINPWM, MAXPWM
  ) : 0;    
}

// -----------------------------------------------------------------------------
// INSTRUCCIONES
// -----------------------------------------------------------------------------
//   Inserta tu código en la función "update_control". Está función se invoca
// períódicamente, pasándole información actualizada sobre el estado del robot.
//
// En ella puedes realizar llamadas a las funciones de la API para acceder a las
// funciones básicas del robot:
//   - set_wheel_speed(int wheel, int direction, int speed) 
//
// Si necesitas que el estado de una variable persista entre llamadas a
// update_control, deberás utilizar una variable global. Utiliza la sección
// comentada unas líneas más abajo. Para evitar conflictos de nombre con 
// variables del firmware, utiliza el prefijo "user_". 

// -----------------------------------------------------------------------------
// Define aquí tus variables globales
// -----------------------------------------------------------------------------
// (Ej.: const int user_time = 0;)

int user_lw=0;
int user_rw=0;
int lw_vel=0;
int rw_vel=0;
double t_tot=0;

int user_state = 0;
float d_max=20;
float d_min=10;

double T = 6;
int i = 0;

//int state = "TESTING";  // Para el codigo de Python, puede ser TESTING o WAITING (ignorar)

void stop(){

  setpointW[LEFT_WHEEL]  = 0;
  setpointW[RIGHT_WHEEL] = 0;

  set_wheel_speed(LEFT_WHEEL,  FORWARD,  0);
  set_wheel_speed(RIGHT_WHEEL, BACKWARD, 0);

}


// -----------------------------------------------------------------------------
// update_control
// -----------------------------------------------------------------------------
//   Bucle de control principal. Ejecuta el código periódicamente, con un
// periódo nominal especificado por el valor sampling_time_us.
//
// params
// ------
//   double count_left_wheel  Incremento de cuentas de encoder de la rueda izquierda. 
//   double count_right_wheel Incremento de cuentas de encoder de la rueda derecha. 
//   double dt_s              Indica el tiempo real, en segundos, transcurrido
//                            desde la última invocación.
// -----------------------------------------------------------------------------


void update_control(double count_left_wheel, double count_right_wheel, double dt_s) {

  pixels.clear();
  //delay(500);
  //serial_limit();

  user_lw += count_left_wheel;
  user_rw += count_right_wheel;

  
  double count_left_RPS  = (count_left_wheel/NT)/dt_s;
  double count_right_RPS = (count_right_wheel/NT)/dt_s;
  double count_left_RPM  = 60*count_left_RPS;
  double count_right_RPM = 60*count_right_RPS;
  double distance = (user_lw*2*PI*R)/(NT);

  //Serial.print("Velocidad encoder (RPS):  ");
  Serial.print(count_left_RPS);
  Serial.print("\t");
  Serial.print(count_right_RPS);
  //Serial.print("\t");
  //Serial.print(dt_s);
  


  //Serial.print("Distancia (m):  ");
  //Serial.println(distance);

  T += dt_s;

  //Serial.print("\t");
  //Serial.print(setpointW[0]*5);


  // #####################################
  //#define CALIBRATION
  //#define CALIBRATION_PYTHON // Para usar el script de python (en pruebas, aun NO funciona)
  #define TEST
  //#define STEADY_STATE
  //#define MOVE_FORWARD


  // Para calibrar los parametros del PID
  #ifdef CALIBRATION

    // Elige el parametro que se desea ajustar
    #define SET_KP
    //#define SET_KI
    //#define SET_KD

    Serial.print("\t");
    Serial.print(K_p[1]);
    Serial.print("\t");
    Serial.print(K_i[1]);
    Serial.print("\t");
    Serial.println(K_d[1]);


    setpointW[LEFT_WHEEL]  = 3;
    setpointW[RIGHT_WHEEL] = 0;

    if (T > 8){
      set_wheel_speed(LEFT_WHEEL,   FORWARD, 0);
      set_wheel_speed(RIGHT_WHEEL,  FORWARD, 0);
      i++;
      T = 0;

      #ifdef SET_KP

        K_p[0] = 30*i;
        K_p[1] = 30*i;

      #elif defined(SET_KI)

        K_i[0] = 100*i;
        K_i[1] = 100*i;

      #elif defined(SET_KD)

        K_d[0] = 3*i;
        K_d[1] = 3*i;

      #endif


      I_prev[0] = 0;
        I_prev[1] = 0;

    }
    else if (T > 4){
      set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
      set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_RPS));

      // Esto es para que no pinte la primera (que viene por defecto, evito tener que ponerlas a 0)
      if (i == 0){
        set_wheel_speed(LEFT_WHEEL,  FORWARD, 0);
        set_wheel_speed(RIGHT_WHEEL, FORWARD, 0);
      }


    }

  #endif



  // Para ver la respuesta final
  #ifdef TEST

    Serial.print("\t");
    Serial.println(setpointW[0]);

    set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
    set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_RPS));
    //set_wheel_speed(RIGHT_WHEEL, FORWARD, feedforward(RIGHT_WHEEL));
    //set_wheel_speed(LEFT_WHEEL,  FORWARD, 150);



    if (T > 4){
      setpointW[LEFT_WHEEL]  = 3.4;
      setpointW[RIGHT_WHEEL] = 3.4;  
      
    }
    if (T > 8){
      setpointW[LEFT_WHEEL]  = 2.7;
      setpointW[RIGHT_WHEEL] = 2.7;
      T = 0;
      i++;
    }

  #endif


  // Para sacar la relacion entre RPM y PWM
  #ifdef STEADY_STATE
    
    if (T > 4){

      //stop();
      //delay(100);

      T = 0;
      i = ((70 + 5*i) == MAXPWM) ? 0 : i + 1;
      int PWM = 70 + 5*i;

      set_wheel_speed(LEFT_WHEEL,  FORWARD, PWM);
      set_wheel_speed(RIGHT_WHEEL, FORWARD, PWM);

    }

    int PWM = 70 + 5*i;
    Serial.print("\t");
    Serial.println(PWM);

  #endif

  
  #ifdef MOVE_FORWARD

    Serial.println("\t");   // Para poner un salto de linea, sino el Serial monitor no funciona

    double speed = 2.5;
    setpointW[LEFT_WHEEL]  = speed;
    setpointW[RIGHT_WHEEL] = speed;

    //set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
    //set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_RPS));

    set_wheel_speed(LEFT_WHEEL,  FORWARD, 150);
    set_wheel_speed(RIGHT_WHEEL, FORWARD, 150);

  #endif



  
}
