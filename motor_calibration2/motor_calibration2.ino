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
//#define H_BRIDGE_RED 
#define H_BRIDGE_BLACK  

// ARDUINO - Uncomment only one option
#define ARDUINO_TYPE_EVERIS  
//#define ARDUINO_TYPE_MKR  

// THE COMPANY - Uncomment only one option
// #define MERRY
// #define FRODO
// #define SAM
#define OTHER

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
  const int pin_right_motor_enable = 9;
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
#define MINPWM 80
#define MAXPWM 200



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

  delay(500);     // A la novena vez que te pilla la mano, ya no hace gracia :)
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


#ifdef OTHER
double K_p[2] = {20, 20};
double K_i[2] = {0, 0};
double K_d[2] = {0.0, 0.0};
double a_ff[2] = {-15, -15};
double b_ff[2] = {2.35/(MAXPWM - MINPWM), 2.35/(MAXPWM - MINPWM)};
//double a_ff[2] = {MINPWM, MINPWM};
//double b_ff[2] = {18.0/(MAXPWM - MINPWM), 18.0/(MAXPWM - MINPWM)};
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
  double P = K_p[motor]*error;
  double I = I_prev[motor] + K_i[motor]*error*elapsed_time*1e-3;
  double D = K_d[motor]*(error - previous_error[motor]) / elapsed_time;
  double u = u0 + P+I+D;
  double v = constrain(u, MINPWM, MAXPWM);
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

double T = 0;
int i = 0;

//int state = "TESTING";  // Para el codigo de Python, puede ser TESTING o WAITING (ignorar)


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

// Para que el serial plotter se vea bien. Limita los valores del eje Y
void serial_limit(){

  Serial.print("0");
  Serial.print("\t");
  Serial.print("16");
  Serial.print("\t");

}



void update_control(double count_left_wheel, double count_right_wheel, double dt_s) {

  pixels.clear();
  //delay(500);
  //serial_limit();

  user_lw += count_left_wheel;
  user_rw += count_right_wheel;

  
  double count_left_RPM  = 60*(count_left_wheel/NT)/dt_s;
  double count_right_RPM = 60*(count_right_wheel/NT)/dt_s;
  double count_left_RPS  = (count_left_wheel/NT)/dt_s;
  double count_right_RPS = (count_right_wheel/NT)/dt_s;
  double distance = (user_lw*2*PI*R)/(NT);

  //Serial.print("Velocidad encoder (RPS):  ");
  Serial.print(count_left_RPS);
  Serial.print("\t");
  Serial.print(count_right_RPS);


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

    Serial.print("\t");
    Serial.print(K_p[0]);
    Serial.print("\t");
    Serial.print(K_i[0]);
    Serial.print("\t");
    Serial.println(pid_right_motor(count_left_wheel)/20.0);

    setpointW[LEFT_WHEEL]  = 3;
    setpointW[RIGHT_WHEEL] = 3;

    if (T > 4){
      set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
      //set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_wheel));
    }
    if (T > 8){
      set_wheel_speed(LEFT_WHEEL,   FORWARD, 0);
      set_wheel_speed(RIGHT_WHEEL,  FORWARD, 0);
      T = 0;
      i++;

      K_i[0] = 5*i;
      K_i[1] = 5*i;

      I_prev[0] = 0;
      I_prev[1] = 0;

    }

  #endif

// Para calibrar los parametros del PID usando Python (ignorar)
#ifdef CALIBRATION_PYTHON

    Serial.print("\t");
    Serial.print(K_p[0]);
    Serial.print("\t");
    Serial.print(K_i[0]);
    Serial.print("\t");
    Serial.println(pid_right_motor(count_left_wheel)/20.0);

    setpointW[LEFT_WHEEL]  = 3;
    setpointW[RIGHT_WHEEL] = 3;

    if (state == "TESTING"){
      if (T > 4){
        set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
        //set_wheel_speed(RIGHT_WHEEL, FORWARD, feedforward[RIGHT_WHEEL]);
      }
      if (T > 8){
        set_wheel_speed(LEFT_WHEEL,   FORWARD, 0);
        set_wheel_speed(RIGHT_WHEEL,  FORWARD, 0);
        T = 0;
        i++;

        if (K_i[0] != 0){
          K_i[0] = i;
          K_i[1] = i;
        }

        else{
          K_p[0] = i;
          K_p[1] = i;
        }
        

        I_prev[0] = 0;
        I_prev[1] = 0;

        state = (K_p[0] < 20) ? "TESTING" : "WAITING";

      }
    }
    else{
      set_wheel_speed(LEFT_WHEEL,   FORWARD, 0);
      set_wheel_speed(RIGHT_WHEEL,  FORWARD, 0);
      if (Serial.available() > 0) {
        // Leer el dato enviado por Python
        String data = Serial.readStringUntil('\n');
        
        // Convertir el dato a un número flotante
        K_p[0] = data.toFloat();

        // Imprimir el valor de Kp en el monitor serial
        Serial.print("Valor de Kp recibido: ");
        Serial.println(K_p[0]);

        state = "TESTING";
        K_i[0] = 0.1;
        K_i[1] = 0.1;
      }
    }

  #endif




  // Para ver la respuesta final
  #ifdef TEST

    Serial.print("\t");
    Serial.println(pid_right_motor(count_left_wheel)/20.0);

    set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
    set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_RPS));

    if (T > 4){
      setpointW[LEFT_WHEEL]  = 3;
      setpointW[RIGHT_WHEEL] = 3;  
      
    }
    if (T > 8){
      setpointW[LEFT_WHEEL]  = 2;
      setpointW[RIGHT_WHEEL] = 2;
      T = 0;
      i++;
    }

  #endif


  // Para sacar la relacion entre RPM y PWM
  #ifdef STEADY_STATE

    int PWM = 70 + 5*i;

    Serial.print("\t");
    Serial.println(PWM);

    if (T > 4){
      set_wheel_speed(LEFT_WHEEL,  FORWARD, PWM);
      set_wheel_speed(RIGHT_WHEEL, FORWARD, PWM);

      T = 0;
      i = ((70 + 5*i) == MAXPWM) ? 0 : i + 1;
    }

  #endif

  
  #ifdef MOVE_FORWARD

    Serial.println("\t");   // Para poner un salto de linea, sino el Serial monitor no funciona

    setpointW[LEFT_WHEEL]  = 10;
    setpointW[RIGHT_WHEEL] = 10;

    set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_RPS));
    set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_RPS));
    //set_wheel_speed(RIGHT_WHEEL, FORWARD, feedforward(LEFT_WHEEL));

  #endif



  //set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_wheel));
  //set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_wheel));

  //set_wheel_speed(LEFT_WHEEL,   FORWARD, 100);
  //set_wheel_speed(RIGHT_WHEEL,  FORWARD, 100);

  
  /*
  if (distance <= 1.5){
    set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_wheel));
    set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_wheel));
  }
  else{
    set_wheel_speed(LEFT_WHEEL,   FORWARD, 0);
    set_wheel_speed(RIGHT_WHEEL,  FORWARD, 0);

  }
  */
  
/*

  float user_d=0;
  float vel_enc_d=0;
  float vel_enc_iz=0;

  user_lw+=count_left_wheel;
  user_rw+=count_right_wheel;
  t_tot+=dt_s;
  
  Serial.print("Kp:\t");
  Serial.print(K_p[0]);
  Serial.println(K_p[1]);
  Serial.print("Ki:\t");
  Serial.print(K_i[0]);
  Serial.println(K_i[1]);
  
  setpointW[0]=0.0;
  setpointW[1]=10;
  
  vel_enc_d=count_right_wheel/t_tot;
  vel_enc_iz=count_left_wheel/t_tot;
  Serial.print("Velocidad encoder:\t");
  Serial.print(vel_enc_d);
  Serial.print("\t");
  Serial.println(vel_enc_iz);
  
  rw_vel=pid_right_motor(vel_enc_d);
  lw_vel=pid_left_motor(vel_enc_iz); 
  Serial.print("Reference vel:\t");
  Serial.print(setpointW[0]);
  Serial.print("\t");
  Serial.println(setpointW[1]);
  Serial.print("Real vel:\t");
  //Serial.print(rw_vel);
  Serial.print(count_right_wheel);
  Serial.print("\t");
  //Serial.println(lw_vel);
  Serial.println(count_left_wheel);
  set_wheel_speed(LEFT_WHEEL,FORWARD,lw_vel);
  set_wheel_speed(RIGHT_WHEEL,FORWARD,rw_vel);

  //distancia += 
  Serial.println(user_lw);
  Serial.println(t_tot);
  if (user_lw < 100) {
    set_wheel_speed(LEFT_WHEEL,FORWARD, 200);
    set_wheel_speed(RIGHT_WHEEL,FORWARD, 200);
  }

  */

  
}
