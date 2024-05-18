// Robotarium-UCM - Agent firmware
// Copyright (C) 2024 UCM-237
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

#include <MeanFilterLib.h>    //https://github.com/luisllamasbinaburo/Arduino-Meanfilter
#include <Adafruit_NeoPixel.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "common.h"
#include <SimpleKalmanFilter.h>
//#include <utility/wifi_drv.h>

using namespace std;

//#define USE_WIFI  // Quitar para no usar el Wifi

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

// Wifi - Uncomment only one option
//#define ROBOTARIUM
#define WIFI_CASA

// ##########################################################
// ###################### PIN LAYOUT ######################
// ##########################################################
#define NUMPIXELS 16  // Numero de pixels
const int LED_RING = 3;  //  LED_RING


#ifdef ARDUINO_TYPE_MKR

  const int pin_left_encoder          = 0;
  const int pin_right_encoder         = 1;
  Adafruit_NeoPixel pixels(NUMPIXELS, LED_RING, NEO_GRB + NEO_KHZ800);    // Por motivos extraños, cambia segun el arduino

#endif

#ifdef ARDUINO_TYPE_EVERIS

  const int pin_left_encoder          = 2;
  const int pin_right_encoder         = 3;
  Adafruit_NeoPixel pixels(NUMPIXELS, LED_RING, NEO_GRB + NEO_KHZ400);    // Por motivos extraños, cambia segun el arduino

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


// ##########################################################
// ###################### WIFI SETUP ###################### 
// ##########################################################

#ifdef ROBOTARIUM
  
  char ssid[] = "RobotariumUCM";   // your network SSID (name)
  char pass[] = "robotario";   // your network password (use for WPA, or use as key for WEP)/

  IPAddress ip_arduino1(192,168,10,6);
  IPAddress ip_server(192,168,10,1);
  unsigned int localPort = 4244;      // local port to listen on

#elif defined(WIFI_CASA)
  
  char ssid[] = "your_home_wifi";       // your network SSID (name)
  char pass[] = "your_password";        // your network password (use for WPA, or use as key for WEP)

  IPAddress ip_arduino1(192,168,1,50);
  IPAddress ip_server(192,168,1,100);
  unsigned int localPort = 4244;      // local port to listen on

#endif

/*-----------ip setup-----------------*/
WiFiUDP Udp;
int ServerPort = 5556;  // Esto diria que no esta haciendo nada
char packetBuffer[256];             //buffer to hold incoming packet

int status = WL_IDLE_STATUS;

//operation variables
struct appdata operation_send;
struct appdata *server_operation;

bool SendDataSerial=false, SendDataUDP=false;
bool serialCom=false;

void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}



// ##########################################################
// ######################## MOTORS ######################### 
// ##########################################################
const int   LEFT_WHEEL = 0;
const int   RIGHT_WHEEL = 1;
const int   PIN_SHARP = 1;
const float R = 0.0325;       // Radio en m
const int   NT = 40;          // Numero de muescas del encoder
const float L_EJE = 0.107;    // Distancia entre ruedas (en m)

#define PI 3.14159265358979323846//...

#define FORWARD LOW
#define BACKWARD HIGH
#define MINPWM 100
#define MAXPWM 200
double LIMIT_PWM[2] = {200.0, 200.0};


#define LEFT  0
#define RIGHT 1



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

// ##########################################################
// ######################## ENCODERS ######################## 
// ##########################################################
const int encoder_resolution_ppt = NT;
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

  

// ##########################################################
// ########################## SETUP ######################### 
// ##########################################################
void setup() {

  setup_pixels();
  #ifdef USE_WIFI
    setup_Wifi();
  #endif
  setup_motors();
  setup_encoders();
  setup_comms();

  //delay(500);
}

void setup_pixels() {
  pixels.begin();

  /*for(int i=0; i<16; i+=2){
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));   // (RGB)
  }
  for(int i=1; i<16; i+=2){
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
  }*/
  pixels.fill(pixels.Color(255, 0, 0));
  pixels.show();

}


void setup_Wifi(){

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  WiFi.config(ip_arduino1);
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);//se prepara el puerto para escuchar
  pixels.fill(pixels.Color(0, 255, 0));
  pixels.show();

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

  pinMode(pin_left_encoder,  INPUT);
  pinMode(pin_right_encoder, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder),  isr_left_encoder_count,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder), isr_right_encoder_count, CHANGE);

}

void setup_comms() {
  Serial.begin(115200);
}

// ##########################################################
// ###################### CONTROL LOOP ###################### 
// ##########################################################

const unsigned long sampling_time_us = 25000;    // 0.025s=25ms
unsigned long last_time_us = 0;

double data_received;

void loop() {
  unsigned long current_time_us = micros();  
  unsigned long dt_us = current_time_us - last_time_us; 

  #ifdef USE_WIFI

    bool serialCom;
    int packetSize = Udp.parsePacket();

    if (packetSize) { //se comprueba si se recibe un paquete
      IPAddress remoteIp = Udp.remoteIP(); //proviene del servidor u ordenador central
      int numbytes = Udp.read((byte*)packetBuffer, MAXDATASIZE); //se guardan los datos en el buffer
      server_operation= (struct appdata*)&packetBuffer;
      //char temp[server_operation->len + 1];

      switch (server_operation->op){
        case OP_SALUDO:
          Serial.print("Case 1: ");
          memcpy(&data_received, server_operation->data, sizeof(double));
          Serial.println(data_received); // Salto de línea para mayor claridad
          break;
        case OP_MOVE_WHEEL:
          Serial.println("Case 2");            
          break;
        default:
          Serial.println("Case 3");     
          break;        
      }
    }

  #endif

  /*
  else if(Serial.available()){
      
    Serial.readBytes(packetBuffer,MAXDATASIZE);
      
    server_operation= (struct appdata*)&packetBuffer;
    serialCom=true;
    switch (server_operation->op){
      case OP_SALUDO:
        Serial.println("Case 1");  
        break;
      case OP_MOVE_WHEEL:
        Serial.println("Case 2");            
        break;
      default:
        Serial.println("Case 2");     
        break;        
    }
  }
  
  if(SendDataSerial){
    Serial.println("Serie");
  }*/


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

// ##########################################################
// ##################### PID CONTROLLER ##################### 
// ##########################################################
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
  double K_p[2] = {10, 80};
  //double K_i[2] = {75, 75};
  double K_i[2] = {0, 0};
  double K_d[2] = {0.0, 0.0};
  double a_ff[2] = {MINPWM, MINPWM+50};
  double b_ff[2] = {1.8/(MAXPWM - MINPWM), 1.8/(MAXPWM - MINPWM)};
#endif

// Estos son los buenos para el mio
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
  LIMIT_PWM[motor] = (1.2*u0 <= 200) ? 1.2*u0 : MAXPWM;  // Para evitar overshoot muy grande
  LIMIT_PWM[motor] = (1.2*u0 >= 100) ? 1.2*u0 : MINPWM;
  double P = K_p[motor]*error;
  double I = I_prev[motor] + K_i[motor]*error*elapsed_time*1e-3;
  double D = (elapsed_time == 0) ? 0 : K_d[motor]*(error - previous_error[motor]) / (elapsed_time*1e-3);
  double u = u0 + P+I+D;
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

// ##########################################################
// ################### VARIABLES GLOBALES ################### 
// ##########################################################


double user_lw = 0;
double user_rw = 0;

double t_tot=0;

String user_state = "FORWARD";

// -----------------------------------------------------
// Coordenadas en las que se encuentra el robot
double theta = PI/2;  // Angulo hacia el que apunta el robot
double x_0 = 0;
double y_0 = 0;

// Coordenadas a las que quiere ir
double x = -0.5;
double y = 0.0;

double epsilon = 0.1;  // Para tener en cuenta el error

// -----------------------------------------------------

int i = 1;
double angle = 180.0;  //  Se usa para el test




// ##########################################################
// ################### FUNCIONES MOTORES #################### 
// ##########################################################


void move_forward(double speed, double count_left_wheel, double count_right_wheel) {


    /*if ((count_left_wheel < 1.75) || (count_right_wheel < 1.75)){
      setpointW[LEFT_WHEEL]  = 2.2;
      setpointW[RIGHT_WHEEL] = 2.2;
    }
    else{*/
      setpointW[LEFT_WHEEL]  = speed;
      setpointW[RIGHT_WHEEL] = speed;
    //}

  set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_wheel));
  set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_wheel));

}


// ------------------------------------------------------------------------------------
// Esta es para girar sobre si mismo
void turn_left(double count_left_wheel, double count_right_wheel){
    //Serial.print("Girando izquierda ...");
    turn(LEFT, count_left_wheel, count_right_wheel);
}

void turn_right(double count_left_wheel, double count_right_wheel){
    Serial.print("Girando derecha ...");
    turn(RIGHT, count_left_wheel, count_right_wheel);
}


void turn(int TURN, double count_left_wheel, double count_right_wheel) {

    setpointW[LEFT_WHEEL]  = 2.25;
    setpointW[RIGHT_WHEEL] = 2.25;

    /*double count = (TURN == LEFT) ? count_right_wheel : count_left_wheel;
    int wheel = (TURN == RIGHT) ? LEFT_WHEEL : RIGHT_WHEEL;
    set_wheel_speed(wheel,  FORWARD,  pid(wheel, count));*/
    //set_wheel_speed(LEFT_WHEEL,  !TURN,  pid_left_motor(count_left_wheel));
    //set_wheel_speed(RIGHT_WHEEL,  TURN,  pid_left_motor(count_right_wheel));

    // +10 por seguridad
    set_wheel_speed(LEFT_WHEEL,  !TURN,  MINPWM+30);
    set_wheel_speed(RIGHT_WHEEL,  TURN,  MINPWM+10);


}


// ------------------------------------------------------------------------------------
// Esta es para girar con un radio concreto (esta es un poco confusa por que hice la cuenta para
// diametro, en vez de radio)
void turn_radius_left(double radius, double count_left_wheel, double count_right_wheel){
  turn_radius(LEFT, radius, count_left_wheel, count_right_wheel);
}
void turn_radius_right(double radius, double count_left_wheel, double count_right_wheel){
  turn_radius(RIGHT, radius, count_left_wheel, count_right_wheel);
}

void turn_radius(int TURN, double radius, double count_left_wheel, double count_right_wheel){
  
  if (radius <= L_EJE/2){     // Ten en cuenta que el diametro minimo es L_EJE/2;
    turn(TURN, count_left_wheel, count_right_wheel);
  }
  else{
    setpointW[TURN]  = 2.5;
    setpointW[!TURN] = setpointW[TURN]*(2*radius + L_EJE)/(2*radius - L_EJE);      

    set_wheel_speed(LEFT_WHEEL,  FORWARD,  pid_left_motor(count_left_wheel));
    set_wheel_speed(RIGHT_WHEEL, FORWARD,  pid_right_motor(count_right_wheel));
  }

}

// ------------------------------------------------------------------------------------

// Gira ligeramente para corregir la trayectoria
void turn_light_left(double count_left_wheel, double count_right_wheel){
  turn_light(LEFT, count_left_wheel, count_right_wheel);
}

void turn_light_right(double count_left_wheel, double count_right_wheel){
  turn_light(RIGHT, count_left_wheel, count_right_wheel);
}

void turn_light(int TURN, double count_left_wheel, double count_right_wheel){

  double speed = 2.7;

  setpointW[!TURN] = speed;
  setpointW[TURN]  = speed*0.75;
  

  set_wheel_speed(LEFT_WHEEL,  FORWARD, pid_left_motor(count_left_wheel));
  set_wheel_speed(RIGHT_WHEEL, FORWARD, pid_right_motor(count_right_wheel));

}


// ------------------------------------------------------------------------------------

void stop(){

  setpointW[LEFT_WHEEL]  = 0;
  setpointW[RIGHT_WHEEL] = 0;

  set_wheel_speed(LEFT_WHEEL,  FORWARD,  0);
  set_wheel_speed(RIGHT_WHEEL, BACKWARD, 0);

}

// ------------------------------------------------------------------------------------

// Simplifica codigo. Resetea las distancias y añade un delay (se puede poner 0)
void reset_state(int time){

  stop();
  delay(time);
  user_lw = 0;
  user_rw = 0;

}

// Pasa el angulo a [0, 2pi]
double normalize_angle(double angle){

  angle = (angle < 0) ? (2*PI + angle) : angle;
  angle = (angle > 2*PI) ? (angle - 2*PI) : angle;
  return angle;

}

// Para calcular cuanto tiene que girar (Ajustado para evitar errores. Usar con angulos mayores de 180º)
double turn_distance(double angle){

  return PI*L_EJE*((angle-100)/360.0);

}


// ##########################################################
// ##################### UPDATE CONTROL ##################### 
// ##########################################################

void update_control(double count_left_wheel, double count_right_wheel, double dt_s) {

  pixels.clear();
  

  user_lw += count_left_wheel;
  user_rw += count_right_wheel;


  double count_left_RPS  = (count_left_wheel/NT)/dt_s;
  double count_right_RPS = (count_right_wheel/NT)/dt_s;

  //double count_left_RPM  = 60*count_left_RPM;
  //double count_right_RPM = 60*count_right_RPM;

  double distance_LEFT  = (user_lw*2.0*PI*R)/(NT);
  double distance_RIGHT = (user_rw*2.0*PI*R)/(NT);
  double distance = (distance_LEFT+distance_RIGHT)/2.0;

  //Serial.print(dt_s);
  //Serial.print("\t");
  Serial.print("Velocidad encoder (RPS):  ");
  Serial.print(count_left_RPS);
  Serial.print("\t");
  Serial.print(count_right_RPS);
  Serial.print("\t");

  Serial.print("Distancia (m):  ");
  Serial.print(distance);
  Serial.print("\t");


  // ELEGIR LO QUE SE QUIERE HACER #############################

  //#define CIRCLE        // Sirve para hacer circulos
  #define SQUARE        // Sirve para hacer cuadrados o para hacer una linea recta
  //#define POINT         // Sirve para hacer que se mueva a las coordenadas requeridas
  //#define MOVE_NO_PID   // Para hacer las pruebas sin que moleste el PID
  //#define TEST          // Para analizar el error en el giro
  //#define TEST_WIFI     // Para analizar el error en el giro usando Wifi (usar esta, es mucho más comoda)
  // Si no usas ninguno se queda parado

  // #########################################################################
  #ifdef CIRCLE // Sirve para hacer circulos

    double radius = 0.3085;
    turn_radius_left(radius, count_left_RPS, count_right_RPS);


  // #########################################################################
  #elif defined(SQUARE) // Sirve para hacer cuadrados o para hacer una linea recta

    double L_SQUARE =  1.0;     // Este es el lado del cuadrado(m)/longitud de la linea
    int LINE = 0;               // 1 para cuadrados, 0 para rectas
    double speed = 2.8;

    if (user_state == "FORWARD"){
      move_forward(speed, count_left_RPS, count_right_RPS);
      if (distance >= L_SQUARE){
        user_state = "WAITING";
        reset_state(1000);
      }
    }
    else if (user_state == "WAITING"){
      stop();
      reset_state(0);
      if (t_tot >= 1.5){
        user_state = "TURN";
        t_tot = 0;
      }
      else{
        t_tot += dt_s;
      }
    }
    else if (user_state == "TURN"){
      turn_left(count_left_RPS, count_right_RPS);
      if (distance >= turn_distance(180 + 90*LINE)){
        user_state = "FORWARD";
        reset_state(500);
      }
    }
    else{ // Por si acaso
      user_state == "FORWARD";
    }

  // #########################################################################
  #elif defined(POINT) // Sirve para hacer que se mueva a las coordenadas requeridas (pendiente de terminar)

    // Coordenadas en metros
    double speed = 2.75;
    
    // Angulo actual
    double theta0 = theta;
    distance_LEFT  = (user_state == "TURN_LEFT")  ? -distance_LEFT :  distance_LEFT;
    distance_RIGHT = (user_state == "TURN_RIGHT") ? -distance_RIGHT : distance_RIGHT;
    distance = (distance_LEFT+distance_RIGHT)/2.0;  // De esta manera la distancia sera ~= 0 cuando gire

    theta = (user_state == "TURN_LEFT") ? (theta - 2*(distance_LEFT)/L_EJE) : (theta + 2*(distance_RIGHT)/L_EJE);
    theta = normalize_angle(theta);

    // Posicion actual
    x_0 += L_EJE*(theta - theta0)*cos(theta);
    y_0 += L_EJE*(theta - theta0)*sin(theta);

    // Calcula hacia donde esta el destino
    double phi = atan2(y - y_0, x - x_0);
    phi = normalize_angle(phi);

    Serial.print("Coordenadas (x, y) (m):  ");
    Serial.print(x_0);
    Serial.print("\t");
    Serial.print(y_0);
    Serial.print("\t");
    Serial.print("Angulo theta (rad):  ");
    Serial.print(theta);
    Serial.print("\t");
    Serial.print("Angulo phi (rad):  ");
    Serial.print(phi);
    Serial.print("\t");
    Serial.print("Distancia de giro:  ");
    Serial.print(2*(distance_RIGHT)/L_EJE);
    Serial.print("\t");


    if ((fabs(x_0 - x) > epsilon*0.4) || (fabs(y_0 - y) > epsilon*0.4)){

      // Alinea el robot hacia el punto correcto (REVISAR)
      if (fabs(phi - theta) > 2*epsilon){    
        if (normalize_angle(theta - phi) < PI){
          turn_right(count_left_RPS, count_right_RPS);
          user_state = "TURN_RIGHT";
        }
        else{
          turn_left(count_left_RPS, count_right_RPS);
          user_state = "TURN_LEFT";
        }
        pixels.fill(pixels.Color(0, 255, 0));
      }

      // Avanza hacia el punto
      else{
        user_state = "FORWARD";
        move_forward(speed, count_left_RPS, count_right_RPS);
        pixels.fill(pixels.Color(0, 0, 255));
      }
    }

    else{
      pixels.fill(pixels.Color(255, 0, 0));
      pixels.show();
      reset_state(5000);
      x = -1.0;
      y = 0.25;
    }

    pixels.show();
    user_lw = 0; user_rw = 0;



  // #########################################################################
  #elif defined(TEST)

    turn_left(count_left_RPS, count_right_RPS);
    
    if (distance >= turn_distance(angle)){
      Serial.print("Grados Girados: ");
      Serial.print(angle);
      angle += 45.0;
      pixels.setPixelColor(2*(i-1), pixels.Color(0, 255, 0));
      pixels.show();
      i++;
      reset_state(3000);
    }


  
  // #########################################################################
  #elif defined(TEST_WIFI) // Test para realizarlo con Wifi, 

    angle = data_received;

    if (angle == 0){
      stop();
      reset_state(0);
      pixels.fill(pixels.Color(0, 255, 0));
      pixels.show();
    }
    else{
      turn_left(count_left_RPS, count_right_RPS);
      pixels.fill(pixels.Color(0, 0, 255));
      pixels.show();
      if (distance >= turn_distance(angle)){
        Serial.print("Grados Girados: ");
        Serial.print(angle);
        angle = 0;
        data_received = 0;
        reset_state(1500);
      }
    }


  // #########################################################################
  #elif defined(LISSAJOUS)

    turn_light_right(count_left_RPS, count_right_RPS); 



  // #########################################################################
  #elif defined(MOVE_NO_PID)

    int PWM = 200;
    
    set_wheel_speed(LEFT_WHEEL,  FORWARD,  PWM);
    set_wheel_speed(RIGHT_WHEEL, FORWARD,  PWM);

  // #########################################################################
  #else

    stop();

  #endif


  Serial.println("\t"); // Para que funcione bien el serial_monitor
  //
  //
  //
}



