 #include <Wire.h>
 #include "LowPass.h"


// Flight Mode Enumeration and Initialization
enum Modo_de_Vuelo{
    FM_Desarmado,
    FM_mounting,
    FM_Estable,
    FM_Ultrasonidos,
  };




 Modo_de_Vuelo modo_vuelo = FM_Desarmado;




 // HC-SR04
#define trigger_pin PC3 // Trigger Pin Ultrasonico (GPIO 11)
#define echo_pin PC2 // Echo Pin Ultrasonico (GPIO 12)

float distancia;

LowPass<2> lp(3, 1e3, true);

float distancia_filtrada;
float sent_last_pulse, tiempo, comienzo_pulso, final_pulso, distancia_calculada;

bool pulso_enviado;
long velocidad_sonido = 343; // a 20 º 343 m/s a 0º 331 m/s
unsigned long timeLast;
float distancia_previa, distancia_previa_filtrada;





// FlightSky i6
#define pin_PPM PB11 // PB11
#define numero_canales 8
uint64_t pulse_instant[numero_canales * 2 + 2];
uint16_t canal_rc[numero_canales];
uint8_t contador_flancos = 1;
int16_t throttle;
int16_t throttle_base;




// Battery(It needs to be a 3S battery or the code should be modified):
#define pin_BAT PA5
float aviso_bateria_baja = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).
float voltaje_bateria;
float compensacion_bateria = 40.0;       // Increment if the drone falls when the battery voltage falls.
                                         // Decrement if the drone rises when the battery voltage falls.




// PID: Variables
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;

float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;

float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;

float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;




float pid_i_mem_altitude, pid_last_altitude_d_error;




// PID: Roll
float pid_p_gain_roll = 0.9;
float pid_i_gain_roll = 0.009;
float pid_d_gain_roll = 4;
int pid_max_roll = 400;


// PID: Pitch
float pid_p_gain_pitch = 0.9;
float pid_i_gain_pitch = 0.009;
float pid_d_gain_pitch = 4;
int pid_max_pitch = 400;


// PID: Yaw
float pid_p_gain_yaw = 0.75;
float pid_i_gain_yaw = 0.01;
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;


// PID: ultrasonido
float Kp = 1.5;  // Ganancia proporcional
float Ki = 0.01; // Ganancia integral
float Kd = 4;  // Ganancia derivativa
    
float pid_max_integral = 400.0;   // Limitar el valor máximo de la salida integral para evitar saturación
float pid_min_integral = -100.0;  // Limitar el valor máximo de la salida integral para evitar saturación
float pid_p;  // Salida proporcional
float pid_d;  // Salida derivativa
float pid_output_US; // Las tres salidas del PID
float min_distance = 50.0;       // Distancia mínima permitida (50 cm)
float target_distance = 50.0;    // Setpoint de distancia objetivo a 50 cm
float error_US;
float pid_last_error;
float pid_i_mem;





// MPU6050: Address, setup and callibration
#define MPU6050_ADDRESS 0x68

boolean auto_level = true;
int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;

int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;

int16_t manual_x_cal_value = 0;
int16_t manual_y_cal_value = 0;
int16_t manual_z_cal_value = 0;

int16_t cal_int;
int16_t temperature;

int16_t acc_x_roll_raw, acc_y_pitch_raw, acc_z_yaw_raw;

int16_t gyro_x_roll_raw, gyro_y_pitch_raw, gyro_z_yaw_raw;

int32_t acc_total_vector;

int32_t gyro_x_roll_cal, gyro_y_pitch_cal, gyro_z_yaw_cal;

int32_t acc_x_roll_cal, acc_y_pitch_cal, acc_z_yaw_cal;

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;




// ESC
#define pin_motor1 PC6 // Pin motor 1 GPIO 6
#define pin_motor2 PC7 // Pin motor 2 GPIO 5
#define pin_motor3 PB9 // Pin motor 3 GPIO 10
#define pin_motor4 PB8 // Pin motor 4 GPIO 9
int16_t esc_1, esc_2, esc_3, esc_4;

TIM_TypeDef *TIM_DEF_M1_M2 = TIM3;
TIM_TypeDef *TIM_DEF_M3_M4 = TIM4;

uint32_t channel_motor1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor1), PinMap_PWM));
uint32_t channel_motor2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor2), PinMap_PWM));
uint32_t channel_motor3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor3), PinMap_PWM));
uint32_t channel_motor4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor4), PinMap_PWM));

HardwareTimer *TIM_M1_M2 = new HardwareTimer(TIM_DEF_M1_M2);
HardwareTimer *TIM_M3_M4 = new HardwareTimer(TIM_DEF_M3_M4);




// Loop timer
uint32_t loop_timer;




// Error signal
uint8_t error;




// Setup routine
void setup() {
  Serial.begin(57600);  // Inicia la comunicación serie a 57600 baudios
  delay(5000);          // Pausa de 5 segundos

  inicializacion_componentes(); // Inicializa los componentes del sistema
  led_off();  // Apaga el LED

  // Espera hasta que los valores de los canales del control remoto sean mayores o iguales a 990 (todos al minimo)
  while (canal_rc[1] < 990 || canal_rc[2] < 990 || canal_rc[3] < 990 || canal_rc[4] < 990) {
    Lectura_Emisora();  // Lee los valores del control remoto
    delay(4); // Pausa de 4 ms entre lecturas
  }

  loop_timer = micros();  // Guarda el tiempo actual en microsegundos
  led_on(); // Enciende el LED
}


// Main routine
void loop() {
  calculo_referencia(); // Calcula las referencias necesarias
  unidad_lectura_procesado(); // Lee y procesa datos de sensores
  Controladores_PID();  // Ejecuta los controladores PID para ajuste de control
  Actuadores(); // Controla los actuadores
  Luces_Led();  // Actualiza el estado de las luces LED

  // Espera hasta que pasen 4000 microsegundos desde el inicio del bucle
  while (micros() - loop_timer < 4000);
  loop_timer = micros();  // Reinicia el temporizador para el próximo ciclo
}
