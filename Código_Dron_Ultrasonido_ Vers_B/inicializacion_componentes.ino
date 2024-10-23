/*
Esta función es un conjunto de inicializaciones. Llama a varias funciones que inicializan componentes específicos del sistema: 
LED, Sensor Ultrasónico, Control remoto (RC), Controladores electrónicos de velocidad (ESC) y Unidad de medición inercial (IMU, la MPU6050).
*/  
void inicializacion_componentes() {
  Inicializacion_led();
  Incializacion_emisora();
  Inicializacion_ESCs();
  Inicializacion_IMU();
  Inicializacion_Ultrasonidos();
}




// Configura el pin PC1 como salida para controlar un LED (es el GPIO 13)
void Inicializacion_led() {
  pinMode(PC1, OUTPUT); // Connected to the red LED next to the USB jack No PWM or alternate uses
}





// Se encarga de inicializar la conexión con la emisora FS-I6X
void Incializacion_emisora() {
  pinMode(pin_PPM, INPUT); // Configura el pin pin_PPM como entrada para recibir señales de control remoto.
  attachInterrupt(digitalPinToInterrupt(pin_PPM), Lectura_PPM, CHANGE);
  // Asocia una interrupción al pin pin_PPM para detectar cambios y llama a la función Lectura_PPM cuando ocurre un cambio.
  // La función Lectura_PPM se encarga de leer y procesar la señal  de radio.
}




// Esto inicializa los controladores electrónicos de velocidad (ESC)[Esto se consigue inicializando las señales PWM] para controlar la velocidad de giro de los motores. 
void Inicializacion_ESCs() {
/*
La función setPWM(channel, pin, frecuencia, duty_cycle) configura una señal PWM en un canal específico, en un pin determinado, con una frecuencia
de 250 Hz y un ciclo de trabajo inicial de 0% (lo que indica que el motor no está girando inicialmente).

  Parametros de configuración: 
    -Channel_motor_X: Son los canales de PWM correspondientes a los motores.
    -Pin_motor_X: Son los pines donde están conectados los motores al microcontrolador.
    -250: Es la frecuencia de la señal PWM en Hz (250Hz es un valor comúnen controladores de motores).
    -0: El duty cycle es 0%, lo que indica que los motores no están recibiendo señal para funcionar (están apagados)
*/ 
  TIM_M1_M2->setPWM(channel_motor1, pin_motor1, 250, 0);  // Configura PWM para el motor 1.
  TIM_M1_M2->setPWM(channel_motor2, pin_motor2, 250, 0);  // Configura PWM para el motor 2.
  TIM_M3_M4->setPWM(channel_motor3, pin_motor3, 250, 0);  // Configura PWM para el motor 3.
  TIM_M3_M4->setPWM(channel_motor4, pin_motor4, 250, 0);  // Configura PWM para el motor 4.
}




/*
- Lectura y procesamiento del sensor MPU6050 para obtención de los ángulos de inclinación
Realiza la configuración inicial y calibración de un sensor MPU6050 para medir ángulos de inclinación mediante la lectura de datos de acelerómetro y giroscopio
*/
void Inicializacion_IMU(void) {
// TwoWire Wire2; [SDA: PB11, SCL: PB10]
  // Las variables ya han sido declaradas en la función main
  Wire.begin();                             // Inicia la comunicación I2C
  Wire.beginTransmission(MPU6050_ADDRESS);  // Comienza la comunicación con el MPU6050 usando su dirección I2C
  error = Wire.endTransmission();           // Aseguramos que la comunicación sea exitosa comprobando el valor de retorno de Wire.endTransmission()
  while (error != 0) {                      // Si hay un error, repite el intento tras un pequeño retardo.
    delay(4);
  }


  // Declaración de variables en la función Main
  // Activación del MPU6050:
  Wire.beginTransmission(MPU6050_ADDRESS);  // Comenzar comunicación con sensor
  Wire.write(0x6B);                         // Registro a modificar
  Wire.write(0x00);                         /* Valor asignado a ese registro (00000000), [Envía el valor 0x00 al registro 0x6B para activar el sensor, ya que por defecto el MPU6050 
                                            esta en modo reposo. Este registro controla la energía del dispositivo y se le asigna ese registro para que empiece a funcionar] */
  Wire.endTransmission();                   // Finalizar comunicación con sensor


  // Configuración del giroscopio:
  Wire.beginTransmission(MPU6050_ADDRESS);  // Comenzar comunicación con sensor
  Wire.write(0x1B);                         // Registro a modificar
  Wire.write(0x08);                         /* Valor asignado a ese registro (00001000). Modifica el registro 0x1B para configurar la escala del giroscopio a 500º/s (cambiando
                                            la configuración predeterminada de 250º/s) */
  Wire.endTransmission();                   // Finalizar comunicación con sensor
  

  // Configuración del acelerometro:
  Wire.beginTransmission(MPU6050_ADDRESS);  // Comenzar comunicación con sensor
  Wire.write(0x1C);                         // Registro a modificar
  Wire.write(0x10);                         /* Valor asignado a ese registro (00010000). Modifica el registro 0x1C para ajustar la escala del acelerómetro a +- 8g
                                            en lugar del valor predeterminado +-2g. */
  Wire.endTransmission();                   // Finalizar comunicación con sensor      

  Wire.beginTransmission(MPU6050_ADDRESS);  // Comenzar comunicación con sensor
  Wire.write(0x1A);                         // Registro a modificar
  Wire.write(0x03);                         /* Valor asignado a ese registro. Modifica el registro 0x1A para establecer el filtro paso bajo en el MPU6050, ajusta el filtro
                                            para una mejor lectura. */
  Wire.endTransmission();                   // Finalizar comunicación con sensor

  uint8_t use_manual_calibration = false;       // Si se utiliza la calibración manual, establece un valor predeterminado (cal_int=2000), de lo contrario se inician las variables de calibración en cero.
  if (use_manual_calibration) cal_int = 2000;
  else {
    cal_int = 0;
    manual_gyro_pitch_cal_value = 0;            // Variable calibración del pitch
    manual_gyro_roll_cal_value = 0;             // Variable calibración del roll
    manual_gyro_yaw_cal_value = 0;              // Variable de calibración del yaw
  }

  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {
    Lectura_IMU();
    gyro_x_roll_cal += gyro_x_roll_raw;
    gyro_y_pitch_cal += gyro_y_pitch_raw;
    gyro_z_yaw_cal += gyro_z_yaw_raw;

    acc_x_roll_cal += acc_x_roll_raw;
    acc_y_pitch_cal += acc_y_pitch_raw;
    acc_z_yaw_cal += acc_z_yaw_raw;
    
    delayMicroseconds(4);
  }

  // División entre el número de muetras para calcular el valor medio, que será el offset de cada eje
  gyro_x_roll_cal = gyro_x_roll_cal / 3000;
  gyro_y_pitch_cal = gyro_y_pitch_cal / 3000;
  gyro_z_yaw_cal = gyro_z_yaw_cal / 3000;

  acc_x_roll_cal  = acc_x_roll_cal / 3000;
  acc_y_pitch_cal  = acc_y_pitch_cal / 3000;
  acc_z_yaw_cal  = acc_z_yaw_cal / 3000;

  manual_gyro_roll_cal_value = gyro_x_roll_cal;
  manual_gyro_pitch_cal_value = gyro_y_pitch_cal;
  manual_gyro_yaw_cal_value = gyro_z_yaw_cal;
  
  manual_x_cal_value = acc_x_roll_cal;
  manual_y_cal_value = acc_y_pitch_cal;
  manual_z_cal_value = acc_z_yaw_cal - 4096; 

  Lectura_IMU();  
  acc_total_vector = sqrt((acc_x_roll_raw * acc_x_roll_raw) + (acc_y_pitch_raw * acc_y_pitch_raw) + (acc_z_yaw_raw * acc_z_yaw_raw));    //Calculate the total accelerometer vector.
  if (abs(acc_y_pitch_raw) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y_pitch_raw / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  
  if (abs(acc_x_roll_raw) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x_roll_raw / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }
  angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
}




// Vinula el método para asignar el comportamiento cuando se detecte una interrupción 
void Inicializacion_Ultrasonidos() {
  pinMode(trigger_pin, OUTPUT); // Configura el pin trigger_pin como salida para el pulso de disparo del sensor ultrasónico.
  digitalWrite(trigger_pin, LOW); // Pone el pin trigger_pin en bajo.
  pinMode(echo_pin, INPUT); // Configura el pin echo_pin como entrada para recibir el eco del sensor.
  attachInterrupt(digitalPinToInterrupt(echo_pin), Lectura_Ultrasonido, CHANGE);  // Asocia una interrupción al pin echo_pin para detectar cambios y llama a la función Lectura_Ultrasonido cuando ocurre un cambio.
}