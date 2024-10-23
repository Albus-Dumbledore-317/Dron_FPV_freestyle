// Controladores PID, para ajustar la posición del dron suavemente.
void Controladores_PID() {
  cnt_attitude_sp();    // Calcula los valores objetivo para los ángulos roll, pitch, yaw a partir de las entradas del control remoto.
  cnt_attitude_pid();   // Aplica los cálculos PID para ajustar los ángulos del dron hacia los valores objetivos calculados
  cnt_altitude_pid_ultrasonido_pid();
}




// Calcula los valores objetivo para los controladores PID de roll, pitch y yaw basandose en las entradas del control remoto
void cnt_attitude_sp() {

  // ROLL: Establece los valors objetivos basado en la entrada del canal 2 del control remoto. Ajusta el valor restando roll_level_adjust y escalándolo.
  pid_roll_setpoint = 0;  // Se iniciliza el valor objetivo en 0.
   /*
  Si la señal del canal 2 del control remoto es mayor a 1501, calcula la diferencia (que representa cuánto debe ajustarse el roll) y asigna ese valor a pid_roll_setpoint. 
  Si es menor a 1499, hace lo mismo pero en sentido contrario.
  */
  if (canal_rc[2] > 1501) {     // Si el canal 2 del roll de la emisora está fuera de la zona muerta (1501 o más) se ajusta   el valor objetivo
    pid_roll_setpoint = canal_rc[2] - 1501;
  }

  else if (canal_rc[2] < 1499) {  // Si el canal 2 del roll de la emisora está fuera de la zona muerta (1499 o menos) se ajusta el valor objetivo
    pid_roll_setpoint = canal_rc[2] - 1499;
  }

  pid_roll_setpoint -= roll_level_adjust; // Se ajusta con el valor del roll_level_adjust del unidad_lectura_procesado para corregir posibles inclinaciones no deseadas
  pid_roll_setpoint /= 2.0;   // Se divide por tres para suavizar la respuesta y reducir la sensibilidad



  // PITCH: Establece los valors objetivos basado en la entrada del canal 1 del control remoto. Ajusta el valor restando pitch_level_adjust y escalándolo.
  pid_pitch_setpoint = 0; // Se inicializa el valor objetivo en 0.
   /*
  Si la señal del canal 1 del control remoto es mayor a 1501, calcula la diferencia (que representa cuánto debe ajustarse el pitch) y asigna ese valor a pid_pitch_setpoint. 
  Si es menor a 1499, hace lo mismo pero en sentido contrario.
  */
  if (canal_rc[1] > 1501) {     // Si el canal 1 del pitch de la emisora está fuera de la zona muerta (1501 o más) se ajusta   el valor objetivo
    pid_pitch_setpoint = canal_rc[1] - 1501;  
  }
  else if (canal_rc[1] < 1499) {  // Si el canal 1 del pitch de la emisora está fuera de la zona muerta (1499 o menos) se ajusta el valor objetivo
    pid_pitch_setpoint = canal_rc[1] - 1499;
  }

  pid_pitch_setpoint -= pitch_level_adjust; // Se ajusta con el valor del pitch_level_adjust del unidad_lectura_procesado para corregir posibles inclinaciones no deseadas
  pid_pitch_setpoint /= 2.0;  // Se divide por tres para suavizar la respuesta y reducir la sensibilidad



  // YAW: Establece pid_yaw_setpoint basado en la entrada del canal 4, pero solo si el canal 3 está por encima de 1050.
  pid_yaw_setpoint = 0; // Inicializa el punto de referencia del PID para el yaw a 0. 

  // Solo ajusta el punto de referencia si la señal del canal 3 es mayor a 1050. Luego, dependiendo de la señal del canal 4, ajusta y escala el valor para pid_yaw_setpoint.
  if (canal_rc[3] > 1050) {
    if (canal_rc[4] > 1550) {
      pid_yaw_setpoint = (canal_rc[4] - 1550) / 2.0;
    }
    else if (canal_rc[4] < 1450) {
      pid_yaw_setpoint = (canal_rc[4] - 1450) / 2.0;
    }
  }
}




  /*
  Esta función calcula las salidas del controlador PID para los ejes de roll (alabeo), pitch (cabeceo) y yaw (guiñada) basándose en las entradas de los sensores 
  y las señales del control remoto. 
  */
void cnt_attitude_pid() {

  // Roll calculations: Ganancia Integral (Roll)
  // Error proporcional (Roll): Control Proporcional
  pid_error_temp = gyro_roll_input - pid_roll_setpoint; // Calcula el error de roll comparando la entrada del giroscopio con el valor objetivo del roll calculado antes.
  // Memoria Integral (Roll):
  // Actualiza la memoria integral sumando el producto de la ganancia integral y el error, y luego limita la memoria integral al máximo permitido para evitar el "windup".
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  
  if (pid_i_mem_roll > pid_max_roll) {    // Se limita la memoria integral para evitar sobrecorreciones
    pid_i_mem_roll = pid_max_roll;
  }
  else if (pid_i_mem_roll < pid_max_roll * -1) {
    pid_i_mem_roll = pid_max_roll * -1;
  } 

  // Salida del PID (Roll):
  /*
  Calcula la salida del PID combinando las ganancias proporcional, integral y derivativa, y luego limita la salida del PID al máximo permitido. 
  Finalmente, guarda el error actual para usarlo en el siguiente cálculo derivativo.
  */
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) {
    pid_output_roll = pid_max_roll;
  }
  else if (pid_output_roll < pid_max_roll * -1) {
    pid_output_roll = pid_max_roll * -1;
  }

  pid_last_roll_d_error = pid_error_temp;   // Calcula la tasa de cambio del error para predecir la tendencia futura y suavizar el sistema, evitando sobrecorreciones bruscas



  //Pitch calculations: Ganancia Integral (Pitch)
  // Error proporcional (Pitch):
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint; // Calcula el error de pitch comparando la entrada del giroscopio con el valor objetivo del pitch calculado antes.

  // Memoria Integral (Pitch):
  // Actualiza la memoria integral sumando el producto de la ganancia integral y el error, y luego limita la memoria integral al máximo permitido para evitar el "windup".
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  if (pid_i_mem_pitch > pid_max_pitch) {
    pid_i_mem_pitch = pid_max_pitch;
  }
  else if (pid_i_mem_pitch < pid_max_pitch * -1) {
    pid_i_mem_pitch = pid_max_pitch * -1; 
  }

  // Salida del PID (Pitch):
  /*
  Calcula la salida del PID combinando las ganancias proporcional, integral y derivativa, y luego limita la salida del PID al máximo permitido. 
  Finalmente, guarda el error actual para usarlo en el siguiente cálculo derivativo.
  */
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  if (pid_output_pitch > pid_max_pitch) {
    pid_output_pitch = pid_max_pitch;
  }
  else if (pid_output_pitch < pid_max_pitch * -1) {
    pid_output_pitch = pid_max_pitch * -1;
  }

  pid_last_pitch_d_error = pid_error_temp;




  // Yaw calculations: Ganancia Integral (Yaw)
  // Error proporcional (Yaw):
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint; // Calcula el error de yaw comparando la entrada del giroscopio con el valor objetivo del yaw calculado antes.

  // Memoria Integral (Yaw):
  // Actualiza la memoria integral sumando el producto de la ganancia integral y el error, y luego limita la memoria integral al máximo permitido para evitar el "windup".
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

  if (pid_i_mem_yaw > pid_max_yaw) {
    pid_i_mem_yaw = pid_max_yaw;
  }
  else if (pid_i_mem_yaw < pid_max_yaw * -1) {
    pid_i_mem_yaw = pid_max_yaw * -1;
  }

  // Salida del PID (Yaw):
  /*
  Calcula la salida del PID combinando las ganancias proporcional, integral y derivativa, y luego limita la salida del PID al máximo permitido. 
  Finalmente, guarda el error actual para usarlo en el siguiente cálculo derivativo.
  */
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

  if (pid_output_yaw > pid_max_yaw) {
    pid_output_yaw = pid_max_yaw;
  }
  else if (pid_output_yaw < pid_max_yaw * -1) {
    pid_output_yaw = pid_max_yaw * -1;
  }

  pid_last_yaw_d_error = pid_error_temp;
}




// PID para control de altitud basado en ultrasonido
void cnt_altitude_pid_ultrasonido_pid() {
  // Calcular la salida proporcional
  pid_p = Kp * error_US;

  // Calcular la salida integral
  pid_i_mem += Ki * error_US;

  // Limitar la memoria integral para evitar acumulación excesiva
  if (pid_i_mem > pid_max_integral) {
    pid_i_mem = pid_max_integral;
  }
  else if (pid_i_mem < pid_min_integral) {
    pid_i_mem = pid_min_integral;
  }

  // Calcular la salida derivativa
  pid_d = Kd * (pid_last_error - error_US);
  pid_last_error = error_US; // Actualizar el error anterior para el siguiente cálculo

  // Sumar las tres salidas (P + I + D) para obtener la salida final del PID
  pid_output_US = pid_p + pid_i_mem + pid_d;
}