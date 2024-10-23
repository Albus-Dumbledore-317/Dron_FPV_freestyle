// Gestiona los actuadores del sistema
void Actuadores() {
  Salida_ESCs();  // Calcula las señales de salida para los ESC en función del modo de vuelo y las salidas PID.
  PWM_ESCs();      // Envía los valores calculados de los ESC a los pines correspondientes para controlar la velocidad de los motores.
  Pulso_Ultrasonido();     // Genera un pulso ultrasónico para medir la distancia
}




// Calcula las salidas de los ESC para controlar los motores del dron en diferentes modos de vuelo 
void Salida_ESCs() {
  
  // MODO Mounting: 
  if (modo_vuelo == FM_mounting) {
    if (throttle > 1200) {  // Se establece un throttle para giarar los motores de 1200 y se asegura que no se exceda este valor (ya que el dron no debe despegar, solo avisar que ya esta operativo para volar).
      throttle = 1200;
    }
    // Calcula las salidas de los ESC (esc_1, esc_2, esc_3, esc_4) basándose en throttle y las salidas del controlador PID para pitch, roll y yaw.
    // Las ecuaciones combinan el throttle más o menos las correcciones de los controladores PID para cada eje (cada motor es ajustado según su posición en el dron).
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
  }


  // MODO Estable: 
  else if (modo_vuelo == FM_Estable) {
    if (throttle > 2000) {  // Se asegura que el throttle nunca pase de 2000 (a través de los pulsos del mando ya sería imposible, pero por si acaso que no pueda pasar de ahí)
      throttle = 2000;
    }
    // Calcula las salidas de los ESC (esc_1, esc_2, esc_3, esc_4) basándose en throttle y las salidas del controlador PID para pitch, roll y yaw.
    // Las ecuaciones combinan el throttle más o menos las correcciones de los controladores PID para cada eje (cada motor es ajustado según su posición en el dron).
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
  }
  
    // MODO Estable: 
  else if (modo_vuelo == FM_Ultrasonidos) {
    if (throttle > 2000) {  // Se asegura que el throttle nunca pase de 2000 (a través de los pulsos del mando ya sería imposible, pero por si acaso que no pueda pasar de ahí)
      throttle = 2000;
    }
    // Calcula las salidas de los ESC (esc_1, esc_2, esc_3, esc_4) basándose en throttle y las salidas del controlador PID para pitch, roll y yaw.
    // Las ecuaciones combinan el throttle más o menos las correcciones de los controladores PID para cada eje (cada motor es ajustado según su posición en el dron).
    esc_1 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
  }


  // OTRO MODO: Si no está en ninguno de los modos anteriores, se fijan las salidas de los ESC a 1000, para que esten apagados y no encendidos y no tengamos una situación peligrosa.
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  // Asegura que las salidas de los ESC no sean menores de 950:
  if (esc_1 < 1000) esc_1 = 950;
  if (esc_2 < 1000) esc_2 = 950;
  if (esc_3 < 1000) esc_3 = 950;
  if (esc_4 < 1000) esc_4 = 950;

  // Asegura que las salidas de los ESC no sean mayores de 2000.
  if (esc_1 > 2000) esc_1 = 2000;
  if (esc_2 > 2000) esc_2 = 2000;
  if (esc_3 > 2000) esc_3 = 2000;
  if (esc_4 > 2000) esc_4 = 2000;

}




// Envía las señales PWM calculadas a los ESC correspondientes (controla los motores a través de temporizadores: TIM_M1_M2 y TIM_M3_M4 ajustando el ciclo de trabajo de la señal PWM)
void PWM_ESCs(){
  TIM_M1_M2->setCaptureCompare(channel_motor1, esc_1, MICROSEC_COMPARE_FORMAT);   // (setCaptureCompare): Envía el valor calculado de PWM a cada motor individual (motor1, motor2, etc.).
  TIM_M1_M2->setCaptureCompare(channel_motor2, esc_2, MICROSEC_COMPARE_FORMAT);   // MICROSEC_COMPARE_FORMAT indica que el valor enviado está en formato de microsegundos, controlando la duración del pulso de PWM para los ESC.
  TIM_M3_M4->setCaptureCompare(channel_motor3, esc_3, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor4, esc_4, MICROSEC_COMPARE_FORMAT);
}




// Maneja el envío de pulsos ultrasónicos para medir distancias
void Pulso_Ultrasonido() {
  if (micros() - sent_last_pulse > 7500) {  // micros devuelve el tiempo actual en microsegundos
                                            // Si ha pasado más de 7500 microsegundos desde el último pulso del sensor ultrasonidos, se procede a enviar otro.

    sent_last_pulse = micros();

    digitalWrite(trigger_pin, HIGH);  // Activa el pin trigger para enviar el pulso ultrasónico

    delayMicroseconds(10);    // Espera 10 microsegundos antes de desactivar el pulso

    digitalWrite(trigger_pin, LOW);   // Apaga el pin, terminando el pulso ultrasónico.

    pulso_enviado = true;  // Se ha enviado el pulso.
  }
}
