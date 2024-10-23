// Esta parte del sistema se encarga de recoger datos de varios sensores y entradas de la emisora. Luego procesa los datos del MPU6050 para calcular la orientación del dron.
void unidad_lectura_procesado() {
  Lectura_bateria();           // Lee el voltaje de la batería.
  Lectura_Emisora();           // Lee los canales del control remoto.
  Lectura_IMU();               // Lee los valores de aceleración y giroscopio del IMU (unidad MPU6050).
  Procesamiento_IMU();         // Procesa los datos del IMU para calcular la orientación del dron.
//  Lectura_PPM();             // Lee las señales PPM del control remoto.   
//  Lectura_Ultrasonido()
}




// Reads battery voltage:
void Lectura_bateria() {
  
  // Load the battery voltage to the voltaje_bateria variable.
  // The STM32 uses a 12 bit analog to digital converter.
  // analogRead => 0 = 0V ..... 4095 = 3.3V
  // The voltage divider (1k & 10k) is 1:11.
  // analogRead => 0 = 0V ..... 4095 = 36.3V
  // 36.3 / 4095 = 112.81.
  // The variable voltaje_bateria holds 1050 if the battery voltage is 10.5V.
  voltaje_bateria = (float)analogRead(PA5) / 112.81;
  
  // The battery voltage is needed for compensation.
  // A complementary filter is used to reduce noise.
  // 1410.1 = 112.81 / 0.08.
  voltaje_bateria = voltaje_bateria * 0.92 + ((float)analogRead(PA5) / 1410.1);                           // If it gives a lower value, map the output to the real values!!!
  
  // Default setting is 10.5V 3S.
  if (voltaje_bateria > 6.0 && voltaje_bateria < aviso_bateria_baja && error == 0)Serial.println("error bat");

}




// Lee los valores de la emisora
void Lectura_Emisora() {
  if (contador_flancos == 18) {
    for (int i = 1; i <= numero_canales; i++) {
      // De estos 18 flancos, el primero y el último no nos aportan información. Recorremos los demás
      // flancos. Para calcular la lontigud de cada pulso, hacemos la resta del flanco actual, menos el
      // flanco anterior. Al haber guardado el instante (micros()) en el que se da cada flanco, con esta
      // resta calculamos la anchura de cada pulso.
      canal_rc[i] = map(pulse_instant[2 * i] - pulse_instant[2 * i - 1], 600, 1600, 1000, 2000);
      
      /*
      canal_del_mando[i] = instante_de_pulso[2 * i] - instante_de_pulso[2 * i - 1];

      // De forma aleatoria el repector envía señales erroneas (ruido). Es necesario filtrar.
      if (i != 5 && canal_anterior[i] > 500 && abs(canal_del_mando[i] - canal_anterior[i]) > 500)canal_del_mando[i] = canal_anterior[i];
      if (abs(canal_del_mando[5] - canal_anterior[5]) > 2000)canal_del_mando[5] = canal_anterior[5];
      canal_anterior[i] = canal_del_mando[i];
      */

    }
  }
  /*
  // Mapeamos las lecturas del mando RC de -30 a 30.
  Mando_Roll      = map(canal_del_mando[1], 600, 1600, -30, 30);    // Mapear pulso entre -30º y 30º
  Mando_Pitch     = map(canal_del_mando[2], 600, 1593, -30, 30);    // Mapear pulso entre -30º y 30º
  Mando_Throttle  = map(canal_del_mando[3], 600, 1600, 1000, 2000); // Mapear pulso entre 1000us y 2000us
  Mando_Yaw       = map(canal_del_mando[4], 600, 1600, -30, 30);    // Mapear pulso entre -30º y 30º
  Mando_SWD       = map(canal_del_mando[5], 600, 1600, 1000, 2000); // Mapear pulso entre 1000us y 2000us (2 pos.)
  Mando_SWC       = map(canal_del_mando[6], 600, 1600, 1000, 2000); // Mapear pulso entre 1000us y 2000us (3 pos.)
  */
}




// Ponemos el contador a 0 entre series de rachas de flancos
void Lectura_PPM() {
  // Aunque el receptor es de 6 canales recibimos 8 pulsos, recibimos 18 flancos (8*2+2). Para
  // transmitir n canales, recibiremos n+2 flancos tanto positivos como negativos.
  // Se pone el contador a 0:
  if (micros() - pulse_instant[contador_flancos - 1] > 2500) contador_flancos = 0;
  // Guardamos en esta variable el instante (micros()) en el que se lee un flanco, tanto positivo como negativo:
  // Índice del array de 0 a 17 --> 18 elementos
  pulse_instant[contador_flancos] = micros();
  contador_flancos++;
}




// Lectura del sensor MPU6050 para obtención de los ángulos de inclinación
void Lectura_IMU(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);          // Comenzar comunicación con sensor
  Wire.write(0x3B);                                 // Registro a modificar
  Wire.endTransmission();                           // Finalizar comunicación con sensor
  Wire.requestFrom(MPU6050_ADDRESS, 14);            // Solicitar un total de 14 registros

  while (Wire.available() < 14);                      // Esperamos hasta recibir los 14 bytes
  acc_x_roll_raw = Wire.read() << 8 | Wire.read();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acc_y_pitch_raw = Wire.read() << 8 | Wire.read();   // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc_z_yaw_raw = Wire.read() << 8 | Wire.read();     // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  temperature = Wire.read() << 8 | Wire.read();       // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)

  gyro_x_roll_raw = Wire.read() << 8 | Wire.read();   // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyro_y_pitch_raw = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyro_z_yaw_raw = Wire.read() << 8 | Wire.read();    // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
  
  // Restar offset
  acc_x_roll_raw -= manual_x_cal_value;
  acc_y_pitch_raw -= manual_y_cal_value;
  acc_z_yaw_raw -= manual_z_cal_value;

  gyro_x_roll_raw -= manual_gyro_roll_cal_value;
  gyro_y_pitch_raw -= manual_gyro_pitch_cal_value;
  gyro_z_yaw_raw -= manual_gyro_yaw_cal_value;
}





// Procesamiento del sensor MPU6050 para obtención de los ángulos de inclinación
void Procesamiento_IMU() {
  // Conversión a grados por segundo (DPS: degree per secord), 65.5 en raw, significa que gira a 1º/s. gyro_roll_raw/65.5 lo pasamos a º/S
  // Calcular ángulo con lecturas de giroscopio (º/s * s = º)// ang(º) = ang_ant(º) + vel_ang(º/s)*dt(s)
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_x_roll_raw / 65.5) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_y_pitch_raw / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_z_yaw_raw / 65.5) * 0.3);


  angle_pitch += (float)gyro_y_pitch_raw * 0.0000611;   // 0.0000611 tiempo en segundos por cada unidad de medida del giroscopio
  angle_roll += (float)gyro_x_roll_raw * 0.0000611;

  // El ángulo de pitch y roll se ajustan para tener en cuenta la rotación en el eje yaw. Evitando que las rotaciones en el eje yaw afecten los ángulos pitch y roll de forma incorrecta.
  angle_pitch -= angle_roll * sin((float)gyro_z_yaw_raw * 0.000001066);
  angle_roll += angle_pitch * sin((float)gyro_z_yaw_raw * 0.000001066);

  // Se calcula la acceleración total en los tres ejes (x,y,z)
  acc_total_vector = sqrt((acc_x_roll_raw * acc_x_roll_raw) + (acc_y_pitch_raw * acc_y_pitch_raw) + (acc_z_yaw_raw * acc_z_yaw_raw));

  // Si los valores de aceleración en los ejes X,Y son menores a la aceleración total, se calculan los ángulos de inclinación de pitch y roll usando funciones trigonometricas
  if (abs(acc_y_pitch_raw) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y_pitch_raw / acc_total_vector) * 57.296;
  }
  if (abs(acc_x_roll_raw) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x_roll_raw / acc_total_vector) * 57.296;
  }

  // Para corregir el drift. Los ángulos finales de pitch y roll se obtienen combinando los ángulos obtenidos del giroscopio y del acelerómetro, aplicando un filtro complementario
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;    // 99,96% del giroscopio
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;       // 0,04% del acelerómetro

  // Se calculan los ajustes de nivelación para pitch y roll  multiplicando los ángulos  por un factor de 15 son los grados (30º a los que esta limitado)
  // Código para el modo de vuelo estable limitadoa 30º y acrobatico con grados ilimitados
  if (modo_vuelo == FM_Estable){
    pitch_level_adjust = angle_pitch * grados_estable;
    roll_level_adjust = angle_roll * grados_estable;
  }
  else if (modo_vuelo == FM_Acrobatico){
    pitch_level_adjust = angle_pitch * grados_ilimitados;
    roll_level_adjust = angle_roll * grados_ilimitados;
  }
  else {
    pitch_level_adjust = angle_pitch * grados_estable;
    roll_level_adjust = angle_roll * grados_estable;
  }

  // Si la función de nivel automático está desactivada, los ajustes se establecen en cero.
  if (!auto_level) {
    roll_level_adjust = 0;
    pitch_level_adjust = 0;
  }
}




// Lectura y procesado del sensor ultrasónico HC-SR04
void Lectura_Ultrasonido() {
    /*
  -pulso_enviado: Indica si un pulso ultrasónico ha sido enviado.
  -echo_pin: Pin donde se recibe la señal de eco del sensor ultrasónico.
  -comienzo_pulso: Marca el tiempo de inicio del eco.
  -final_pulso: Marca el tiempo de finalización del eco.
  -tiempo: Duración del eco en microsegundos.
  -velocidad_sonido: Velocidad del sonido en el aire (usualmente ~34300 cm/s).
  -distancia: Distancia calculada del objeto.
  -distancia_filtrada: Distancia filtrada.
  -lp: Filtro de paso bajo (low-pass filter).
  -timeLast: Tiempo de la última medición. 
  */

  // // CHEQUEO DEL PULSE SENT: Se comprueva si se ha enviado un pulso ultrasónico
  if (pulso_enviado) {   // La función sólo procede si el pulso se ha enviado, lo que significa que un pulso ultrasónico ha sido enviado y estamos esperando el eco.
    
    // CHEQUEO DEL ESTADO DEL PIN ECHO_PIN: 
    if (digitalRead(echo_pin) == HIGH) {    // Si el pin echo esta en HIGH sgifnifica que el eco ha comenzado
      comienzo_pulso = micros();               // Se registra el tiempo de inicio en el que el eco se ha empezado ha recivirse
    
    // ECHO FINALIZADO:
    } else {  // Si el pin echo no está en estado HIGH (asumiendo que está en LOW), significa que el echo ha terminado. 
      final_pulso = micros();   // Se registra el tiempo de finalización 
      tiempo = final_pulso - comienzo_pulso;   // Se calcula la duración del echo, como la diferencia entre el timepo de fin y el tiempo de inicio (en microsegundos)

      //CÁLCULO DE LA DISTANCIA:
      distancia_calculada = tiempo / 1e6 * velocidad_sonido * 1e2 / 2.0;  // La distancia se calcula usando la duración del eco y la velocidad del sonido en el aire. Se divide por 2 porque el sonido viaja hacia el objeto y regresa.
      // distancia(metros) = velocidad(metros/segundos)*tiempo(segundos), se divide por dos porque va y vuelve, y por 1e6 por el factor de conversión de microsegundos a segundos

      // AJUSTE DE LA DISTANCIA MÍNIMA:
      if (distancia_calculada < 10) {   // Si la distancia calculada es menor que 10 cm, se ajusta a 10 cm para evitar mediciones erróneas.
        distancia_calculada = 10;       // Se ajusta para que nunca sea más pequeña que este valor
      }

      // PROCESAMIENTO DE LA DISTANCIA VÁLIDA: 
      if (distancia_calculada < 200) {    // Si la distancia calculada es menor que 200 cm, se considera válida.

        // Se actualizan las variables distancia_previa y distancia_previa_filtrada con los valores actuales
        distancia_previa = distancia;       // Almacenamos la distancia en la distancia anterior (previa)
        distancia_previa_filtrada = distancia_filtrada; // Lo mismo pero para la distancia anterior con el filtro paso bajo

        distancia = distancia_calculada;   // Esta distancia es la actual la previa es la anterior a esta
        distancia_filtrada = lp.filt(distancia);    // Se aplican filtros paso bajo para suavizar las lecturas

        timeLast = micros();    // Actualización del tiempo de la última medición
      }
      pulso_enviado = false;   // El pulso ultrasónico ha sido procesado
    }
  }
}