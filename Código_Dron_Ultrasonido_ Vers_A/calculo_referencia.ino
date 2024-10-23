// Este código se utiliza para gestionar y generar referencias en un sistema de control de vuelo basado en el estado y las entradas del control remoto. 
void calculo_referencia(){
  // Llama a las funciones para gestionar el modo y generar las referencias.
  Gestion_modos_vuelo();        // Gestiona el modo de vuelo basado en las entradas del control remoto.
  Generacion_referencias();     // Genera las referencias para los diferentes modos de vuelo.
} 




// Se decide en que modo de vuelo se encuentra el dron en función de los valores del canal 6 del control remoto.
void Gestion_modos_vuelo(){
    // Si el canal 6 está por debajo de 1300, el dron está desarmado, por lo tanto modo_vuelo = FM_Desarmado
    if (canal_rc[6] < 1200) {
      modo_vuelo = FM_Desarmado;
    }
    // Si el canal 6 está entre 1300 y 1700, el dron pasa a estado armado, por lo tanto modo_vuelo = FM_Estable
    else if (canal_rc[6] >= 1400 && canal_rc[6] < 1600) {
      modo_vuelo = FM_Estable;       // El dron esta listo para volar, solo queda seleccionar el modo de vuelo deseado.
    }
    // Si el canal 6 es mayor a 1700 y el dron está armado, pasa a modo de vuelo estable, por lo tanto modo_vuelo = FM_Ultrasonidos
    else if (canal_rc[6] >= 1800) {
      modo_vuelo = FM_Ultrasonidos;
    }
}




// Se generan las referencias específicas (throttle, ángulos, puntos de referencia de altitud) para controlar el dron según el modo de vuelo que se haya establecido en modo_vuelo.
void Generacion_referencias(){

  // Modo Disabled: El dron permence quieto, sin capcaidad para actuar los motores.
  if(modo_vuelo == FM_Desarmado) {
    led_on();               // Enciende un LED indicando que el sistema está preparado para volar, sin hacer girar las hélices.
    // Inicializa los ángulos de pitch y roll.
    angle_pitch = angle_pitch_acc;    
    angle_roll = angle_roll_acc;

    // Resetea los controladores PID.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_output_roll = 0;


    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_output_pitch = 0;


    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    pid_output_yaw = 0;  

  }

  
  // Modo Mounting: Modo de armado haciendo girar motores (se encienden las helices y giran según el trhottle especificado).
  else if(modo_vuelo == FM_mounting){
    throttle = 1200;    // Se fija el acelerador en un valor de 1200 (los motores empiezan a girar indicando que ya se puede volar).

    led_armado();   // Se enciende un indicador de que el dron está armado (led_armado(), es un led rojo de seguridad que empieza a parpadear indicando que ya no se puede manipular el dron.

    // Inicializa los ángulos de pitch y roll con los valores del acelerómetro.
    angle_pitch = angle_pitch_acc;      // Se calibra el ángulo del roll inicial del dron.
    angle_roll = angle_roll_acc;        // Se calibra el ángulo del pitch inicial del dron.

    // Resetea las memorias y errores de los controladores PID para roll, pitch y yaw, para comenzar desde cero en estos parametros.
    pid_i_mem_roll = 0;                 // Reinicio de la variable de control del roll.
    pid_last_roll_d_error = 0;          // Reinicio de la variable de control del roll.

    pid_i_mem_pitch = 0;                // Reinicio de la variable de control del pitch.
    pid_last_pitch_d_error = 0;         // Reinicio de la variable de control del pitch.

    pid_i_mem_yaw = 0;                  // Reinicio de la variable de control del yaw.
    pid_last_yaw_d_error = 0;           // Reinicio de la variable de control del yaw.
  }

  // Modo estable: Ahora ya podemos volar nosotros mismos en un modo estabilizado (la altura no es mantenida ni la posición) pero siempre buscara estar plano el dron.
  else if(modo_vuelo == FM_Estable){
    led_off();
    throttle = canal_rc[3];
    pid_i_mem_altitude = 0;             // Se reinicia los valores del PID para el control de altitud, indicando un vuelo más preciso durante este modo de vuelo.
    pid_last_altitude_d_error = 0;
  }

    // Modo ultrasonidos: Ahora ya podemos volar nosotros mismos en un modo estabilizado con el sistema de colisiones.
  else if(modo_vuelo == FM_Ultrasonidos){
    led_on();
    
    // Si el sensor de ultrasonidos detecta una distancia inferior a la mínima permitida
    if (distancia_filtrada < min_distance){
      // Usar control PID para mantener la distancia objetivo de 50 cm
      error_US = target_distance - distancia_filtrada;
      
      // Ajustar el acelerador (throttle) con el valor calculado por el PID
      throttle_base = canal_rc[3];
      throttle = throttle_base + pid_output_US;  // throttle_base es el valor base de potencia
    }
    // Si la distancia es mayor o igual a la mínima, volar como en modo estable
    else if (distancia_filtrada >= min_distance){
      throttle = canal_rc[3];   // El acelerador se controla manualmente por el canal 3 de la emisora
      // Reiniciar valores de memoria del PID cuando no se usa el control de altitud
    }
    pid_i_mem_altitude = 0;             
    pid_last_altitude_d_error = 0;
  }
} 