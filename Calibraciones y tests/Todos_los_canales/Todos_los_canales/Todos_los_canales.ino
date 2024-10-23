/*
   Generación de señales PWM para control de motores
   Versión para STM32.
*/

// Tiempo de ciclo
#define tiempo_ciclo 5000

// Pines asignados
#define pin_RC        PB11   // Pin para lectura del mando vía PPM
#define pin_motor1    PC6    // Pin motor 1
#define pin_motor2    PC7    // Pin motor 2
#define pin_motor3    PB9    // Pin motor 3
#define pin_motor4    PB8    // Pin motor 4

// Receptor RC
#define numero_canales 8
/*
   Mando_canal[0] = -
   Mando_canal[1] = ROLL
   Mando_canal[2] = PITCH
   Mando_canal[3] = THROTTLE
   Mando_canal[4] = YAW
   Mando_canal[5] = SWD º
   Mando_canal[6] = SWC
   Mando_canal[7] = -
*/

uint32_t pulso_instante[numero_canales * 2 + 2], rise_instante_ant;
uint16_t Mando_canal[numero_canales], canal_ant[numero_canales];
uint16_t Mando_Throttle, Mando_Roll, Mando_Pitch, Mando_Yaw;
uint16_t contador_flaco = 1;

// Tiempo ciclo
uint32_t tiempo_nuevo_ciclo, tiempo_motores_start, contador_ciclos;

// Periodo de las señales PWM
uint16_t ESC1_us, ESC2_us, ESC3_us, ESC4_us;

void setup() {
  Serial.begin(115200);   // Para Serial.print()

  // Declarar pines PWM y poner señales a LOW
  pinMode(pin_motor1, OUTPUT);    // MOTOR 1
  pinMode(pin_motor2, OUTPUT);    // MOTOR 2
  pinMode(pin_motor3, OUTPUT);    // MOTOR 3
  pinMode(pin_motor4, OUTPUT);    // MOTOR 4
  digitalWrite(pin_motor1, LOW);  // MOTOR 1
  digitalWrite(pin_motor2, LOW);  // MOTOR 2
  digitalWrite(pin_motor3, LOW);  // MOTOR 3
  digitalWrite(pin_motor4, LOW);  // MOTOR 4

  // Declarar interrupción en pin_RC. CHANGE = se activa tanto con flanco positivo como con flanco negativo
  pinMode(pin_RC, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_RC), interrupt_RC, CHANGE);
}

void loop() {
  // Un nuevo ciclo cada 5000us (5ms)
  while (micros() - tiempo_nuevo_ciclo < tiempo_ciclo);
  tiempo_nuevo_ciclo = micros();

  PWM_GEN();     // Generar señales PWM para los motores

  // Visualizar por Monitor Serie
  Serial.print("Throttle: ");
  Serial.print(Mando_Throttle);
  Serial.print(" Roll: ");
  Serial.print(Mando_Roll);
  Serial.print(" Pitch: ");
  Serial.print(Mando_Pitch);
  Serial.print(" Yaw: ");
  Serial.println(Mando_Yaw);

  contador_ciclos++;
}

void RECEPTOR_RC() {
  // Solo ejecutamos esta parte si hemos recibido toda la ráfaga, los 18 flancos con la informacion
  // de todos los canales.
  if (contador_flaco == 18) {
    for (uint8_t i = 1; i <= numero_canales; i++) {
      Mando_canal[i] = pulso_instante[2 * i] - pulso_instante[2 * i - 1];

      // Filtrado de señales erroneas (ruido)
      if (i != 5 && canal_ant[i] > 500 && abs(Mando_canal[i] - canal_ant[i]) > 500) {
        Mando_canal[i] = canal_ant[i];
      }
      if (abs(Mando_canal[5] - canal_ant[5]) > 2000) {
        Mando_canal[5] = canal_ant[5];
      }
      canal_ant[i] = Mando_canal[i];
    }
  }

  // Mapeamos las lecturas del mando RC de 1000 a 2000.
  Mando_Throttle  = map(Mando_canal[3], 600, 1600, 1000, 2000);
  Mando_Roll      = map(Mando_canal[1], 600, 1600, 1000, 2000);
  Mando_Pitch     = map(Mando_canal[2], 600, 1600, 1000, 2000);
  Mando_Yaw       = map(Mando_canal[4], 600, 1600, 1000, 2000);
}

void interrupt_RC() {
  if (micros() - pulso_instante[contador_flaco - 1] > 2500) contador_flaco = 0;
  pulso_instante[contador_flaco] = micros();
  contador_flaco++;
}

void PWM_GEN() {
  digitalWrite(pin_motor1, HIGH);  // MOTOR 1
  digitalWrite(pin_motor2, HIGH);  // MOTOR 2
  digitalWrite(pin_motor3, HIGH);  // MOTOR 3
  digitalWrite(pin_motor4, HIGH);  // MOTOR 4
  tiempo_motores_start = micros();

  RECEPTOR_RC(); // Leer mando RC

  // Ajustamos los motores considerando el efecto del ROLL, PITCH y YAW además del THROTTLE.
  ESC1_us = Mando_Throttle + Mando_Roll + Mando_Pitch - Mando_Yaw;  // Motor 1
  ESC2_us = Mando_Throttle - Mando_Roll + Mando_Pitch + Mando_Yaw;  // Motor 2
  ESC3_us = Mando_Throttle - Mando_Roll - Mando_Pitch - Mando_Yaw;  // Motor 3
  ESC4_us = Mando_Throttle + Mando_Roll - Mando_Pitch + Mando_Yaw;  // Motor 4

  // Limitar los valores PWM
  ESC1_us = constrain(ESC1_us, 1000, 2000);
  ESC2_us = constrain(ESC2_us, 1000, 2000);
  ESC3_us = constrain(ESC3_us, 1000, 2000);
  ESC4_us = constrain(ESC4_us, 1000, 2000);

  // Generar PWM
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW); // MOTOR 1
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW); // MOTOR 2
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW); // MOTOR 3
    if (tiempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW); // MOTOR 4
  }
}
