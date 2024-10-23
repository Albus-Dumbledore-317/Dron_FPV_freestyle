void Luces_Led() {
  led_on();
  led_off();
}

// LED ON
void led_on() {
  digitalWrite(PC1, HIGH);
}

// LED OFF
void led_off() {
  digitalWrite(PC1, LOW);
}


void led_armado() {
  unsigned long startTime = millis(); // Guarda el tiempo de inicio
  while (millis() - startTime < 100) { // Duración total de 3 segundos
    led_on();
    delay(500); // Mantén el LED encendido durante 1 segundo
    led_off();
    delay(500); // Mantén el LED apagado durante 1 segundo
  }
}