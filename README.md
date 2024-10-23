# Dron_FPV_freestyle
Las fuentes principales y secundarias de este proyecto han sido: 
  - Brokking.net el enlace de la web es http://www.brokking.net/ymfc-al_main.html
  - Arduproject.es
  - El proyecto de: Design and implementation of a new quadcopter drone prototype controlled through STM32 using the Arduino environment de León Enrique Prieto Bailo.
  - El proyecto de: Design and implementation of advanced location systems for quadrotor drones controlled by STM32 de Alejandro Boadella

En este repositorio de GitHub encontrareis tres carpetas:
  - Primera Carpeta: Calibraciones y tests:
    Aquí se encuentrar diferentes pruevas de testeo y calibracion, como las señales PWM y Motores (podemos calibrar los ESCs desde ahí), testear las señales del
    radio control y del receptor y verlos por pantalla como son esas señales, podemos provar el testeo de nuestra MPU6050 y ver que valores sacamos según las
    las inclinaciones que tenemos (si funciona correctamente), y luego todos los canales donde podemos configurar el mando si es en PPM o señales separadas por
    canales.
    
  - Segunda Carpeta: Código_Dron_Ultrasonido_Vers_A
    Aquí se encuentra la versión del dron funcional (la que está instalada por defecto), esta no incorpora la versión del modo de vuelo acrobatico. Pero ya implementa
    el PID extra del sensor de ultrasonidos y su sistema de colisiones provisional que se tiene que mejorar.
    
  - Tercera Carpeta: Código_Dron_Ultrasonido_Vers_B
    Aquí se encuentra la última versión del dron con la que se ha estado trabajando, no se ha testeado (habrá que revisarlo bien), ya que se ha implementado
    el modo acrobático, y se han realizado intentos de mejora para el sistema de colisiones. Se han realizado pruevas de hacer efecto de amortiguador
    con valores minimos, y con PIDs. 
