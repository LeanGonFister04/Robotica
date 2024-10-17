#include <Servo.h>
#include <IRremote.hpp>

// Control manual/automatico
boolean ES_MANUAL;
constexpr uint16_t CONTROL = 0x9; // Botón Cambia de Control

// Pines del receptor IR
constexpr uint8_t RECV_PIN{2}; // Pin del receptor IR

// Pines del L298N para los motores
const int motor1Pin1 = 7;
const int motor1Pin2 = 8;
const int motor2Pin1 = 9;
const int motor2Pin2 = 11;

const int en_A = 5;
const int en_B = 6;

// Valores de los botones
constexpr uint16_t POWER = 0x43;     // Botón POWER
constexpr uint16_t ADELANTE = 0x18;  // Botón ADELANTE
constexpr uint16_t ATRAS = 0x52;     // Botón ATRÁS
constexpr uint16_t IZQUIERDA = 0x8;  // Botón IZQUIERDA
constexpr uint16_t DERECHA = 0x5A;   // Botón DERECHA

// Valores del sensor y servo
Servo servoMotor;
const int trigPin = A5; // out
const int echoPin = A4; // in
const int servoPin = 3;

// VALORES DE LOS PINES DE LOS LEDS
const int pinRojo = 4;
const int pinVerde = 1;
const int pinAzul = 13;

unsigned long lastCommandTime = 0;  // Para registrar el último tiempo en el que se recibió una señal
const unsigned long commandTimeout = 150;  // Tiempo en milisegundos para determinar si el botón fue soltado


// Función para recibir el código IR
uint16_t irReceive()
{
    uint16_t received{0};

    if (IrReceiver.decode())
    {
        IrReceiver.printIRResultShort(&Serial);
        if (IrReceiver.decodedIRData.protocol == UNKNOWN)
        {
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        if (IrReceiver.decodedIRData.protocol == NEC)
        {
            received = IrReceiver.decodedIRData.command;
        }
        IrReceiver.resume();
    }
    return received;
}

void setup()
{
    Serial.begin(9600);

    ES_MANUAL = true;

     // Configuración de los pines del motor como salida
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(en_A, OUTPUT);
    pinMode(en_B, OUTPUT);

    // CONFIGURAR PINES DEL LEDs
    pinMode(pinRojo, OUTPUT);
    pinMode(pinVerde, OUTPUT);
    pinMode(pinAzul, OUTPUT);

    // Inicializar los motores apagados
    apagarMotores();
  
  	// Configurar el servo
  	servoMotor.attach(servoPin);
  
  	// Configurar los pines del sensor ultrasónico
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Inicializar la recepción del IR
    IrReceiver.begin(RECV_PIN);
}

void loop() {
    uint16_t irCode = irReceive();

    if (irCode == CONTROL) {
        ES_MANUAL = !ES_MANUAL;
        setColor(180, 0, 0);
    }

    if (ES_MANUAL) {
        setColor(0, 0, 0);
        if (irCode != 0) {
            lastCommandTime = millis();
        }
        // Control manual por IR
        switch (irCode) {
            case POWER:
                apagarMotores();
                break;
            case ADELANTE:
                irAdelante();
                break;
            case IZQUIERDA:
                irIzquierda();
                break;
            case ATRAS:
                irAtras();
                break;
            case DERECHA:
                irDerecha();
                break;
        }
        if (millis() - lastCommandTime > commandTimeout) {
            apagarMotores();
        }
    } else {
        // Modo automático con mejoras
        int distanciaIzquierda, distanciaCentro, distanciaDerecha;
        int distanciaMinima = 20;  // Umbral de distancia mínima en cm

        // Medir las tres distancias sin retrasos excesivos
        servoMotor.write(0);    // Mover a la izquierda
        delay(200);             // Reducir el tiempo de espera
        distanciaIzquierda = medirDistancia();

        servoMotor.write(90);   // Mover al centro
        delay(200);             // Reducir el tiempo de espera
        distanciaCentro = medirDistancia();

        servoMotor.write(180);  // Mover a la derecha
        delay(200);             // Reducir el tiempo de espera
        distanciaDerecha = medirDistancia();

        // Tomar decisiones basadas en la distancia
        if (distanciaCentro < distanciaMinima) {
            apagarMotores();
            delay(300);  // Se redujo el delay para respuesta rápida

            // Si ambos lados tienen obstáculos cercanos, retroceder y girar más tiempo
            if (distanciaIzquierda < distanciaMinima && distanciaDerecha < distanciaMinima) {
                atras();
                delay(1000);  // Retrocede más tiempo
                derecha();    // Gira completamente a la derecha
                delay(1500);  // Más tiempo para el giro completo
            }
            // Si el obstáculo está más cerca de la derecha, girar a la izquierda
            else if (distanciaDerecha < distanciaIzquierda) {
                izquierda();
                delay(1000);  // Gira durante más tiempo
            }
            // Si el obstáculo está más cerca de la izquierda, girar a la derecha
            else {
                derecha();
                delay(1000);  // Gira durante más tiempo
            }
        } else {
            // Si no hay obstáculos, sigue adelante
            adelante();
        }
    }
}

int medirDistancia() {
  // Enviar un pulso al pin TRIG
  digitalWrite(trigPin, LOW); // Se asegura que cualquier anterir este apagado
  delayMicroseconds(2); // esperar
  digitalWrite(trigPin, HIGH); // alta durante 10 ms
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW); // fin del pulso

  // Leer el tiempo que tarda el pulso en volver al pin ECHO
  long duracion = pulseIn(echoPin, HIGH);

  // Calcular la distancia en centímetros
  int distancia = duracion * 0.034 / 2;
  
  return distancia;
}

void irDerecha() {
  // Ambos motores hacia adelante
  Serial.print("derecha");
  digitalWrite(motor1Pin1, LOW); // Motores izquierdo atras
  digitalWrite(motor1Pin2, HIGH); // Motores izquierdo adelante
  digitalWrite(motor2Pin1, LOW); // Motores derechos atras
  digitalWrite(motor2Pin2, HIGH); // Motores derechos adelante
  analogWrite(en_A, 200); 
  analogWrite(en_B, 200);
}

void irIzquierda() {
  // Ambos motores hacia atrás
  Serial.print("izquierda");

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(en_A, 200); 
  analogWrite(en_B, 200);
}

void irAtras() {
  // Motor derecho hacia adelante, motor izquierdo hacia atrás
  Serial.print("atras");

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(en_A, 200); 
  analogWrite(en_B, 200);
}

void irAdelante() {
  // Motor derecho hacia atrás, motor izquierdo hacia adelante
  Serial.print("adelante");

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(en_A, 200); 
  analogWrite(en_B, 200);
}

void apagarMotores() {
  // Apagar todos los motores
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(en_A, 0); 
  analogWrite(en_B, 0);
}

void setColor(int rojo, int verde, int azul) {
  analogWrite(pinRojo, rojo);
  analogWrite(pinVerde, verde);
  analogWrite(pinAzul, azul);
}
