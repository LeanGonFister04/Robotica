#include <Servo.h>
#include <IRremote.hpp>

// Control manual/automatico
boolean ES_MANUAL;
constexpr uint16_t CONTROL = 0x7; // Botón Cambia de Control

// Pines del receptor IR
constexpr uint8_t RECV_PIN{2}; // Pin del receptor IR

// Pines del L298N para los motores
const int motor1Pin1 = 7;
const int motor1Pin2 = 8;
const int motor2Pin1 = 9;
const int motor2Pin2 = 11;

const int en_A = 5;
const int en_B = 6;

//SERVO
Servo servoMotor;

// SENSORES DE PRESENCIA
const int presenciaIzq = 12;
const int presenciaDer = 10;

// Valores de los botones
constexpr uint16_t POWER = 0x43;     // Botón POWER
constexpr uint16_t ADELANTE = 0x18;  // Botón ADELANTE
constexpr uint16_t ATRAS = 0x52;     // Botón ATRÁS
constexpr uint16_t IZQUIERDA = 0x8;  // Botón IZQUIERDA
constexpr uint16_t DERECHA = 0x5A;   // Botón DERECHA

// Valores del sensor y servo
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
            // Serial.print("Command: 0x");
            // Serial.println(received, HEX);
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
  
  	// Configuración del sensor de distancia
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Configuración del servomotor
    servoMotor.attach(servoPin);
    servoMotor.write(90);  // Posición inicial del servo

    // Configuración de los sensores de presencia
    pinMode(presenciaIzq, INPUT);
    pinMode(presenciaDer, INPUT);

    // Inicializar la recepción del IR
    IrReceiver.begin(RECV_PIN);
    // Serial.print(F("Listo para recibir señales IR en el pin "));
    // Serial.println(RECV_PIN);
}

void loop()
{
    uint16_t irCode = irReceive();

    if (irCode == CONTROL)
    {
        ES_MANUAL = !ES_MANUAL;
        setColor(180,0,0);
    }

    if (ES_MANUAL)
    {
        setColor(0,0,180);
        if (irCode != 0)
        {
            lastCommandTime = millis();  // Actualiza el tiempo de la última señal recibida
        }
        switch (irCode)
        {
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

        default:
            // No hacer nada si se presiona un botón diferente
            break;
        }
        
        
        // Apagar motores si no se recibe señal por más de "commandTimeout" ms
        if (millis() - lastCommandTime > commandTimeout)
        {
            apagarMotores();
        }
    }
    else{
        automaticMode();
    }
}


void automaticMode() {
    // Leer sensores de presencia
    bool sensorIzq = digitalRead(presenciaIzq);
    bool sensorDer = digitalRead(presenciaDer);
    long distance = obtenerDistancia();

    // // Mostrar la distancia en el monitor serial
    // Serial.print("Distancia: ");
    // Serial.println(distance);

    // Lógica de decisiones basadas en sensores
    if (!sensorIzq) {
        Serial.println("presencia a la izquierda.");
        servoMotor.write(40);  // Posición inicial del servo
        irDerecha();
    } else if (!sensorDer) {
        Serial.println("presencia a la derecha.");
        servoMotor.write(140);  // Posición inicial del servo
        irIzquierda();
    } else if (distance < 15) {
        apagarMotores();
    } else {
        irAdelante();
    }
}   

long obtenerDistancia() {
    // Generar pulso de trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Medir el tiempo de regreso del eco y calcular la distancia
    long duracion = pulseIn(echoPin, HIGH);
    return duracion / 58;  // Conversión a centímetros
}

void irDerecha() {
  // Ambos motores hacia adelante
  // Mirando la pila hacia abajo
  // Serial.print("derecha");
  digitalWrite(motor1Pin1, LOW); // Motores izquierdo atras
  digitalWrite(motor1Pin2, HIGH); // Motores izquierdo adelante
  digitalWrite(motor2Pin1, LOW); // Motores derechos atras
  digitalWrite(motor2Pin2, HIGH); // Motores derechos adelante
  analogWrite(en_A, 200); 
  analogWrite(en_B, 200);
  if(!ES_MANUAL){
    servoMotor.write(90);
  }
}

void irIzquierda() {
  // Ambos motores hacia atrás
  // Serial.print("izquierda");

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(en_A, 200); 
  analogWrite(en_B, 200);
  if(!ES_MANUAL){
    servoMotor.write(90);
  }
}

void irAtras() {
  // Motor derecho hacia adelante, motor izquierdo hacia atrás
  // Serial.print("atras");

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(en_A, 150); 
  analogWrite(en_B, 150);
}

void irAdelante() {
  // Motor derecho hacia atrás, motor izquierdo hacia adelante
  // Serial.print("adelante");

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(en_A, 100); 
  analogWrite(en_B, 100);
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

 
