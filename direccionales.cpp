#include <Servo.h>
#include <IRremote.hpp>

// Control manual/automatico
boolean ES_MANUAL;
constexpr uint16_t CONTROL = 0x2; // Botón Cambia de Control

// Pines del receptor IR
constexpr uint8_t RECV_PIN{2}; // Pin del receptor IR

// Pines del L293D para los motores
constexpr int motor1A = 7;
constexpr int motor1B = 8;
constexpr int motor2A = 9;
constexpr int motor2B = 11;

// Valores de los botones
constexpr uint16_t POWER = 0xC;     // Botón POWER
constexpr uint16_t ADELANTE = 0x1;  // Botón ADELANTE
constexpr uint16_t ATRAS = 0x6;     // Botón ATRÁS
constexpr uint16_t IZQUIERDA = 0x4; // Botón IZQUIERDA
constexpr uint16_t DERECHA = 0x9;   // Botón DERECHA

// Valores del sensor y servo
Servo servoMotor;
const int trigPin = A5; // out
const int echoPin = A4; // in
const int servoPin = 3;

unsigned long lastCommandTime = 0;  // Para registrar el último tiempo en el que se recibió una señal
const unsigned long commandTimeout = 150;  // Tiempo en milisegundos para determinar si el botón fue soltado


// Función para recibir el código IR
uint16_t irReceive()
{
    uint16_t received{0};

    if (IrReceiver.decode())
    {
        //IrReceiver.printIRResultShort(&Serial);
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
    pinMode(motor1A, OUTPUT);
    pinMode(motor1B, OUTPUT);
    pinMode(motor2A, OUTPUT);
    pinMode(motor2B, OUTPUT);

    // Inicializar los motores apagados
    apagarMotores();
  
  	// Configurar el servo
  	servoMotor.attach(servoPin);
  
  	// Configurar los pines del sensor ultrasónico
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

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
    }

    if (ES_MANUAL)
    {
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
            adelante();
            break;

        case IZQUIERDA:
            izquierda();
            break;

        case ATRAS:
            atras();
            break;

        case DERECHA:
            derecha();
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
    else
    {
        int distanciaIzquierda, distanciaCentro, distanciaDerecha;

        // Medir la distancia hacia la izquierda
        servoMotor.write(0);
        delay(500);
        distanciaIzquierda = medirDistancia();

        // Medir la distancia al frente
        servoMotor.write(90);
        delay(500);
        distanciaCentro = medirDistancia();

        // Medir la distancia hacia la derecha
        servoMotor.write(180);
        delay(500);
        distanciaDerecha = medirDistancia();

        // Determinar la dirección en función de las distancias medidas
        if (distanciaCentro < 20) {
            apagarMotores();
            delay(500);

            // Si el obstáculo mas cerca a la derecha derecha, gira a la izquierda
            if (distanciaDerecha < distanciaIzquierda) {
                izquierda();
                delay(1000);
            }
            // Si el obstáculo esta más cerca a la izquierda, gira a la derecha
            else if (distanciaIzquierda < distanciaDerecha) {
                derecha();
                delay(1000);
            }
            else {
                // Si el obstáculo está de frente pero ambas direcciones son iguales,
                // simplemente retrocede y gira a la derecha por defecto
                atras();
                delay(1000);
                derecha();
                delay(1000);
            }
            apagarMotores();
            delay(500);
        }
        else {
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

void adelante()
{
    // Ambos motores hacia adelante
    digitalWrite(motor1A, HIGH); // HIGH SOLO, adelante Motor1
    digitalWrite(motor1B, LOW);// HIGH SOLO, atras Motor1
    digitalWrite(motor2A, LOW); // HIGH SOLO, atras Motor2
    digitalWrite(motor2B, HIGH); // HIGH SOLO, adelante Motor1
}

void atras()
{
    // Ambos motores hacia atrás
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, HIGH);
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, HIGH);
}

void izquierda()
{
    // Motor derecho hacia adelante, motor izquierdo hacia atrás
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);
}

void derecha()
{
    // Motor derecho hacia atrás, motor izquierdo hacia adelante
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, HIGH);
    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);
}

void apagarMotores()
{
    // Apagar todos los motores
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, LOW);
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, LOW);
}

