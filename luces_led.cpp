// VALORES DE LOS PINES DE LOS LEDS
const int pinRojo = 4;
const int pinVerde = 0;
const int pinAzul = 12;

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
  
    // CONFIGURAR PINES DEL LEDs
    pinMode(pinRojo, OUTPUT);
    pinMode(pinVerde, OUTPUT);
    pinMode(pinAzul, OUTPUT);

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
      setColor(0,0,0);
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

void setColor(int rojo, int verde, int azul) {
  analogWrite(pinRojo, rojo);
  analogWrite(pinVerde, verde);
  analogWrite(pinAzul, azul);
}