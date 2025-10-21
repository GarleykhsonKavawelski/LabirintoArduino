#include <Servo.h>

// Definição de pinos para o sensor de movimento e som
const int sensorMovimentoPin = 2; // Pino digital conectado ao sensor de movimento
const int sensorSomPin = 3;       // Pino digital conectado ao sensor de som
const int ledPin = 13;            // Pino digital conectado ao LED

// Variáveis para o sensor de movimento
int estadoSensorMovimento = LOW; // Variável para armazenar o estado do sensor de movimento

// Variáveis para o joystick e servos
Servo servo1;
Servo servo2;
int joyX = A0; // Pino analógico para o eixo X
int joyY = A1; // Pino analógico para o eixo Y

int servoVal;
unsigned long previousMillis = 0; // Variável para o temporizador da saída Serial
const long interval = 500;        // Intervalo para a saída Serial em milissegundos

void setup() {
  // Inicializa a comunicação serial (para depuração)
  Serial.begin(9600);

  // Configura os pinos como entrada ou saída
  pinMode(sensorMovimentoPin, INPUT);
  pinMode(sensorSomPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Configura os servos
  servo1.attach(5); // Conecta o servo1 ao pino digital 5
  servo2.attach(6); // Conecta o servo2 ao pino digital 6
}

void loop() {
  // Leitura do sensor de movimento
  estadoSensorMovimento = digitalRead(sensorMovimentoPin);

  // Verifica se o sensor de movimento detectou movimento
  if (estadoSensorMovimento == HIGH) {
    Serial.println("Movimento detectado!"); // Imprime no monitor serial
    digitalWrite(ledPin, HIGH);             // Acende o LED

    // Ativa o sensor de som (emite a onda sonora)
    digitalWrite(sensorSomPin, HIGH);
    delay(1000); // Mantém o som por 1 segundo (ajuste conforme necessário)
    digitalWrite(sensorSomPin, LOW);
    digitalWrite(ledPin, LOW); // Desliga o LED
  } else {
    // Se não houver movimento, desliga o sensor de som e o LED
    digitalWrite(sensorSomPin, LOW);
    digitalWrite(ledPin, LOW);
  }

  // Leitura e mapeamento do joystick para os servos
  servoVal = analogRead(joyX);
  int servo1Pos = map(servoVal, 0, 1023, 0, 180); // Ajuste os limites conforme necessário
  servo1.write(servo1Pos);

  servoVal = analogRead(joyY);
  int servo2Pos = map(servoVal, 0, 1023, 0, 180); // Ajuste os limites conforme necessário
  servo2.write(servo2Pos);

  // Saída Serial (apenas para depuração e monitoramento)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Serial.print("Joystick X: ");
    Serial.print(analogRead(joyX));
    Serial.print(" -> Servo1 Position: ");
    Serial.println(servo1Pos);

    Serial.print("Joystick Y: ");
    Serial.print(analogRead(joyY));
    Serial.print(" -> Servo2 Position: ");
    Serial.println(servo2Pos);
  }

  delay(50); // Pequeno atraso para estabilidade dos servos e leituras
}