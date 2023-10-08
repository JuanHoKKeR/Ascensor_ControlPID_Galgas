#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <TimerOne.h>

#define Encoder      2 // Quadrature encoder A pin
#define Encoder_B    8
#define IN1          3
#define IN2          11
#define END_stop1    13
//-------Variables Controlador-------------
float kp = 6;
float ki = 0.02, kd = 0.02;
float numero;
float input, output, setpoint;
float iTerm = 0, lastInput = 0, dInput = 0, error = 0;
int outMin = -250, outMax = 250;
int sampleTime = 10; // This value is in milliseconds
//----------------------------------------
volatile long encoderPos = 0;
volatile long lastEncoderPos = 0; // Variable to store the previous encoder position
//----------------------------------------
const int VALOR_PISO_1 = 1;
const int VALOR_PISO_2 = 2900;
const int VALOR_PISO_3 = 5600;
const int VALOR_PISO_4 = 8300;
int pisoActual = 1;
int pisoObjetivo = 1;
//----------------------------------------
float piso1=0;
float piso2=0;
float piso3=0;
float piso4=0;
int valor1=0;
int valor2=0;
int valor3=0;
int valor4=0;

void setup() {
  Serial.begin(115200);
  pinMode(Encoder, INPUT);
  pinMode(Encoder_B, INPUT);
  pinMode(END_stop1, INPUT);
  //----------------------------------------
  attachPCINT(digitalPinToPCINT(Encoder), encoder, CHANGE); // Detect both rising and falling edges
  attachPCINT(digitalPinToPCINT(Encoder_B), encoder, CHANGE);
  //----------------------------------------
  Timer1.initialize(sampleTime * 1000); // Setup Timer1
  Timer1.attachInterrupt(moveTo);
}

void loop() {
  Serial.print("Pasos: ");
  Serial.println(encoderPos);

  if (Serial.available() > 0) {
    int nuevoNumero = Serial.parseInt();
    if (nuevoNumero != 0) {
      numero = nuevoNumero;
      Serial.print("Número recibido: ");
      Serial.println(numero);
    }
  }
  Serial.print("Número:   ");
  Serial.println(numero);
  Seleccionar();
  delay(100);
}

void encoder() {
  int MSB = digitalRead(Encoder);
  int LSB = digitalRead(Encoder_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoderPos << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }

  lastEncoderPos = encoded; // Update the last encoder position
}

void moveTo() {
  setpoint = numero;
  input = encoderPos;
  error = setpoint - input;
  iTerm += ki * error * sampleTime;
  if (iTerm > outMax) iTerm = outMax;
  else if (iTerm < outMin) iTerm = outMin;
  dInput = (input - lastInput) / sampleTime;
  output = kp * error + iTerm - kd * dInput;
  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;
  lastInput = input;
  pwmOut(output);
}

void pwmOut(int out) {
  if (out > 0) {
    analogWrite(IN1, out);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(out));
  }
}

bool ver;

void Seleccionar(){
  valor1 = analogRead(0);
  piso1=valor1*(5.0/1023.0);
  valor2 = analogRead(1);
  piso2=valor2*(5.0/1023.0);
  valor3 = analogRead(2);
  piso3=valor3*(5.0/1023.0);
  valor4 = analogRead(4);
  piso4=valor4*(5.0/1023.0);
  Serial.print("Lectura 1: ");Serial.println(piso1);Serial.print("Lectura 2: ");Serial.println(piso2);Serial.print("Lectura 3: ");Serial.println(piso3);Serial.print("Lectura 4: ");Serial.println(piso4);



  if (piso1>2.86) {
    numero = VALOR_PISO_1;
    pisoObjetivo=1;
    verificar();
  } else if (piso2>2.82) {
    numero = VALOR_PISO_2;
    pisoObjetivo=2;
    verificar();
  } else if (piso3>2.82) {
    numero = VALOR_PISO_3;
    pisoObjetivo=3;
    verificar();
  } else if (piso4>2.86) {
    numero = VALOR_PISO_4;
    pisoObjetivo=4;
    verificar();
  }
  

}
void verificar(){
  int rangoMaximo=numero+10;
  int rangoMinimo=numero-10;
  pisoActual=encoderPos;
  if(pisoActual>=rangoMinimo && pisoActual<=rangoMaximo){
    Serial.print("Piso Actual: ");Serial.println(pisoObjetivo);
  }
  else{
    Serial.println("Esperando llegar al Piso Seleccionado");
    while(veri());
  }
}

bool veri(){
  int rangoMaximo=numero+10;
  int rangoMinimo=numero-10;
  pisoActual=encoderPos;
  if(pisoActual>=rangoMinimo && pisoActual<=rangoMaximo){
    return false;
  }
  else{
    return true;
  }
}
