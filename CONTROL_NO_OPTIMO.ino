// ******************************************************** //
// Control No Óptimo – Realimentación de Estados (NO LQR)
// u(t) = Kr * r - k1 * x1 - k2 * x2
// ******************************************************** //

// ---------- Muestreo ----------
unsigned long TS = 50;
float Tseg = 0.05;

// ---------- Ganancias del control NO óptimo ----------
const float k1 = 0.50;     // Ganancia estado X1
const float k2 = 1.00;     // Ganancia estado X2
const float Kr = 2.0;      // Ganancia de referencia

// ---------- Pines ----------
#define pR 0
#define pX1 1
#define pX2 2
#define pU 10

#define pSW2 2
#define pSW3 3
#define pLED8 8
#define pLED9 9

// ---------- Escalamiento ADC ----------
#define V_REF 5.0
#define ADC_MAX 1023.0
#define mX (V_REF/ADC_MAX)
#define mU (ADC_MAX/V_REF)

unsigned long TIC = 0, TS_code = 0, TC = 0;

float R=0, X1=0, X2=0, Y=0, U=0;
int Ui=0;
bool Habilitado = 0;

// ******************************************************** //
// Inicio
// ******************************************************** //
void setup(){
  Serial.begin(9600);

  pinMode(pSW2, INPUT);
  pinMode(pSW3, INPUT);
  pinMode(pLED8, OUTPUT);
  pinMode(pLED9, OUTPUT);
}

// ******************************************************** //
// LOOP
// ******************************************************** //
void loop(){
  proc_entradas();
  control_no_optimo();
  proc_salidas();
  coms_arduino_ide();
  espera();
}

// ******************************************************** //
// Ley de control NO ÓPTIMO
// ******************************************************** //
void control_no_optimo(){

  U = Kr * R - k1 * X1 - k2 * X2;

  // Saturación
  if(U >= V_REF) U = V_REF;
  else if(U < 0) U = 0;
}

// ******************************************************** //
// Entradas — Estados y Referencia
// ******************************************************** //
void proc_entradas(){
  X1 = analogRead(pX1) * mX;
  X2 = analogRead(pX2) * mX;

  R = Habilitado * (analogRead(pR)*mX);
  Y = X2;
}

// ******************************************************** //
// Salidas — PWM
// ******************************************************** //
void proc_salidas(){
  Ui = int(U * mU);
  analogWrite(pU, Ui);

  botonesyleds();
}

// ******************************************************** //
void botonesyleds(){

  if(digitalRead(pSW2)) Habilitado = 1;
  if(digitalRead(pSW3)) Habilitado = 0;

  digitalWrite(pLED9, !Habilitado);
}

// ******************************************************** //
void espera(){
  TS_code = millis() - TIC;
  TC = TS - TS_code;
  if(TS_code < TS) delay(TC);
  TIC = millis();
}

// ******************************************************** //
// Serial Monitor
// ******************************************************** //
void coms_arduino_ide(){
  Serial.print(R); Serial.print(",");
  Serial.print(Y); Serial.print(",");
  Serial.print(X1); Serial.print(",");
  Serial.println(U);
}
