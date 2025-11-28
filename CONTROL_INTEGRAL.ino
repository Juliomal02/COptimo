// ******************************************************** //
// Control con Integración del Error (I-STATE)
// u = Kr*r - k1*x1 - k2*x2 + kI * xI
// xI[k] = xI[k-1] + T*(r - y)
// ******************************************************** //

// ---------- Muestreo ----------
unsigned long TS = 50;
float Tseg = 0.05;

// ---------- Ganancias ----------
const float k1 = 0.40;
const float k2 = 0.40;
const float kI = 0.80;
const float Kr = 1.0;

// ---------- Pines ----------
#define pR 0
#define pX1 1
#define pX2 2
#define pU 10

#define pSW2 2
#define pSW3 3
#define pLED8 8
#define pLED9 9

// ---------- ADC ----------
#define V_REF 5.0
#define ADC_MAX 1023.0
#define mX (V_REF/ADC_MAX)
#define mU (ADC_MAX/V_REF)

unsigned long TIC=0, TS_code=0, TC=0;

float R=0, X1=0, X2=0, Y=0, U=0;
float xI = 0;     // INTEGRADOR
int Ui=0;
bool Habilitado = 0;

// ******************************************************** //
void setup(){
  Serial.begin(9600);
  pinMode(pSW2,INPUT);
  pinMode(pSW3,INPUT);
  pinMode(pLED8,OUTPUT);
  pinMode(pLED9,OUTPUT);
}
// ******************************************************** //
void loop(){
  proc_entradas();
  control_integral();
  proc_salidas();
  coms_arduino_ide();
  espera();
}
// ******************************************************** //
// Control con Integrador
// ******************************************************** //
void control_integral(){

  // Integrador de error xI[k] = xI + T*(r - y)
  xI = xI + Tseg * (R - Y);

  // Ley de control
  U = Kr*R - k1*X1 - k2*X2 + kI*xI;

  // Saturación física
  if(U >= V_REF) U = V_REF;
  if(U < 0) U = 0;
}

// ******************************************************** //
void proc_entradas(){
  X1 = analogRead(pX1)*mX;
  X2 = analogRead(pX2)*mX;
  R = Habilitado*(analogRead(pR)*mX);
  Y = X2;
}
// ******************************************************** //
void proc_salidas(){
  Ui = int(U*mU);
  analogWrite(pU,Ui);
  botonesyleds();
}
// ******************************************************** //
void botonesyleds(){
  if(digitalRead(pSW2)) Habilitado = 1;
  if(digitalRead(pSW3)) Habilitado = 0;
  digitalWrite(pLED9,!Habilitado);
}
// ******************************************************** //
void espera(){
  TS_code = millis()-TIC;
  TC = TS - TS_code;
  if(TS_code < TS) delay(TC);
  TIC = millis();
}
// ******************************************************** //
void coms_arduino_ide(){
  Serial.print(R); Serial.print(",");
  Serial.print(Y); Serial.print(",");
  Serial.print(X1); Serial.print(",");
  Serial.println(U);
}
