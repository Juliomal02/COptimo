// Control-LQR-Seguimiento

// ******************************************************** //
// Control Óptimo (LQR) para Seguimiento de Referencia      //
// Implementación para sistema RC-RC-R (3 Resistencias, 2 Capacitores)
// Etapa 2 del Proyecto Integrador
// Basado en el modelo: R1=R2=R3=1MΩ, C1=C2=1μF
// ******************************************************** //

// ******************************************************** //
//----------  Muestreo  --------//
// ******************************************************** //
  unsigned long TS = 50;      // Muestreo TS miliseg
  float Tseg = 0.05;          // Muestreo en Tseg segundos
// ******************************************************** //

// ******************************************************** //
//----------  Parámetros del Controlador Óptimo (LQR)  -------- //
// ******************************************************** //
// Las ganancias K1, K2 y Kp (GAIN_R_LQR) se calculan a partir
// del diseño LQR (Etapa 2)
  const float K1_LQR = 2.2312;    // Ganancia de retroalimentación para X1 (VC1)
  const float K2_LQR = 1.0716;    // Ganancia de retroalimentación para X2 (VC2)
  
// Ganancia de Feedforward (Kp) calculada para seguimiento:
// Kp = K1*x1r/r + K2*x2r/r + ur/r = K1*(2) + K2*(1) + 3
// 2.2312*(2) + 1.0716*(1) + 3 = 8.534
  const float GAIN_R_LQR = 8.534; // Kp - Ganancia total de feedforward para R
// ******************************************************** //


// ******************************************************** //
//----------  Constantes  --------//
// ******************************************************** //


//---------- Definición Pines IO analogicos--------//
  #define pR 0                // pin de referencia (Entrada Analógica A0)
  #define pX1 1               // pin de estado 1 (VC1) (Entrada Analógica A1)
  #define pX2 2               // pin de estado 2 (VC2 = V0) (Entrada Analógica A2)
  #define pU 10               // pin de salida de control (Entrada a Planta, PWM)

//---------- Definición Pines IO discretos--------//
  #define pLED8 8             // LED 8 en tarjeta
  #define pLED9 9             // LED 9 en tarjeta
  #define pSW2 2              // SW 2 en tarjeta (Para Habilitar)
  #define pSW3 3              // SW 3 en tarjeta (Para Deshabilitar)

//---------- Escalamientos para analogicas de 0 a 5 V --------//
  #define V_REF 5.0           // Voltaje de referencia (generalmente 5V)
  #define ADC_MAX 1023.0      // Valor máximo del ADC
  #define mX (V_REF/ADC_MAX)  // Pendiente 0-1023 -> 0 - 5
  #define bX 0                // Ajuste cero
  #define mU (ADC_MAX/V_REF)  // Pendiente 0 - 5 -> 0 - 1023
  #define bU 0                // Ajuste cero



// ******************************************************** //
//----------  Variables globales  --------//
// ******************************************************** //

//---------- Tiempo --------//
  unsigned long TS_code = 0;  // Tiempo que tarda programa
  unsigned long TIC = 0;      // Estampa de tiempo inicio ciclos
  unsigned long TC = 0;       // Faltante para TS
    
//----------  Señales --------//
  float R = 0;                // Referencia (V)
  float Y = 0;                // Salida (V)
  float X1 = 0;               // Estado 1 (VC1) (V)
  float X2 = 0;               // Estado 2 (VC2) (V)
  float U = 0;                // Salida control (Vi) (V)
  int Ui = 0;                 // Salida control tarjeta (PWM 0-255)

//---------- Otros --------//
  bool Habilitado = 0;        // Señal {0,1} para entradas escalón


// ******************************************************** //
//----------  Rutinia de inicio --------//
// ******************************************************** //


void setup() {
  //--Inicia serial--//
  Serial.begin(9600);

  //--Configura pines digitales--//  
  pinMode(pSW2, INPUT);
  pinMode(pSW3, INPUT);
  pinMode(pLED8, OUTPUT);
  pinMode(pLED9, OUTPUT);
  
  // El pin pU (10) se configura para salida PWM en proc_salidas()

}


// ******************************************************** //
//---------- Rutinia principal  --------//
// ******************************************************** //


void loop() {                    
  proc_entradas();                  // Procesamiento de Entradas (Lectura ADC)
  control();                        // Control (Cálculo LQR)
  proc_salidas();                   // Procesado de Salidas (Escalamiento y PWM)
  coms_arduino_ide();             // Comunicaciones (Serial Plotter)
  espera();                         // Control de Tiempo (Muestreo Uniforme)
}


// ******************************************************** //
//---------- Rutinias de control --------        //
// ******************************************************** //


//-- Control --//
void control(){
  // Ley de control Óptimo (LQR) de Seguimiento:
  // U = GAIN_R_LQR * R - K1_LQR * X1 - K2_LQR * X2

    U = GAIN_R_LQR * R - K1_LQR * X1 - K2_LQR * X2; 
    
  // Saturacion (La entrada Vi no puede ser mayor a V_REF o menor a 0)
  if(U >= V_REF) U = V_REF;     // Saturacion superior (ej. 5.0V)
  else if(U < 0) U = 0;         // Saturacion inferior (0V)
}


// ******************************************************** //
//---------- Rutinias de IO y control de tiempo  --------//
// ******************************************************** //


//-- Procesado de entradas --//
void proc_entradas(){
  // Lectura del ADC y conversión a Voltios (float)
  X1 = analogRead(pX1)*mX+bX;
  X2 = analogRead(pX2)*mX+bX;
  
  // Lectura de referencia y aplicación de Habilitado
  R = Habilitado*(analogRead(pR)*mX+bX);
  Y = X2;
}


//-- Procesado de salidas --//
void proc_salidas(){
  // Escalamiento del voltaje de control (U) a un valor de PWM (0-255)
  // Ui = int(U * 255 / V_REF)
  Ui = int(U * mU);

  // Salida PWM en pin pU
  analogWrite(pU, Ui);
  
  botonesyleds(); // Manejo de IO discretas
}


//-- Memoria {0,1} para entrada escalón y LEDs --//
void botonesyleds(){

  // El sistema se habilita si se presiona SW2, y se deshabilita con SW3
  if(digitalRead(pSW2) == HIGH) Habilitado = 1;
  else if(digitalRead(pSW3) == HIGH) Habilitado = 0;

  // Indicador de Habilitado (LED9)
  if(Habilitado == 0) digitalWrite(pLED9, HIGH);   // LED9 = Encendido (Control Deshabilitado)
  else digitalWrite(pLED9, LOW);                  // LED9 = Apagado (Control Habilitado)
}


//-- Para muestreo uniforme --//
void espera(){    
  TS_code = millis()- TIC;            // Tiempo de ciclo
  TC = TS - TS_code;                  // Calcula faltante para TS
  if (TS_code < TS) delay(TC);        // Espera para completar ciclo de TS    
  TIC = millis();
}


//-- Comunicación con monitor (Para Serial Plotter) --//
void coms_arduino_ide(){  
  // Formato: Referencia,Salida,Estado1,Entrada
  Serial.print(R);                  // y_d(t)
  Serial.print(",");                // Separador
  Serial.print(Y);                  // y(t)
  Serial.print(",");                // Separador
  Serial.print(X1);                 // x1(t)
  Serial.print(",");                // Separador
  Serial.println(U);                // u(t)
}
