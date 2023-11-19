//Programa para obtener modelo de motor DC mediante entrada de señal escalón
#include <util/atomic.h>
// Libreria Wire para I2C
#include <Wire.h>

//Pines Encoder
#define ENCA 2
#define ENCB 7
//Pines Driver
#define Adelante 4
#define Atras 8
#define PWM1 3

//VARIABLES GLOBALES
float contTiempo = 0;
float tiempo = 0;
int TOPcount = 51; //PWM
int B; //Encoder
int contPos=0;
long t=0; //Calculo RPMs
int tOverflow = 0;
float rpm = 0.0;
float rpmEMA = 0; //Filtro EMA
float alfa = 0.6;
float e_k, u_k; //PID
float ref = 0;
float u_k_1 = 0;
float e_k_1 = 0;
float e_k_2 = 0;
float DeltaTheta; //I2C
char buff_env[8];
float tAnt = 0;
int addr = 0x11; //Motor 4

////AJUSTADO kp1.1ki36kd0.004
float q0 = 3.136;
float q1 = -5.064;
float q2 = 2;

void setup() {
//  Encoders
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  
//  Pines Sentido de Giro Driver
  pinMode(Adelante, OUTPUT);
  pinMode(Atras, OUTPUT);
  
//  PWM Timer2
  pinMode(PWM1,OUTPUT);
//  WGM22:0=5=0b101 para establecer "Phase Correct PWM Mode" con TOP siendo el registro OCR2A
//  CS22:0=2=0b010 para prescalador = 8
  TCCR2A &= ~(1<<WGM21);  
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
//  PWM_freq = fclk/(2 x Prescalador x TOP) -> fclk = 16MHz, Prescalador = 8, TOP = OCR2A = 51
//  PWM_freq = 19.6 kHz
  OCR2A = TOPcount;

//  Interrupciones Encoder Timer1
  attachInterrupt(digitalPinToInterrupt(ENCA), lecEncoder, RISING);
//  Prescalador = 1 -> CS12:0=1=0b001
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS10); //Prescalador N = 1
//  Contador Timer1 comienza en 0
  TCNT1 = 0;
//  Habilitar interrupciones por Overflow
  TIMSK1 |=  (1<<TOIE1);

//  Interrupciones Muestreo Timer0
  TCCR0A = 0;
  TCCR0B = 0;
// Habilitar CTC mode (Clear Timer on Compare Match) WGM02:0=2=0b010
  TCCR0A |= (1 << WGM01);
// Prescalador N=256 CS02:0=4=0b100
  TCCR0B |= (1 << CS02);
// fmuestreo = fclk/(N*(OCR0A+1)) -> fclk = 16MHz, N = 256, OCR0A = 124 -> fmuestreo = 500 Hz 
  OCR0A = 124;
//  Contador Timer1 comienza en 0
  TCNT0  = 0;
//  Habilitar interrupciones por Compare Match entre TCNT0 y OCR0A  
  TIMSK0 |= (1 << OCIE0A);

//  Habilitar interrupciones
  sei();
  
  // Join I2C bus as slave with address 8
//  Wire.begin(0x8);
//  Wire.begin(0x9);
  Wire.begin(addr);
//  Wire.begin(0x11);
  // Call receiveEvent when data received
  Wire.onReceive(receiveEvent);
  // Handler
  Wire.onRequest(sendData);
  
  Serial.begin(115200);
}

void loop() {
  //  Filtro Exponential Moving Average
  rpmEMA = alfa*rpm + (1-alfa)*rpmEMA;

  if(tiempo - contTiempo >= 0.5 && ref != 0)
  {
    ref = 0;
    Wire.begin(addr);
  }
    //  OCR2B define ciclo de trabajo -> Ciclo de Trabajo = OCR2B/TOPcount*100%
  //  Giro positivo
  if(u_k >= 0)
  {
    digitalWrite(Atras, LOW);
    digitalWrite(Adelante, HIGH);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      OCR2B = map(int(u_k),0,TOPcount,27,TOPcount);;
    }
  }
  //  Giro Negativo
  else
  {
    digitalWrite(Adelante, LOW);
    digitalWrite(Atras, HIGH); 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      OCR2B = -map(int(u_k),-TOPcount,0,-TOPcount,-27);
    }
  }
  Serial.print(tiempo,4);
  Serial.print(",");
  Serial.print(u_k);
  Serial.print(",");
  Serial.print(rpm);
  Serial.print(",");
  Serial.print(rpmEMA);
  Serial.print(",");
  Serial.println(ref);
}

//INERRUPCION ENCODER
void lecEncoder()
{
//  Bloqueo interrupciones
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
//    Obtener valor contador Timer1 TCNT1
    t = tOverflow*65.536e3 + TCNT1 + 1;
    TCNT1 = 0;
    B = digitalRead(ENCB);
    if(t < 17.91e3)
    {
      t = tAnt;
    }
    tAnt = t;
  //  Determinar sentido de giro
  //  RPM = fclk*60/(N*GR*CPR*GReng) / TCNT -> RPM = 537.049717e3/TCNT -> GR=297.924, CPR=3, GReng=2, fclk=16MHz, N=1
    if(B == 1)
    {
      rpm = 537.049717e3/t;
      contPos ++;
    }
    else 
    {
      rpm = -537.049717e3/t;
      contPos --;
    }
    tOverflow = 0;
  }
}

//INTERRUPCION OVERFLOW TIMER1
ISR (TIMER1_OVF_vect)
{
//  Reinicio contador TCNT1
  TCNT1 = 0;
//  Si no hay interrupciones por encoder se aproxima RPM=0
  if(tOverflow >= 30)
  {
    rpm = 0;
    tOverflow ++;
    if(tOverflow >= 35000)
    {
      tOverflow = 3000;
    }
  }
  else
  {
    tOverflow ++;
  }
}

ISR (TIMER0_COMPA_vect)
{
  tiempo = tiempo + 2.0e-3;
//  Error
  e_k = ref - rpm;
//  Ley de control
  u_k = u_k_1 + q0*e_k + q1*e_k_1 + q2*e_k_2;
//  Truncamiento señal de control
  if (u_k > TOPcount)
  {
     u_k = TOPcount;
  }
  else if (u_k < -TOPcount)
  {
    u_k = -TOPcount;
  }
  u_k_1 = u_k;
  e_k_2 = e_k_1;
  e_k_1 = e_k;
}

//-->I2C<--
// Funcion cuando Master envia referencia
void receiveEvent(int howMany) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (howMany > 1) {
      char buff_rec[howMany];
      for (int i = howMany - 1; i >= 0; i--) {
        buff_rec[i] = Wire.read();
      }
//      Chequeo de información recibida
      if(isDigit(buff_rec[0]) || buff_rec[0] == '-')
      {
        ref = atof(buff_rec);
        contTiempo = tiempo; 
      }
    }
    else{
      Wire.read();
    }
  }
}

//funcion para enviar datos cuando Master lo requiera
void sendData() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
//    Giro de rueda
//    DeltaTheta = contPos/1787.544; //revoluciones
//    DeltaTheta = contPos/11.23147e3; //radianes
    DeltaTheta = contPos/284.49646; //radianes
    contPos = 0;
    dtostrf(DeltaTheta*1e2, 2, 6, buff_env);
    Wire.write(buff_env);
    Serial.println("");
    Serial.println(DeltaTheta*1e2,6);
  }
}
