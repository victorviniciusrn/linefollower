
# define STOP 0
# define FOLLOWING_LINE 1
# define NO_LINE 2

//Definição dos pinos de controle do motor
#define M1 11 // Pino_Velocidade 1º Motor ( 0 a 255)_ Porta ATV_A ponte H;
#define M2 9 //Pino_Velocidade 2º Motor ( 0 a 255) _ Porta ATV_B ponte H;
#define dir1 8 //Pino_Direção do 1º Motor: Para frente / Para trás (HIGH ou LOW)_ porta IN1 ponte H;
#define dir2 10 //Pino_Direção do 2º Motor: Para frente / Para trás (HIGH ou LOW)_ porta IN3 ponte H;

//Definição dos pinos dos sensores
#define pin_S1 5        // D0
#define pin_S2 6        // D0
#define pin_S3 7        // D0
#define ground 0        // terra
#define sensor_1 3      // 5 V
#define sensor_2 2      // 5 V 
#define MIN_POT 110
#define LIGADO 0
#define DESLIGADO 1

int mode = 0;
// Constantes PID

float Kp=40;
float Ki=0.0;
float Kd=1.5;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

/////
bool Sensor1 = 0;
bool Sensor2 = 0;
bool Sensor3 = 0; 

//variável responsável por controlar a velocidade dos motores
int velocidade = 140;


//void curvaSuaveAEsquerda(){
//  analogWrite(M1, MIN_POT);
//  analogWrite(M2, 110); 
//}

// void curvaSuaveADireita(){
//  analogWrite(M1, 110); 
//  analogWrite(M2, MIN_POT); 
//}

void readLFSsensors()
{
  int LFSensor[3] = {0,0,0};
  
  LFSensor[0] = digitalRead(pin_S3);
  LFSensor[1] = digitalRead(pin_S2);
  LFSensor[2] = digitalRead(pin_S1);

  
 if((     LFSensor[0]== LIGADO )&&(LFSensor[1]== LIGADO )&&(LFSensor[2]== DESLIGADO ))  {mode = FOLLOWING_LINE; error = 2;}
  else if((LFSensor[0]== LIGADO )&&(LFSensor[1]== DESLIGADO )&&(LFSensor[2]== DESLIGADO ))  {mode = FOLLOWING_LINE; error = 1;}
  else if((LFSensor[0]== LIGADO )&&(LFSensor[1]== DESLIGADO )&&(LFSensor[2]== LIGADO ))  {mode = FOLLOWING_LINE; error = 0;}
  else if((LFSensor[0]== DESLIGADO )&&(LFSensor[1]== DESLIGADO )&&(LFSensor[2]== LIGADO ))  {mode = FOLLOWING_LINE; error = -1;}
  else if((LFSensor[0]== DESLIGADO )&&(LFSensor[1]== LIGADO )&&(LFSensor[2]== LIGADO ))  {mode = FOLLOWING_LINE; error = -2;}
  else if((LFSensor[0]== DESLIGADO )&&(LFSensor[1]== DESLIGADO )&&(LFSensor[2]== DESLIGADO ))  {mode = NO_LINE;}
  
}

void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = ((Kp*P) + (Ki*I) + (Kd*D));
  previousError = error;
}

void motorPIDcontrol()
{
  
  int leftMotorSpeed = velocidade + PIDvalue;
  int rightMotorSpeed = velocidade - PIDvalue;
  
  // The motor speed should not exceed the max PWM value
  constrain(leftMotorSpeed, 50, 250);
  constrain(rightMotorSpeed, 50, 250);

  analogWrite(M1,leftMotorSpeed);
  analogWrite(M2, rightMotorSpeed);  
}


//void curvaAcentuadaAEsquerda(){
//  analogWrite(M1, 0); 
//  analogWrite(M2, MIN_POT + 10); 
//}
//
//
//void curvaAcentuadaADireita (){
//  analogWrite(M1, MIN_POT + 30); 
//  analogWrite(M2, 0); 
//}
//
//void continuarReto(){
//  analogWrite(M1, velocidade); 
//  analogWrite(M2, velocidade); 
//}

void setup(){
//Setamos os pinos de controle dos motores como saída
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);
pinMode(dir1, OUTPUT);
pinMode(dir2, OUTPUT);
pinMode(ground, OUTPUT); 

//Setamos a direção inicial do motor como 0, isso fará com que ambos os motores girem para frente
digitalWrite(dir1, LOW);
digitalWrite(dir2, LOW);
digitalWrite(ground, LOW); 

//Setamos os pinos dos sensores como entrada
pinMode(pin_S1, INPUT);
pinMode(pin_S2, INPUT);
pinMode(pin_S3, INPUT); 

pinMode(sensor_1, OUTPUT); 
pinMode(sensor_2, OUTPUT); 

digitalWrite(sensor_1, HIGH); 
digitalWrite(sensor_2, HIGH); 
}

void loop(){
//Neste processo armazenamos o valor lido pelo sensor na variável que armazena tais dados.
Sensor1 = digitalRead(pin_S1);
Sensor2 = digitalRead(pin_S2);
Sensor3 = digitalRead(pin_S3);


readLFSsensors();
switch(mode){
  case FOLLOWING_LINE:
    calculatePID();
    motorPIDcontrol();
    break;

  case NO_LINE:
    analogWrite(M1, 130); 
    analogWrite(M2, 130);
    
  case STOP:
    analogWrite(M1, 0); 
    analogWrite(M2, 0);     
}

//if((Sensor2 == LIGADO && Sensor3 == LIGADO) && (Sensor1 == DESLIGADO)){
//  curvaAcentuadaAEsquerda();
//} 
//
//if((Sensor2 == LIGADO && Sensor1 == LIGADO) && (Sensor3 == DESLIGADO)){
//  curvaAcentuadaADireita(); 
//}
//
//if((Sensor2 == DESLIGADO) && (Sensor1 == LIGADO && Sensor3 == LIGADO)){
//  continuarReto(); 
//}

//if(Sensor1 == LIGADO && Sensor2 == LIGADO && Sensor3 == LIGADO) {
//  continuarReto(); 
//}


}
