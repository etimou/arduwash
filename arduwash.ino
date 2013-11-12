#include <TimerOne.h>
#include <MsTimer2.h>

//define pins
#define ZERO_DETECT_PIN 2
#define TACHY_COUNT_PIN 3
#define TRIAC_MOTOR_PIN 4
#define ROTAT_MOTOR_PIN 5
#define SECURE_DOOR_PIN 6
#define WATER_EMPTY_PIN 7
#define WATER_FILL__PIN 8
#define PUSH_BUTT_1_PIN 9
#define LAMP_INFO_1_PIN 10
#define LAMP_INFO_2_PIN 11
#define LAMP_INFO_3_PIN 12
#define WATER_HEAT__PIN 13
#define THERM_SENSR_PIN 14
#define WATER_LVL_0_PIN 15
#define WATER_LVL_1_PIN 16
#define WATER_LVL_2_PIN 17
#define PUSH_BUTT_2_PIN 18
#define PUSH_BUTT_3_PIN 19

//for LCD
#define DATA  11
#define STR   12
#define CLOCK 10

//Constants
#define CYCLE_DURATION 100
#define MIN_SPEED 50
#define MAX_SPEED 3500

//Default values of global variables
int delay_triac=10000; //10000->No power    0->Full power
int tachy_counter=0;
int speed_setpoint=0; //in Hz 0 -> 3500
unsigned long last_zerocross = 0;
int count = 0; //count the number of zero cross for each cycle
int Speed = 0;



int printString(char *s, char column, char line);


//Setup
void setup() {

  pinMode(ZERO_DETECT_PIN, INPUT);
  pinMode(TACHY_COUNT_PIN, INPUT);
  pinMode(TRIAC_MOTOR_PIN, OUTPUT);
  pinMode(ROTAT_MOTOR_PIN, OUTPUT);
  pinMode(SECURE_DOOR_PIN, OUTPUT);
  pinMode(WATER_FILL__PIN, OUTPUT);
  pinMode(WATER_EMPTY_PIN, OUTPUT);
  pinMode(PUSH_BUTT_1_PIN, INPUT);
  pinMode(LAMP_INFO_1_PIN, OUTPUT);
  pinMode(LAMP_INFO_2_PIN, OUTPUT);
  pinMode(LAMP_INFO_3_PIN, OUTPUT);
  pinMode(WATER_HEAT__PIN, OUTPUT);
  pinMode(THERM_SENSR_PIN, INPUT);
  pinMode(WATER_LVL_0_PIN, INPUT);
  pinMode(WATER_LVL_1_PIN, INPUT);
  pinMode(WATER_LVL_2_PIN, INPUT);
  pinMode(PUSH_BUTT_2_PIN, INPUT);
  pinMode(PUSH_BUTT_3_PIN, INPUT);

  
  digitalWrite(TRIAC_MOTOR_PIN,LOW);
  digitalWrite(ROTAT_MOTOR_PIN,LOW);
  digitalWrite(SECURE_DOOR_PIN,LOW);
  digitalWrite(WATER_FILL__PIN,LOW);
  digitalWrite(WATER_EMPTY_PIN,LOW);
  digitalWrite(LAMP_INFO_1_PIN,LOW);
  digitalWrite(LAMP_INFO_2_PIN,LOW);
  digitalWrite(LAMP_INFO_3_PIN,LOW);
  digitalWrite(WATER_HEAT__PIN,LOW);
  
  //Serial initialization
  Serial.begin(19200);
  
  //LCD initialization
  delay(1000);
    
  sendCommand(0, 0, 0, 0, 1, 1);
  sendCommand(0, 0, 0, 0, 1, 1);
  sendCommand(0, 0, 0, 0, 1, 1);
  sendCommand(0, 0, 0, 0, 1, 0);
  sendCommand(0, 0, 0, 0, 1, 0);//4bits 2 lines
  sendCommand(0, 0, 1, 0, 0, 0);
    
  sendCommand(0, 0, 0, 0, 0, 0);//clear
  sendCommand(0, 0, 0, 0, 0, 1);
  sendCommand(0, 0, 0, 0, 0, 0);//cursor
  sendCommand(0, 0, 1, 1, 0, 0);

   
  //interrupts initialization
  Timer1.initialize(); // Initialize TimerOne library 
  MsTimer2::set(CYCLE_DURATION, cycle); // 
  MsTimer2::start();
  attachInterrupt(0, zeroCrossAC, RISING); //pin2, function to call or RISING/FALLING
  attachInterrupt(1, topTachy, RISING);     //pin3, function to call or RISING/FALLING
}



void loop() {
  
  //begining of 1 washing cycle
  printString("ARDUWASH v0.0",1,1);
  printString("Waiting to start",2,1);
  waitForPinStatus(PUSH_BUTT_3_PIN, true);
  printString("                ",1,1);
  
  for (int i=0; i<2; i++)
  {
    fillWater();
    wash();
    empty();
  }
  
  shake();
  
  printString("End             ",2,1);
  delay(5000);


  
}



void zeroCrossAC()
{
  //uncomment the followig to verify that there are not too many interrupts  
  count ++;
   
  //ignore if elapsed time since last interrupt is less than 9ms
  if ( (micros() - last_zerocross) < 9000 )
    return;

  //here is a good interrupt  (zero-crossing :))
  last_zerocross = micros();
  
  if (delay_triac < 1000)
  {
    digitalWrite(TRIAC_MOTOR_PIN,HIGH);
  }
  else if (delay_triac > 9000)
  {
    digitalWrite(TRIAC_MOTOR_PIN,LOW);
  }
  
  else
  {
    Timer1.attachInterrupt(manageTriac, delay_triac);
  }
}


void manageTriac()
{
  digitalWrite(TRIAC_MOTOR_PIN,HIGH);
  delayMicroseconds(50);
  digitalWrite(TRIAC_MOTOR_PIN,LOW);
 
  Timer1.stop();
  Timer1.detachInterrupt();
}


void cycle ()
{
  //actual speed of the motor
  Speed = tachy_counter*1000/CYCLE_DURATION;
  
  //calculate the delay_triac here
  //speed_setpoint=500;
  //speed_regulation();
  if (speed_setpoint!=0)
  {
    speed_regulation();
  }
  

  //Command the triac delay using serial terminal
  if (Serial.available() > 0) {
  // get incoming byte:
    char inByte = Serial.read();

    if (inByte=='a')
    {
      delay_triac = delay_triac + 100;
    }
    if (inByte=='z')
    {
      delay_triac = delay_triac - 100;
    }
    if (inByte=='e')
    {
      delay_triac = 10000;
      speed_setpoint = 0;
    }
    if (inByte=='r')
    {
      speed_setpoint = 80;
    }  
  }

  Serial.println(delay_triac);
  Serial.println(Speed);
  //RaZ counters
  count = 0;
  tachy_counter=0;  
}


void topTachy()
{
  //occurs when the tachy sensor value reaches 0
  tachy_counter++;
}

void speed_regulation()
{
  int new_delta = speed_setpoint - Speed;
  
  static int old_speed = 0;
  static int old_delta = 0;
  
  if (abs(new_delta)>=abs(old_delta))
  {
    //not improved since last time
    if (new_delta>0)
    {
      //go faster
      delay_triac-=50;
    }
    else
    {
      //go slower
      delay_triac+=50;
    }    
  }
  else
  {
    //at least as good as last time
    if (abs(new_delta-old_delta)>=abs(new_delta))
    {
      if (new_delta>0)
      {
        delay_triac+=50;
      
      }
      else
      {
        delay_triac-=50;
      }       
    }
  }
  
  
  if ((speed_setpoint)<MIN_SPEED)
  {
    delay_triac = 10000;
  }
  
  old_speed = Speed;
  old_delta = new_delta;
}


int sendCommand(boolean RS, boolean RW, boolean DB7, boolean DB6, boolean DB5, boolean DB4)
{
    digitalWrite(STR, LOW);
    digitalWrite(CLOCK, LOW);
    
    int tab[8];
    tab[0] = DB4;
    tab[1] = DB5;
    tab[2] = DB6;
    tab[3] = DB7;    
    tab[4] = 1; //EN   
    tab[5] = RW;
    tab[6] = RS;
    tab[7] = 0;    
         
    for (int n=0; n<8; n++)
    {
      digitalWrite(DATA, tab[n]);
      delayMicroseconds(1);
      digitalWrite(CLOCK, HIGH);
      delayMicroseconds(1);
      digitalWrite(CLOCK, LOW);
    }
    
    digitalWrite(STR, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STR, LOW);
    
    tab[4] = 0;
    
    for (int n=0; n<8; n++)
    {
      digitalWrite(DATA, tab[n]);
      delayMicroseconds(1);
      digitalWrite(CLOCK, HIGH);
      delayMicroseconds(1);
      digitalWrite(CLOCK, LOW);
    }
    
    digitalWrite(STR, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STR, LOW);
    
}

int printChar(char c)
{
    sendCommand(1, 0, c & 128 , c & 64, c & 32, c & 16);
    sendCommand(1, 0, c & 8, c & 4, c & 2, c & 1); 
}

int printString(char *s, char line=1, char column=1)
{
  char c = column-1;
  if (line==1)
  {
    sendCommand(0, 0, 1, 0, 0, 0);
    sendCommand(0, 0, c & 8, c & 4, c & 2, c & 1);
  }
  else if (line==2)
  {
    sendCommand(0, 0, 1, 1, 0, 0);
    sendCommand(0, 0, c & 8, c & 4, c & 2, c & 1);
  }
  
  for (int i=0; i<128; i++)
  {
    char a = s[i];
    if (a == 0)
      break;  
    else
    {    
      printChar(a);
    }
  }
}

bool validatePinStatus(int pin, bool Status)
{
  if (digitalRead(pin) != Status)
      return false;
  else
  {
      delayMicroseconds(200000);
	  if (digitalRead(pin) != Status)
        return false;
	  else
	    return true;
  }
}

int waitForPinStatus(int pin, bool Status)
{
  do {}
  while(!validatePinStatus(pin,Status));
}

int fillWater()
{
  printString("Filling...      ", 2,1);
  digitalWrite(WATER_FILL__PIN, HIGH);
  delay(10000);
  startMotor(80,0);
  waitForPinStatus(WATER_LVL_2_PIN, true);
  delay(5000);
  digitalWrite(WATER_FILL__PIN, LOW);
  stopMotor();
  printString("Full            ", 2,1);
  delay(2000);
}

int wash()
{
  bool rotation=false;
  digitalWrite(WATER_HEAT__PIN, HIGH);
  printString("Heating         ", 2,1);
  
  for (int i=0; i<15; i++)
  {
    startMotor(80,rotation);
    delay(60000);
    stopMotor();
    
    if (analogRead(A0)<500)
    {
      digitalWrite(WATER_HEAT__PIN, LOW);
      printString("Stop Heating    ", 2,1);
    }
    rotation = !rotation;
    
  }
  digitalWrite(WATER_HEAT__PIN, LOW);
}
int empty()
{
 printString("Emptying         ", 2,1);
 digitalWrite(WATER_EMPTY_PIN, HIGH);
 startMotor(80,0);
 
 waitForPinStatus(WATER_LVL_0_PIN, HIGH);
 
 delay(30000);
 digitalWrite(WATER_EMPTY_PIN, LOW);
 stopMotor();

 //
 delay(10000);
 startMotor(200,0);
 digitalWrite(WATER_EMPTY_PIN, HIGH);
 delay(20000);
 stopMotor();
 digitalWrite(WATER_EMPTY_PIN, LOW);

 
}

int shake()
{
 printString("Shaking         ", 2,1);
 digitalWrite(WATER_EMPTY_PIN, HIGH);
 startMotor(500,0);
 delay(30000);
 digitalWrite(WATER_EMPTY_PIN, LOW);
 stopMotor();
}

int startMotor(int MySpeed, bool rotation)
{
  if (rotation)
  {
    printString("<M ",1,1);
  }
  else
  {
    printString(" M>",1,1);
  }
  do {delay(100);}
  while(Speed!=0);
  
  digitalWrite(ROTAT_MOTOR_PIN, rotation);
  delay(1000);
  
  speed_setpoint = MySpeed;
}

int stopMotor()
{
  speed_setpoint = 0;
  delay_triac = 10000;
  digitalWrite(TRIAC_MOTOR_PIN, LOW);

  do {delay(100);}
  while(Speed!=0);

  digitalWrite(ROTAT_MOTOR_PIN, LOW);
  
  printString("   ",1,1);
}
