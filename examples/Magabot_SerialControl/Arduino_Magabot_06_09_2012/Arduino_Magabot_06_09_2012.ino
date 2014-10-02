#include <Wire.h>


#define REGISTER_CONFIG (16)
#define REGISTER_OUTPUT (16)

int dir1=0,dir2=0;
int inByte = 'p';
int ponteiro=0;
int veloc1 = 15;
int veloc2 = 15;
int reading = 0;
int Sonar_read=0;
int Sonar_number=1;
int Sonars[6];

int RGB[3];
int val,baterias;

char order = '2';

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(13, OUTPUT);    
  pinMode(12, OUTPUT);    

  //RGB led pins
  pinMode(11, OUTPUT);     
  pinMode(10, OUTPUT); 
  pinMode(9, OUTPUT);   
  
  //Switchs
  pinMode(2, INPUT);
  pinMode(3, INPUT);
 // pinMode(4, INPUT);
 // pinMode(5, INPUT);

  pinMode(6, OUTPUT);     
  pinMode(7, OUTPUT);     
  pinMode(8, OUTPUT);     
  
  Wire.begin();
  Serial.begin(9600);
  
  analogWrite(11,255);
  analogWrite(10,255);
  analogWrite(9,255);
  
  Wire.beginTransmission(0x39);
  Wire.write((byte) 0);
  Wire.write((byte) 0x77);
  Wire.endTransmission();



  
}

void loop() 
{ 

// changeAddress(0x70, 0xE4);
// while(1);

  read_BAT ();
  read_SRF ();

  Wire.beginTransmission(0x39);
  Wire.write((byte) 1);
  Wire.write((byte) 0x6F);
  Wire.write((byte) 0x6F);
  Wire.write((byte) 0x03);
  Wire.write((byte) 0x03);
  Wire.endTransmission();
 
  if (Serial.available() > 0) 
  {
    inByte = Serial.read();
    
    MagaBotControllerSerial();
    
    /*if(inByte == '1') order = inByte;
    if(inByte == '2') order = inByte;
    if(inByte == '3') order = inByte;
    if(order == '1') SerialAnalyze();*/
  }
/*  if(order == '1') AssistedNavigation();
  if(order == '2') obstacleAvoid();    */
}


int becoState =0;
unsigned long becoTime;
unsigned long bTime;
unsigned long iTime;
int bumper = 0; // 1 = left, 2 = right
boolean ir[3];
boolean bump[2];
int maxIR = 700;
unsigned long time;


//*****************************************************//
//******Actuate motors with left and righ velocity*****//
//*****************************************************//

void actuateMotors(int vel1, int vel2)
{
  vel2 = -vel2;
  byte v1b1 = vel1 >> 8;
  byte v1b2 = vel1 & 0xFF;
  byte v2b1 = vel2 >> 8;
  byte v2b2 = vel2 & 0xFF;
  
  Wire.beginTransmission(0x15);
  Wire.write((byte) 0);
  Wire.write((byte) v1b1);
  Wire.write((byte) v1b2);
  Wire.write((byte) 1);    //  high byte
  Wire.endTransmission();
  
  Wire.beginTransmission(0x16);
  Wire.write((byte) 0);
  Wire.write((byte) v2b1);
  Wire.write((byte) v2b2);
  //     Wire.write((byte) veloc2);  //  low byte
  //      Wire.write((byte) dir2);    //  high byte
  Wire.write((byte) 1);    //  high byte
  
  Wire.endTransmission();
}

//********************************//
//******Read Battery Status*******//
//********************************//
void read_BAT ()
{
  baterias = analogRead(3);
  baterias = analogRead(3);
          
  if (baterias>690) {digitalWrite(13, LOW);digitalWrite(12, LOW);}
  else if (baterias>616){digitalWrite(13, HIGH);digitalWrite(12, LOW);}
  else if (baterias>584){digitalWrite(13, HIGH);digitalWrite(12, HIGH);}
  else {digitalWrite(13, LOW);digitalWrite(12, HIGH);}
}


//***************************************//
//*****SONARS READ AND CHANGE ADRESS*****//
//***************************************//
void read_SRF ()
{
 
  if (Sonar_read==0)
  {
    Wire.beginTransmission(0x70+Sonar_number);
    Wire.write((byte) 0);
    Wire.write((byte) 0x51);
    Wire.endTransmission();
    Sonar_read=1;
    time = millis()+50;
  }
  else if (millis()>time)
  {
 // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(0x70+Sonar_number); // transmit to device #112
    Wire.write((byte) 0x02);             // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
    Wire.requestFrom(0x70+Sonar_number, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
    if(2 <= Wire.available())    // if two bytes were received
    {
      reading = Wire.read();  // receive high byte (overwrites previous reading)
      reading = reading << 8;    // shift high byte to be high 8 bits
      reading |= Wire.read(); // receive low byte as lower 8 bits

      Sonars[Sonar_number]=reading;       
      Sonar_read=0;
      Sonar_number++;
      if (Sonar_number==6) Sonar_number=1; //Sonar_number==6
    }
    else
    {
      Sonar_read=0;
      Sonar_number++;
      if (Sonar_number==6) Sonar_number=1; //Sonar_number==6      
    }
  }
}
//Change I2C Sonars Address
void changeAddress(byte oldAddress, byte newAddress)
{
  Wire.beginTransmission(oldAddress);
  Wire.write((byte) 0x00);
  Wire.write((byte) 0xA0);
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write((byte) 0x00);
  Wire.write((byte) 0xAA);
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write((byte) 0x00);
  Wire.write((byte) 0xA5);
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write((byte) 0x00);
  Wire.write((byte) newAddress);
  Wire.endTransmission();
}

// SONARS  treshold utility for a 20cm value

boolean sonarRead()
{
  for (int o=0;o<5;o++)
   {
     if(Sonars[o] < 20)
       return true;//?true:false;
   }
   return false;
}

//*************************************//
//****IR ground sensors read***********//
//*************************************//
void irRead() 
{
    digitalWrite(8, HIGH);
    analogRead(2);
    ir[0] = (analogRead(2)>maxIR)?true:false;
    digitalWrite(8, LOW);
    
    digitalWrite(7, HIGH);
    analogRead(1);
    ir[1] = (analogRead(1)>maxIR)?true:false;
    digitalWrite(7, LOW);
    
    digitalWrite(6, HIGH);
    analogRead(0);
    ir[2] = (analogRead(0)>maxIR)?true:false;
    digitalWrite(6, LOW);
}

//***************************************//
//****Front bumpers read function********//
//***************************************//
void bumpRead()
{
   bump[0] =(digitalRead(3)==1)?false:true;
   bump[1] = (digitalRead(2)==1)?false:true;
}

//**************************************//
//***********Obstacle avoidance*********//
//*********For autonomous control*******//
///*************************************//
void obstacleAvoid()
{
  irRead();
  bumpRead();
  
  veloc1 = 15;
  veloc2 = 15;
 
  int frontDistance = 100;
  int sideDistance = 50;


  if(bump[0])
  {
    becoState = 1;

    actuateMotors(-5,-veloc2);  
    becoTime = millis()+1000;  
  }
  else if(bump[1])
  {
    becoState = 1;
    actuateMotors(-veloc1,-5);  
    becoTime = millis()+1000;  
  }
  if(ir[1])
  {
    becoState = 1;
    actuateMotors(-10,-veloc2);  
    becoTime = millis()+1000;  
  }
  else if(ir[1])
  {
    becoState = 1;
    actuateMotors(-10,-veloc2);  
    becoTime = millis()+1000;  
  }
  else if(ir[2])
  {
    becoState = 1;

    actuateMotors(-veloc1,-10);  
    becoTime = millis()+1000;  
  }
  
  if (becoState ==1 && millis()>becoTime)
  {
    becoState = 0;
  }
  int bestDirection = 0;
  int bestDirectionValue = 0;

  for (int count = 1;count<6;count++)
  {
    if(Sonars[count] > bestDirectionValue)
    {
      bestDirectionValue = Sonars[count];
      bestDirection = count;
    }
  }
  String dir;
  if (becoState == 0)
  {    
    if(Sonars[3] > frontDistance)
    {
      actuateMotors(veloc1,veloc2);
      analogWrite(9,  255);
      analogWrite(10, 255);
      analogWrite(11, 0);
      if(Sonars[1] < sideDistance || Sonars[2] < sideDistance)
      {
        actuateMotors(veloc1,0);  
        analogWrite(9,  255);
        analogWrite(10, 0);
        analogWrite(11, 0);
      }
      else if(Sonars[4] < sideDistance || Sonars[5] < sideDistance)
      {
        actuateMotors(0,veloc2);  
        analogWrite(9,  255);
        analogWrite(10, 0);
        analogWrite(11, 0);
      } 
    }
    else
    {
      actuateMotors(veloc1,0);
      analogWrite(9,  255);
      analogWrite(10, 0);
      analogWrite(11, 0);
    }
  }
  else
  {
    analogWrite(9,  255);
    analogWrite(10, 0);
    analogWrite(11, 255);
  }
}


//***************************//
//****** Odometer ***********//
//***************************//
void read_clicks (void)
{
  //primeira parte
      Wire.beginTransmission(0x15);
      Wire.write((byte) 0x19);
      Wire.write((byte) 1);
      Wire.endTransmission();

delay(1);  

      Wire.beginTransmission(0x16);
      Wire.write((byte) 0x19);
      Wire.write((byte) 1);  
      Wire.endTransmission();

delay(1);  

    Wire.beginTransmission(0x15); // transmit to device 0x15
    Wire.write((byte) 0x15);             // sets register pointer to echo #1 register (0x15)
    Wire.endTransmission();

    Wire.requestFrom(0x15, 2);
    
    if(2 <= Wire.available())    // if two bytes were received
    {
       Serial.write(Wire.read());
       Serial.write(Wire.read());
    }
    
     Wire.beginTransmission(0x16); // transmit to device 0x16
     Wire.write((byte) 0x15);             // sets register pointer to echo #1 register (0x15)
     Wire.endTransmission();
    
    Wire.requestFrom(0x16, 2);
    if(2 <= Wire.available())    // if two bytes were received
    {
       Serial.write(Wire.read());
       Serial.write(Wire.read());
    }
  //primeira parte
    Wire.beginTransmission(0x15);
    Wire.write((byte) 0x14);
    Wire.write((byte) 0);
    Wire.endTransmission();
    delay(1);  
  
    Wire.beginTransmission(0x16);
    Wire.write((byte) 0x14);
    Wire.write((byte) 0);
    Wire.endTransmission();
    delay(1);  
}

int velocity =10;
int turnVel =20;
boolean fordward = true;
boolean stopped = true;
boolean  turning = false;
boolean right = false;


// for full low level control serial protocol
void MagaBotControllerSerial()
{
  if (ponteiro==0)
    {
      if (inByte==0x83)
      {
        for (int o=1;o<6;o++)
        {
          Serial.write((unsigned char) (Sonars[o] >> 8));
          Serial.write((unsigned char) (Sonars[o] & 0xFF));
        }
      }
      else if (inByte==0x74)
      {
        read_clicks();        
      }
      else if (inByte==0x73) //I IR read
      {
                
        digitalWrite(8, HIGH);
        val = analogRead(2);
        val = analogRead(2);
        digitalWrite(8, LOW);
        Serial.write((unsigned char) (val >> 8));
        Serial.write((unsigned char) (val & 0xFF));
        
        digitalWrite(7, HIGH);
        val = analogRead(1);
        val = analogRead(1);
        digitalWrite(7, LOW);
        Serial.write((unsigned char) (val >> 8));
        Serial.write((unsigned char) (val & 0xFF));
        
        digitalWrite(6, HIGH);
        val = analogRead(0);
        val = analogRead(0);
        digitalWrite(6, LOW);
        Serial.write((unsigned char) (val >> 8));
        Serial.write((unsigned char) (val & 0xFF));
                
      } 
     else if  (inByte==0x4B) //Baterias
     {
          Serial.write((unsigned char) (baterias >> 8));
          Serial.write((unsigned char) (baterias & 0xFF));        
     }
     else if (inByte==0x66) //B bumpers
      {
          int sensorValue1 = digitalRead(5);
          if (sensorValue1==1) {Serial.write((unsigned char) 0);}else Serial.write((unsigned char) 1);

          int sensorValue2 = digitalRead(4);
          if (sensorValue2==1) {Serial.write((unsigned char) 0);}else Serial.write((unsigned char) 1);
          
          int sensorValue3 = digitalRead(3);
          if (sensorValue3==1) {Serial.write((unsigned char) 0);}else Serial.write((unsigned char) 1);
          
          int sensorValue4 = digitalRead(2);
          if (sensorValue4==1) {Serial.write((unsigned char) 0);}else Serial.write((unsigned char) 1);
      }
      else if (inByte==0x76)  ponteiro=5; //LEDS
      else if (inByte==0x86) ponteiro=1;
    }
    else if (ponteiro==1) { veloc1 = inByte;ponteiro=2;}
    else if (ponteiro==2) { dir1 = inByte;ponteiro=3;}
    else if (ponteiro==3) { veloc2 = inByte;ponteiro=4;}
    else if (ponteiro==4) 
    {
      dir2 = inByte;
      ponteiro=0;
//      Wire.beginTransmission(0x18);
        Wire.beginTransmission(0x15);
      Wire.write((byte) 0);
      Wire.write((byte) veloc1);  //  low byte
      Wire.write((byte) dir1);    //  high byte
//      Wire.write((byte) veloc1);  //  low byte
//      Wire.write((byte) dir1);    //  high byte
      Wire.write((byte) 1);    //  high byte
      Wire.endTransmission();
      
      Wire.beginTransmission(0x16);
      Wire.write((byte) 0);
      Wire.write((byte) veloc2);  //  low byte
      Wire.write((byte) dir2);    //  high byte
//     Wire.write((byte) veloc2);  //  low byte
//     Wire.write((byte) dir2);    //  high byte
      Wire.write((byte) 1);    //  high byte
      Wire.endTransmission();
      
    }
    else if (ponteiro==5) { RGB[0] = inByte; ponteiro=6; analogWrite(11, (unsigned char) RGB[0]);}
    else if (ponteiro==6) { RGB[1] = inByte;ponteiro=7; analogWrite(10, (unsigned char) RGB[1]);}
    else if (ponteiro==7) { RGB[2] = inByte;ponteiro=0; analogWrite(9, (unsigned char) RGB[2]);}
}


//Simple characters instructions Serial control protocol
int SerialAnalyze()
{
  if (inByte=='a' || inByte == 'A')
  {
    right = false;
    turn();
    turning = true;   
    //actuateMotors(0,velocity); 
    //Serial.print("a");
  }
  //d
  else if(inByte=='d'|| inByte=='D')
  {
    right=true;
    turn();
  }
  //s
  else if(inByte=='s'|| inByte=='S')
  {
    veloc1 = -velocity;
    veloc2 = -velocity;
    fordward = false;
    turning = false;
    stopped = false;
  }
  //w
  else if(inByte=='w'|| inByte=='W')
  {
    veloc1 = velocity+5;
    veloc2 = velocity+5;
    fordward = true;
    turning = false;
    stopped = false;
    
  }
  //p
  else if(inByte=='p'|| inByte=='P' || inByte=='G' || inByte=='g')
  {
    veloc1 = 0;
    veloc2 = 0;
    turning = false;
    stopped = true;
  }
  
  else if(inByte== '+' || inByte == '-')//ox43 ='+' 0x45='-'
  {
    velocity= (inByte== '+') ? velocity+1 : velocity-1; 
    if (!stopped)
    {
      if(turning)
      {
        turn();
      }
      else if (fordward)
      {
        veloc1 = velocity;
        veloc2 = velocity;
      }  
      else//backward
      {
        veloc1 = -velocity;
        veloc2 = -velocity;
      }
    }
  }
  actuateMotors(veloc1,veloc2);
}

void turn()
{  
 if (!right)
 {
    if (stopped)
    {
       veloc1 = -10;
       veloc2 = 10;
    }
    else if (fordward) 
    {
      veloc1 = velocity-turnVel;
      veloc2 = velocity;
    }
    else
    {
      veloc1 = -velocity+turnVel;
      veloc2 = -velocity;
    }
  }
  else 
  {
    if (stopped)
    {
       veloc1 = 10;
       veloc2 = -10;
    }
    else if (fordward) 
    {
      veloc1 = velocity;
      veloc2 = velocity-turnVel;
    }
    else
    {
      veloc1 = -velocity;
      veloc2 = -velocity+turnVel;
    }
  }
}


void AssistedNavigation()
{
  irRead();
  bumpRead();
  
  /*if (ir[0])
  {
    actuateMotors(-velocity,0);
  }
  else if(ir[1])
    actuateMotors(-velocity,0);
  else if(ir[2])
    actuateMotors(0,-velocity);
  else
    actuateMotors(veloc1,veloc2);  
  */
  //delay(100);
  //boolean ola = sonarRead();
  if( (bump[0]||bump[1]) && millis()>bTime)
  {
    Serial.println('b');
    bTime = millis()+500;
  }
  
  if( (ir[0]||ir[1]||ir[2]) && millis()>iTime) 
  {
    Serial.println('i');
    iTime = millis()+500;
  }
  
  if( bump[0]||bump[1]||ir[0]||ir[1]||ir[2])
  {
    becoState =1;
    actuateMotors(-velocity,-velocity);  
    becoTime = millis()+500;  
  }
  if (becoState ==1 && millis()>becoTime)
  {
    veloc1 = 0;
    veloc2 = 0;
    actuateMotors(veloc1,veloc2);
    becoState = 0;
  }

  if (becoState ==1)
  {
    analogWrite(9,  255);
    analogWrite(10, 0);
    analogWrite(11, 255);
  }
  else
  {
    analogWrite(9,  0);
    analogWrite(10, 255);
    analogWrite(11, 255);
  }
  
}


