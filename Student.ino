#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RH_ASK.h>
#include <SPI.h>

//initializing the class of "rh_ask" library 
RH_ASK driver;

//initializing the class of the "Adafruit_PWMServoDriver" library
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


/*---------------=Options=---------------*/
/*----<Max & Min Define>----*/
// Servo-Pulse max & min
#define ServoMin                  140
#define ServoMax                  560
// Servo-Plate angle min & max
#define PlateMax                  180
#define PlateMin                    0
// Servo-MainHand angle min & max
#define MainMax                   180
#define MainMin                     0
// Servo-SecondHand angle min & max
#define SecondMax                 180
#define SecondMin                   0
// Servo-Fingers angle min & max
#define FingersMax                 90
#define FingersMin                  0

/*----<Globals>----*/
typedef enum 
{
  Plate = 0,
  Main,
  Second,
  Fingers
} StudentArmParts;

typedef enum 
{
  Save = 1,
  FastSave,
  Delete,
  Move,
  Load, 
  FastLoad
} Modes; // idle is the default in the switch case

typedef struct
{
  unsigned char ang_plate;
  unsigned char ang_main;
  unsigned char ang_second;
  unsigned char ang_fingers;
} Positions_t;

Positions_t Coordinates[10];
Positions_t FastCoordinates[100]; // this is enough for 5 second save
unsigned int pos_counter = 0;
unsigned int fast_pos_counter = 0;

typedef struct 
{
  unsigned char mode;
  unsigned char ang_plate;
  unsigned char ang_main;
  unsigned char ang_second;
  unsigned char ang_fingers;
} Msg_t __attribute__((packed));

unsigned long last_millis_load = 0;
unsigned long last_millis_fload = 0;
/*----<Setup>----*/
void setup() 
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  if (!driver.init())
         Serial.println("init failed");
  Serial.println("Done Setup.");
  pinMode(8, OUTPUT); // this is 5v power for the radio reciever
  digitalWrite(8,HIGH);
}


/*----<Loop>----*/
void loop() 
{
  pinMode(8, OUTPUT);
  digitalWrite(8,HIGH);
  uint8_t buf[6] = {0};
  uint8_t buflen = sizeof(buf);
  Msg_t msg;
  int pos = 0;
  if (driver.recv(buf, buflen)) // Non-blocking 
  {
    msg.mode = buf[0];
    msg.ang_plate = buf[1];
    msg.ang_main = buf[2];
    msg.ang_second = buf[3];
    msg.ang_fingers = buf[4];
    Serial.println("Got msg from Teacher");
    Serial.println((char*)buf);
    Serial.println((int)buf[0]);
    Serial.print((int)buf[1]);
    Serial.print((int)buf[2]);
    Serial.print((int)buf[3]);
    Serial.print((int)buf[4]);
    switch ((int)msg.mode)
    {
      case Save:
          if(!(pos_counter >= 10))
          {
            Coordinates[pos_counter].ang_plate = msg.ang_plate;
            Coordinates[pos_counter].ang_main = msg.ang_main;
            Coordinates[pos_counter].ang_second = msg.ang_second;
            Coordinates[pos_counter].ang_fingers = msg.ang_fingers;
            pos_counter++;
          }
          else
            Serial.println("Can't save more positions");
          break;
      case FastSave:
          FastSaveCoordinates(msg);
          break;
      case Delete:
          Clear();
          break;
      case Move: // CPU_IDLE
          MoveIdle(msg);
          break;
      case Load:
          if(millis() - last_millis_load > 1500)
          {
            last_millis_load = millis();
            LoadCoordinates(pos_counter);
            pos_counter++;
          }
          break;
      case FastLoad:
          if(millis() - last_millis_fload > 50)
          {
            last_millis_fload = millis();
            FastLoadCoordinates();
            fast_pos_counter++;
          }
          break;
      default:
          Serial.println("Check msg from radio");
          break;
    }
  }
}


/*----<Functions>----*/

// this function maps angle to pulse
int angleToPulse(int ang)
{
   int pulse = map(ang, 0, 180, ServoMin, ServoMax);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}

// this function gets part num and angle, and then sends to the right servo the needed angle
void SendCommand(int part, int angle)
{
  switch (part)
  {
    case Plate:
        Serial.println(angle);
        if ( angle <= PlateMax && angle >= PlateMin )
          pwm.setPWM(Plate, 0, angleToPulse(angle));
        break;
    case Main:
        if ( angle <= MainMax && angle >= MainMin )
          pwm.setPWM(Main, 0, angleToPulse(angle));
        break;
    case Second:
        if ( angle <= SecondMax && angle >= SecondMin )
          pwm.setPWM(Second, 0, angleToPulse(angle));
        break;
    case Fingers:
        if ( angle <= FingersMax && angle >= FingersMin )
          pwm.setPWM(Fingers, 0, angleToPulse(angle));
        break;
    default: //checking if i got a wrong arm num
        Serial.println("Check part num.");
        break;
  }
}

//this function loads the exsisting coordinates
void LoadCoordinates(int coordinate)
{
    SendCommand(Plate, Coordinates[coordinate].ang_plate);
    SendCommand(Main, Coordinates[coordinate].ang_main);
    SendCommand(Second, Coordinates[coordinate].ang_second);
    SendCommand(Fingers, Coordinates[coordinate].ang_fingers);
}

//this function clears coordinates array
void Clear()
{
  for(int i = 0; i < 10; i++)
  {
      Coordinates[i].ang_plate = 0;
      Coordinates[i].ang_main = 0;
      Coordinates[i].ang_second = 0;
      Coordinates[i].ang_fingers = 0;
  }
  for(int i = 0; i < 100; i++)
  {
      FastCoordinates[i].ang_plate = 0;
      FastCoordinates[i].ang_main = 0;
      FastCoordinates[i].ang_second = 0;
      FastCoordinates[i].ang_fingers = 0;
  }
  pos_counter = 0;
  fast_pos_counter = 0;
}

void MoveIdle(Msg_t msg)
{
  Serial.println("Moving idle");
  SendCommand(Plate, (int)msg.ang_plate);
  SendCommand(Main, (int)msg.ang_main);
  SendCommand(Second, (int)msg.ang_second);
  SendCommand(Fingers, (int)msg.ang_fingers);
}

void FastSaveCoordinates(Msg_t msg)
{
  FastCoordinates[fast_pos_counter].ang_plate = msg.ang_plate;
  FastCoordinates[fast_pos_counter].ang_main = msg.ang_main;
  FastCoordinates[fast_pos_counter].ang_second = msg.ang_second;
  FastCoordinates[fast_pos_counter].ang_fingers = msg.ang_fingers;
}

void FastLoadCoordinates()
{
  SendCommand(Plate, FastCoordinates[fast_pos_counter].ang_plate);
  SendCommand(Main, FastCoordinates[fast_pos_counter].ang_main);
  SendCommand(Second, FastCoordinates[fast_pos_counter].ang_second);
  SendCommand(Fingers, FastCoordinates[fast_pos_counter].ang_fingers);
}
