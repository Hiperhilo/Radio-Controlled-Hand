#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <String.h>
#include <HiperButton.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

//initializing the class of "rh_ask" library 
RH_ASK driver;

// init. lcd
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F, 16, 2);

// buttons
HiperButton Switch(8);
HiperButton LoadCo(9);
HiperButton Record(10);
HiperButton Stop(11);

// msg struct
typedef struct 
{
  unsigned char Mode;
  unsigned char ang_plate;
  unsigned char ang_main;
  unsigned char ang_second;
  unsigned char ang_fingers;
} Msg_t __attribute__((packed));

enum Modes
{
  Save = 1,
  FastSave,
  Delete,
  Move,
  Load, 
  FastLoad
}; // idle is the Move mode in the switch case of the Student

// this is a custom char for the ":)" symbol
// made this symbol in https://maxpromer.github.io/LCD-Character-Creator/
byte customChar[] = {
  0x00,
  0x00,
  0x0A,
  0x00,
  0x11,
  0x0E,
  0x00,
  0x00
};

// for timers
unsigned long print_time = 0;
unsigned long send_time = 0;

// ---------------------------------  Globals  ---------------------------
// -------------< Counters >------------
unsigned int old_pos = 0;
unsigned int old_fpos = 0;
unsigned int pos_counter = 0;
unsigned int fast_pos_counter = 0;
// -------------< Flags >---------------
bool fast_record = 0;
bool mode = 0;
bool load = 0;
bool allstop = 0;
bool record = 0;
bool alldelete = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
  lcd.init();
  lcd.backlight();
  if (!driver.init())
    Serial.println("init failed");
  InitAnimation();
}

void loop() {
  // ------------------------------------  Updating Buttons  -------------------------
  Switch.UpdateState();
  LoadCo.UpdateState();
  Record.UpdateState();
  Stop.UpdateState();
  // ------------------------------------  Prepare for sending  ----------------------
  ClickCheck();
  PrintData(mode, pos_counter, fast_pos_counter);
  // --------------------------------------------  Sending  --------------------------
  if(millis() - send_time > 50)
  {
    send_time = millis();
    //SendData();
  }
}

// this function is made for the init. animation
void InitAnimation()
{
  char animation[12] = "Welcome User";
  char ini[14] = "Init. Complete";
  lcd.createChar(0, customChar);
  lcd.setCursor(0, 0);
  for(int i = 0; i < 12; i++)
  {
    lcd.setCursor(i, 0);
    lcd.print(animation[i]);
    delay(50);
  }
  lcd.print(": ");
  lcd.write(0);
  delay(300);
  for(int i = 0; i < 14; i++)
  {
    lcd.setCursor(i, 1);
    lcd.print(ini[i]);
    delay(50);
  }
  delay(500);
  lcd.clear();
}

// this function will print the data (like mode and number of pos') to the lcd screen
void PrintData(int mode, int pos, int fast_pos)
{
  char str[3] = {0};
  //Serial.println(mode);
  if (millis() - print_time > 100) // PRINT
  {
    print_time = millis();
    if(pos != old_pos || old_fpos != fast_pos)
      lcd.clear();
    switch (mode) // false = regular, true = precise
    {
      case false:
        lcd.setCursor(0, 0);
        lcd.print("Mode: Regular");
        lcd.setCursor(0, 1);
        lcd.print("Num of pos: ");
        lcd.print(itoa(pos, str, 10));
        old_pos = pos;
        break;
      case true:
        lcd.setCursor(0, 0);
        lcd.print("Mode: Precise");
        lcd.setCursor(0, 1);
        lcd.print("Num of pos: ");
        lcd.print(itoa(fast_pos, str, 10));
        old_fpos = fast_pos;
        break;
      default:
        break;
    }
  }
}

//this function is checking buttons state, and then changing globals by that
void ClickCheck()
{
  if (Switch.IsClick())                             // SWITCH
  {
    mode = !mode;
    Serial.println("Click Switch");
  }
  if (Record.IsClick())                             // RECORD
  {
    if(mode == false && pos_counter < 10)           //<-Regular->
    {
      pos_counter++;
      record = true;
    }
    else if(mode == false && fast_pos_counter < 100)
      fast_record = true;
    Serial.println("Click Record"); 
  }
  if (LoadCo.IsClick())                               // LOAD
  {
    load = true;
    Serial.println("Click Load");                   
  }
  if (Stop.IsClick())                               // STOP
  {
    allstop = true;
    Serial.println("Click Stop");                   
  }
  if (Stop.IsPress())                              // DELETE
  {
    alldelete = true;
    pos_counter = 0;
    fast_pos_counter = 0;
  }
  //Serial.println(digitalRead(9));  
  //Serial.println(digitalRead(8));  
}

//this function will send a new msg to the student according to the flags in the globals
void SendData()
{
  unsigned char msg[6] = {0};
  //checking the button pressed flags and by that sending the msg
  if(allstop == true) // STOP
  {
    allstop = false;
    msg[0] = Move;
  }
  else if(record == true) // RECORD
  {
    record = false;
    msg[0] = Save;
  }
  else if(fast_record == true)
  {
    FastRecord();
  }
  else if(load == true) // LOAD
  {
    load = false;
    if(mode == 0)
      msg[0] = Load;
     else
      msg[0] = FastLoad;
  }
  else if(alldelete == true) // DELETE
  {
    alldelete = false;
    msg[0] = Delete;
  }
  else // CPU_IDLE
    msg[0] = Move;
  //Serial.println(msg[0]);
  //Serial.println(analogRead(A0));
  //Serial.println(map(analogRead(A0), 140, 860, 0, 180)); // 140 and 869 because between these two the resistor does 180 degrees
  if(analogRead(A0) >= 140 && analogRead(A0) <= 860) // Plate
    msg[1] = map(analogRead(A0), 140, 860, 0, 180);
  else
    msg[1] = 90;
  if(analogRead(A1) >= 140 && analogRead(A1) <= 860) // Main
    msg[2] = map(analogRead(A1), 140, 860, 0, 180);
  else
    msg[2] = 90;
  if(analogRead(A2) >= 140 && analogRead(A2) <= 860) // Second
    msg[3] = map(analogRead(A2), 140, 860, 0, 180);
  else
    msg[3] = 90;
  if(analogRead(A3) >= 140 && analogRead(A3) <= 860) // Fingers
    msg[4] = map(analogRead(A3), 140, 860, 0, 180);
  else
    msg[4] = 90;
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  //Serial.println("Sent");
}

//this function is made to do fast record
void FastRecord()
{
  unsigned char msg[6] = {0};
  while(!Stop.IsClick() && fast_pos_counter < 100)
  {
    msg[0] = FastSave;
    if(analogRead(A0) >= 140 && analogRead(A0) <= 860) // Plate
      msg[1] = map(analogRead(A0), 140, 860, 0, 180);
    else
      msg[1] = 90;
    if(analogRead(A1) >= 140 && analogRead(A1) <= 860) // Main
      msg[2] = map(analogRead(A1), 140, 860, 0, 180);
    else
      msg[2] = 90;
    if(analogRead(A2) >= 140 && analogRead(A2) <= 860) // Second
      msg[3] = map(analogRead(A2), 140, 860, 0, 180);
    else
      msg[3] = 90;
    if(analogRead(A3) >= 140 && analogRead(A3) <= 860) // Fingers
      msg[4] = map(analogRead(A3), 140, 860, 0, 180);
    else
      msg[4] = 90;
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    Serial.println("Sent Fast ;D");
    fast_pos_counter++;
    delay(50);
  }
}
