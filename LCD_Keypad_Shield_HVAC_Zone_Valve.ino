//Include the LiquidCrystal header file which is inside the Arduino IDE
#include <EEPROM.h>
#include <LiquidCrystal.h> 
#include <OneWire.h>
#include <DallasTemperature.h>
// Can't use Servo.h - causes conflicts with LCD Keypad shield.
// #include <Servo.h>

// Servo damper;
#define damperPIN 3
#define T_ADDR 0
int t_setpoint = 22;
boolean bDamperOpen = true;
int state = 0; // 0 = Auto, 1= Open, 2 = Closed

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress room = {0x10, 0x56, 0x22, 0x49, 0x00, 0x08, 0x00, 0x19};
DeviceAddress plenum = {0x10, 0x6D, 0x30, 0x49, 0x00, 0x08, 0x00, 0xF3};
#define ROOM 0
#define PLENUM 1
int deviceCount;
int temp[2];

/* ==== PINS ASSIGNMNET ==========
 * LCD RS pin to digital pin 8
 * LCD EN pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * Backlight PWM control to Pin 10
 * LCD R/W pin to ground
*/
// Set the I/O pin for LCD 4-bit mode following the library assignment: 
//  LiquidCrystal(rs, en, d4, d5, d6, d7).
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int backLightTimeout = 5000;
int backLightTimer;
int analogPin = A0;  //Define the A0 as analogPin as integer type.
int adc_key_old;
int adc_key_in;
int NUM_KEYS = 5;
int key=-1;
int adc_key_val[5] ={30, 150, 360, 535, 760 }; //Define the value at A0 pin 
                                               // when a key is pressed.

// Define each key as character string type for keypress display.               
char msgs[5][15] = {"Right Key OK ", 
                    "Up Key OK    ", 
                    "Down Key OK  ", 
                    "Left Key OK  ", 
                    "Select Key OK"};
                    
                    
/*******************************************************************************
* PRIVATE FUNCTION: setup()
* PARAMETERS:void
* RETURN:void
* DESCRIPTIONS:
* Define of I/O pin as Input or Output 
*
*******************************************************************************/
// The setup() method runs once, when the sketch starts
void setup ()
{
  // Debug setup
  Serial.begin(115200);

  // OneWire setup
  dumpOneWireAddresses();
  sensors.begin();
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount(); 
  Serial.print(deviceCount);
  Serial.println(" devices");

  // LCD keypad shield setup
  lcd.begin(16, 2);     // set the lcd type: 16-character by 2-lines
  lcd.clear();          // LCD screen clear
  analogWrite(10,250);  // Light it up

  // Setup the servo/damper
  // Manual bit-banging the pin as the Servo library kills too many I/O pins
  pinMode(damperPIN, OUTPUT);
  pulseDamper(!bDamperOpen);
  pulseDamper(bDamperOpen);

  // Check if a (reasonable) setpoint has been stored in EEPROM
  // reasonable being > 0 < 40?
  int tmp_t = EEPROM.read(T_ADDR);
  if (tmp_t > 0 && tmp_t < 40) {
    t_setpoint = tmp_t;
    Serial.print("Stored setpoint:");
  }
  else 
    Serial.print("No stored setpoint, using default: ");
  Serial.println(t_setpoint);
}


/*******************************************************************************
* PRIVATE FUNCTION: loop()
* PARAMETERS: void
* RETURN: void
* DESCRIPTIONS:
* Non-Stop looping 
*******************************************************************************/
void loop()
{
  processKeyPad();  

  // State Auto/Open/Closed
  switch(state) {
    case 0:   // Auto
      logTemps();
      updateDisplay();
      controlDamper();
      break;
    case 1:   // Open
    case 2:   // Closed
      pulseDamper(bDamperOpen);
      break;
  }

  dimBackLight();
}
  

/*******************************************************************************
* PRIVATE FUNCTION: get_key
* PARAMETERS: integer
* RETURN:unsigned int input
* DESCRIPTIONS:
* convert the ADC value to number between 0 to 4
*******************************************************************************/
int get_key()
{
  int k;
  int input = analogRead(analogPin);  // Read the value at analogPin A0 and store 
                                      // the value in the adc_key_in register
    
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {
      backLightTimer = backLightTimeout;
      return k;
    }
  }
    
  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed
    
  return k;
}

void processKeyPad()
{
  static int tc = 1;
  adc_key_in = get_key(); // check for key press.

// 0  Right
// 1  Up
// 2  Down
// 3  Left
// 4  Select

  if (adc_key_in >= 0) tc++; else tc=1;
  switch(adc_key_in) {
    case 0:   // right - cycle Auto->Open->Closed->Auto
      if ( (tc%5000) == 0) {
        lcd.clear(); lcd.setCursor(0,0);
        if (++state >= 3) { state = 0;  }
//        Serial.print("State:"); Serial.println(state);
        if (state != 0) bDamperOpen = (state == 1);
        if (bDamperOpen)
          lcd.print("DAMPER OPEN");
        else
          lcd.print("DAMPER CLOSED");
      }
      break;
    case 1:   // up
      // if tic-count is mod 10? then do something.
      if ( (tc%5000) == 0) t_setpoint++;
      break;
    case 2:   // down
      // if tic-count is mod 10? then do something.
      if ( (tc%5000) == 0) t_setpoint--;
      break;
    case 3:   // left - cycle Auto->Closed->Open->Auto
      if ( (tc%5000) == 0) {
        lcd.clear(); lcd.setCursor(0,0);
        if (--state < 0) { state = 2; }
        if (state != 0) bDamperOpen = (state == 1);
        if (bDamperOpen)
          lcd.print("DAMPER OPEN");
        else
          lcd.print("DAMPER CLOSED");
      }
      break;
    case 4:   // select
      if ( (tc%250) ==0) {
        lcd.clear();                        // LCD screen clear
        lcd.setCursor(0,0);           // set the position of next message string: 
        EEPROM.update(T_ADDR, t_setpoint);
        lcd.print("SETPOINT STORED");
        Serial.print("Storing new setpoint: "); Serial.println(t_setpoint);
        delay(2500);
        backLightTimer = 255;   // force dim
      }
      break;
  }
}

void dimBackLight()
{
  static int tc = 10; tc--; if (tc) return; tc = 10;
  
  backLightTimer--;if (backLightTimer<0) backLightTimer=0;

  if (backLightTimer < 255)
  {
    analogWrite(10,backLightTimer);
  } else {
    analogWrite(10,250);
  }
}

void logTemps()
{
  float t;  // temp temperature for sanity checking.
  static long tc = 100000; // tick counter - only work every x times called.
  
  tc--;
  if (tc) {
    return;
  }
  tc = 100000;
  
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Trps:");
  sensors.requestTemperatures(); // Send the command to get temperatures

  t = sensors.getTempC(room);
  if (t<85) { temp[ROOM] = t*10.0; Serial.print(temp[ROOM]/10.0); } else { Serial.print("R"); }
  Serial.print(",");
  t = sensors.getTempC(plenum);
  if (t<85) { temp[PLENUM] = t*10.0; Serial.print(temp[PLENUM]/10.0); } else { Serial.print("P"); }

  Serial.print(","); Serial.println( t_setpoint );
}

void updateDisplay()
{
  static int tc = 1000; // tick counter - only work every x times called.
  
  tc--;
  if (tc) {
    return;
  }

  tc = 1000;

  lcd.clear();                        // LCD screen clear
  lcd.setCursor(0,0);           // set the position of next message string: 
  lcd.print("T[R]=");
  lcd.print(temp[ROOM]/10);
  lcd.print(" T[P]=");
  lcd.print(temp[PLENUM]/10);

  lcd.setCursor(0,1);
  lcd.print("TS=");
  lcd.print(t_setpoint); // Serial.println(t_setpoint);
  lcd.print(" V=");
  if (bDamperOpen) lcd.print("OPEN"); else lcd.print("CLOSED");
}

void dumpOneWireAddresses()
{
  // oneWire
    byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  
  Serial.print("Looking for 1-Wire devices...\n\r");
  while(oneWire.search(addr)) {
    Serial.print("\n\rFound \'1-Wire\' device with address:\n\r");
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print("CRC is not valid!\n");
        return;
    }
  }
  Serial.print("\n\r\n\rThat's it.\r\n");
  oneWire.reset_search();
  return;
}

// Cannot use Servo.h as that disables the PWM on pin 10 (LCD backlight)
// and seems to cause other timing issues.
void pulseDamper(bool bOpen){
  static boolean bState = true;
  static int tc;
  int Pttl = 3000;    // Total pulse length (delay between pulses to let servo catch it's breath
  int P0 = 500;       // Open pulse length
  int P1 = 2250;      // Close pulse length
  static int Pc = P1; // current position
  static int Ps = 1;  // step
  
  if (bOpen != bState)
  {
    bState = bOpen;
    if (bOpen) Serial.println("D:O"); else Serial.println("D:C");
    tc = 125;
    Ps = 1;
  }
  
  if (!tc) {
    return;
  }
  tc--;

  if (bOpen) {
    Pc -= Ps; Ps++;
    if (Pc < P0) Pc = P0;
  } else {
    Pc += Ps; Ps++;
    if (Pc > P1) Pc = P1;
  }
  
  digitalWrite(damperPIN, HIGH);
//  if (bOpen) delayMicroseconds(P0); else delayMicroseconds(P1);
  delayMicroseconds(Pc);
  digitalWrite(damperPIN, LOW);
//  if (bOpen) delayMicroseconds(Pttl - P0); else delayMicroseconds(Pttl - P1);
  delayMicroseconds(Pttl - Pc);
}

//
// controlDamper
//
// Logic to determine if the air in the plenum/vents is wanted/helpful or not.
//
// If the room temp is ok, damper should be open for fresh air.
// If the room temp is too warm, then the damper should close if plenum is also warm.
// If the room temp is too cold, then the damper should close if plenum is also cold.
//
// If Open Then
//    Close if (Tr+0.5 > Ts && Tp-0.5 > Ts) // too warm and heating
//      || (Tr-0.5 < Ts && Tp+0.5 > Ts) // too cool and cooling
//  Else
//    Open if (Tr+0.5 > Ts && Tp+0.5 < Ts) // too warm and cooling
//      || (Tr-0.5 < Ts && Tp-0.5 > Ts)  // too cool and heating
//
void controlDamper_old()
{
  if (bDamperOpen) {
    bDamperOpen = ! ( ((temp[ROOM] - 5) > (t_setpoint*10) && temp[PLENUM] > (temp[ROOM] + 5) )   // Too hot - don't heat
                || ((temp[ROOM] + 5) < (t_setpoint*10) && temp[PLENUM] < (temp[ROOM] - 5) ) ); // Too cold - don't cool
  } else {
    bDamperOpen =  ( ((temp[ROOM] - 5) > (t_setpoint*10) && temp[PLENUM] < (temp[ROOM] - 5) )   // Too hot - please cool
                || ((temp[ROOM] + 5) < (t_setpoint*10) && temp[PLENUM] > (temp[ROOM] + 5) ) ); // Too cold - please heat
  }

  pulseDamper(bDamperOpen);
}

// Matt's version to introduce better hysteresis
void controlDamper()
{
  int closeEnough = 10; // within a degree of setpoint, always stay open
  int hysteresis = 5; // how far into close zone must room temperature get before closing?
        // also, how far into the open quadrant must plenum temperature get before opening?
  
  bool withinTRCE = (    temp[ROOM] - closeEnough < t_setpoint    &&    temp[ROOM] + closeEnough > t_setpoint    ); // is the room temperature close enough to the setpoint to just stay open?
  bool withinTRH = (    temp[ROOM] - closeEnough - hysteresis < t_setpoint    &&    temp[ROOM] + closeEnough + hysteresis > t_setpoint    ); // is the room temperature within the hysteresis distance of the close enough zone?
  bool withinTPH = (    temp[PLENUM] - hysteresis < temp[ROOM]    &&    temp[PLENUM] + hysteresis > temp[ROOM]    ); // is the plenum temperature within the hysteresis distance of the room?
  bool openQuadrant = (    temp[ROOM] < t_setpoint    ); // are we in one of the quadrants where the valve should be open?
      // in other words, is the plenum closer to the setpoint then the room is?
    
  if (    temp[PLENUM] < temp[ROOM]    )
    openQuadrant = ! openQuadrant;
  
  if (bDamperOpen) {    // damper is open, stay open as long as we are in the open quadrants OR in room temperature hysteresis zone
    bDamperOpen = openQuadrant || withinTRH;
  } else {      // damper is closed, open if we enter the close enough zone or past the damper hysteresis in an open quadrant
    bDamperOpen = withinTRCE    ||    ( openQuadrant && ! withinTPH);
  }
  

  pulseDamper(bDamperOpen);

}
