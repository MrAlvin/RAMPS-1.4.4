/*
 * This sketch will test several elements on the RAMPS 1.4.4 shield
 * 
 *  - Use the program 'puTTY' as the 'serial terminal'/'monitor program' to see results and readouts via the serial port
 *  
 *       putTTY understands VT100 screen ESC codes, and this will 
 *       show the results on a stable screen image, 
 *       rather than a continued scroll, as the Arduino IDE built-in serial monitor will do.
 *
 *       puTTY can be found here: https://www.putty.org/
 *       
 */
 
#define RAMPS144        // For testing RAMPS 1.4.4 shield, on any Controller board. 
#define RAMPS144_DUE  // Add this for testing on DUE  - DUE is different on analog pins
//#define AGCM4         // Add this for testing on AGCM4 - Adafruit Grand Central M4 has different Digital to Analog pin number correlations
  
//#define REV_B         // For testing shield 1.7B
//#define REV_C         // For testing shield 1.7C 

#define BAUD_RATE 115200

#define Direct_LCD // 20x4 Text LCD - RRD Smart LCD controller

   //Testing I2C_LCD requres a 23017 chip to actually be connected. 
   //this sketch will lock-up (as in halt/wait) if no 23017 chip is found on the I2C bus
//#define I2C_LCD // 20x4 Text LCD via MCP23017 I2C I/O expander chip, PANELOLU2 style
                  
                    
/* The following sections on the RAMPS 1.4.4 shield will be tested:
 *  - Mosfets   
 *       one mosfet is blinked three times, then the next mosfet is being blinked three times
 *       continues rotation of which mosfet to blink while the sketch is running
 *  - Temperature pins  
 *       are read, and raw value is displayed.
 *       Connect any resistor between each of the two pins, 
 *       to change the number being displayed.
 *  - End-stops
 *       reads and shows the status of all end-stops.
 *       Connect any end-stop to GND to test that it works. 
 *       Any end-stop will also make the Debug_LED (D13) light up. 
 *       (Debug_LED (D13 will also still be blinked, in its mosfet rotation).
 *  - Z-probe
 *       reads and shows the status of the Z-probe at the end of the End-stop status line
 *       Connect the Z-probe 12V plus, to the Z-probe S pin (Signall pin), 
 *       to test Z-probe functionality
 *  - Stepper drivers X, Y, Z, E0, E1 (with A4988 setting)
 *       All steppers are moved for 2 seconds in one direction, then direction is changed,
 *       and this continues, while the sketch is running.
 *  - TMC2130 testing on X, Y, Z, E0, E1
 *       All steppers are moved for 2 seconds in one direction, then direction is changed,
 *       and this continues, while the sketch is running.
 *  - EEPROM   
 *       press E to read and write to/from the EEPROM
 *  - PS_ON
 *       connect a resistor and LED between Vcc and PS_ON, and it will be continually blinked
 *       A resistor of 220 ohm to 570 ohm should work fine.
 *      
 */

//***** For End-stops
#define X_MIN_PIN           3
#define X_MAX_PIN           2
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#ifdef REV_B
  #define Z_PROBE_PIN      64
#else
  #ifdef REV_C
    #define Z_PROBE_PIN    48
  #else  // RAMPS 1.4.4
    #define Z_PROBE_PIN    19  // Z-min
  #endif
#endif


//misc variables
int Use_puTTY = true;
int Use_TMC = false;    // activate TMC2130 testing ny pressing T on the keyboard, using the 'serial terminal' window
int incomingByte = 0;   // for incoming serial data
int do_test_i2c_eeprom = false;

const int DebugLed_pin = 13;
bool PauseEndStopBlink = false;
int eeprom_data_1 =0;


#ifdef Direct_LCD
  int do_LCD_update = false;
#endif 

  
// PS_ON test - connect a resistor and LED between Vcc and "P-On" pins, and the diode will be blinked
#if defined(RAMPS144) || defined(RAMPS144_DUE)
  const int PS_ON_pin = 39; 
#else
  const int PS_ON_pin = 39; 
#endif

//program flow
int do_home = false; 

//***** For Blink n-times of mosfets
byte _blinks = 3;
byte _blink_minor = 0;

int blinkN_pin = 13;    //the pin number that the LED is connected to
int blinkN_on_delay = 100;    //change this value to adjust the number of MilliSeconds the LED is ON
int blinkN_off_delay = 100;   //change this value to adjust the number of MilliSeconds the LED is OFF
int blinkN_long_off_delay = 400;


//***** For EEPROM 
#include <Wire.h>     // for I2C
 
#define eeprom_address 0x50    // device address 
byte d = 0;
byte b = 0;
int  i = 0;

//***** Analog Read and Serial Out example, but without the use of delay() *****
#if defined(RAMPS144_DUE)
  const int analogInPin_1 = A9;  // Analog input pin that the potentiometer is attached to
  const int analogInPin_2 = A10;  // Analog input pin that the potentiometer is attached to
  const int analogInPin_3 = A11;  // Analog input pin that the potentiometer is attached to
#elif defined(RAMPS144)
  const int analogInPin_1 = A13;  // Analog input pin that the potentiometer is attached to
  const int analogInPin_2 = A14;  // Analog input pin that the potentiometer is attached to
  const int analogInPin_3 = A15;  // Analog input pin that the potentiometer is attached to
#else  // 1.7
  const int analogInPin_1 = A5;  // Analog input pin that the potentiometer is attached to
  const int analogInPin_2 = A6;  // Analog input pin that the potentiometer is attached to
  const int analogInPin_3 = A7;  // Analog input pin that the potentiometer is attached to
#endif

//***** For 5 Stepper drivers on RAMPS 1.7
// For 4988 style stepper boards. 

// X-stepper
#ifdef RAMPS144
  #define enaPinX    38
  #define csPinX     47
  #if defined(AGCM4)
    #define stepPinX   67  // A0  (ACGM4 digital numbering is different for Analog pins)
    #define directPinX 68  // A1  (ACGM4 digital numbering is different for Analog pins)
  #else
    #define stepPinX   54  // A0
    #define directPinX 55  // A1
  #endif

#else //for RAMPS 1.7, rev B & C
  #define enaPinX    55
  #define stepPinX   56
  #define directPinX 57
  #ifdef REV_B
    #define csPinX   46
  #else //rev_C
    #define csPinX   62
  #endif
#endif

// Y-stepper
#ifdef REV_B
  #define enaPinY    58
  #define stepPinY   62
  #define directPinY 63
  #define csPinY     42
#else
  #ifdef REV_C
    #define enaPinY    58
    #define stepPinY   46
    #define directPinY 42
    #define csPinY     63
   #else // RAMPS 1.4.4
    #if defined(AGCM4)
      #define enaPinY    69  //A2  (ACGM4 digital numbering is different for Analog pins)
      #define stepPinY   73  //A6  (ACGM4 digital numbering is different for Analog pins)
      #define directPinY 74  //A7  (ACGM4 digital numbering is different for Analog pins)
    #else
      #define enaPinY    56  //A2
      #define stepPinY   60  //A6
      #define directPinY 61  //A7
    #endif
    #define csPinY     45
   #endif
#endif

// Z-stepper
#ifdef RAMPS144
  #if defined(AGCM4)
    #define enaPinZ    54  //A8  (ACGM4 digital numbering is different for Analog pins)
  #else
    #define enaPinZ    62  //A8
  #endif
  #define stepPinZ   46
  #define directPinZ 48
  #define csPinZ     32
#else //for RAMPS 1.7, rev B & C
  #define enaPinZ    67
  #define stepPinZ   68
  #define directPinZ 69
  #ifdef REV_B
    #define csPinZ   48
  #else
    #define csPinZ   66
  #endif
#endif

// E0-stepper
#ifdef RAMPS144
  #define enaPinE0    24
  #define stepPinE0   26
  #define directPinE0 28
  #define csPinE0     22
#else //for RAMPS 1.7, rev B & C
  #define enaPinE0    30
  #define stepPinE0   34
  #define directPinE0 36
  #ifdef REV_B
    #define csPinE0   38
  #else
    #define csPinE0   64
  #endif
#endif

// E1-stepper
#ifdef RAMPS144
  #define enaPinE1    30    
  #define stepPinE1   36
  #define directPinE1 34
  #define csPinE1     43
#else //for RAMPS 1.7, rev B & C
  #define enaPinE1    22    
  #define stepPinE1   24
  #define directPinE1 26
  #ifdef REV_B
    #define csPinE1    6
  #else
    #define csPinE1    6
  #endif
#endif


// Library needed: https://github.com/teemuatlut/TMCStepper
// But follow this install guide: http://marlinfw.org/docs/hardware/tmc_drivers.html#installing-the-library
// the install guide lets you install the library from within Arduino IDE
#include <TMC2130Stepper.h>

TMC2130Stepper driverX = TMC2130Stepper(enaPinX, directPinX, stepPinX, csPinX);
TMC2130Stepper driverY = TMC2130Stepper(enaPinY, directPinY, stepPinY, csPinY);
TMC2130Stepper driverZ = TMC2130Stepper(enaPinZ, directPinZ, stepPinZ, csPinZ);
TMC2130Stepper driverE0 = TMC2130Stepper(enaPinE0, directPinE0, stepPinE0, csPinE0);
TMC2130Stepper driverE1 = TMC2130Stepper(enaPinE1, directPinE1, stepPinE1, csPinE1);

// For standard RRD Text LCD 
#ifdef Direct_LCD
  #include <LiquidCrystal.h>
  
  // These are for the NewLiquidCrystal_151 library
  // LiquidCrystal lcd(en, rs, d4, d5, d6, d7, bl, blpol)
  // See: https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
  //
  // Delete or move the Arduino default 'LiquidCrystal' library, 
  // to avoid using the wrong library.
  //
  // Look at C:\Program Files (x86)\Arduino\libraries and 
  // move the folder 'LiquidCrystal' out of the 'libraries' folder. 
  // It is not enough to rename it, it needs to be removed!
  
  // these pin definitions all go to Aux-4
  #define LCD_PINS_RS     16
  #define LCD_PINS_ENABLE 17
  #define LCD_PINS_D4     23
  #define LCD_PINS_D5     25
  #define LCD_PINS_D6     27
  #define LCD_PINS_D7     29
  
  LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7);
  
  // these pin definitions all go to Aux-4
  #define BEEPER_PIN        37
  #define BTN_EN1           31
  #define BTN_EN2           33
  #define BTN_ENC           35
  #define KILL_PIN          41
#endif  

//#ifdef I2C_LCD
  // include the library code:
  #include <Wire.h>
  #include <LiquidTWI2.h>
  
  // Connect via i2c, address 0x20 (A0-A2 not jumpered)
  LiquidTWI2 i2c_lcd(0x20);
//#endif 

// pre-defining functions - for some reason the compiler complained today (and this fixed it)
void putty_ready();
void test_I2C_EEPROM_Main();
void test_i2c_eeprom();
void writeData(unsigned int eaddress, byte data);
byte readData(unsigned int eaddress);
void read_serial();
void AnalogRead_and_SerialOut_example();
void Endstops();
void EndstopsDebugLED();
void ShowEndstopsHL();
void ShowEndstops10();
void Blink(int blink_pin);
void blinkN(byte bl);
void blinkN(byte major, byte minor);
void blinkN();
void SetDirection();
void OneStepAllPins();
void TMC_ready();
void set_A4988_style();
void remove_A4988_style();
void list_serial_commands();
void Update_eeprom_info_on_display();
void show_last_eeprom();
void setup_DirectLCD();
void update_LCD();
void LCD_count();
void setup_I2C_LCD();
void I2C_LCD_count();

//Note to self: fix variable names (to be nouns) and function names (to be adjectives)
//or just keep collecting ramdom stuff, and let your mood and every whip inspire you, like now ;-)



//*****************************************************//
//****                                             ****//
//****                 SETUP                       ****//
//****                                             ****//
//*****************************************************//

void setup()
{
  // initialize end-stop pins 
  pinMode(X_MIN_PIN, INPUT_PULLUP);  
  pinMode(X_MAX_PIN, INPUT_PULLUP);  
  pinMode(Y_MIN_PIN, INPUT_PULLUP);  
  pinMode(Y_MAX_PIN, INPUT_PULLUP);  
  pinMode(Z_MIN_PIN, INPUT_PULLUP);  
  pinMode(Z_MAX_PIN, INPUT_PULLUP);  
  pinMode(Z_PROBE_PIN, INPUT_PULLUP);

  // initialize digital pin 13 (Built-in LED) 
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // initialize PS-ON test, using a LED - you need to add the LED 
  pinMode(PS_ON_pin, OUTPUT);
  digitalWrite(PS_ON_pin, LOW);
  
  // initialize mosfet pins 
  pinMode(8, OUTPUT);  //fan2 (1), 1.4.4 = bed
  digitalWrite(8, LOW);
  pinMode(9, OUTPUT);  //heat-bed, 1.4.4 = fan0
  digitalWrite(9, LOW);
  pinMode(10, OUTPUT); //fan1,     1.4.4 = hot-end
  digitalWrite(10, LOW);
  #if defined(REV_B) || defined(REV_C)
    pinMode(11, OUTPUT); //hot-end
    digitalWrite(11, LOW);
  #endif
  pinMode(12, OUTPUT); //pwr-on or fan3 (2), 1.4.4 = fan2
  digitalWrite(12, LOW);
  #ifdef REV_C
    pinMode(6, OUTPUT); //fan(3)
    digitalWrite(6, LOW);
    pinMode(7, OUTPUT); //fan(4)
    digitalWrite(7, LOW); 
  #endif
  #if defined(RAMPS144) || defined(RAMPS144_DUE)
    pinMode(7, OUTPUT); //1.4.4 = fan1
    digitalWrite(7, LOW);
  #endif

  Serial.begin(BAUD_RATE); // Initialize the serial
  if(Use_puTTY){
    putty_ready();
    Serial.println( F("Hello World!           Using puTTY (VT100 esc codes)") );
    putty_wait_position();
    list_serial_commands();
  }else{
    Serial.println( F("Hello World!") );
    Serial.println();
    list_serial_commands();
    Serial.print( F("blinking pin ") );
    Serial.print(blinkN_pin );
  }

  set_A4988_style();
  

  // TMC_ready();
  
  #ifdef Direct_LCD
    setup_DirectLCD();
  #endif

  #ifdef I2C_LCD
    setup_I2C_LCD();
  #endif
}

void set_A4988_style(){
  // Initialize stepper driver pins
  pinMode(enaPinX,OUTPUT);  // enable pin
  pinMode(enaPinY,OUTPUT);  // enable pin
  pinMode(enaPinZ,OUTPUT);  // enable pin
  pinMode(enaPinE0,OUTPUT); // enable pin
  pinMode(enaPinE1,OUTPUT); // enable pin
  
  pinMode(stepPinX,OUTPUT);  // step pin
  pinMode(stepPinY,OUTPUT);  // step pin
  pinMode(stepPinZ,OUTPUT);  // step pin
  pinMode(stepPinE0,OUTPUT); // step pin
  pinMode(stepPinE1,OUTPUT); // step pin
    
  pinMode(directPinX,OUTPUT);  // direct pin
  pinMode(directPinY,OUTPUT);  // direct pin
  pinMode(directPinZ,OUTPUT);  // direct pin
  pinMode(directPinE0,OUTPUT); // direct pin
  pinMode(directPinE1,OUTPUT); // direct pin

  //set direction pins 
  digitalWrite(directPinX,HIGH);  //set rotation direction
  digitalWrite(directPinY,HIGH);  //set rotation direction
  digitalWrite(directPinZ,HIGH);  //set rotation direction
  digitalWrite(directPinE0,HIGH); //set rotation direction
  digitalWrite(directPinE1,HIGH); //set rotation direction


  //set enable pins to: Enable
  digitalWrite(enaPinX,LOW);  //enable pin pulled low
  digitalWrite(enaPinY,LOW);  //enable pin pulled low
  digitalWrite(enaPinZ,LOW);  //enable pin pulled low
  digitalWrite(enaPinE0,LOW); //enable pin pulled low
  digitalWrite(enaPinE1,LOW); //enable pin pulled low
}

void remove_A4988_style(){

  //set direction pins 
  digitalWrite(directPinX,LOW);  //set rotation direction
  digitalWrite(directPinY,LOW);  //set rotation direction
  digitalWrite(directPinZ,LOW);  //set rotation direction
  digitalWrite(directPinE0,LOW); //set rotation direction
  digitalWrite(directPinE1,LOW); //set rotation direction

    //reset set enable pins
  digitalWrite(enaPinX,LOW);  //enable pin pulled low
  digitalWrite(enaPinY,LOW);  //enable pin pulled low
  digitalWrite(enaPinZ,LOW);  //enable pin pulled low
  digitalWrite(enaPinE0,LOW); //enable pin pulled low
  digitalWrite(enaPinE1,LOW); //enable pin pulled low
  
  // reset stepper driver pins
  pinMode(enaPinX,INPUT);  // enable pin
  pinMode(enaPinY,INPUT);  // enable pin
  pinMode(enaPinZ,INPUT);  // enable pin
  pinMode(enaPinE0,INPUT); // enable pin
  pinMode(enaPinE1,INPUT); // enable pin
  
  pinMode(stepPinX,INPUT);  // step pin
  pinMode(stepPinY,INPUT);  // step pin
  pinMode(stepPinZ,INPUT);  // step pin
  pinMode(stepPinE0,INPUT); // step pin
  pinMode(stepPinE1,INPUT); // step pin
    
  pinMode(directPinX,INPUT);  // direct pin
  pinMode(directPinY,INPUT);  // direct pin
  pinMode(directPinZ,INPUT);  // direct pin
  pinMode(directPinE0,INPUT); // direct pin
  pinMode(directPinE1,INPUT); // direct pin
}


//*****************************************************//
//****                                             ****//
//****      MAIN LOOP                              ****//
//****                                             ****//
//*****************************************************//

void loop(){
  read_serial();
  OneStepAllPins();
  AnalogRead_and_SerialOut_example();  // reads and displays an analog value
  OneStepAllPins();
  Endstops(); //reads and shows endstop pin status
  OneStepAllPins();
  blinkN(); //blinks a mosfet three times, then moves to the next mosfet
  OneStepAllPins();
  Blink(PS_ON_pin); //blinks an LED (with suitable resistor), if LED (and resistor) is set between a + and and PS-On pin
  OneStepAllPins();
  SetDirection(); // switches direction of steppers every 1 second
  OneStepAllPins(); //moves the steppers
  if(do_test_i2c_eeprom) test_I2C_EEPROM_Main();
  Update_eeprom_info_on_display(); //update puTTY screen
  #ifdef Direct_LCD
    if(do_LCD_update) update_LCD();
    LCD_count(); // update the number counter on the text LCD
  #endif
  OneStepAllPins();
  #ifdef I2C_LCD
    I2C_LCD_count(); // update the number counter on the text LCD
  #endif
  OneStepAllPins();
}


//*****************************************************//
//****                                             ****//
//****     Serial commands                         ****//
//****                                             ****//
//*****************************************************//
void read_serial(){
  if(Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
  
    switch(incomingByte) {
      case 'P':
      case 'p':
        Use_puTTY = true;
        putty_ready();
        break;
      case 'M':
      case 'm':
        Use_puTTY = false;
        break;
      case 'T':
      case 't':
        Use_TMC = true;
        //remove_A4988_style();
        TMC_ready();
        Serial.println( F("TMC2130-style     ") );
        break;
      case 'A':
      case 'a':
        Use_TMC = false;
        set_A4988_style();
        Serial.println( F("A4988-style     ") );
        break;
      case 'E':
      case 'e':
        do_test_i2c_eeprom = true;
        break;
      case 'C':
      case 'c':
        list_serial_commands();
        break;
    }
  }
}

void list_serial_commands(){
  Serial.println( F("type commands at any time:") );
  Serial.println( F("P = use puTTY (default)") );
  Serial.println( F("M = use Arduino Monitor") );
  Serial.println( F("T = test TMC2130 stepper drivers") );
  Serial.println( F("A = test A4988 stepper drivers (default)") );
  Serial.println( F("E = test i2C EEPROM") );
  Serial.println( F("C = show this list of commands") );
}

//*****************************************************//
//****                                             ****//
//****      TMC2130 stuff                          ****//
//****                                             ****//
//*****************************************************//
void TMC_ready(){
  SPI.begin();
  pinMode(MISO, INPUT_PULLUP); // MISO is recognized. I wonder where it is declared 
  
  driverX.begin();       // Initiate pins and registeries
  driverY.begin();       // Initiate pins and registeries
  driverZ.begin();       // Initiate pins and registeries
  driverE0.begin();       // Initiate pins and registeries
  driverE1.begin();       // Initiate pins and registeries
  
  driverX.rms_current(300);  // Set stepper current to 300mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driverY.rms_current(300);
  driverZ.rms_current(300);
  driverE0.rms_current(300);
  driverE1.rms_current(300);
  
  driverX.stealthChop(1);  // Enable extremely quiet stepping
  driverY.stealthChop(1);
  driverZ.stealthChop(1);
  driverE0.stealthChop(1);
  driverE1.stealthChop(1);

  driverX.microsteps(16);  // Because of program overhead, this is a better setting for this test sketch,
  driverY.microsteps(16);  // this does however cause the stepper to make more sound.
  driverZ.microsteps(16);  // But do not worry, in the real firmware, this is handled much better.
  driverE0.microsteps(16); // This is just a test sketch after all.
  driverE1.microsteps(16); // Otherwise try out the test sketch 'TMC2130-simple-all.ino'

  //set enable pins to: Enable
  digitalWrite(enaPinX,LOW);  //enable pin pulled low
  digitalWrite(enaPinY,LOW);  //enable pin pulled low
  digitalWrite(enaPinZ,LOW);  //enable pin pulled low
  digitalWrite(enaPinE0,LOW); //enable pin pulled low
  digitalWrite(enaPinE1,LOW); //enable pin pulled low
}
  
//*****************************************************//
//****                                             ****//
//****      eeprom stuff                           ****//
//****                                             ****//
//*****************************************************//

void test_i2c_eeprom(){
  int d = 0;
  int b = 0;
  int i = 0;
  
  Serial.println( F("Testing EEPROM") );
  show_last_eeprom();
  Wire.begin();

  //check to see if this eeprom test has been done before
  d = readData(1);
  if (d > 0) b = d+1;
 
  //write data out
  Serial.println("Writing data.");
  for ( i = 0; i < 10; i++ )
  {
    writeData(i,i+b);
  }
  Serial.println("Done writing");
  //read data back
  Serial.println("Reading data.");
  for ( i = 0; i < 10; i++ )
  {
    Serial.print(i);
    Serial.print(" : ");
    d = readData(i);
    if(i == 0) eeprom_data_1 = d; //remember this first number
    Serial.println(d, DEC);
  }
  Serial.println( F("EEPROM Test Completed"));
  Serial.println();
}
 
// writes a byte of data in i2c memory location eaddress
void writeData(unsigned int eaddress, byte data) 
{
  Wire.beginTransmission(eeprom_address);
  // set the pointer position
  Wire.write((int)(eaddress >> 8));
  Wire.write((int)(eaddress & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}
 
// reads a byte of data from i2c memory location eaddress
byte readData(unsigned int eaddress) 
{
  byte result;
  Wire.beginTransmission(eeprom_address);
  // set the pointer position
  Wire.write((int)(eaddress >> 8));
  Wire.write((int)(eaddress & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(eeprom_address,1); // get the byte of data
  result = Wire.read();
  return result;
}

void Update_eeprom_info_on_display(){
  if(Use_puTTY) putty_position(12, 0); 
  show_last_eeprom();
}

void show_last_eeprom(){
  if(eeprom_data_1 > 0) {
    Serial.print("Last eeprom data was: ");
    Serial.println(eeprom_data_1, DEC);
  }
}

void test_I2C_EEPROM_Main(){
  do_test_i2c_eeprom = false;
  if(Use_puTTY) {
      putty_cls();
      putty_home();
    }
    test_i2c_eeprom();
    Serial.println( F("Waiting 5 seconds") );
    delay(5000);
    if(Use_puTTY) {
      putty_cls();
      putty_home();
    }
}

                
//*****************************************************//
//****                                             ****//
//****     VT100 / puTTY screen commands           ****//
//****                                             ****//
//*****************************************************//

void putty_home(){
  Serial.print("\x1B[H");     // cursor to home command
}


void putty_cls(){
  Serial.print("\x1B[2J" );   // clear screen command
}

void putty_position(int Line, int Column ){
  Serial.print("\x1B[");   // ESC command
  Serial.print(Line);
  Serial.print(";");
  Serial.print(Column);
  Serial.print("H");  
}

void putty_wait_position(){
  putty_position(15, 0);
}

void putty_ready(){
    putty_cls();
    putty_home();  
}

 
//*****************************************************//
//****                                             ****//
//****      Analog Temperature pins                ****//
//****                                             ****//
//*****************************************************//

void AnalogRead_and_SerialOut_example() {
  static unsigned long analog_delay = 500;      // milli seconds to wait between analog readings
  static unsigned long analog_millis = 0;
  static int sensorValue_1 = 0;        // value read from the pot
  static int sensorValue_2 = 0;        // value read from the pot
  static int sensorValue_3 = 0;        // value read from the pot
  
  //static int outputValue = 0;        // value output to the PWM (analog out)

  if (millis() - analog_millis > analog_delay )  // if its time to make a new reading
  { 
    // read the analog in value:
    sensorValue_1 = analogRead(analogInPin_1);  
    sensorValue_2 = analogRead(analogInPin_2);
    sensorValue_3 = analogRead(analogInPin_3); 
             
    // map it to the range of the analog out:
    //outputValue = map(sensorValue, 0, 1023, 0, 255);  
    // change the analog out value:
    //analogWrite(analogOutPin, outputValue);           

    if(Use_puTTY) putty_position(3, 0);	
	
	Serial.println( F("Showing analog"));
  
    // print the results to the serial monitor:
    Serial.print( F("sensor1 = ") );                       
    Serial.print(sensorValue_1);   
	Serial.println( F("    ") );
    Serial.print( F("sensor2 = ") );                       
    Serial.print(sensorValue_2); 
	Serial.println( F("    ") );
    Serial.print( F("sensor3 = ") );                       
    Serial.print(sensorValue_3);
	Serial.println( F("    ") );
    Serial.println();    
    //Serial.print( F("\t output = ") );      
    //Serial.println(outputValue);   

    //ready to wait for next read and show
    analog_millis = millis();
    do_home = true;
  }
}

//*****************************************************//
//****                                             ****//
//****       Endstops and Z-probe                  ****//
//****                                             ****//
//*****************************************************//

void Endstops() { 
  static unsigned long endstop_millis = 0;
  static unsigned long endstop_interval = 100;
  if ( millis() - endstop_millis >  endstop_interval )  { // if its time to show status
	if(Use_puTTY) putty_position(8, 0);
    Serial.println( F("Endstops") );
    ShowEndstopsHL();  //show logic level
    ShowEndstops10();  //show on/off style
    Serial.println();
	
	if(Use_puTTY) putty_wait_position();

    //ready to wait for next show
    endstop_millis = millis();
    do_home = true;
  }
  EndstopsDebugLED(); 
}

void EndstopsDebugLED() {     
  bool EndsDebugLedOn = 0;
  if( !digitalRead(X_MIN_PIN) ) EndsDebugLedOn = 1;
  if( !digitalRead(X_MAX_PIN) ) EndsDebugLedOn = 1;
  if( !digitalRead(Y_MIN_PIN) ) EndsDebugLedOn = 1;
  if( !digitalRead(Y_MAX_PIN) ) EndsDebugLedOn = 1;
  if( !digitalRead(Z_MIN_PIN) ) EndsDebugLedOn = 1;
  if( !digitalRead(Z_MAX_PIN) ) EndsDebugLedOn = 1;
  if( !digitalRead(Z_PROBE_PIN) ) EndsDebugLedOn = 1;

  if( !PauseEndStopBlink ) {
    if( (EndsDebugLedOn == 1)  ) { //endstop LED indication is paused during the three blinks of DebugLED
      digitalWrite(DebugLed_pin, HIGH);   // set the LED on
    }else{
      digitalWrite(DebugLed_pin, LOW);   // set the LED ooff
    }
  }
}

void ShowEndstopsHL() {     
  if( digitalRead(X_MIN_PIN) ) Serial.print("H"); else Serial.print("L");
  if( digitalRead(X_MAX_PIN) ) Serial.print("H"); else Serial.print("L");
  Serial.print(" ");
  if( digitalRead(Y_MIN_PIN) ) Serial.print("H"); else Serial.print("L");
  if( digitalRead(Y_MAX_PIN) ) Serial.print("H"); else Serial.print("L");
  Serial.print(" ");
  if( digitalRead(Z_MIN_PIN) ) Serial.print("H"); else Serial.print("L");
  if( digitalRead(Z_MAX_PIN) ) Serial.print("H"); else Serial.print("L");
  Serial.print("-");
  if( digitalRead(Z_PROBE_PIN) ) Serial.println("H"); else Serial.println("L");
  do_home = true;
}

void ShowEndstops10() {     
  if( digitalRead(X_MIN_PIN) ) Serial.print("0"); else Serial.print("1");
  if( digitalRead(X_MAX_PIN) ) Serial.print("0"); else Serial.print("1");
  Serial.print(" ");
  if( digitalRead(Y_MIN_PIN) ) Serial.print("0"); else Serial.print("1");
  if( digitalRead(Y_MAX_PIN) ) Serial.print("0"); else Serial.print("1");
  Serial.print(" ");
  if( digitalRead(Z_MIN_PIN) ) Serial.print("0"); else Serial.print("1");
  if( digitalRead(Z_MAX_PIN) ) Serial.print("0"); else Serial.print("1");
  Serial.print("-");
  if( digitalRead(Z_PROBE_PIN) ) Serial.println("0"); else Serial.println("1");
  do_home = true;
}

//*****************************************************//
//****                                             ****//
//****       Blinky stuff                          ****//
//****                                             ****//
//*****************************************************//

void Blink(int blink_pin) 
{ 
  static unsigned long blink_millis = 0;
  static unsigned long blink_interval = 0;
  static boolean do_on = true;
  static int on_delay = 100;    //change this value to adjust the number of MilliSeconds the LED is ON
  static int off_delay = 100;   //change this value to adjust the number of MilliSeconds the LED is OFF
  
  if ( millis() - blink_millis >  blink_interval )  { // if its time to change the blink
    if (do_on) { //use a flag to determine wether to turn on or off the Blink LED
      digitalWrite(blink_pin, HIGH);   // set the LED on, if okay to use power for it
      blink_millis = millis();
      blink_interval = on_delay; // wait for a second
      do_on = false;
    }else{
      digitalWrite(blink_pin, LOW);    // set the LED off
      // set the time to do next blink 
      blink_millis = millis();
      blink_interval = off_delay;  // wait for a second
      do_on = true;
    } 
  }
}


void blinkN(byte bl){
  _blinks = bl;
}

void blinkN(byte major, byte minor){
  _blinks = major;
  _blink_minor = minor; 
}


void blinkN() {
  static unsigned long blinkN_millis = 0;
  static unsigned long blinkN_interval = 0;
  static boolean do_N_on = true;
  static int idx = 0;
  if(_blinks > 0) { // still do a blink session 
    if ( millis() - blinkN_millis >  blinkN_interval )  { // 
      if(do_N_on) { //use a flag to determine wether to turn on or off the Blink LED
        digitalWrite(blinkN_pin, HIGH);   // set the LED on, if okay to use power for it
        blinkN_millis = millis();
        blinkN_interval = blinkN_on_delay; // wait for a second
        do_N_on = false;
      }else{
        digitalWrite(blinkN_pin, LOW);    // set the LED off
        // set the time to do next blink 
        blinkN_millis = millis();
        blinkN_interval = blinkN_off_delay;  // wait for a second
        do_N_on = true;
        _blinks--;
        if(_blinks == 0) {
          blinkN_interval = blinkN_long_off_delay; // a longer pause is necessary before doing next blink session
          //next blink session will be; 
          if (_blink_minor) {
            _blinks = _blink_minor;
            _blink_minor = 0; 
          }
        }
      } 
    }
  }else{ // no more blink sessions
    // do next blink  
    switch (idx){
      case 0:
        blinkN_pin = 9; 
        _blinks = 3;
        PauseEndStopBlink = false;
        idx++; 
        break;
       case 1:
        blinkN_pin = 10;
        _blinks = 3;
        idx++; 
        #ifdef RAMPS144
          idx++; // skip blinking D11
        #endif
        break;
       case 2:
        blinkN_pin = 11;
        _blinks = 3; 
        idx++; 
        break;
       case 3:
        blinkN_pin = 8;
        _blinks = 3;
        idx++; 
        break;
       case 4:
        blinkN_pin = 12;
        _blinks = 3;
        idx++; 
        break;
       case 5:
        blinkN_pin = 13;
        _blinks = 3;
        PauseEndStopBlink = true;
        #ifdef REV_B  // end the loop of what to blink
          idx = 0; 
        #else
          #ifdef REV_C
            idx++; //continue the loop of what to blink
          #else // ramps 1.4.4
            idx = 8; // also blink D7
          #endif
        #endif
        break;
       case 6: //only REV_C
        blinkN_pin = 6;
        _blinks = 3;
        idx++; 
        break;
       case 7: //only REV_C
        blinkN_pin = 7;
        _blinks = 3;
        idx = 0; 
        break;
        case 8: //only ramps 1.4.4
        blinkN_pin = 7;
        _blinks = 3;
        idx = 0; 
        break;
    }
	if(Use_puTTY) putty_position(1, 0);
    Serial.print( F("blinking pin ") );
    Serial.print(blinkN_pin );
	Serial.print( F("        ") );
	if(Use_puTTY) putty_wait_position();
  }
}

//*****************************************************//
//****                                             ****//
//****       Stepper driver stuff                  ****//
//****                                             ****//
//*****************************************************//
void SetDirection(){
  static unsigned long direction_millis = 0;
  static unsigned long direction_interval = 2000;  // move one directon for 2000 milli seconds = 2 seconds
  static boolean step_direction = true;
  if ( millis() - direction_millis >  direction_interval )  {
    if(step_direction) {
      if(Use_TMC){
        driverX.shaft_dir(0);
        driverY.shaft_dir(0);
        driverZ.shaft_dir(0);
        driverE0.shaft_dir(0);
        driverE1.shaft_dir(0);
      }else{
        digitalWrite(directPinX,HIGH);  //set rotation direction
        digitalWrite(directPinY,HIGH);  //set rotation direction
        digitalWrite(directPinZ,HIGH);  //set rotation direction
        digitalWrite(directPinE0,HIGH); //set rotation direction
        digitalWrite(directPinE1,HIGH); //set rotation direction
      }
      step_direction = !step_direction; // ready to reverse direction
    }else{
      if(Use_TMC){
        driverX.shaft_dir(1);
        driverY.shaft_dir(1);
        driverZ.shaft_dir(1);
        driverE0.shaft_dir(1);
        driverE1.shaft_dir(1);
      }else{
        digitalWrite(directPinX,LOW);  //set rotation direction
        digitalWrite(directPinY,LOW);  //set rotation direction
        digitalWrite(directPinZ,LOW);  //set rotation direction
        digitalWrite(directPinE0,LOW); //set rotation direction
        digitalWrite(directPinE1,LOW); //set rotation direction
      }
      step_direction = !step_direction; // ready to reverse direction
    } // if step_direction
    direction_millis = millis();
  } // if millis
} // StepAllPins


void OneStepAllPins() {
  static int idxS = 0;
  static unsigned long pause_micros = 0;
  static unsigned long pause_interval = 0;
  switch(idxS) {
    case 0:
      digitalWrite(stepPinX,HIGH);
      digitalWrite(stepPinY,HIGH);
      digitalWrite(stepPinZ,HIGH);
      digitalWrite(stepPinE0,HIGH);
      digitalWrite(stepPinE1,HIGH);
      pause_micros = micros();
      if (Use_TMC) {
        idxS = 2;
      }else{
        pause_interval = 10; // pause 20 micro seconds in next step
        idxS++;
      }
      break;
    case 1:
      if ( micros() - pause_micros >  pause_interval )  { //pause min. 4 micro seconds, however not really needed because program overhead is already >4uS
        idxS++;
      }
      break;
    case 2:
      digitalWrite(stepPinX,LOW);
      digitalWrite(stepPinY,LOW);
      digitalWrite(stepPinZ,LOW);
      digitalWrite(stepPinE0,LOW);
      digitalWrite(stepPinE1,LOW);
      if (Use_TMC) {
        idxS = 0;
      }else{
        pause_micros = micros();
        pause_interval = 100; // 1000 for no micro step
        idxS++;
      }
    case 3:
     if ( micros() - pause_micros >  pause_interval )  { //pause min. 4 micro seconds, however not really needed because program overhead is already >4uS
        idxS = 0;
      }
      break;
      default:
        idxS = 0;
      break;
  } //switch
} // OneStepAllPins


//*****************************************************//
//****                                             ****//
//****       Direct RRD LCD  (20x4 text)           ****//
//****                                             ****//
//*****************************************************//
#ifdef Direct_LCD
  void setup_DirectLCD() {
    // set up the LCD's number of columns and rows: 
    lcd.begin(20, 4);
    // Print a message to the LCD.
    lcd.print( F("hello, world!") );
  }
  
  void update_LCD(){
    do_LCD_update = false;
    // set the cursor to column 0, line 0 - (0,0) is top left - (0,1) is buttom left (on 2-line display)
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 0);
    lcd.print( F("hello, world!") );
  }
  
  void LCD_count() {  
    // set the cursor to column 0, line 1 - (0,0) is top left - (0,1) is buttom left (on 2-line display)
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(1, 1);
    // print the number of seconds since reset:
    lcd.print(millis()/1000);
  }  
#endif

//*****************************************************//
//****                                             ****//
//****       I2C RRD LCD  (20x4 text)              ****//
//****       via MCP23017 I/O expander             ****//
//****                                             ****//
//*****************************************************//
#ifdef I2C_LCD
  void setup_I2C_LCD() {
    // set the LCD type
    // lcd.setMCPType(LTI_TYPE_MCP23008); 
    i2c_lcd.setMCPType(LTI_TYPE_MCP23017); 
    // set up the LCD's number of rows and columns:
    i2c_lcd.begin(20, 4);
    // Print a message to the LCD.
    i2c_lcd.print("hello, world!");
  }
  
  void I2C_LCD_count() {
   // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    i2c_lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    i2c_lcd.print(millis()/1000);
  }
#endif
