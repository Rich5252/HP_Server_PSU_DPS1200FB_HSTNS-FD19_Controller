//--------------------------------------------------------------------------
//Derived from: https://github.com/raplin/DPS-1200FB
// Lots of other useful info here: https://github.com/slundell/dps_charger
//--------------------------------------------------------------------------

//===============================================================================
//Features PSU power up/down depending on USB power (ie controlled from PC).
//Monitors current and trips the PSU if it exceeds a selectable current.
//  Current trip level is selectable via the rotary control.
//  Reset is via a long push on rotary control switch.
//Also has a 4 digit display with selectable status register displays selected
//  via the push switch on the rotary control.
//Low update rate USB serial output of all PSU registers (9600 baud).
//Developed for HP DPS-1200FB (HSTNS-PD19). However should work with many others.
//73 - G4AHN
//================================================================================

//SoftI2C-master lib MUST BE THIS ONE:
//https://github.com/yasir-shahzad/SoftI2C
//

#include <SoftI2C.h>

//initialise I2C
// Nano pins are A4 - sda and A5 - scl
// NOTE PSU pin 30 - Gnd, 31 - SDA, 32 - SCL
// Also requires pullups enabled on Nano (external pullups therefore not required)

SoftI2C I2C = SoftI2C(A4, A5, true);            //sda, scl, pullup

//some useful PSU data registers
//more options listed at end of sketch
struct RegDef {
        int    reg;       //register number
        float  scale;     //scale factor from 16 bit value
        int    nDec;      //number of dec places to display
        String strName;   //description
        String strUnits;  //units
};

RegDef RegList[] = {                                          //index
        {0x07, 254.5 * 12 / 14.3,1, "OUTPUT_VOLTAGE","V"},    //0
        {0x08, 128.0, 1, "AMPS_OUT","A"},                     //1
        {0x04, 32.0, 0, "INPUT_VOLTAGE","V"},                 //2
        {0x05, 128.0, 1, "AMPS_IN","A"},                      //3
        {0x06, 2.0, 0, "WATTS_IN", "W"},                      //4
        {0x0d, 32.0, 0, "TEMP1_INTAKE","F"},                  //5
        {0x0e, 32.0, 0, "TEMP2_INTERNAL","F"},                //6
        {0x0f, 4, 0, "FAN_SPEED_RPM", "r"}                    //7
      };
int RegListSize = 8;

#include <EEPROM.h>
#include <TM1637.h>
#include <TM16xxDisplay.h>
//other pin allocations
#define PSUpowerOnPin A3          //drive PSU power on input
#define USBPowerDetectPin 2       //connected to USB power supply, used to switch on PSU
#define ToggleSwitchPin 3         //aux power off switch, "on" && USB power turns on PSU
#define DispCLKpin 4              //4 digit local display
#define DispDIOpin 5              // ditto
#define RotBtnPin 8               // rotary switch push button
                                  //    short push changes displayed register
                                  //    long push resets current trip

TM1637 DispModule(DispDIOpin, DispCLKpin);
TM16xxDisplay Disp(&DispModule, 4);    // TM16xx object, 4 digits

#define ItripEEaddr 0
int ItripAmps = 20;     //Current trip value, Amps * 2, ie half amp resolution
bool ITRIPPED = false;

#define DisplayRegIndexEEaddr 1
#define DisplayUpdateMillis  1000
int DisplayRegIndex = 0;


//Rotary encoder
#include <RotaryEncoder.h>
#define Pin_Rot1 7
#define Pin_Rot2 6

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

//interupt routine for rotary encoder
void checkPosition()
{
  encoder->tick();          // check the state for each input change detected
}

//============  SETUP  ==========================
void setup()
{
  //recall current trip and init if its never been written
  if (EEPROM.read(ItripEEaddr) > 100) {
    EEPROM.write(ItripEEaddr, 10);
  }
  ItripAmps = EEPROM.read(ItripEEaddr);

  //ditto for the status register to display
  if (EEPROM.read(DisplayRegIndexEEaddr) > RegListSize) {
    EEPROM.write(DisplayRegIndexEEaddr, 0);
  }
  DisplayRegIndex = EEPROM.read(DisplayRegIndexEEaddr);
  
  Serial.begin(9600);
  Serial.println("\nHP-DPS1200FB_Controller");

  // init I2C interface
  I2C.begin();

  DispModule.clearDisplay();
  DispModule.setupDisplay(true, 2);  // set the brightness (0 to 7, 0 = 0ff) (although only 0-4 work)
  Disp.println("INIT");

  pinMode(ToggleSwitchPin,INPUT_PULLUP);
  pinMode(USBPowerDetectPin, INPUT);
  pinMode(PSUpowerOnPin, OUTPUT);
  pinMode(RotBtnPin, INPUT_PULLUP);
  pinMode(Pin_Rot1, INPUT_PULLUP);
  pinMode(Pin_Rot2, INPUT_PULLUP);

  //rotary control setup
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(Pin_Rot1, Pin_Rot2, RotaryEncoder::LatchMode::TWO03);
  
  // register interrupt routine for rotary control
  attachInterrupt(digitalPinToInterrupt(Pin_Rot1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_Rot2), checkPosition, CHANGE);  
}


//==============  MAIN LOOP  =====================================
void loop()
{
  //read PSU current every loop for max protection
  //Trip switches off PSU until push button gets a long push then released.
  float Current = readPSU(RegList[1].reg, RegList[1].scale);
  if (!ITRIPPED) {ITRIPPED = (Current >= ItripAmps / 2.0); }
  
  //power on/off
  digitalWrite(PSUpowerOnPin, !ITRIPPED && digitalRead(USBPowerDetectPin) && digitalRead(ToggleSwitchPin));  

  //start looking for next Display value - update at **below
  String str = "TRIP";
  if (!ITRIPPED)
  {
    //This is locally selected register
    str = strReadPSUstatusRegsiter();       //returns "" if no update this time
  }
  
  //rotary control change?
  static int pos = 0;  
  static long StartTripDisp = 0;
  encoder->tick();                // call tick() to check the state.  
  int newPos = encoder->getPosition();
  
  //If changed then update trip current and display this for 500ms 
  if (pos != newPos) {
    ItripAmps = ItripAmps + newPos - pos;     //change trip current (half amp resolution)
    pos = newPos;
    StartTripDisp = millis();
  }
  //trip display timer
  if (StartTripDisp > 0) {
    long TripDeltaT = millis() - StartTripDisp;    
    if (TripDeltaT < 500) {    
      //display new Trip current setting for 500ms
      str = GetStr(ItripAmps / 2.0, 1) + "t";      
    }
    else {
      EEPROM.update(ItripEEaddr, ItripAmps);          //doing this here reduces write cycles on EEPROM
      StartTripDisp = 0;      //end of trip displ
    }
  }

  //Display update **
  static String strLastDisp = "";  
  if (str != "" && str != strLastDisp) {
    Disp.println(str);
    strLastDisp = str; 
  }

  //Detect short or long push button press
  if (ShortBtn() && !ITRIPPED) {
    // cycle the register that is displayed
    // if display index == size then diplay all in rotation
    if (++DisplayRegIndex > RegListSize) { DisplayRegIndex = 0; }
    EEPROM.update(DisplayRegIndexEEaddr, DisplayRegIndex);
  }
  if (LongBtn()) {
    //reset a trip
    ITRIPPED = false;
  }
  
}  // =================  END MAIN LOOP  ============================


//============  READ PSU REGISTER (i2c)  ============================
// basic function to read a single 16 bit register from PSU
float readPSU(uint8_t reg, float scale)
{
  // i2c address is made up of base and slot value. Slot is 3 bits on PSU pins 27 (lsb). 28 and 29 (msb
  // if all left open circuit slot is 7
  uint8_t slot = 7;
  uint8_t i2cAddress = 0x58 + slot;

  //calculate essential checksum that is part of data request instruction
  uint8_t cs = (reg << 1) + (i2cAddress << 1);
  uint8_t regCS = ((0xff-cs)+1)&0xff;             

  //Basic sequence is to write out the data request instruction
  //  followed by reading the two byte reply.
  //write [address,register,checksum] and then read two bytes (send address+read bit, read lsb,read msb)
  // These can be done in seperate "transmission" sections
  
  // Send request to PSU
  I2C.beginTransmission(i2cAddress);   //enables Nano as controller and send/checks address OK
  I2C.write(reg << 1);                 //register to read
  I2C.write(regCS);                    //essential checksum otherwise not accepted.
  uint8_t ret = I2C.endTransmission();

  String str = "i2cAddress accepted";
  if (ret) {str = "i2cAddress not acknowledged error";}

  //read of the PSU register
  //first request read from PSU i2cAddress (it was primed with the instruction written above)
  // the "requestFrom" function reads into library's local buffer
  // return value "n" is number of successfully read bytes
  uint8_t n = I2C.requestFrom(i2cAddress, (uint8_t) 2);  //get 2 bytes from i2c device
  int lsb = I2C.read();                                  //then read the pair of bytes from buffer
  int msb = I2C.read();
  int val = msb * 256 + lsb;
  
  return (float) val / scale;
}


//  ===================  GET NEXT VALUE FOR DISPLAY =================
//Get data for local display and stream all to Serial interface
//Update at slow rate to speed up the core loop
String strReadPSUstatusRegsiter()
{
  static long lastDisplay = 0;
  static int  localRegIndex = -1;
  static int  serialRegIndex = -1;

    if ((millis() - lastDisplay) > DisplayUpdateMillis) {
      if (DisplayRegIndex < RegListSize)
      {
         localRegIndex = DisplayRegIndex;   //fixed local reg
      }
      else
      {
        if (++localRegIndex >= RegListSize) { localRegIndex = 0; }  //cycle local reg
      }
      //READ selected register
      float val = readPSU(RegList[localRegIndex].reg, RegList[localRegIndex].scale);
      String strUnits = RegList[localRegIndex].strUnits;
      if (strUnits == "F") {
        strUnits = "C";
        val = (val - 32) * 5 / 9;
      }
      //modify str to fit 4 digit display
      String strDisp = GetStr(val,RegList[localRegIndex].nDec) + strUnits;
      lastDisplay = millis();
      
      //next serial output
      if (++serialRegIndex >= RegListSize) { serialRegIndex = 0; }
      if (serialRegIndex != localRegIndex)
      {
        //need to read different reg for serial
        //READ selected register
        val = readPSU(RegList[serialRegIndex].reg, RegList[serialRegIndex].scale);
        strUnits = RegList[serialRegIndex].strUnits;
        if (strUnits == "F") {
          strUnits = "C";
          val = (val - 32) * 5 / 9;
        }       
      }
      //output serial data
      Serial.println(RegList[serialRegIndex].strName + ": " + String(val) + " " + strUnits);

      return strDisp;      
    }
    return "";    //indicates no local display update
}


// Function to return String to fit 4 char display from float
String GetStr (float x, int nDec) {
  int nChar = 4;                    // 5 chars incl dec point
  char ascArray[20] = "";

  if (nDec == 0) {
    nChar = 3;        // no dec point if zero decimal places
  }
  dtostrf(x, nChar, nDec, ascArray);   // double float to nChar string with nDec decimal places
  
  String str = String(ascArray);
  return str;
}


//===================  Button detection  ====================
#define ShortBtnTime 100
#define LongBtnTime 1000
long FirstDownTime = 0;

//check button input and detect a short press
//Activate function on release
bool ShortBtn(){
  
  if (digitalRead(RotBtnPin)) {
    //button up
    long msDelta = millis() - FirstDownTime;
    if (FirstDownTime != 0 && msDelta >= ShortBtnTime && msDelta < LongBtnTime) {
      FirstDownTime = 0;
      return true;
    }
    return false;
  }
  else {
    //button down
    if (FirstDownTime == 0) { FirstDownTime = millis(); }
    return false;
  }
}

//check button input and detect a long press
//Activate function on release
bool LongBtn(){
  if (digitalRead(RotBtnPin)) {
    //button up
    long msDelta = millis() - FirstDownTime;
    if (FirstDownTime != 0 && msDelta >= LongBtnTime) {
      FirstDownTime = 0;
      return true;
    }
    return false;
  }
  else {
    //button down
    if (FirstDownTime == 0) { FirstDownTime = millis(); }
    return false;
  }
}


//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
// register info from original python source
//      https://github.com/raplin/DPS-1200FB
//---------------------------------------------------------------------------------------
// HEX register value needs a shift left applied when sent to I2C and used to calc checksum
// HEX like "0x50>>1" means use 0x50 shifted right as input to readPSU function
//---------------------------------------------------------------------------------------
/*
    #Readable registers - some of these are slightly guessed - comments welcome if you figure something new out or have a correction.
    REGS={
        #note when looking at PIC disasm table; "lookup_ram_to_read_for_cmd", below numbers are <<1
        #the second arg is the scale factor
        0x01:["FLAGS",0],           #not sure but includes e.g. "power good"
        0x04:["INPUT_VOLTAGE",32.0], #e.g. 120 (volts)
        0x05:["AMPS_IN",128.0],
        0x06:["WATTS_IN",2.0],
        0x07:["OUTPUT_VOLTAGE",254.5], #pretty sure this is right; unclear why scale is /254.5 not /256 but it's wrong - can't see how they'd not be measuring this to high precision
        0x08:["AMPS_OUT",128.0],  #rather inaccurate at low output <10A (reads under) - appears to have internal load for stability so always reads about 1.5 even open circuit
        0x09:["WATTS_OUT",2.0],
        0x0d:["TEMP1_INTAKE_FARENHEIT",32.0],   # this is a guess - may be C or F but F looks more right
        0x0e:["TEMP2_INTERNAL_FARENHEIT",32.0], 
        0x0f:["FAN_SPEED_RPM",1], #total guess at scale but this is def fan speed it may be counting ticks from the fan sensor which seem to be typically 2 ticks/revolution
        0x1a:["?flags",0],                      #unknown (from disassembly)
        0x1b:["?voltage",1],                    #unknown (from disassembly)
        (0x2c>>1):["WATT_SECONDS_IN",-4.0], #this is a special case; uses two consecutive regs to make a 32-bit value (the minus scale factor is a flag for that)
        (0x30>>1):["ON_SECONDS",2.0],
        (0x32>>1):["PEAK_WATTS_IN",2.0],
        (0x34>>1):["MIN_AMPS_IN",128.0],
        (0x36>>1):["PEAK_AMPS_OUT",128.0],
        (0x3A>>1):["COOL_FLAGS1",0],             #unknown (from disassembly)
        (0x3c>>1):["COOL_FLAGS2",0],             #unknown (from disassembly)
        (0x40>>1):["FAN_TARGET_RPM",1],          #unknown (from disassembly)
        (0x44>>1):["VOLTAGE_THRESHOLD_1",1],    #unknown (from disassembly)
        (0x46>>1):["VOLTAGE_THRESHOLD_2",1],    #unknown (from disassembly)
        (0x50>>1):["MAYBE_UNDERVOLTAGE_THRESH",32.0],    #unknown (from disassembly)
        (0x52>>1):["MAYBE_OVERVOLTAGE_THRESH",32.0],    #unknown (from disassembly)
        #reading 0x57 reads internal EEPROM space in CPU (just logging info, e.g. hours in use)
        }
 */
