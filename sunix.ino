//*****************************************************
//SWISSNIXIE - SUNIX (Universal Nixie Clock Control) SOFTWARE
//VERSION: 1.2
//May 5 2017
//www.swissnixie.com
//info@swissnixie.com
//*****************************************************
//This software is open source and can be used or altered
//for free!
//But also this software comes with ABSOLUTE NO WARRANTY!
//Enjoy!
//*****************************************************




//****HARDWARE I/O DEFINITION****

#define thePort PORTD         //define Hardware port where Shift-Registers are connected
#define DATA PD5              //define Dataline
#define OE 6                  //define latch pin
#define CLK PD7               //define clockline
#define ROT_A 2               //Rotatry Encoder Pin1
#define ROT_B 3               //Rotary Encoder Pin2
#define ROT_BTN A3            //Rotary Button
#define BTN A2                //Pushbutton
#define LED 4                 //Pin where the Led is connected
#define BUZZER 8              //Pin where the Buzzer is connected
#define PWR 9                 //Power Supply Enable pin
#define ETXD 99               //External TXD  (uses Dummy-Number)
#define ERXD 12               //External RXD
#define EXTINT 13              //External pin for activation (PIR or etc.)

//*****

//****EEPROM and I2C Bus Addresses

#define RTC_ADDRESS 0x68        //define I2C bus Address for RTC, in case of DS3231 its 0x68
#define STORAGE_ADDRESS 0x50    //define I2C bus Address for EEPROM
#define EEPROM_LED 0x10         //Define EEPROM byte adresses
#define EEPROM_BRIGHT 0x11
#define EEPROM_CYCLE 0x12
#define EEPROM_LZ 0x13
#define EEPROM_AMPM 0x14
#define EEPROM_TSM 0x15
#define EEPROM_TSD 0x16
#define EEPROM_ACTS 0x17
#define EEPROM_ACTT 0x18
#define EEPROM_ACTP 0x19
#define EEPROM_BDRT 0x20
#define EEPROM_DSTE 0x21
#define EEPROM_BM 0x22
#define EEPROM_CMODE 0x23
#define EEPROM_ENTS 0x24
#define EEPROM_TMZ 0x25
#define EEPROM_DEFAULT 0x02

byte oldSecond = 0;
byte oldMinute = 0;
byte oldHour = 0;
byte mstHour = 0;
byte mstMinute = 0;
byte mstSecond = 0;
byte DST_value = 0;
byte TMZ_eeprom = 0;
byte extActTime = 0;
byte leadingZero = 0;
byte isAmPmMode = 0;
byte red = 0;
byte blue = 0;
byte green = 0;
byte cycletime = 0;
byte cyclemode = 1;
byte menu_item = 0;
byte menuItems = 16;
byte bulbMode = 0;

int cycle_speed = 300;
int menu_timeout = 5000;
int eeprom_address = 0;
int encMovement = 0;
int encValueMenu = 0;
int encValueItem = 0;
int encValueMst = 0;
int lastValueMst = 0;
int mstPointer = 0;
int lastMenu = 0;
int lastItem = 0;
int currentVal = 0;
int buzzer_time = 10;
int ms_timeout = 10000;
int ms_ontime = 0;
int ms_tcount = 0;

bool ms_active = false;
bool mstPointerChange = false;
bool ms_on = false;
bool display_time_true = 1;
bool menu_true = 0;
bool menu_item_active = 0;
bool p_once = 0;
bool p_save = 0;
bool menuOnceFlag = 0;
bool buzzer_on = 0;
bool buzzer_set = 0;
bool menu_reset = 0;
bool menu_item_reset = 0;
bool bulbOne = 0;
bool bulbTwo = 0;
bool enTimeSync = 0;
bool exTSvalid = false;
bool needsTS = false;
bool msbOnce = false;
bool extAct = false;
bool extActPos = true;

unsigned long oldTimeStamp = 0;
unsigned long register_output_1 = 1;
unsigned long register_output_2 = 1;
unsigned long last_menu = 0;
unsigned long last_buzzer = 0;
unsigned long ms_last = 0;
unsigned long last_act = 0;
char incoming[15] = { };

int TMZvalues[] = { 0, 0, 60, 120, 180, 210, 240, 270, 300, 330, 345, 360, 390,
                    420, 480, 510, 525, 540, 570, 600, 630, 660, 720, 765, 780, 840, -60,
                    -120, -180, -210, -240, -300, -360, -420, -480, -540, -570, -600, -660,
                    -720
                  };

#include <Wire.h>                           //include library for I2C communication
#include <Adafruit_NeoPixel.h>              //include library for backlight led
#include <ClickEncoder.h>                   //include library for rotary encoder
#include <TimerOne.h>                       //include library timer for rotary encoder
#include <NeoSWSerial.h>                    //include library for software serial, use this library, not the standart
#include <NMEAGPS.h>                        //include library for GPS decoding for all NMEA GPS receivers and "nwts" device
#include <TimeLib.h>                        //include library for internal time
#include <Timezone.h>           //include library for timezone

TimeChangeRule TMEX_D = { "DST", First, Sun, Apr, 2, +60 };
TimeChangeRule TMEX_N = { "NOR", Last, Sun, Oct, 2, +0 };
Timezone tzmex(TMEX_D, TMEX_N);
TimeChangeRule TFDJ_D = { "DST", First, Sun, Nov, 2, +60 };
TimeChangeRule TFDJ_N = { "NOR", Third, Sun, Jan, 2, +0 };
Timezone tzfdj(TFDJ_D, TFDJ_N);
TimeChangeRule TAUS_D = { "DST", First, Sun, Oct, 2, +60 };
TimeChangeRule TAUS_N = { "NOR", First, Sun, Apr, 2, +0 };
Timezone tzaus(TAUS_D, TAUS_N);
TimeChangeRule TPAR_D = { "DST", First, Sun, Oct, 2, +60 };
TimeChangeRule TPAR_N = { "NOR", Fourth, Sun, Mar, 2, +0 };
Timezone tzpar(TPAR_D, TPAR_N);
TimeChangeRule TJOR_D = { "DST", Last, Fri, Mar, 2, +60 };
TimeChangeRule TJOR_N = { "NOR", Last, Fri, Oct, 2, +0 };
Timezone tzjor(TJOR_D, TJOR_N);
TimeChangeRule TEUR_D = { "DST", Last, Sun, Mar, 2, +60 };
TimeChangeRule TEUR_N = { "NOR", Last, Sun, Oct, 2, +0 };
Timezone tzeur(TEUR_D, TEUR_N);
TimeChangeRule TNZL_D = { "DST", Last, Sun, Sep, 2, +60 };
TimeChangeRule TNZL_N = { "NOR", First, Sun, Apr, 2, +0 };
Timezone tznzl(TNZL_D, TNZL_N);
TimeChangeRule TCHL_D = { "DST", Second, Sun, Aug, 2, +60 };
TimeChangeRule TCHL_N = { "NOR", Second, Sun, May, 2, +0 };
Timezone tzchl(TCHL_D, TCHL_N);
TimeChangeRule TUSA_D = { "DST", Second, Sun, Mar, 2, +60 };
TimeChangeRule TUSA_N = { "NOR", First, Sun, Nov, 2, +0 };
Timezone tzusa(TUSA_D, TUSA_N);
TimeChangeRule TBRA_D = { "DST", Third, Sun, Oct, 2, +60 };
TimeChangeRule TBRA_N = { "NOR", Third, Sun, Feb, 2, +0 };
Timezone tzbra(TBRA_D, TBRA_N);

TimeChangeRule *tcr;     //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

extern void RTCsetTime(byte setSeconds, byte setMinutes, byte setHours,
                       byte setWeekday, byte setMonthday, byte setMonth, byte setYear); // forward declaration

//Definitions for Peripherals
ClickEncoder *encoder;
#define       DEBUG_PORT   Serial
NeoSWSerial gps_port(ERXD, ETXD);
NMEAGPS gps;
gps_fix fix;
NeoGPS::clock_t earliestTime;
NeoGPS::clock_t gpsTime;
tmElements_t tm;

int16_t last, value;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, LED, NEO_GRB + NEO_KHZ800); //Adarfuit WS2812 Led initialisation

void setup() {

  //Initialize Communication Ports
  Wire.begin();
  DEBUG_PORT.begin(9600);
  gpsSerialInit();
  gps_port.attachInterrupt(gpsIsr);

  //  Ignore GPS times before this
  earliestTime = NeoGPS::time_t().parse((str_P) F("2016-12-22 00:00:00"));

  //initialize encoder
  encoder = new ClickEncoder(ROT_A, ROT_B, ROT_BTN, 2, HIGH); // avoids linking in heap library
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  last = -1;

  //pinModes:
  pinMode(OE, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PWR, OUTPUT);
  pinMode(ROT_A, INPUT);
  pinMode(ROT_B, INPUT);
  pinMode(ROT_BTN, INPUT);
  pinMode(BTN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(EXTINT, INPUT);
  digitalWrite(PWR, LOW);

  //check if eeprom values are valid, if not, write defaults
  if (readEEPROM(STORAGE_ADDRESS, EEPROM_DEFAULT) != 1)
    eeprom_first();

  //initalize routines
  strip.begin();
  initDST();
  initTMZ();
  setTimeFromRTC();
  initBulbs();
  initColor(1);
  initCycleTime();
  initLeadingZero();
  initAmPm();
  initCycleMode();
  timeService();
  initTimeSync();
  initActInput();

}

void loop() {

  timeService();    //Main function for time,display and cycling handling

  checkEncoder();     //Function for checking the rotary encoder

  setSerialTime();      //Setting time over serial port

  msHandler();        //Function for setting the time manually

  menuHandler();      //Function for the settings menu

  exTimeSyncTest();     //Function for the GPS/NWTS receiver

  buzzing();        //Buzzer service

  if (extAct == true) {
    actService();   //Function for external activation (PIR or similar)
  }

}

void gpsIsr(uint8_t c) {

  //GPS interrupt handle
  gps.handle(c);

}

void gpsDebug() {

  //Function for debugging purposes of the gps receiver

  if ((millis() > 5000) && (gps.statistics.chars < 10)) {
    //DEBUG_PORT.println( F("No GPS detected: check wiring.") );
    while (true)
      ;
  }
}

void gpsSerialInit() {

  //This function loads saved baud-rate and initializes the gps_port.

  byte bdrt = readEEPROM(STORAGE_ADDRESS, EEPROM_BDRT);

  switch (bdrt) {
    case 1:
      gps_port.begin(9600);
      break;
    case 2:
      gps_port.begin(4800);
      break;
    case 3:
      gps_port.begin(19200);
      break;
    case 4:
      gps_port.begin(38400);
      break;
    case 5:
      gps_port.begin(57600);
      break;
    case 6:
      gps_port.begin(115200);
      break;
  }

}

void exTimeSyncTest() {

  //This function checks for a valid time signal from the gps device

  while (gps.available() > 0) {
    fix = gps.read();

    //  Make sure it has been long enough since a reset,
    //      that GPS syncing is enabled, and
    //         that the GPS fix has valid date and time fields.
    if ((millis() > 20000) && (enTimeSync == 1) && fix.valid.date
        && fix.valid.time) {
      gpsTime = fix.dateTime;

      exTSvalid = true;
    } else {
      exTSvalid = false;
    }

  }
}

void exTimeSyncSet() {

  //This function will set the time from gps if the seconds are differing by two from the rtc

  if (gpsTime > earliestTime) {

    int exYear = fix.dateTime.full_year();
    int exMonth = fix.dateTime.month;
    int exDay = fix.dateTime.date;
    int exHour = fix.dateTime.hours;
    int exMinute = fix.dateTime.minutes;
    int exSecond = fix.dateTime.seconds;

    if ((second() > exSecond + 2) || (second() < exSecond - 2)) {
      RTCsetTime(exSecond, exMinute, exHour, 0, exDay, exMonth, exYear);
    }
  }
  needsTS = 0;
}


void actService() {

  //This function checks handles the external activiation signal

  if ((millis() - last_act) > (extActTime * 60000)) {
    digitalWrite(PWR, HIGH);
  }

  if (extActPos == true) {
    if (digitalRead(EXTINT) == HIGH) {
      digitalWrite(PWR, LOW);
      last_act = millis();
    }
  } else {
    if (digitalRead(EXTINT) == LOW) {
      digitalWrite(PWR, LOW);
      last_act = millis();
    }
  }

}

void msHandler() {
  //Handling for manual time seting

  if (ms_on == true && ms_active == false) {
    ms_on = false;
    ms_active = true;
    ms_last = millis();
    doubleBeep();
    display_time_true = false;
    setLedColor(4, 0);
    mstHour = hour();
    mstMinute = minute();
    mstSecond = second();
    formatOutput(mstHour, 255, 255, bulbOne, bulbTwo);
    setOutputs(register_output_1, register_output_2);

  }

  if (ms_active == true && msbOnce == true) {

    RTCsetTime(mstSecond, mstMinute, mstHour, 0, day(), month(), year());
    setTimeFromRTC();
    doubleBeep();

    initColor(0);
    mstPointer = 0;
    display_time_true = true;
    msbOnce = false;
    ms_active = false;
    ms_on = false;

  }

  if (ms_active == true && millis() - ms_last > ms_timeout) {
    ms_active = false;
    ms_on = false;
    display_time_true = true;
    mstPointer = 0;
    initColor(0);
  }

  if (ms_active == true && msbOnce == false) {
    msService();
  }

}

void msEvent() {
  //event counter
  ms_last = millis();
}

void msService() {

  //this function takes care of manual time-stetting

  int mstMov = 0;
  mstMov = encoder->getValue();

  if (mstMov != 0 || mstPointerChange == true) {
    msEvent();
    mstPointerChange = false;

    if (mstPointer == 0) {
      byte newMstHour = mstHour + mstMov;
      if (newMstHour >= 0 && newMstHour <= 23) {
        mstHour = mstHour + mstMov;
        formatOutput(mstHour, 255, 255, bulbOne, bulbTwo);
        setOutputs(register_output_1, register_output_2);
      }
    }

    if (mstPointer == 1) {
      byte newMstMinute = mstMinute + mstMov;
      if (newMstMinute >= 0 && newMstMinute <= 59) {
        mstMinute = mstMinute + mstMov;
        formatOutput(mstHour, mstMinute, 255, bulbOne, bulbTwo);
        setOutputs(register_output_1, register_output_2);
      }

    }
    if (mstPointer == 2) {
      byte newMstSecond = mstSecond + mstMov;
      if (newMstSecond >= 0 && newMstSecond <= 59) {
        mstSecond = mstSecond + mstMov;
        formatOutput(mstHour, mstMinute, mstSecond, bulbOne, bulbTwo);
        setOutputs(register_output_1, register_output_2);
      }

    }
  }

}

void doubleBeep() {
  //puts out a double-beep
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
  delay(250);
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
}

void errorBeep() {
  //puts out a tripple-beep for error
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
  delay(250);
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
  delay(250);
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
}

void menuHandler() {

  //this function handles the menu action

  if ((menu_true == 1 && millis() - last_menu > menu_timeout)
      || menu_reset == 1) {
    menu_true = 0;
    display_time_true = 1;
    menu_item_active = 0;
    initColor(0);
    p_once = 0;
    p_save = 0;
    encValueMenu = 0;
    encValueItem = 0;
    menuOnceFlag = 0;
    menu_reset = 0;
    menu_item_reset = 0;
  }

  if ((menu_true == 1 && menu_item_active == 0) || menu_item_reset == 1) {

    encMovement += encoder->getValue();
    encValueMenu = encMovement;

    if (encValueMenu < 0)
      encValueMenu = 0;
    if (encValueMenu > menuItems)
      encValueMenu = menuItems;

    if ((encValueMenu != lastMenu && encValueMenu > 0)
        || menu_item_reset == 1) {
      menuEvent();
      lastMenu = encValueMenu;
      formatOutput(encValueMenu, 255, 255, 0, 0);
      setOutputs(register_output_1, register_output_2);
    }
    menu_item_reset = 0;
  }

  if (menu_item_active == 1) {

    switch (menu_item) {
      case 2:
        menuItemHandler(16, EEPROM_LED, 1);
        break;
      case 3:
        menuItemHandler(99, EEPROM_BRIGHT, 2);
        break;
      case 4:
        menuItemHandler(3, EEPROM_CYCLE, 0);
        break;
      case 5:
        menuItemHandler(5, EEPROM_CMODE, 0);
        break;
      case 6:
        menuItemHandler(2, EEPROM_LZ, 0);
        break;
      case 7:
        menuItemHandler(4, EEPROM_BM, 0);
        break;
      case 8:
        menuItemHandler(12, EEPROM_DSTE, 0);
        break;
      case 9:
        menuItemHandler(2, EEPROM_AMPM, 0);
        break;
      case 10:
        menuItemHandler(2, EEPROM_ACTS, 0);
        break;
      case 11:
        menuItemHandler(99, EEPROM_ACTT, 0);
        break;
      case 12:
        menuItemHandler(2, EEPROM_ACTP, 0);
        break;
      case 13:
        menuItemHandler(6, EEPROM_BDRT, 0);
        break;
      case 14:
        menuItemHandler(2, EEPROM_ENTS, 0);
        break;
      case 15:
        menuItemHandler(39, EEPROM_TMZ, 0);
        break;
      default:
        break;
    }
  }

}



void initTimeSync() {
  int enTsEval = readEEPROM(STORAGE_ADDRESS, EEPROM_ENTS);
  if (enTsEval == 2) { // <-- semi-colon removed!
    enTimeSync = 1;
  }
}

void initCycleTime() {
  cycletime = readEEPROM(STORAGE_ADDRESS, EEPROM_CYCLE);
}
void initCycleMode() {
  cyclemode = readEEPROM(STORAGE_ADDRESS, EEPROM_CMODE);
}
void initLeadingZero() {
  leadingZero = readEEPROM(STORAGE_ADDRESS, EEPROM_LZ);
}
void initAmPm() {
  isAmPmMode = readEEPROM(STORAGE_ADDRESS, EEPROM_AMPM);
}

void initDST() {
  DST_value = readEEPROM(STORAGE_ADDRESS, EEPROM_DSTE);

}

void initTMZ() {
  TMZ_eeprom = readEEPROM(STORAGE_ADDRESS, EEPROM_TMZ);
}

void initActInput() {
  if (readEEPROM(STORAGE_ADDRESS, EEPROM_ACTS) == 2)
    extAct = true;
  extActTime = readEEPROM(STORAGE_ADDRESS, EEPROM_ACTS);
  if (readEEPROM(STORAGE_ADDRESS, EEPROM_ACTP) == 2)
    extActPos = true;
  last_act = millis();

}

void menuItemHandler(int maxVal, int eeaddr, int type) {

  if (menuOnceFlag == 0) {

    int oldVal = readEEPROM(STORAGE_ADDRESS, eeaddr);
    formatOutput(menu_item, 255, oldVal, 0, 0);
    setOutputs(register_output_1, register_output_2);
    currentVal = oldVal;
    if (type == 1) {
      setLedColor(oldVal, 0);
    }
    menuOnceFlag = 1;
  }
  if (menuOnceFlag == 1 && p_save == 0) {

    int encVal = encoder->getValue();
    int copVal = currentVal + encVal;
    if (copVal != currentVal && copVal >= 1 && copVal <= maxVal) {
      menuEvent();
      currentVal = copVal;
      formatOutput(menu_item, 255, currentVal, 0, 0);
      setOutputs(register_output_1, register_output_2);

      if (type == 1) {
        setLedColor(currentVal, 0);
      }
      if (type == 2) {
        ledBrightness(currentVal);
      }

    }
  }
  if (p_save == 1) {

    if (type == 1) {
      writeEEPROM(STORAGE_ADDRESS, EEPROM_LED, currentVal);
      setLedColor(currentVal, 1);
    }

    if (type == 2) {
      writeEEPROM(STORAGE_ADDRESS, EEPROM_BRIGHT, currentVal);
      ledBrightness(currentVal);
    }

    if (type == 0) {
      writeEEPROM(STORAGE_ADDRESS, eeaddr, currentVal);

    }

    p_save = 0;
    p_once = 0;
    menuOnceFlag = 0;
    menu_reset = 1;
  }

}

void buzzing() {
  if (buzzer_on == 0 && buzzer_set == 1) {
    digitalWrite(BUZZER, HIGH);
    last_buzzer = millis();
    buzzer_on = 1;
    buzzer_set = 0;
  }
  if (buzzer_on == 1 && ((millis() - last_buzzer) > buzzer_time)) {
    digitalWrite(BUZZER, LOW);
    buzzer_on = 0;
  }
}

void checkEncoder() {
  ClickEncoder::Button b = encoder->getButton();
  switch (b) {
    case ClickEncoder::Held:

      if (digitalRead(BTN) == HIGH) {
        if (ms_active == false && ms_on == false) {
          ms_on = true;
        }
      } else {
        if (menu_true == 0 && ms_active == false) {
          menu_true = 1;
          buzzer_set = 1;
          menuEvent();
          display_time_true = 0;
          formatOutput(value, 255, 255, 0, 0);
          setOutputs(register_output_1, register_output_2);
        }
      }

      break;
    case ClickEncoder::DoubleClicked:

      if (menu_true == 1 && menu_item_active == 1 && p_once == 0) {
        buzzer_set = 1;
        p_save = 1;
        p_once = 1;

      }

      if (menu_true == 1 && menu_item_active == 0) {
        menu_item_active = 1;
        buzzer_set = 1;
        menu_item = encValueMenu;
        menuEvent();
        encMovement = 0;
        formatOutput(menu_item, 255, 00, 0, 0);
        setOutputs(register_output_1, register_output_2);
      }

      if (ms_active == true && msbOnce == false) {
        msbOnce = true;
      }

      break;
    case ClickEncoder::Clicked:

      if (mstPointer <= 2) {
        mstPointer++;
        mstPointerChange = true;
      }

      break;

  }
}

uint32_t DST_calculation(unsigned long inputTime) {

  //This function will caluclate if there is a Day-Light-Saving enabled and an ajustment needed.

  uint32_t daylightSavingValue = 0;
  switch (DST_value) {

    case 1:
      daylightSavingValue = inputTime;
      break;
    case 2:
      daylightSavingValue = inputTime + 3600;
      break;
    case 3:
      daylightSavingValue = tzeur.toLocal(inputTime, &tcr);
      break;
    case 4:
      daylightSavingValue = tzusa.toLocal(inputTime, &tcr);
      break;
    case 5:
      daylightSavingValue = tzaus.toLocal(inputTime, &tcr);
      break;
    case 6:
      daylightSavingValue = tznzl.toLocal(inputTime, &tcr);
      break;
    case 7:
      daylightSavingValue = tzbra.toLocal(inputTime, &tcr);
      break;
    case 8:
      daylightSavingValue = tzchl.toLocal(inputTime, &tcr);
      break;
    case 9:
      daylightSavingValue = tzmex.toLocal(inputTime, &tcr);
      break;
    case 10:
      daylightSavingValue = tzfdj.toLocal(inputTime, &tcr);
      break;
    case 11:
      daylightSavingValue = tzpar.toLocal(inputTime, &tcr);
      break;
    case 12:
      daylightSavingValue = tzjor.toLocal(inputTime, &tcr);
  }

  return daylightSavingValue;
}

long TMZ_calculation() {
  long TMZ_offset = TMZvalues[TMZ_eeprom] * 60;
  return TMZ_offset;
}

void initBulbs() {
  int BM = readEEPROM(STORAGE_ADDRESS, EEPROM_BM);
  switch (BM) {
    case 1:
      bulbMode = 0; //Both Neon Bulbs OFF
      bulbOne = 0;
      bulbTwo = 0;
      break;
    case 2:
      bulbMode = 1; //Both Neon Bulbs ON (continous)
      bulbOne = 1;
      bulbTwo = 1;
      break;
    case 3:
      bulbMode = 2; //Neon Bulbs Change each second
      break;
    case 4:
      bulbMode = 3; //both Neon Bulbs blink reversely
      bulbOne = 1;
      bulbTwo = 0;
      break;
    default:
      bulbMode = 0;

  }

}

void bulbHandler() {
  if (bulbMode == 2) {
    if (bulbOne == 1 || bulbTwo == 1) {
      bulbOne = 0;
      bulbTwo = 0;
    } else {
      bulbOne = 1;
      bulbTwo = 1;
    }
  };

  if (bulbMode == 3) {
    byte bulbHelperOne = bulbOne;
    byte bulbHelperTwo = bulbTwo << 1;
    int bulbHelper = bulbHelperOne | bulbHelperTwo;
    switch (bulbHelper) {
      case 1:
        bulbOne = 0;
        bulbTwo = 1;
        break;
      case 2:
        bulbOne = 1;
        bulbTwo = 0;
        break;
    }
  }

}

void checkSerial() {
  int rec = 0;
  char* cmdbuf;
  char c;
  int i;
  while ( DEBUG_PORT.available() && c != '\n') { // buffer up a line
    c = DEBUG_PORT.read();
    cmdbuf[i++] = c;
  }

  i = 0;
  while (cmdbuf[++i] != ' ')
    ; // find first space
  cmdbuf[i] = 0;          // null terminate command
  char* cmd = cmdbuf;     //
  int cmdlen = i;         // length of cmd

  int args[5], a;         // five args max, 'a' is arg counter
  char* s;
  char* argbuf = cmdbuf + cmdlen + 1;
  while ((s = strtok(argbuf, " ")) != NULL && a < 5) {
    argbuf = NULL;
    args[a++] = (byte) strtol(s, NULL, 0); // parse hex or decimal arg
    DEBUG_PORT.println((byte) strtol(s, NULL, 0));
  }
  int argcnt = a;         // number of args read

}

void initColor(bool hasDelay) {
  //Function for initialisiation of LED illumination, from a unkown state. This will read from EEPROM
  int color = readEEPROM(STORAGE_ADDRESS, EEPROM_LED);
  setLedColor(color, hasDelay);
  initBrightness();
}

void initBrightness() {
  int bright = (255 / 100) * readEEPROM(STORAGE_ADDRESS, EEPROM_BRIGHT);
  strip.setBrightness(bright);
  strip.show();
}

void ledBrightness(int theBrightness) {
  initColor(0);
  strip.setBrightness((255 / 100) * theBrightness);
  strip.show();
}

void setLedColor(int preset, bool hasDelay) {

  // This function selects a preset color and handles it over to the led setting function
  // More available colors can be found here:
  //https://forums.adafruit.com/viewtopic.php?f=8&t=80684

  switch (preset) {
    case 1:
      setLed(0, 0, 0, hasDelay); // OFF
      break;
    case 2:
      setLed(0, 255, 0, hasDelay); // Full Green
      break;
    case 3:
      setLed(0, 0, 255, hasDelay); // Full Green
      break;
    case 4:
      setLed(255, 0, 0, hasDelay); // Full Red
      break;
    case 5:
      setLed(255, 255, 0, hasDelay); // Full Yellow
      break;
    case 6:
      setLed(255, 255, 255, hasDelay); // Full White
      break;
    case 7:
      setLed(0, 255, 255, hasDelay); //  Turquoise
      break;
    case 8:
      setLed(51, 0, 255, hasDelay); // Dark Blue
      break;
    case 9:
      setLed(204, 0, 102, hasDelay); // Dark Red
      break;
    case 10:
      setLed(102, 204, 0, hasDelay); // Dark Green
      break;
    case 11:
      setLed(153, 0, 255, hasDelay); // Purple-like
      break;
    case 12:
      setLed(0, 255, 153, hasDelay); // Neon-Green
      break;
    case 13:
      setLed(255, 102, 0, hasDelay); // Orange
      break;
    case 14:
      setLed(255, 153, 0, hasDelay); // Another Orange
      break;
    case 15:
      setLed(51, 255, 255, hasDelay); // Neon Blue
      break;
    case 16:
      setLed(255, 0, 255, hasDelay); // Sort of Red
      break;
    default:
      break;
  }
}

void setLed(int r, int g, int b, bool hasDelay) {
  //Function to set every pixel of the LED illumination. If hasDelay is true, a small delay will be added between each pixel to show a lightning bar effect
  red = r;
  green = g;
  blue = b;
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, r, g, b);
    strip.show();
    if (hasDelay != 0) {
      delay(200);
    }
  }
}

void setOutputs(unsigned long val_one, unsigned long val_two) { //Function to shift out 2 x 32bit fast enough to prevent flicker!

  // ------------WARNING!--------------------
  // This functions operates directly on ports, not via digitalWrite()
  // because digitalWrite() would be to slow, and display would flicker
  // if different pins are used, you maybe hav to change the variable "thePort"
  // to the matching I/O port letter of the controller!

  digitalWrite(OE, LOW); //Disable Outputs to prevent flicker

  //Send first 32-bit variable value

  for (int i = 0; i < 32; i++) {
    thePort &= ~_BV(DATA);  //Data LOW
    if ( bitRead(val_one, i) == 1) {
      thePort |= _BV(DATA);  //Data HIGH
    }
    thePort |= _BV(CLK);  //CLK HIGH
    thePort &= ~_BV(CLK); //CLK LOW
  }

  //Send second 32-bit variable value
  for (int i = 0; i < 32; i++) {
    thePort &= ~_BV(DATA);  //Data LOW
    if ( bitRead(val_two, i) == 1) {
      thePort |= _BV(DATA);  //Data HIGH
    }
    thePort |= _BV(CLK);  //CLK HIGH
    thePort &= ~_BV(CLK); //CLK LOW
  }
  digitalWrite(OE, HIGH); //Enable Outputs

}

void formatOutput(byte IN1, byte IN2, byte IN3, byte LIN1, int LIN2) {

  byte var = DecToBcd(IN1);
  int value_1 = BcdToDec(var >> 4);
  var = var << 4;
  int value_2 = BcdToDec(var >> 4);

  var = DecToBcd(IN2);
  int value_3 = BcdToDec(var >> 4);
  var = var << 4;
  int value_4 = BcdToDec(var >> 4);

  var = DecToBcd(IN3);
  int value_5 = BcdToDec(var >> 4);
  var = var << 4;
  int value_6 = BcdToDec(var >> 4);

  if (value_1 > 0) {
    value_1 = value_1 - 1;
  } else {
    value_1 = 9;
  };

  if (value_2 > 0) {
    value_2 = value_2 - 1;
  } else {
    value_2 = 9;
  };

  if (value_3 > 0) {
    value_3 = value_3 - 1;
  } else {
    value_3 = 9;
  };

  if (value_4 > 0) {
    value_4 = value_4 - 1;
  } else {
    value_4 = 9;
  };

  if (value_5 > 0) {
    value_5 = value_5 - 1;
  } else {
    value_5 = 9;
  };

  if (value_6 > 0) {
    value_6 = value_6 - 1;
  } else {
    value_6 = 9;
  };

  unsigned long T1 = 2147483648UL >> ((unsigned long) value_1);
  unsigned long T2 = 2147483648UL >> ((unsigned long) value_2 + 10);
  unsigned long T3 = 2147483648UL >> ((unsigned long) value_3 + 20);
  unsigned long L1 = 0;
  unsigned long L2 = 0;

  if (LIN1 > 0) {
    L1 = 1;
  }
  if (LIN2 > 0) {
    L2 = 2;
  }

  unsigned long T4 = 2147483648UL >> ((unsigned long) value_4);
  unsigned long T5 = 2147483648UL >> ((unsigned long) value_5 + 10);
  unsigned long T6 = 2147483648UL >> ((unsigned long) value_6 + 20);

  if (IN1 > 99) {
    T1 = 0;
    T2 = 0;
  }
  if (IN2 > 99) {
    T3 = 0;
    T4 = 0;
  }
  if (IN3 > 99) {
    T5 = 0;
    T6 = 0;
  }
  if (display_time_true == 1 && leadingZero == 2 && value_1 == 9) {
    T1 = 0;
  }

  register_output_1 = T1 | T2 | T3 | L1 | L2;
  register_output_2 = T4 | T5 | T6;

}

void timeService() {

  bool updated = false;
  bool needsCycle = false;
  byte AMPM_value = 0;

  if (oldTimeStamp != now()) {
    oldTimeStamp = now();
    updated = 1;

    if (needsTS == true) {

      if (exTSvalid == true) {
        exTimeSyncSet();
        setTimeFromRTC();
      } else {
        setTimeFromRTC();
      }
      needsTS = false;

    }

    if (oldMinute != minute()) {
      needsTS = true;
      byte oldMTT = oldMinute / 10;
      byte currentMTT = minute() / 10;

      if (cycletime == 1 && oldMTT != currentMTT) {
        needsCycle = true;
      }

      if (cycletime == 2) {
        needsCycle = true;
      }
      oldMinute = minute();

    }
    if (oldHour != hour()) {

      if (cycletime == 3) {
        needsCycle = true;
      }

      oldHour = hour();
    }

    if (isAmPmMode == 2) {
      AMPM_value = 12;
    }
  }

  if (needsCycle == true && display_time_true == 1) {
    cycleHandler();
  }

  if (updated == 1 && display_time_true == 1) {

    bulbHandler();

    formatOutput(hour() - AMPM_value, minute(), second(), bulbOne, bulbTwo);
    setOutputs(register_output_1, register_output_2);

  }

}

void cycleHandler() {

  switch (cyclemode) {
    case 1:
      cycle_display_1();
      break;
    case 2:
      cycle_display_2();
      break;
    case 3:
      cycle_display_3();
      break;
    case 4:
      cycle_display_4();
      break;
    case 5:
      cycle_display_5();
      break;
    default:
      cycle_display_1();
      break;
  }

}

void printTime() {
  DEBUG_PORT.print(F("Time is  "));
  if (hour() < 10)
    DEBUG_PORT.print('0');
  DEBUG_PORT.print(hour());
  DEBUG_PORT.print(':');
  if (minute() < 10)
    DEBUG_PORT.print('0');
  DEBUG_PORT.print(minute());
  DEBUG_PORT.print(':');
  if (second() < 10)
    DEBUG_PORT.print('0');
  DEBUG_PORT.println(second());

}

void cycle_display_1() {

  //*******************Simply cycle all digits 3times from 0-9****************
  for (int t = 0; t < 3; t++) {
    for (int i = 0; i < 10; i++) {
      formatOutput(i * 11, i * 11, i * 11, bulbOne, bulbTwo);
      setOutputs(register_output_1, register_output_2);
      delay(30);
    }
  }
}

void cycle_display_2() {

  //*****************Cycle all digits 3times from 0-9 with gaps (off) between digits**************
  for (int t = 0; t < 3; t++) {
    for (int i = 0; i < 10; i++) {
      formatOutput(i * 11, i * 11, i * 11, bulbOne, bulbTwo);
      setOutputs(register_output_1, register_output_2);
      delay(20);
      formatOutput(255, 255, 255, bulbOne, bulbTwo);
      setOutputs(register_output_1, register_output_2);
      delay(50);
    }
  }
}

void cycle_display_3() {

  //*******************Shift digits 0-9 from left to right****************
  for (int t = 0; t < 2; t++) {
    for (int i = 0; i < 10; i++) {

      int cd3_i = 0;

      if (i == 0)
        cd3_i = 9;
      if (i > 0)
        cd3_i = i - 1;

      for (int j = 0; j < 6; j++) {

        unsigned long cd3_oval1 = 0;
        unsigned long cd3_oval2 = 0;
        unsigned long cd3_lamp1 = bulbOne;
        unsigned long cd3_lamp2 = bulbTwo << 1;

        if (j < 3) {
          cd3_oval1 = 2147483648UL
                      >> ((unsigned long) cd3_i + (10 * j));
          cd3_oval2 = 0;

        };
        if (j > 2) {
          cd3_oval2 = 2147483648UL
                      >> ((unsigned long) cd3_i + (10 * (j - 3)));
          cd3_oval1 = 0;
        };

        unsigned long cd3_reg1 = cd3_oval1 | cd3_lamp1 | cd3_lamp2;
        setOutputs(cd3_reg1, cd3_oval2);
        delay(100);
      }
    }
  }
}

void cycle_display_4() {

  for (int t = 0; t < 5; t++) {

    if (t < 3) {
      for (int i = 0; i < 10; i++) {
        formatOutput(i * 11, i * 11, i * 11, bulbOne, bulbTwo);
        setOutputs(register_output_1, register_output_2);
        delay(30);
      }
    }
    if (t == 3) {

      for (int i = 0; i < 10; i++) {
        formatOutput(i * 11, i * 11, second(), bulbOne, bulbTwo);
        setOutputs(register_output_1, register_output_2);
        delay(30);
      }
    }
    if (t == 4) {
      for (int i = 0; i < 10; i++) {
        formatOutput(i * 11, minute(), second(), bulbOne, bulbTwo);
        setOutputs(register_output_1, register_output_2);
        delay(30);
      }
    }

  }

}
;

void cycle_display_5() {

  //************************Pick a random number for each digit for 50 times**********
  for (int i = 0; i < 50; i++) {
    int cval4_1 = random(0, 99);
    int cval4_2 = random(0, 99);
    int cval4_3 = random(0, 99);
    formatOutput(cval4_1, cval4_2, cval4_3, bulbOne, bulbTwo);
    setOutputs(register_output_1, register_output_2);
    delay(50);
  }
}
;

void menuEvent() {

  //if any menu event occurs, the timeout for displaying the menu will be reset.
  last_menu = millis();
}

void setSerialTime() {

  //read incoming
  while (DEBUG_PORT.available() > 0) {
    static int i = 0;
    char c = DEBUG_PORT.read();

    //time will be send in format HHmmssddMMyyw, z.B. 1329002001137

    if ((i == 12) && (c == 'X')) {
      // If the 13th character was an X, it's confirmed that the serial transmission is right.
      char SIhr[3] = { };
      char SImin[3] = { };
      char SIsec[3] = { };
      char SIday[3] = { };
      char SImonth[3] = { };
      char SIyr[3] = { };
      char SIweekday[2] = { };

      SIhr[0] = incoming[0];
      SIhr[1] = incoming[1];
      SImin[0] = incoming[2];
      SImin[1] = incoming[3];
      SIsec[0] = incoming[4];
      SIsec[1] = incoming[5];
      SIday[0] = incoming[6];
      SIday[1] = incoming[7];
      SImonth[0] = incoming[8];
      SImonth[1] = incoming[9];
      SIyr[0] = incoming[10];
      SIyr[1] = incoming[11];
      SIweekday[0] = incoming[12];

      byte Ssecond = atoi(SIsec);     //0-59
      byte Sminute = atoi(SImin);     //0-59
      byte Shour = atoi(SIhr);      //0-23
      byte SweekDay = atoi(SIweekday); //0-6 -> sunday - Saturday
      byte SmonthDay = atoi(SIday);     //1-31
      byte Smonth = atoi(SImonth);   //1-12
      int Syear = atoi(SIyr);      //0-99

      RTCsetTime(Ssecond, Sminute, Shour, SweekDay, SmonthDay, Smonth,
                 Syear);
      printTime();
      //      printTime();        <-- twice ???

      // reset for next command
      i = 0;

    } else if ((c == 10) || (c == 13)) {
      // End of line, reset
      i = 0;

    } else if ((' ' <= c) && (c <= '~')) {

      // Accumulate another byte of the command
      if (i < sizeof(incoming))
        incoming[i++] = c;
    }
  }
}

void RTCsetTime(byte setSeconds, byte setMinutes, byte setHours,
                byte setWeekday, byte setMonthday, byte setMonth, int setYear) // forward declaration
{
  //This function sends a given time to the RTC module and updates the day of the last setting.

  //Send data:
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0x00);
  Wire.write(DecToBcd(setSeconds));
  Wire.write(DecToBcd(setMinutes));
  Wire.write(DecToBcd(setHours));
  Wire.write(DecToBcd(setWeekday));
  Wire.write(DecToBcd(setMonthday));
  Wire.write(DecToBcd(setMonth));
  Wire.write(DecToBcd(setYear - 2000));
  Wire.endTransmission();

  //Update day and month of setting, for day-light saving
  writeEEPROM(STORAGE_ADDRESS, EEPROM_TSM, setMonth);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_TSD, setMonthday);

}

void setTimeFromRTC() {

  //This function reads the current time in UTC saved in the RTC Module and sets the arduino time.

  //collect data
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDRESS, 7);
  byte rtcSeconds = BcdToDec(Wire.read());
  byte rtcMinutes = BcdToDec(Wire.read());
  byte rtcHours = BcdToDec(Wire.read());
  bool dummyDay = BcdToDec(Wire.read());
  byte rtcDay = BcdToDec(Wire.read());
  byte rtcMonth = BcdToDec(Wire.read());
  int rtcYear = BcdToDec(Wire.read());

  //Fill tm elements

  tm.Second = rtcSeconds;
  tm.Hour = rtcHours;
  tm.Minute = rtcMinutes;
  tm.Day = rtcDay;
  tm.Month = rtcMonth;
  tm.Year = (rtcYear + 30);

  //convert rtc time to timestamp
  uint32_t tempTime = makeTime(tm) + TMZ_calculation();
  //calculate adjustment
  uint32_t compiled_time = DST_calculation(tempTime);

  //set arduino time
  setTime(compiled_time);

}

void writeEEPROM(int deviceaddress, int eeaddress, byte data) {

  //This function writes given data to given eeprom adress

  Wire.beginTransmission(deviceaddress);
  Wire.write(eeaddress);
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

int readEEPROM(int deviceaddress, byte eeaddress) {

  // Function to read a value from the external EEPROM chip

  byte rdata = 0xFF;

  Wire.beginTransmission(deviceaddress);
  Wire.write(eeaddress); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 2);

  if (Wire.available()) {
    rdata = Wire.read();
  }
  return rdata;
}

void timerIsr() {
  //Encoder Library neccesarry function
  encoder->service();
}

void eeprom_first() {
  //This function fills the eeprom with default values if the values are not already there

  writeEEPROM(STORAGE_ADDRESS, EEPROM_LED, 2);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_BRIGHT, 99);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_CYCLE, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_LZ, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_AMPM, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_TSM, 0);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_TSD, 0);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_ACTS, 0);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_ACTT, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_ACTP, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_BDRT, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_DSTE, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_BM, 2);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_CMODE, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_ENTS, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_TMZ, 1);
  writeEEPROM(STORAGE_ADDRESS, EEPROM_DEFAULT, 1);

}

// HELPING FEATURES AND FUNCTIONS!
// MAY OR MAY NOT USED IN THE CODE
// NOT NEEDED FOR NORMAL OPERATION

static int BcdToDec(int var) {
  return var - 6 * (var >> 4);
}
;
static int DecToBcd(int var) {
  return (var / 10 * 16) + (var % 10);
}
;

void print_long(uint32_t value) { //prints a 32bit variable with leading zeros, usefull for testing/debuging

  for (byte i = 0; i < 32; i++) {
    DEBUG_PORT.print(bitRead(value, 31 - i));
    if (i < 31)
      continue;
    DEBUG_PORT.println();
  }
}

void print_byte(uint8_t value) { //prints a 8bit variable with leading zeros, usefull for testing/debuging

  for (byte i = 0; i < 8; i++) {
    DEBUG_PORT.print(bitRead(value, 7 - i));
    if (i < 7)
      continue;
    DEBUG_PORT.println();
  }
}


