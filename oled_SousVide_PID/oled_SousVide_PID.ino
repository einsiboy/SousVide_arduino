/*
 * Using PID algorithm to keep a water bath at a constant temperature.
 * Required for Sous Vide, can also be useful for mashing when homebrewing beer.
 * This sketch is based on the tutorial from adafruit: 
 * https://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino
 * 
 * The main differences are that this implementation does not use a shield. 
 * I use a 1.3" 128x64 OLED screen (SPI), and the buttons are currently connected on a breadboard.
*/

//Universal 8bit Graphics Library, http://code.google.com/p/u8glib/
#include "U8glib.h"

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>


// ===================================
// ==== States =======================
//====================================
//enum OperatingState { OFF = 0, SETP, RUN, TUNE_KP, TUNE_KI, TUNE_KD, AUTO};
enum OperatingState {OFF, SETP, RUN, TUNE_KP, TUNE_KI, TUNE_KD, AUTO};
OperatingState opState = OFF;
//OperatingState opState = RUN;

// ===================================
// ==== Sensor =======================
//====================================
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;


// ===================================
// ==== Screen =======================
//====================================
// the oled screen object
U8GLIB_SH1106_128X64 display(4, 5, 6, 7, 3);  // SW SPI Com: SCK = 4, MOSI = 5, CS = 6, A0 (DC) = 7


// ===================================
// ==== PID variables ================
//====================================

//Define Variables we'll be connecting to
double setpoint;
double input;
double output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ===================================
// ==== Autotune variables ===========
//====================================
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&input, &output);


// ===============================
// ==== Variables ================
// ===============================
#define BUTTON_UP 10
#define BUTTON_DOWN 11
#define BUTTON_LEFT 12
#define BUTTON_RIGHT 9
#define BUTTON_SELECT 8

#define BUTTON_SHIFT BUTTON_SELECT

// Output Relay
#define RelayPin 13

unsigned long lastInput = 0; // last button press

// For blinking title text in RUN state
unsigned long textOnTime = millis();
unsigned long textOffTime = millis();
boolean drawRunTitle = true;

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ===============================
// ==== Strings ==================
// ===============================
const char *STR_SOUS_VIDE = "Sous Vide!";

void setup() {
  Serial.begin(9600);
  //display.begin();
  
  setpoint = 65;
  input = 20;

  // BUTTON INIT
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);

  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start

  // temp sensor setup
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  sensors.getAddress(tempSensor, 0);
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);

  loadParameters();

  myPID.SetTunings(Kp,Ki,Kd);

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);
  
  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
  
  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
}

void loadParameters(){
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
  
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


void loop() {
  // as a starting point let's try to do both the updage logic and draw in the picture loop.
   // picture loop
    /*if(isButtonPressed()){
      lastInput = millis();
    }*/
    if(!(opState == OFF)){
      doControl();
    }
    
    display.firstPage();  
    do {
     switch (opState)
     {
     case OFF:
        Off();
        break;
     case SETP:
        tuneSp();
        break;
      case RUN:
        Run();
        break;
     case TUNE_KP:
        tuneKp();
        break;
     case TUNE_KI:
        tuneKi();
        break;
     case TUNE_KD:
        tuneKd();
        break;
     }
    
    drawSousVide();
  } while( display.nextPage() );
}

//void drawTitleString(const char *titleStr){
void drawTitleString(const char* titleStr){
  display.setFont(u8g_font_6x10);
  int fontHeight = display.getFontAscent()-display.getFontDescent(); // used as offset
  int y = 1;
  int x = 0;

  y += fontHeight; //starts drawing from lower left of string
  display.setPrintPos(x, y);
  display.print(titleStr);
}

// ===================================
// ==== Off state ====================
//====================================


void Off()
{
  updateOff();
  drawOff();
}

void updateOff(){
  digitalWrite(RelayPin, LOW);  // make sure it is off
  myPID.SetMode(MANUAL);
  
  if(!digitalRead(BUTTON_RIGHT)){
    sensors.requestTemperatures(); // Start an asynchronous temperature reading
    
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    
    opState = RUN;
    delay(200);
  }
}

void drawOff(){
  drawTitleString("Off"); 
  
  int x, y, fontHeight;
  display.setFont(u8g_font_7x13);
  fontHeight = display.getFontAscent()-display.getFontDescent(); // used as offset

  y = 0.3*display.getHeight() + fontHeight;
  x = 0.075*display.getWidth();
  display.setPrintPos(x, y);
  display.print("Press right");
  y += fontHeight;
  display.setPrintPos(x,y);
  display.print("to start");
}


// ===================================
// ==== Running state ================
//====================================

/*
 * Main running state, should display the status and listen for button presses to change screen
 */
void Run(){
  // update logic, then draw
  updateRun();
  drawRun();
}

void updateRun(){
   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

  if(!digitalRead(BUTTON_SELECT) 
     && !digitalRead(BUTTON_RIGHT) ) { // only allow autotune if close to steady state
      if(abs(input - setpoint) < 1.5){
        startAutoTune();
      } else {
        Serial.println("Can't start autotune, not within 1.0 °C of setpoint");
        delay(200);
      }
  } else if(updateStateChange(OFF, SETP)) {
    delay(200);
    return;
  }

  // Have the title text blink while running.
  if(millis() - textOnTime > 2000 && drawRunTitle){
    drawRunTitle = false;
    textOffTime = millis();
  }
  if(millis() - textOffTime > 300 && !drawRunTitle){
    drawRunTitle = true;
    textOnTime = millis();
  }

  // periodically log to serial port in csv format
  if (millis() - lastLogTime > logInterval)  
  {
    Serial.print(input);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(tuning);
    lastLogTime = millis();
  }
}

void drawRun(){
  if(drawRunTitle){
    drawTitleString("Running");
    if(tuning){
      display.print(" - T");    
    }
  }
  
  int x, y, fontHeight;
  display.setFont(u8g_font_7x13);
  fontHeight = display.getFontAscent()-display.getFontDescent(); // used as offset

  y = 0.3*display.getHeight() + fontHeight;
  x = 0.0675*display.getWidth();
  display.setPrintPos(x, y);
  display.print("Target: ");
  display.print(setpoint);
  display.print((char)176);
  display.print("C");

  y += fontHeight+1;
  //x -= 3;
  display.setPrintPos(x, y);
  display.print("Current: "); 
  display.print(input);
  display.print((char)176);
  display.print("C");
  
  y += fontHeight+1;
  float pct = map(output, 0, WindowSize, 0, 1000);
  display.setPrintPos(x, y);
  display.print(pct/10);
  display.print("%");
}

// ===================================
// ==== Tuning setpoint ==============
//====================================

void tuneSp(){
  updateSp();
  drawSp();
}

void updateSp(){
   //DoControl();
   //}
  updateSettingsScreen(setpoint, 1, 0.1, RUN, TUNE_KP);
}

void drawSp(){
  drawTitleString("Set Target");
  
  int x, y, fontHeight;
  display.setFont(u8g_font_7x13);
  fontHeight = display.getFontAscent()-display.getFontDescent(); // used as offset

  y = 0.3*display.getHeight() + fontHeight;
  x = 0.075*display.getWidth();

  display.setPrintPos(x, y);
  display.print("Target: ");
  display.print(setpoint);
  display.print((char)176);
  display.print("C");
}

// ===================================
// ==== Tuning Kp ====================
// ===================================

void tuneKp(){
  updateKp();
  drawKp();
}

void updateKp(){
    updateSettingsScreen(Kp, 1, 10, SETP, TUNE_KI);
}

void drawKp(){
  drawTitleString("Set Kp");
  drawTuningConstant("Kp", Kp);
}


// ===================================
// ==== Tuning Ki ====================
// ===================================

void tuneKi(){
  updateKi();
  drawKi();
}

void updateKi(){
  updateSettingsScreen(Ki, 0.01, 10, TUNE_KP, TUNE_KD);
}

void drawKi(){
  drawTitleString("Set Ki");
  drawTuningConstant("Ki", Ki);
}

// ===================================
// ==== Tuning Kd ====================
//====================================

void tuneKd(){
  updateKd();
  drawKd();
}

void updateKd(){
  updateSettingsScreen(Kd, 0.01, 10, TUNE_KI, RUN);
}

void drawKd(){
  drawTitleString("Set Kd");
  drawTuningConstant("Kd", Kd);
}

// ===================================
// ==== misc =========================
//====================================

void debugButtons(){
  Serial.print("BUTTON_UP: "); Serial.println(digitalRead(BUTTON_UP));
  Serial.print("BUTTON_DOWN: "); Serial.println(digitalRead(BUTTON_DOWN));
  Serial.print("BUTTON_LEFT: "); Serial.println(digitalRead(BUTTON_LEFT));
  Serial.print("BUTTON_RIGHT: "); Serial.println(digitalRead(BUTTON_RIGHT));
  Serial.print("BUTTON_SELECT: "); Serial.println(digitalRead(BUTTON_SELECT));
}

// ************************************************
// Execute the control loop
// ************************************************
void doControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        finishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = output; 
  
}

// =============================================
// ======== Auto Tune ==========================
// =============================================
void startAutoTune(){
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}


void finishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

void drawTuningConstant(const char *constName, double constVar){
  int x, y, fontHeight;
  display.setFont(u8g_font_7x13);
  fontHeight = display.getFontAscent()-display.getFontDescent(); // used as offset

  y = 0.3*display.getHeight() + fontHeight;
  x = 0.075*display.getWidth();

  display.setPrintPos(x, y);
  display.print(constName);
  display.print(": ");
  display.print(constVar);
}

void updateSettingsScreen(double &constVar, float increment, float shiftFactor, OperatingState leftOpState, OperatingState rightOpState){
  if (!digitalRead(BUTTON_SHIFT))
  {
    increment *= shiftFactor;
  }
  if(updateStateChange(leftOpState, rightOpState)){
    // state has changed
    delay(200);
    return;
  }
  if (!digitalRead(BUTTON_UP))
  {
     constVar += increment;
     delay(200);
  }
  if (!digitalRead(BUTTON_DOWN))
  {
     constVar -= increment;
     delay(200);
  }

  /*if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
  {
     opState = RUN;
     return;
  }*/
}

boolean updateStateChange(OperatingState leftOpState, OperatingState rightOpState){
  if (!digitalRead(BUTTON_LEFT))
  {
     opState = leftOpState;
     return true;
  }
  if (!digitalRead(BUTTON_RIGHT))
  {
     opState = rightOpState;
     return true;
  }
  return false;
}

boolean isButtonPressed(){
  if ( !digitalRead(BUTTON_LEFT) || !digitalRead(BUTTON_UP) || !digitalRead(BUTTON_DOWN) || !digitalRead(BUTTON_RIGHT) || !digitalRead(BUTTON_SELECT) ){
    return true;
  } else{
    return false;
  }
}

/*
 * Draws text in the upper right corner, to be used on all or at least most screens
 */
void drawSousVide(void){
  display.setFont(u8g_font_4x6);
  //display.setFont(u8g_font_unifont);
  
  int strW = display.getStrWidth(STR_SOUS_VIDE);
  int x = display.getWidth() - strW;
  display.drawStr(x, display.getFontAscent()-display.getFontDescent(), STR_SOUS_VIDE);
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}


