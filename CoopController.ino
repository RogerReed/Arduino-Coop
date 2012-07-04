#include <IRremote.h>
#include <IRremoteInt.h>
#include <Wire.h>
#include <OneWire.h>
#include <TimeLord.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>

/**
* Copyright 2012, Roger Reed
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 
* 02110-1301, USA.
*/

// Whether to print debug messages to Serial
const boolean SERIAL_DEBUG = false;

// Pin definitions
const byte LIGHT_TOGGLE_BUTTON_PIN = 2, 
  WATER_BUTTON_PIN = 3,
  DOOR_RELAY_CLOSE_PIN = 4,
  DOOR_RELAY_OPEN_PIN = 5, 
  AIR_TEMP_SENSOR_PIN = 6,
  IR_RECV_PIN = 7,
  LIGHT_RELAY_PIN = 8,
  FEEDER_RELAY_PIN = 9,
  FAN_RELAY_PIN = 10,
  WATER_SOLENOID_RELAY_PIN = 11,
  DOOR_OPEN_SWITCH_PIN = A0, 
  DOOR_CLOSE_SWITCH_PIN = A1, 
  FEED_BUTTON_PIN = A2,
  WATER_FLOAT_SWITCH_PIN = A3,
  DOOR_CLOSE_STOP_PIN = A6,
  DOOR_OPEN_STOP_PIN = A7;

// Infrared constants
const int
  SONY_IR_0 = 0x910,
  SONY_IR_1 = 0x10,
  SONY_IR_2 = 0x810,
  SONY_IR_3 = 0x410,
  SONY_IR_4 = 0xC10,
  SONY_IR_5 = 0x210,
  SONY_IR_6 = 0xA10,
  SONY_IR_7 = 0x610,
  SONY_IR_8 = 0xE10,
  SONY_IR_9 = 0x110;
  
// Delay constants 
const int POST_IR_HANDLE_DELAY_MS = 1250,
  POST_BUTTON_HANDLE_DELAY_MS = 750;

// Duration constants
const int FEED_DURATION_SEC = 20,
  WATER_DURATION_SEC = 20;

// Time constants 
const int DOOR_SHUT_AFTER_SUNSET_MIN = 30,
  LIGHT_PER_DAY_HRS = 16,
  WATER_AFTER_SUNRISE_MIN = 15,
  FEED_MORNING_AFTER_SUNRISE_MIN = 30,
  FEED_AFTERNOON_BEFORE_SUNSET_HR = 4,
  FEED_EVENING_BEFORE_SUNSET_HR = 1,
  FEED_MID_DAY_HR = 12,
  FEED_MID_DAY_MIN = 0,
  LIGHT_ON_BEFORE_SUNSET_HRS = 2;

// Temperature constants  
const float MIN_REASONABLE_AIR_TEMP_F = 0.0f,  // low air temp we should never see; if we do most likely a temp probe issue
  MAX_REASONABLE_AIR_TEMP_F = 150.0f, // high air temp we should never see
  INIT_TEMP = -1000.0f,
  FAN_ON_TEMP_F = 77.0f,  // temp fan is turned on
  FAN_OFF_TEMP_F = 70.0f;  // temp fan is turned off;

// Location constants
const float LATITUDE = 33.0369867,
    LONGITUDE = -117.2919818;
const int TIMEZONE = -8;
    
// Infrared variables
IRrecv irRecv(IR_RECV_PIN);
decode_results irResults;

// Temperature variables
OneWire airTempOneWire(AIR_TEMP_SENSOR_PIN); 
boolean airTempError = false;
float airTemp = INIT_TEMP;

// Time variables
TimeLord coopTimeLord;
long feedOnTimeSec = 0,
  waterOnTimeSec = 0,
  lastLoopSec = now();
byte openDoorHr, openDoorMin, closeDoorHr, closeDoorMin,
  lightOnHr, lightOnMin, lightOffHr, lightOffMin,
   waterHr, waterMin, feedMorningHr, feedMorningMin,
   feedAfternoonHr, feedAfternoonMin,
   feedEveningHr, feedEveningMin;
   
// Control variables
boolean manualWater = false;

void setup() {    
  if(SERIAL_DEBUG){
    Serial.begin(57600); // initialize hardware serial port    
  }
  
  Wire.begin();
  irRecv.enableIRIn();
   
  setSyncProvider(RTC.get);

  coopTimeLord.TimeZone(TIMEZONE * 60);
  coopTimeLord.Position(LATITUDE, LONGITUDE);
  
  setupOutputPins();
  
  initState();
  scheduleDailyAlarms();
  scheduleTodayAlarms();
}

void loop(){
  if (SERIAL_DEBUG) {
    serialDebugState();
  }

  handleDoor();
  updateAirTemp();
  handleFan();
  handleDurations();
  handleControls();
  handleFloat();
  handleInfrared();  
  
  Alarm.delay(0);
  lastLoopSec = now();
}

/**
 * Calculates alarm times used for scheduling and init state.
 */
void calculateAlarmTimes(){
  int nowHour = hour(),
   nowMinute = minute(),
   nowDay = day(),
   nowMonth = month(),
   nowYear = year();
   
  Serial.print("nowHour: ");
  Serial.println(nowHour);
  Serial.print("nowMinute: ");
  Serial.println(nowMinute);  
  Serial.print("nowDay: ");
  Serial.println(nowDay);
  Serial.print("nowMonth: ");
  Serial.println(nowMonth);
  Serial.print("nowYear: ");
  Serial.println(nowYear);
   
  byte timeLordSunRise[]  = {0, 0, 0, nowDay, nowMonth, nowYear};
  byte timeLordSunSet[]  = {0, 0, 0, nowDay, nowMonth, nowYear};

  coopTimeLord.SunRise(timeLordSunRise);
  coopTimeLord.SunSet(timeLordSunSet);
  
  if(SERIAL_DEBUG){
    Serial.print("sunrise: ");
    Serial.print(timeLordSunRise[2]);
    Serial.print(":");
    Serial.println(timeLordSunRise[1]);
    Serial.print("sunset: ");
    Serial.print(timeLordSunSet[2]);
    Serial.print(":");
    Serial.println(timeLordSunSet[1]);
  }
  
  openDoorHr = timeLordSunRise[2];
  openDoorMin = timeLordSunRise[1];
  closeDoorHr = timeLordSunSet[2];
  closeDoorMin = timeLordSunSet[1];
  
  if(closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN >= 60){
    closeDoorHr += (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN) / 60;
    closeDoorMin = (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN) % 60;
  }else{
    closeDoorMin += DOOR_SHUT_AFTER_SUNSET_MIN;
  }
  
  if(SERIAL_DEBUG){
    Serial.print("open door: ");
    Serial.print(openDoorHr);
    Serial.print(":");
    Serial.println(openDoorMin);
    Serial.print("close door: ");
    Serial.print(closeDoorHr);
    Serial.print(":");
    Serial.println(closeDoorMin);
  }

  float naturalDaylightHr;
  naturalDaylightHr = 12.0f-(timeLordSunRise[2] + timeLordSunRise[1]/60.0f);
  naturalDaylightHr += (timeLordSunSet[2] + timeLordSunSet[1]/60.0f)-12.0f;
  
  if(SERIAL_DEBUG){
    Serial.print("natural daylight (hrs): ");
    Serial.println(naturalDaylightHr);
  }
  
  lightOnHr = timeLordSunSet[2];
  lightOnMin = timeLordSunSet[1];
  lightOffHr = timeLordSunSet[2];
  lightOffMin = timeLordSunSet[1];
  
  float lightNeededHrs = LIGHT_PER_DAY_HRS - naturalDaylightHr;
  int lightNeededMins = lightNeededHrs * 60;
  
  Serial.print("light needed (mins): ");
  Serial.println(lightNeededMins);

  lightOnHr -= LIGHT_ON_BEFORE_SUNSET_HRS;

  if(lightOffMin + lightNeededMins >= 60){
    lightOffHr += (lightOffMin + lightNeededMins) / 60;
    lightOffMin = (lightOffMin + lightNeededMins) % 60;
  }else{
    lightOffMin += lightNeededMins;
  }
  
  if(SERIAL_DEBUG){
    Serial.print("light on: ");
    Serial.print(lightOnHr);
    Serial.print(":");
    Serial.println(lightOnMin);
    Serial.print("light off: ");
    Serial.print(lightOffHr);
    Serial.print(":");
    Serial.println(lightOffMin);
  }
  
  waterHr = timeLordSunRise[2];
  waterMin = timeLordSunRise[1];
  feedMorningHr = timeLordSunRise[2];
  feedMorningMin = timeLordSunRise[1];
  feedAfternoonHr = timeLordSunSet[2];
  feedAfternoonMin = timeLordSunSet[1];
  feedEveningHr = timeLordSunSet[2];
  feedEveningMin = timeLordSunSet[1];
  
  if(waterMin + WATER_AFTER_SUNRISE_MIN >= 60){
    waterHr += (waterMin + WATER_AFTER_SUNRISE_MIN) / 60;
    waterMin = (waterMin + WATER_AFTER_SUNRISE_MIN) % 60;
  }else{
    waterMin += WATER_AFTER_SUNRISE_MIN;
  }
  
  if(feedMorningMin + FEED_MORNING_AFTER_SUNRISE_MIN >= 60){
    feedMorningHr += (feedMorningMin + FEED_MORNING_AFTER_SUNRISE_MIN) / 60;
    feedMorningMin = (feedMorningMin + FEED_MORNING_AFTER_SUNRISE_MIN) % 60;
  }else{
    feedMorningMin += FEED_MORNING_AFTER_SUNRISE_MIN;
  }
  
  feedAfternoonHr -= FEED_AFTERNOON_BEFORE_SUNSET_HR;
  feedEveningHr -= FEED_EVENING_BEFORE_SUNSET_HR;

  if(SERIAL_DEBUG){
    Serial.print("water: ");
    Serial.print(waterHr);
    Serial.print(":");
    Serial.println(waterMin);
    Serial.print("feed (morning): ");
    Serial.print(feedMorningHr);
    Serial.print(":");
    Serial.println(feedMorningMin);
    Serial.print("feed (afternoon): ");
    Serial.print(feedAfternoonHr);
    Serial.print(":");
    Serial.println(feedAfternoonMin);
    Serial.print("feed (evening): ");
    Serial.print(feedEveningHr);
    Serial.print(":");
    Serial.println(feedEveningMin);
  } 
}

/**
 * Schedule daily alarms.
 */
void scheduleDailyAlarms(){
  Alarm.alarmRepeat(1, 0, 0, scheduleTodayAlarms);
  Alarm.alarmRepeat(FEED_MID_DAY_HR, FEED_MID_DAY_MIN, 0, feed); 
}

/**
 * Schedule today alarms.
 */
void scheduleTodayAlarms(){
  calculateAlarmTimes();
  
  int nowHr = hourMinuteToHour(hour(), minute());
  
  if(nowHr < hourMinuteToHour(openDoorHr, openDoorMin)){
    Alarm.alarmOnce(openDoorHr, openDoorMin, 0, enableDoorOpening);
  } 
  
  if(nowHr < hourMinuteToHour(closeDoorHr, closeDoorMin)){
    Alarm.alarmOnce(closeDoorHr, closeDoorMin, 0, enableDoorClosing);
  } 
  
  if(nowHr < hourMinuteToHour(lightOnHr, lightOnMin)){
    Alarm.alarmOnce(lightOnHr, lightOnMin, 0, lightOn); 
  }
  
  if(nowHr < hourMinuteToHour(lightOffHr, lightOffMin)){
    Alarm.alarmOnce(lightOffHr, lightOffMin, 0, lightOff);
  }
  
  if(nowHr < hourMinuteToHour(waterHr, waterMin)){
    Alarm.alarmOnce(waterHr, waterMin, 0, water); 
  }
  
  if(nowHr < hourMinuteToHour(feedMorningHr, feedMorningMin)){
    Alarm.alarmOnce(feedMorningHr, feedMorningMin, 0, feed); 
  }
  
  // mid day feed scheduled as daily at noon in scheduleDailyAlarms method
  
  if(nowHr < hourMinuteToHour(feedAfternoonHr, feedAfternoonMin)){
    Alarm.alarmOnce(feedAfternoonHr, feedAfternoonMin, 0, feed);
  } 
  
  if(nowHr < hourMinuteToHour(feedEveningHr, feedEveningMin)){
    Alarm.alarmOnce(feedEveningHr, feedEveningMin, 0, feed);
  } 
}

/**
 * Initialize state based on current time 
 */
void initState(){
  fanOff();
  feedOff();
  waterOff();
  
  calculateAlarmTimes();
  
  int nowHr = hourMinuteToHour(hour(), minute());

  if(nowHr > hourMinuteToHour(lightOnHr, lightOnMin) && nowHr < hourMinuteToHour(lightOffHr, lightOffMin)){
    Serial.println("light init on");
    lightOn();
  }else{
    Serial.println("light init off");
    lightOff();
  }
  
  if(nowHr > hourMinuteToHour(openDoorHr, openDoorMin) && nowHr < hourMinuteToHour(closeDoorHr, closeDoorMin)){
    Serial.println("door init open");
    enableDoorOpening();
  }else{
    Serial.println("door init close");
    enableDoorClosing();
  }  
}
 
/**
 * Set pin modes
 */
void setupOutputPins(){
  // relays
  pinMode(DOOR_RELAY_OPEN_PIN, OUTPUT);
  pinMode(DOOR_RELAY_CLOSE_PIN, OUTPUT);
  pinMode(FEEDER_RELAY_PIN, OUTPUT);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(WATER_SOLENOID_RELAY_PIN, OUTPUT);
}
    
void serialDebugState(){    
//  Serial.print("LIGHT_TOGGLE_BUTTON_PIN (");
//  Serial.print(LIGHT_TOGGLE_BUTTON_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(digitalRead(LIGHT_TOGGLE_BUTTON_PIN));
//  Serial.print(", WATER_BUTTON_PIN (");
//  Serial.print(WATER_BUTTON_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(digitalRead(WATER_BUTTON_PIN));
//  Serial.print(", DOOR_OPEN_SWITCH_PIN (");
//  Serial.print(DOOR_OPEN_SWITCH_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(digitalRead(DOOR_OPEN_SWITCH_PIN));
//  Serial.print(", DOOR_CLOSE_SWITCH_PIN (");
//  Serial.print(DOOR_CLOSE_SWITCH_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(digitalRead(DOOR_CLOSE_SWITCH_PIN));
//  Serial.print(", WATER_FLOAT_SWITCH_PIN (");
//  Serial.print(WATER_FLOAT_SWITCH_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(digitalRead(WATER_FLOAT_SWITCH_PIN));
//  Serial.print(", FEED_BUTTON_PIN (");
//  Serial.print(FEED_BUTTON_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(digitalRead(FEED_BUTTON_PIN));
//  Serial.print(", DOOR_CLOSE_STOP_PIN (");
//  Serial.print(DOOR_CLOSE_STOP_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(analogRead(DOOR_CLOSE_STOP_PIN) > 500);
//  Serial.print(", DOOR_OPEN_STOP_PIN (");
//  Serial.print(DOOR_OPEN_STOP_PIN, DEC);
//  Serial.print("): ");
//  Serial.print(analogRead(DOOR_OPEN_STOP_PIN) > 500);
//  Serial.println();
  
  Serial.print("airTemp: ");
  if(!airTempError){
    Serial.print(airTemp);
    Serial.println();
  }else{
    Serial.println("ERROR");
  }
  
  Serial.print("time: ");
  Serial.print(year(), DEC);
  Serial.print('/');
  Serial.print(month(), DEC);
  Serial.print('/');
  Serial.print(day(), DEC);
  Serial.print(' ');
  Serial.print(hour(), DEC);
  Serial.print(':');
  Serial.print(minute(), DEC);
  Serial.print(':');
  Serial.print(second(), DEC);
  Serial.println();
}

/**
 * Handles any action that needs to be taken onfloat switch.
 */
void handleFloat(){
  if(digitalRead(WATER_FLOAT_SWITCH_PIN)){
    waterOn();
  }else if(!manualWater){
    waterOff();
  }
}

/**
 * Handles any action that needs to be taken on control buttons or switches.
 */
void handleControls(){  
  if(isLightToggleButton()){
    Serial.println("toggle light button");
    toggleLight();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
  
  if(isFeedButton()){
    Serial.println("feed button");
    feed();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }

  if(isWaterButton()){
    Serial.println("water button");
    manualWater = true;
    water();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
  
  if(isDoorOpenSwitch()){
    Serial.println("door open switch");
    enableDoorOpening();
  }
  
  if(isDoorCloseSwitch()){
    Serial.println("door close switch");
    enableDoorClosing();
  }
}

/**
 * Handles any action that needs to be taken on the fan.
 */
void handleFan() {
 if(!airTempError){
   if(airTemp >= FAN_ON_TEMP_F){
     fanOn();
   }
  
   if(airTemp <= FAN_OFF_TEMP_F){
     fanOff();
   }
 }
}

/**
 * Updates air temp and checks for probe error.
 */
void updateAirTemp() {
  airTempError = !updateTemp(&airTempOneWire, &airTemp, airTempError);
  if(airTempError || airTemp <= MIN_REASONABLE_AIR_TEMP_F || airTemp >= MAX_REASONABLE_AIR_TEMP_F){
    airTempError = true;
  }
}

/**
 * Handles any action that needs to be taken on the door motor.
 */
void handleDoor() {
  if(isDoorClosing() && isDoorCloseStop()){
    disableDoor();
  }
  
  if(isDoorOpening() && isDoorOpenStop()){
    disableDoor();
  }
}

/**
 * Handles any durations (e.g. feed, water) that need to be monitored and turned off after set time.
 */
void handleDurations(){
  if(isFeedOn()){
    feedOnTimeSec += now() - lastLoopSec;
    if(SERIAL_DEBUG){
        Serial.print("feeding (");
        Serial.print(feedOnTimeSec);
        Serial.println(" sec)");
    }
    if(feedOnTimeSec >= FEED_DURATION_SEC){
      feedOff();
    }
  }
  
  if(isWaterOn()){
    waterOnTimeSec += now() - lastLoopSec;
    if(SERIAL_DEBUG){
        Serial.print("watering (");
        Serial.print(waterOnTimeSec);
        Serial.println(" sec)");
    }
    if(waterOnTimeSec >= WATER_DURATION_SEC){
      waterOff();
      manualWater = false;
    }
  }
}

/**
 * Handles any action that need to be taken from Infrared
 */
void handleInfrared() {
  if (!irRecv.decode(&irResults)) {
    irRecv.resume();
    return;
  }
  
  if(SERIAL_DEBUG){
    Serial.print("irResults.value: ");
    Serial.println(irResults.value, HEX);
  }
  
  switch(irResults.value){
    case SONY_IR_4:
      // water
      Serial.println("water remote");
      manualWater = true;
      water();
      delay(POST_IR_HANDLE_DELAY_MS);
      break;
    case SONY_IR_5:
      // toggle light
      Serial.println("toggle light remote");
      toggleLight();
      delay(POST_IR_HANDLE_DELAY_MS);
      break;
    case SONY_IR_6:
      // feed
      Serial.println("feed remote");
      feed();
      delay(POST_IR_HANDLE_DELAY_MS);
      break;
    case SONY_IR_7:
      // open door
      Serial.println("door open remote");
      enableDoorOpening();
      delay(POST_IR_HANDLE_DELAY_MS);
      break;
    case SONY_IR_8:
      // close door
      Serial.println("door close remote");
      enableDoorClosing();
      delay(POST_IR_HANDLE_DELAY_MS);
      break;
    case SONY_IR_9:
      // toggle fan
      Serial.println("toggle fan remote");
      toggleFan();
      delay(POST_IR_HANDLE_DELAY_MS);
      break;
  }
  
  irRecv.resume();
}

// feed methods
void feed(){
  feedOnTimeSec = 0;
  feedOn();
}

void feedOn(){
  digitalWrite(FEEDER_RELAY_PIN, LOW);
}

void feedOff(){
  digitalWrite(FEEDER_RELAY_PIN, HIGH);
}

boolean isFeedOn(){
  return digitalRead(FEEDER_RELAY_PIN) == LOW;
}

boolean isFeedButton(){
  return digitalRead(FEED_BUTTON_PIN) == HIGH;
}
// end feed methods

// water methods
void water(){
  waterOnTimeSec = 0;
  waterOn();
}

void waterOn(){
  digitalWrite(WATER_SOLENOID_RELAY_PIN, HIGH);
}

void waterOff(){
  digitalWrite(WATER_SOLENOID_RELAY_PIN, LOW);
}

boolean isWaterOn(){
  return digitalRead(WATER_SOLENOID_RELAY_PIN) == HIGH;
}

boolean isWaterButton(){
  return digitalRead(WATER_BUTTON_PIN) == HIGH;
}
// end feed methods

// light methods
void toggleLight(){
  if(isLightOn()){
    lightOff();  
  }else{
    lightOn();
  }
}

boolean isLightOn(){
  return digitalRead(LIGHT_RELAY_PIN) == LOW;
}

void lightOn(){
  digitalWrite(LIGHT_RELAY_PIN, LOW);
}

void lightOff(){
  digitalWrite(LIGHT_RELAY_PIN, HIGH);
}

boolean isLightToggleButton(){
  return digitalRead(LIGHT_TOGGLE_BUTTON_PIN) == HIGH;
}
// end light methods

// fan methods
void toggleFan(){
  if(isFanOn()){
    fanOff();  
  }else{
    fanOn();
  }
}

boolean isFanOn(){
  return digitalRead(FAN_RELAY_PIN) == LOW;
}

void fanOn(){
  digitalWrite(FAN_RELAY_PIN, LOW);
}

void fanOff(){
  digitalWrite(FAN_RELAY_PIN, HIGH);
}
// end fan methods

// door methods
boolean isDoorOpenSwitch(){
  return digitalRead(DOOR_OPEN_SWITCH_PIN);
}

boolean isDoorCloseSwitch(){
  return digitalRead(DOOR_CLOSE_SWITCH_PIN);
}

boolean isDoorClosing(){
  return digitalRead(DOOR_RELAY_CLOSE_PIN) == HIGH;
}

boolean isDoorOpening(){
  return digitalRead(DOOR_RELAY_OPEN_PIN) == HIGH;
}

void enableDoorClosing(){
  if(!isDoorCloseStop()){
    digitalWrite(DOOR_RELAY_OPEN_PIN, LOW);
    digitalWrite(DOOR_RELAY_CLOSE_PIN, HIGH);
  }
}

void enableDoorOpening(){
  if(!isDoorOpenStop()){
    digitalWrite(DOOR_RELAY_CLOSE_PIN, LOW);
    digitalWrite(DOOR_RELAY_OPEN_PIN, HIGH);
  }
}

boolean isDoorInMiddle(){
  return !isDoorOpenStop() && !isDoorCloseStop();
}

boolean isDoorCloseStop(){
  return analogRead(DOOR_CLOSE_STOP_PIN) > 500;
}

boolean isDoorOpenStop(){
  return analogRead(DOOR_OPEN_STOP_PIN) > 500;
}

void disableDoor(){
   digitalWrite(DOOR_RELAY_OPEN_PIN, LOW);
   digitalWrite(DOOR_RELAY_CLOSE_PIN, LOW);
}
// end door methods

/**
 * Updates a single temp
 */
boolean updateTemp(OneWire * oneWireTherm, float * fltTemp, boolean previousError){
  byte thermAddr[8], data[12];
  oneWireTherm->reset_search();   
  oneWireTherm->search(thermAddr); 

  if(OneWire::crc8(thermAddr,7)!=thermAddr[7]) { // checksum invalid
    return false;
  }
 
  if(!oneWireTherm->reset()){
    return false; 
  }
  
  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0x44, 1); // start conversation w/ parasite power
  
  if(previousError){
    delay(1000);
  }
  
  if(!oneWireTherm->reset()){
    return false; 
  }
  
  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0xBE); // read scratchpad  
  for(int i=0;i<9;i++) { // need 9 bytes
    data[i] = oneWireTherm->read();
  }

  *fltTemp = getTemperature(data[0], data[1]);

  return true;
}

/**
 * Converts celsius to fahrenheit
 */ 
float convertCeliusToFahrenheit(float c) {
  return((c*1.8)+32); 
}

/**
 * Converts fahrenheit to celsius
 */ 
float convertFahrenheitToCelius(float f) {
  return((f-32)*0.555555556); 
}

float hourMinuteToHour(int hour, int minute){
  return hour + minute/60.0f;
}

/**
 * Gets temp from bytes for OneWire
 */
float getTemperature(int lowByte, int highByte) {
  int intPostDecimal, boolSign, intHexTempReading, intTempReadingBeforeSplit, preDecimal, i;
  float fltPostDecimal, fltTemp;
  intHexTempReading = (highByte << 8) + lowByte;
  boolSign = intHexTempReading & 0x8000;
  if(boolSign) {
    intHexTempReading = (intHexTempReading ^ 0xffff) + 1;
  }
  intTempReadingBeforeSplit = 6.25 * intHexTempReading; // multiply by (100 * precision) = (100 * 0.0625) = 6.25 = 12-bit precision
  preDecimal = intTempReadingBeforeSplit / 100;
  intPostDecimal = intTempReadingBeforeSplit % 100;
  fltPostDecimal = intPostDecimal;
  if(intPostDecimal<10) {
    fltTemp = preDecimal+(fltPostDecimal/1000);
  }
  else {
    fltTemp = preDecimal+(fltPostDecimal/100);
  }  
  if(boolSign) { 
    fltTemp = -fltTemp; 
  }
  return convertCeliusToFahrenheit(fltTemp);
}


