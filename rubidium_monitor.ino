/*
 * Rubidium_Monitor by Jonathan Zepp - @DaJMasta
 * Version 1.01 - September 25, 2017
 * Simple management and serial switching software for my Salvage Rubidium GPSDO project
 * 
 * Version 1.01 - Removed old error condition, tweaked values, changed error indicator LED behavior
 * 
  */
  
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <OneWire.h>

#define settlingLED 5
#define stabilizedLED 6
#define tempLED 7
#define tempBus 2
#define gpsRX 9
#define gpsTX 8
#define rbTX 16
#define rbRX 10
#define gpsPPS 3
#define rbPPS 4

#define warmUpTime 3600     //1 hour
#define settleTime 86400    //1 day
#define fullStable 604800   //1 week

#define blinkSpeed 250
#define tempPollSpeed 1000

#define centerTempBoard 51.0
#define centerTempPSU 49.5
#define centerTempRb 54.5
#define allowedTempVariation 5.0
#define maxTempVariation 8.5

#define commandBuffer 16

long gpsUptime, rbUptime, unitUptime, timeToLockGPS, timeToLockRb, countDifferential, blinkTimer, lastTempPoll, tempErrorTime ;
boolean bLastGPSPPS, bLastRbPPS, bTempLED, bSettlingLED, bStabilizedLED, bBlinkTemp, bBlinkSettling, bBlinkStabilized, bBlinkToggle, 
        bChangeMode, bChangeSerial, bHadTempError ;
byte runMode, serialMode ;
float rbTemp, boardTemp, psuTemp, rbMin, rbMax, boardMin, boardMax, psuMin, psuMax ;
int gpsUnlocks ;

OneWire oneWire(tempBus) ;
DallasTemperature tempSensors(&oneWire) ;
DeviceAddress rbTherm = {0x28, 0xFF, 0xCB, 0x89, 0x01, 0x17, 0x04, 0x18} ;               //Addresses of installed sensors, found with "Tester" example sketch
DeviceAddress boardTherm = {0x28, 0xFF, 0x73, 0xDD, 0x00, 0x17, 0x05, 0x4A} ;
DeviceAddress psuTherm = {0x28, 0xFF, 0x7D, 0x88, 0x01, 0x17, 0x04, 0x44} ;

SoftwareSerial rbPort(rbTX, rbRX, SERIAL_8N1) ;
SoftwareSerial gpsPort(gpsTX, gpsRX) ;

void setup() {
  Serial.begin(57600) ;
  Serial.println("Rubidium GPSDO monitor initializing...") ;

  pinMode(settlingLED, OUTPUT) ;
  pinMode(stabilizedLED, OUTPUT) ;
  pinMode(tempLED, OUTPUT) ;
  pinMode(gpsPPS, INPUT) ;
  pinMode(rbPPS, INPUT) ;

  gpsUptime = 0 ;
  rbUptime = 0 ;
  unitUptime = 0 ;
  timeToLockGPS = 0 ;
  timeToLockRb = 0 ;
  gpsUnlocks = 0 ;
  bLastGPSPPS = digitalRead(gpsPPS) ;
  bLastRbPPS = digitalRead(rbPPS) ;
  bTempLED = true ;
  bSettlingLED = true ;
  bStabilizedLED = true ;
  bBlinkTemp = false ;
  bBlinkSettling = false ;
  bBlinkStabilized = false ;
  bBlinkToggle = false ;
  bChangeMode = true ;
  bChangeSerial = false ;
  countDifferential = 0 ;
  runMode = 0 ;
  serialMode = 0 ;
  blinkTimer = 0 ;
  lastTempPoll = 0 ;
  rbTemp = 0 ;
  boardTemp = 0 ;
  psuTemp = 0 ;
  rbMin = 100 ;
  rbMax = -100 ;
  boardMin = 100 ;
  boardMax = -100 ;
  psuMin = 100 ;
  psuMax = -100 ;
  bHadTempError = false ;
  
  updateLEDs() ;

  rbPort.begin(9600) ;                                  //Mode 1
  gpsPort.begin(9600) ;                                 //Mode 2
  rbPort.setTimeout(50) ;
  gpsPort.setTimeout(50) ;

  tempSensors.begin() ;
  if(tempSensors.getDeviceCount() != 3)
    Serial.println("Temp sensor(s) not found!") ;

  tempSensors.setResolution(rbTherm, 9) ;
  tempSensors.setResolution(boardTherm, 9) ;
  tempSensors.setResolution(psuTherm, 9) ;

  checkPPS() ;
  checkTemps() ;

  runMode = 1 ;
  Serial.println("Initialized!") ;
  Serial.println() ;
}

void loop() {
  checkPPS() ;
  trackTime() ;
  
  if(lastTempPoll + tempPollSpeed <= millis())
    checkTemps() ;

  if(unitUptime % 60 == 0){
    if(bHadTempError && !bBlinkTemp && runMode > 2){
      runMode = 3 ;
      bChangeMode = true ;
      bHadTempError = false ;
      tempErrorTime = unitUptime ;
      returnTime(unitUptime) ;
      Serial.println(", Temperature out of bounds, resettling...") ;
      Serial.println() ;
    }
  }
    
  evalStatus() ;
  updateLEDs() ;
  doSerial() ;
}

void trackTime(){
  if(gpsUptime == 0 || rbUptime == 0)
    unitUptime = millis() / 1000 ;
  else{
    if(runMode > 1)
      unitUptime = (timeToLockRb / 1000) + rbUptime ;
    else
      unitUptime = (timeToLockGPS / 1000) + gpsUptime ;
  }
}

void updateLEDs(){
  if(blinkTimer + blinkSpeed <= millis()){
    blinkTimer = millis() ;
    bBlinkToggle = !bBlinkToggle ;
  }
  
  if(bBlinkSettling)
    digitalWrite(settlingLED, bBlinkToggle) ;
  else
    digitalWrite(settlingLED, bSettlingLED) ;

  if(bBlinkStabilized)
    digitalWrite(stabilizedLED, bBlinkToggle) ;
  else
    digitalWrite(stabilizedLED, bStabilizedLED) ;

  if(bBlinkTemp)
    digitalWrite(tempLED, bBlinkToggle) ;
  else
    digitalWrite(tempLED, bTempLED) ;
}

void checkPPS(){
  boolean readVal, didSomething ;
  didSomething = false ;
  
  readVal = digitalRead(gpsPPS) ;
  if(readVal != bLastGPSPPS){
    if(gpsUptime == 0)
        timeToLockGPS = millis() ;
      gpsUptime++ ;
      bLastGPSPPS = readVal ;
      didSomething = true ;
  }

  readVal = digitalRead(rbPPS) ;
  if(readVal != bLastRbPPS){
    if(rbUptime == 0)
      timeToLockRb = millis() ;
    rbUptime++ ;
    bLastRbPPS = readVal ;
    didSomething = true ;
  }
  if(gpsUptime > 0 && rbUptime > 0 && didSomething){
    if(countDifferential == 0)
      countDifferential = gpsUptime - rbUptime ;

    /* if((countDifferential + rbUptime) > (gpsUptime + 2) && runMode != 255){
      runMode = 255 ;
      bChangeMode = true ;
      returnTime(unitUptime) ;
      Serial.println(", PPS DESYNC ERROR:") ;
      Serial.print("GPS Uptime: ") ;
      returnTime(gpsUptime) ;
      Serial.println() ;
      Serial.print("Rubidium Uptime: ") ;
      returnTime(rbUptime) ;
      Serial.println() ;
    }
    else */ if((countDifferential + rbUptime) < (gpsUptime - 2)){
      runMode = 1 ;
      bChangeMode = true ;
      gpsUptime = 0 ;
      countDifferential = 0 ;
      gpsUnlocks++ ;
      returnTime(unitUptime) ;
      Serial.println(", GPS lock lost, reacquiring...") ;
      Serial.println() ;
    }
  }
}

void checkTemps(){
  lastTempPoll = millis() ;

  tempSensors.requestTemperatures();

  rbTemp = tempSensors.getTempC(rbTherm) ;
  boardTemp = tempSensors.getTempC(boardTherm) ;
  psuTemp = tempSensors.getTempC(psuTherm) ;

  if(rbTemp > rbMax)
    rbMax = rbTemp ;
  if(rbTemp < rbMin)
    rbMin = rbTemp ;
    
  if(boardTemp > boardMax)
    boardMax = boardTemp ;
  if(boardTemp < boardMin)
    boardMin = boardTemp ;
    
  if(psuTemp > psuMax)
    psuMax = psuTemp ;
  if(psuTemp < psuMin)
    psuMin = psuTemp ;

  bTempLED = false ;
  bBlinkTemp = false ;

  if(runMode < 2){
    bTempLED = true ;
    return ;
  }

  if((rbTemp > centerTempRb + maxTempVariation) || (rbTemp < centerTempRb - maxTempVariation)){
    bTempLED = true ;
    bBlinkTemp = true ;
    if(runMode > 2){
      if(!bHadTempError){
        returnTime(unitUptime) ;
        Serial.print(", Rubidium temperature error: ") ;
        Serial.println(rbTemp) ;
        Serial.println() ;
        bHadTempError = true ;
      }
    }
  }
  else if((rbTemp > centerTempRb + allowedTempVariation) || (rbTemp < centerTempRb - allowedTempVariation)){
    bTempLED = true ;
    bBlinkTemp = false ;
  }
  else{
    bTempLED = false ;
    bBlinkTemp = false ;
  }

  if((psuTemp > centerTempPSU + maxTempVariation) || (psuTemp < centerTempPSU - maxTempVariation)){
    bTempLED = true ;
    bBlinkTemp = true ;
    if(runMode > 2){
      if(!bHadTempError){
        returnTime(unitUptime) ;
        Serial.print(", PSU temperature error: ") ;
        Serial.println(psuTemp) ;
        Serial.println() ;
        bHadTempError = true ;
      }
    }
  }
  else if((psuTemp > centerTempPSU + allowedTempVariation) || (psuTemp < centerTempPSU - allowedTempVariation)){
    bTempLED = true ;
    if(!bBlinkTemp)
      bBlinkTemp = false ;
  }

  if((boardTemp > centerTempBoard + maxTempVariation) || (boardTemp < centerTempBoard - maxTempVariation)){
    bTempLED = true ;
    bBlinkTemp = true ;
    if(runMode > 2){
      if(!bHadTempError){
        returnTime(unitUptime) ;
        Serial.print(", Board temperature error: ") ;
        Serial.println(boardTemp) ;
        Serial.println() ;
        bHadTempError = true ;
      }
    }
  }
  else if((boardTemp > centerTempBoard + allowedTempVariation) || (boardTemp < centerTempBoard - allowedTempVariation)){
    bTempLED = true ;
    if(!bBlinkTemp)
      bBlinkTemp = false ;
  }
}

void evalStatus(){
  switch(runMode){
    case 0:   Serial.println("Init Failed") ;                                     //Setup progresses to mode 1
              Serial.println() ;
              runMode = 255 ;
              bChangeMode = true ;
              break ;
    case 1:   if(bChangeMode){
                returnTime(unitUptime) ;
                Serial.println(", Waiting for GPS/Rb Locks...") ;
                Serial.println() ;
              }
              bChangeMode = false ;
              if(gpsUptime > 0 && rbUptime > 0){                                   //Waiting for locks
                runMode++ ;
                bChangeMode = true ;
              }
              bSettlingLED = true ;
              bStabilizedLED = true ;
              bBlinkSettling = true ;
              bBlinkStabilized = true ;
              break ;
    case 2:   if(bChangeMode){
                returnTime(unitUptime) ;
                Serial.println(", Locks acquired, proceeding to warm up") ;
                Serial.println() ;
              }
              bChangeMode = false ;
              if(gpsUptime > warmUpTime && rbUptime > warmUpTime){         //Waiting to warm up
                runMode++ ;
                bChangeMode = true ;
                countDifferential = gpsUptime - rbUptime ;
              }
              bSettlingLED = true ;
              bStabilizedLED = false ;
              bBlinkSettling = true ;
              bBlinkStabilized = false ;
              break ;
    case 3:   if(bChangeMode){
                returnTime(unitUptime) ;
                Serial.println(", Warmed up, proceeding to settle") ;
                Serial.println() ;
              }
              bChangeMode = false ;
              if((gpsUptime > settleTime && rbUptime > settleTime && !bHadTempError) || 
                (tempErrorTime + settleTime > unitUptime && bHadTempError)) {               //Waiting to settle
                runMode++ ;
                bChangeMode = true ;
              }
              bSettlingLED = true ;
              bStabilizedLED = true ;
              bBlinkSettling = false ;
              bBlinkStabilized = true ;
              break ;
    case 4:   if(bChangeMode){
                returnTime(unitUptime) ;
                Serial.println(", Settled, proceeding to stabilize") ;
                Serial.println() ;
              }
              bChangeMode = false ;
              if((gpsUptime > fullStable && rbUptime > fullStable && !bHadTempError) || 
                (tempErrorTime + fullStable > unitUptime && bHadTempError)){                //Waiting to stabilize
                runMode++ ;
                bChangeMode = true ;
                bHadTempError = false ;
              }
              bSettlingLED = true ;
              bStabilizedLED = true ;
              bBlinkSettling = true ;
              bBlinkStabilized = false ;
              break ;
    case 5:   if(bChangeMode){
                returnTime(unitUptime) ;
                Serial.println(", Fully stabilized") ;         //Normal operation
                Serial.println() ;
              }
              bChangeMode = false ;
              bSettlingLED = false ;
              bStabilizedLED = true ;
              bBlinkSettling = false ;
              bBlinkStabilized = false ;
              break ;
    case 255: if(bChangeMode){
                returnTime(unitUptime) ;
                Serial.println(", ERROR") ;                     //Error
                Serial.println() ;
      }
              bChangeMode = false ;
              bSettlingLED = true ;
              bStabilizedLED = true ;
              bBlinkSettling = true ;
              bBlinkStabilized = true ;
              bTempLED = true ;
              bBlinkTemp = true ;
              break ;
    default:  returnTime(unitUptime) ;
              Serial.print(", Invalid run mode: ") ;
              Serial.println(runMode) ;
              Serial.println() ;
              bChangeMode = true ;
              runMode = 255 ;
  }
}

void doSerial(){
 
  if(bChangeSerial){
    if(serialMode == 1) 
      rbPort.listen() ;
    else if(serialMode == 2) 
      gpsPort.listen() ;
    else if(serialMode == 0) ;
    else{
      returnTime(unitUptime) ;
      Serial.print(", Invalid serial mode: ") ;
      Serial.println(serialMode) ;
      Serial.println() ;
      serialMode = 0 ;
    }
    bChangeSerial = false ;
  }

  Serial.flush() ;
  
  switch(serialMode){
    case 0:   receiveCommand() ;
              break ;
    case 1:   if(rbPort.available())
                Serial.write(rbPort.read()) ;
              else
                receiveCommand() ;
              break ;
    case 2:   if(gpsPort.available())
                Serial.write(gpsPort.read()) ;
              else
                receiveCommand() ;
              break ;
    default:  returnTime(unitUptime) ;
              Serial.println(", invalid serial mode.") ;
              Serial.println() ;
              break ;
  }
}

/*    Command format:
 *     <*CMMD*>
 * 
 *    Commands:
 *    RTMP - return temperatures
 *    RTST - return temperature statistics
 *    RTUT - return uptimes
 *    RSER - return serial mode
 *    RFUL - return full statistics
 *    
 *    SSER - Switch serial mode
 *    RSTP - reset temperature statistics
 *    RLCK - restart locking and settling procedure
 */

void receiveCommand(){
  char command[commandBuffer] ;
  byte counter ;
  boolean bForMonitor = false ;
  
  if(!Serial.available())
    return ;

  for(counter = 0; counter < commandBuffer; counter++)
    command[counter] = ' ' ;
  
  if(!Serial.readBytesUntil('\n', command, commandBuffer))
    return ;
  while(Serial.available())
    Serial.read() ;

  for(counter = 0; counter < commandBuffer - 1; counter++){
    if(command[counter] == '<' && command[counter + 1] == '*')
      bForMonitor = true ;
  }
  
  if(bForMonitor){
    for(counter = 0; counter < commandBuffer - 8; counter++){
      if(command[counter] == '<' && command[counter + 1] == '*' && command[counter + 6] == '*' && command[counter + 7] == '>'){
        switch(command[counter + 2]){
          case 'R':   switch(command[counter + 3]){
                        case 'T':     switch(command[counter + 4]){
                                        case 'M':   if(command[counter + 5] == 'P'){          //Return temperatures
                                                      returnTime(unitUptime) ;
                                                      Serial.print(", Temperatures - Board temp: ") ;
                                                      Serial.print(boardTemp) ;
                                                      Serial.print("C, PSU temp: ") ;
                                                      Serial.print(psuTemp) ;
                                                      Serial.print("C, Rubidium module temp: ") ;
                                                      Serial.print(rbTemp) ;
                                                      Serial.println("C") ;
                                                      Serial.println() ;
                                                    }
                                                    else{
                                                      invalidCommand(command) ;
                                                      return ;
                                                    }
                                                    break ;
                                        case 'S':   if(command[counter + 5] == 'T'){          //Return temperature statistics
                                                      returnTime(unitUptime) ;
                                                      Serial.print(", Temperature stats - board max: ") ;
                                                      Serial.print(boardMax) ;
                                                      Serial.print("C, board min: ") ;
                                                      Serial.print(boardMin) ;
                                                      Serial.println("C") ;
                                                      Serial.print("PSU max: ") ;
                                                      Serial.print(psuMax) ;
                                                      Serial.print("C, PSU min: ") ;
                                                      Serial.print(psuMin) ;
                                                      Serial.println("C") ;
                                                      Serial.print("Rubidium module max: ") ;
                                                      Serial.print(rbMax) ;
                                                      Serial.print("C, Rb module min: ") ;
                                                      Serial.print(rbMin) ;
                                                      Serial.println("C") ;
                                                      Serial.println() ;
                                                    }
                                                    else{
                                                      invalidCommand(command) ;
                                                      return ;
                                                    }
                                                    break ;
                                        case 'U':   if(command[counter + 5] == 'T'){          //Return uptimes          
                                                      Serial.print("Uptimes - Unit: ") ;
                                                      returnTime(unitUptime) ;
                                                      Serial.println() ;
                                                      Serial.print("GPS: ") ;
                                                      returnTime(gpsUptime) ;
                                                      Serial.println() ;
                                                      Serial.print("Rubidium: ") ;
                                                      returnTime(rbUptime) ;
                                                      Serial.println() ;
                                                      Serial.print("GPS to Rb count difference: ") ;
                                                      Serial.println(countDifferential) ;
                                                      Serial.print("GPS unlock count: ") ;
                                                      Serial.println(gpsUnlocks) ;
                                                      Serial.println() ;
                                                    }
                                                    else{
                                                      invalidCommand(command) ;
                                                      return ;
                                                    }
                                                    break ;
                                        default:    invalidCommand(command) ;
                                                    return ;
                                      }
                                      break ;
                        case 'S':     switch(command[counter + 4]){
                                        case 'E':   if(command[counter + 5] == 'R'){          //Return serial mode
                                                      returnTime(unitUptime) ;
                                                      Serial.print(", serial connected to ") ;
                                                      if(serialMode == 0)
                                                        Serial.println("monitor") ;
                                                      else if(serialMode == 1)
                                                        Serial.println("rubidium module") ;
                                                      else if(serialMode == 2)
                                                        Serial.println("GPS") ;
                                                      Serial.println() ;
                                                    }
                                                    else{
                                                      invalidCommand(command) ;
                                                      return ;
                                                    }
                                                    break ;
                                        case 'T':   if(command[counter + 5] == 'P'){          //Reset temperature statistics
                                                      rbMin = 100 ;
                                                      rbMax = -100 ;
                                                      boardMin = 100 ;
                                                      boardMax = -100 ;
                                                      psuMin = 100 ;
                                                      psuMax = -100 ;
                                                      bTempLED = false ;
                                                      bHadTempError = false ;
                                                      returnTime(unitUptime) ;
                                                      Serial.println(", temp stats reset") ;
                                                      Serial.println() ;
                                                    }
                                                    else{
                                                      invalidCommand(command) ;
                                                      return ;
                                                    }
                                                    break ;
                                        default:    invalidCommand(command) ;
                                                    return ;
                                      }
                                      break ;
                        case 'L':     if(command[counter + 4] == 'C' && command[counter + 5] == 'K'){          //Restart locking and settling
                                        runMode = 1 ;
                                        bChangeMode = true ;
                                        gpsUptime = 0 ;
                                        rbUptime = 0 ;
                                        bHadTempError = false ;
                                        gpsUnlocks = 0 ;
                                        returnTime(unitUptime) ;
                                        Serial.println(", relocking and resettling...") ;
                                        Serial.println() ;
                                        return ;
                                      }
                                      else{
                                        invalidCommand(command) ;
                                        return ;
                                      }
                                      break ;
                        case 'F':     if(command[counter + 4] == 'U' && command[counter + 5] == 'L'){          //Return full info
                                         Serial.print("Full report - Operating mode: ") ;
                                         Serial.print(runMode) ;
                                         Serial.print(" - ") ;
                                         switch(runMode){
                                            case 1:   Serial.println("Waiting for GPS/Rb Locks") ;
                                                      break ;
                                            case 2:   Serial.println("Warming up") ;
                                                      break ;
                                            case 3:   Serial.println("Settling") ;
                                                      break ;
                                            case 4:   Serial.println("Stabilizing") ;
                                                      break ;
                                            case 5:   Serial.println("Normal operation") ;
                                                      break ;
                                            default:  Serial.println("Error") ;
                                                      break ;
                                         }
                                         Serial.print("Unit uptime: ") ;
                                         returnTime(unitUptime) ;
                                         Serial.println() ;
                                         Serial.print("GPS uptime: ") ;
                                         returnTime(gpsUptime) ;
                                         Serial.println() ;
                                         Serial.print("Rubidium uptime: ") ;
                                         returnTime(rbUptime) ;
                                         Serial.println() ;
                                         Serial.print("GPS to Rb count difference: ") ;
                                         Serial.println(countDifferential) ;
                                         Serial.print("GPS unlock count: ") ;
                                         Serial.println(gpsUnlocks) ;
                                         Serial.print("Board temp: ") ;
                                         Serial.print(boardTemp) ;
                                         Serial.print("C, PSU temp: ") ;
                                         Serial.print(psuTemp) ;
                                         Serial.print("C, Rubidium temp: ") ;
                                         Serial.print(rbTemp) ;
                                         Serial.println("C") ;
                                         Serial.print("Board max temp: ") ;
                                         Serial.print(boardMax) ;
                                         Serial.print("C, board min: ") ;
                                         Serial.print(boardMin) ;
                                         Serial.println("C") ;
                                         Serial.print("PSU max: ") ;
                                         Serial.print(psuMax) ;
                                         Serial.print("C, PSU min: ") ;
                                         Serial.print(psuMin) ;
                                         Serial.println("C") ;
                                         Serial.print("Rubidium module max: ") ;
                                         Serial.print(rbMax) ;
                                         Serial.print("C, Rb module min: ") ;
                                         Serial.print(rbMin) ;
                                         Serial.println("C") ;
                                         Serial.print("Serial connected to ") ;
                                         if(serialMode == 0)
                                           Serial.println("monitor") ;
                                         else if(serialMode == 1)
                                           Serial.println("rubidium module") ;
                                         else if(serialMode == 2)
                                           Serial.println("GPS") ;
                                         Serial.println() ;
                                      }
                                      else{
                                        invalidCommand(command) ;
                                        return ;
                                      }
                                      break ;
                        default:      invalidCommand(command) ;
                                      return ;
                      }
                      break ;
          case 'S':   switch(command[counter + 3]){
                        case 'S':     if(command[counter + 4] == 'E' && command[counter + 5] == 'R'){          //Switch serial mode
                                        serialMode++ ;
                                        bChangeSerial = true ;
                                        if(serialMode > 2)
                                          serialMode = 0 ;
                                        returnTime(unitUptime) ;
                                        Serial.print(", serial switched to ") ;
                                        if(serialMode == 0)
                                          Serial.println("monitor") ;
                                        else if(serialMode == 1)
                                          Serial.println("rubidium module") ;
                                        else if(serialMode == 2)
                                          Serial.println("GPS") ;
                                        Serial.println() ;
                                        return ;
                                      }
                                      else{
                                        invalidCommand(command) ;
                                        return ;
                                      }
                                      break ;
                        default:      invalidCommand(command) ;
                                      return ;
                      }
                      break ;
          default:    invalidCommand(command) ;
                      return ;
        }
      }
    }
  }
  else{
    if(serialMode == 1){
      for(counter = 0; counter < commandBuffer; counter++){
        if(!isAscii(command[counter]))
          command[counter] = ' ' ;
      }
      rbPort.print(command) ;
      rbPort.flush() ;
    }
    else if(serialMode == 2){
      gpsPort.print(command) ;
      gpsPort.flush() ;
    }
  }
}

void invalidCommand(char* command){
  returnTime(unitUptime) ;
  Serial.print(", invalid command: ") ;
  Serial.println(command) ;
  Serial.println() ;
}

void returnTime(long toReturn){
  long days = toReturn / 86400 ;
  long hours = (toReturn - (days * 86400)) / 3600 ;
  long minutes = (toReturn - (days * 86400) - (hours * 3600)) / 60 ;
  
  Serial.print(days) ;
  Serial.print("d ") ;
  Serial.print(hours) ;
  Serial.print("h ") ;
  Serial.print(minutes) ;
  Serial.print("m ") ;
  Serial.print(toReturn % 60) ;
  Serial.print("s") ;
}

