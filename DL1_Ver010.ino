 #include <Average.h>
#include <Bounce.h>
#include <EEPROM.h>
#include <pgmspace.h>


/*
Program for communicating between a DL1 data logger and Arduino mega board
Communication is done on Serial2 port Serial port was used to debug reading the DL1 data
Program by 
Colin Byrne colinbyrne105@optusnet.com.au
Justin Mitchell

*/

// maximum message length from msg_len[]
#define MAX_LEN    42
#define MAX_MESS  107

//Assign Input Pins No
#define BUTTONIN1   22
#define BUTTONIN2   21
#define BUTTONIN3   24
#define BUTTONIN4   28
#define BUTTONIN5   26
#define BUTTONIN6   20
#define DIGITALIN1  19

// Assign Output Pins No
#define RelayOut1  13
#define RelayOut2  12
#define RelayOut3  11
#define RelayOut4  10
#define RelayOut5  9
#define RelayOut6  8
#define BuzzerOut1  52

// Header addresses for Analoge Output messages
#define HeaderAnalogue9   35
#define HeaderAnalogue10  33
#define HeaderAnalogue11  34
#define HeaderAnalogue12  32
#define HeaderAnalogue13  31
#define HeaderAnalogue14  29
#define HeaderAnalogue15  30
#define HeaderAnalogue16  28
#define HeaderAnalogue17  36
#define HeaderAnalogue18  37
#define HeaderAnalogue19  38
#define HeaderAnalogue20  39
#define HeaderAnalogue21  40
#define HeaderAnalogue22  41

// Misc defines
#define FuelRateArraySize 1024

// Global Variables for Analog inputs
int analogInput1;
int analogInput2;
int analogInput3;
int analogInput4;
int analogInput5;
int analogInput6;
int analogInput7;
int analogInput8;
int analogInput9;

// Variables used to decode serial data from DL1
int buffer[MAX_LEN] = {0};       //Our Buffer 
unsigned int bytesInBuffer = 0;  //Keep track of bytes in Our Buffer
byte messageLengths[]={9,11,0,7,21,6,6,6,5,14,10,3,0,5,5,5,5,5,0,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,0,11,6,10,10,10,11,11,11,11,11,3,5,30,11,4,4,42,42,3,5,5,5,6,24,3,6,4,4,5,5,5,5,10,5,5,5,5,6,5,4,5,6,5,10,8,0,0,0,19,0,17,9,11,0,0};
int currentMessageLength = 0;    //number of bytes in the current message
//int i;                           //generic, used for all for loops
int bytesNeededInBuffer;         //Amount of bytes needed in buffer to complete message
byte calculatedCheckSum;         //checksum calculated from bytes in message

// Variables used to decode serial data from Dash 3
int first_byte2 = 0; //
int serial_available2 = 0;
int trash;
int buffer2[4] = {0};
int bc2 = 0;  // buffer count
int x2 = 0;
int serialReadCount = 0;

//
float timeStamp = 0;
//int gpsSpeed = 0;
int  UnitControlChannel = 0;

//Variables used for Distance Claculation from pulse count message
unsigned long pulseCount1 = 0, pulseCountMem1 = 0, pulseCountDiff1 = 0, pulseCountTotal = 0;
unsigned long distance1 = 0;
int wheelSpeedTeeth = 20;
int rollingCircumfrance = 1960;  //in mm
unsigned long gpsSpeed = 0;

//Variables used for Fuel Claculation from pulse count message
long pulseCountFuel = 0;
unsigned long fuelPulseMillsMem = 0;
int FuelPulseDuration = 0;
long FuelPulseMax = 59871; //55L x 1270Pulses = 69850
int FuelRateArray[FuelRateArraySize];
int FuelAverageArray[255];
int FuelRate = 0;
int FuelAverage = 0;
int FuelRateCounter = 0;  //records when to write the average rate to the average array
int fuelPulsesPerLitre  = 1088;

//Variables used for Fuel Rate Calculation
long fuelPulsesPerDistance = 936025; //(((1000*1000*100/rollingCircumfrance)*wheelSpeedTeeth)/fuelPulsesPerLitre)*1000;
unsigned long pulseCount2 = 0, pulseCountMem2 = 20, pulseCountDiff2 = 0, pulseCountTotal2 = 0;

  
//Assign Bounce objects
Bounce BounceButton1 = Bounce( BUTTONIN1,50 );
Bounce BounceButton2 = Bounce( BUTTONIN2,50 ); 
Bounce BounceButton3 = Bounce( BUTTONIN3,50 ); 
Bounce BounceButton4 = Bounce( BUTTONIN4,50 );
Bounce BounceButton5 = Bounce( BUTTONIN5,50 );
Bounce BounceButton6 = Bounce( BUTTONIN6,50 );

//Variables used eeprom storage during power off
int distaneEEPROMAddres = 0;
int fuelCountEEPROMAddres = 4;
boolean saveFlag = 0;
int saveDelay = 1000;
int saveDelayCounter = 0;

//Variables Used for SOTimer
long millisMemSOTimer = 0;
long SOTimerDelay = 5000;  //5 Seconds
boolean SOTimerStartFlag = false;


//Variables used for Go Pro
boolean GoProRecording = 0;
boolean GoProButtonPressed = 0;
int GoProOnDelayMS = 2000;
int GoProOffDelayMS = 4000;
long GoProTimmer = 0;

//Variables used for Thermostat
int ThermoFanTempOn = 247;   //Raw Value for On Temp = 80.0 deg C
int ThermoFanTempOff = 253;   //Raw Value for Off Temp = 78.6 deg C
int TempMin;
int TempMax;
int TempDiff;
int ThermoFanOn = 0;
int ThermoFanOnSpeed = 80;

// Bit Value 206.,208.,210.,212.,215.,217.,219.,221.,223.,225.,227.,229.,231.,233.,235.,237.,239.,241.,243.,245.,247.,249.,251.,253.,256.,258.,260.,262.,264.,266.,268.,270.,272.,274.,276.,278.,280.
// Voltage   1.01,1.02,1.03,1.04,1.05,1.06,1.07,1.08,1.09,1.10,1.11,1.12,1.13,1.14,1.15,1.16,1.17,1.18,1.19,1.20,1.21,1.22,1.23,1.24,1.25,1.26,1.27,1.28,1.29,1.30,1.31,1.32,1.33,1.34,1.35,1.36,1.37
// Temp      89.3,88.9,88.5,88.1,87.7,87.3,87.0,86.6,86.2,85.8,85.4,85.1,84.7,84.3,84.0,83.6,83.2,82.9,82.5,82.1,81.8,81.4,81.1,80.7,80.4,80.0,79.6,79.3,78.9,78.6,78.2,77.9,77.6,77.2,76.9,76.5,76.2

int tempConvert[] ={371,1368,1365,1362,1359,1357,1354,1351,1348,1346,1343,1340,1337,1335,1332,1329,1327,1324,1321,1318,1316,1313,1310,1308,1305,1302,1300,1297,1294,1292,1289,1286,1284,1281,1279,1276,1273,1271,1268,
1266,1263,1260,1258,1255,1253,1250,1247,1245,1242,1240,1237,1235,1232,1230,1227,1225,1222,1220,1217,1215,1212,1210,1207,1205,1202,1200,1197,1195,1192,1190,1187,1185,1182,1180,1178,1175,1173,1170,
1168,1165,1163,1161,1158,1156,1153,1151,1149,1146,1144,1142,1139,1137,1135,1132,1130,1127,1125,1123,1121,1118,1116,1114,1111,1109,1107,1104,1102,1100,1097,1095,1093,1091,1088,1086,1084,1082,1079,
1077,1075,1073,1070,1068,1066,1064,1062,1059,1057,1055,1053,1050,1048,1046,1044,1042,1040,1037,1035,1033,1031,1029,1027,1024,1022,1020,1018,1016,1014,1012,1010,1007,1005,1003,1001,999,997,995,
993,991,989,987,984,982,980,978,976,974,972,970,968,966,964,962,960,958,956,954,952,950,948,946,944,942,940,938,936,934,932,930,928,926,924,922,920,918,916,
914,912,911,909,907,905,903,901,899,897,895,893,891,889,888,886,884,882,880,878,876,874,873,871,869,867,865,863,862,860,858,856,854,852,851,849,847,845,843,
841,840,838,836,834,832,831,829,827,825,824,822,820,818,817,815,813,811,810,808,806,804,803,801,799,797,796,794,792,791,789,787,785,784,782,780,779,777,775,
774,772,770,769,767,765,764,762,760,759,757,755,754,752,750,749,747,745,744,742,741,739,737,736,734,732,731,729,728,726,724,723,721,720,718,716,715,713,712,
710,709,707,705,704,702,701,699,698,696,695,693,691,690,688,687,685,684,682,681,679,678,676,675,673,672,670,669,667,666,664,663,661,660,658,657,655,654,652,
651,649,648,646,645,643,642,641,639,638,636,635,633,632,630,629,628,626,625,623,622,620,619,618,616,615,613,612,610,609,608,606,605,603,602,601,599,598,596,
595,594,592,591,590,588,587,585,584,583,581,580,579,577,576,574,573,572,570,569,568,566,565,564,562,561,560,558,557,556,554,553,552,550,549,548,546,545,544,
542,541,540,538,537,536,535,533,532,531,529,528,527,525,524,523,522,520,519,518,516,515,514,513,511,510,509,508,506,505,504,502,501,500,499,497,496,495,494,
492,491,490,489,487,486,485,484,482,481,480,479,477,476,475,474,472,471,470,469,468,466,465,464,463,461,460,459,458,457,455,454,453,452,451,449,448,447,446,
444,443,442,441,440,438,437,436,435,434,433,431,430,429,428,427,425,424,423,422,421,419,418,417,416,415,414,412,411,410,409,408,406,405,404,403,402,401,399,
398,397,396,395,394,392,391,390,389,388,387,385,384,383,382,381,380,379,377,376,375,374,373,372,370,369,368,367,366,365,364,362,361,360,359,358,357,356,354,
353,352,351,350,349,348,346,345,344,343,342,341,340,338,337,336,335,334,333,332,331,329,328,327,326,325,324,323,321,320,319,318,317,316,315,314,312,311,310,
309,308,307,306,304,303,302,301,300,299,298,297,295,294,293,292,291,290,289,288,286,285,284,283,282,281,280,279,277,276,275,274,273,272,271,269,268,267,266,
265,264,263,262,260,259,258,257,256,255,254,253,251,250,249,248,247,246,245,243,242,241,240,239,238,237,236,234,233,232,231,230,229,228,226,225,224,223,222,
221,220,218,217,216,215,214,213,212,210,209,208,207,206,205,204,202,201,200,199,198,197,195,194,193,192,191,190,189,187,186,185,184,183,182,180,179,178,177,
176,175,173,172,171,170,169,168,166,165,164,163,162,161,159,158,157,156,155,153,152,151,150,149,148,146,145,144,143,142,140,139,138,137,136,134,133,132,131,
130,128,127,126,125,124,122,121,120,119,118,116,115,114,113,112,110,109,108,107,105,104,103,102,100,99,98,97,96,94,93,92,91,89,88,87,86,84,83,
82,81,79,78,77,76,74,73,72,71,69,68,67,66,64,63,62,60,59,58,57,55,54,53,51,50,49,48,46,45,44,42,41,40,39,37,36,35,33,
32,31,29,28,27,25,24,23,21,20,19,17,16,15,13,12,11,9,8,7,5,4,3,1,0,-1,-3,-4,-5,-7,-8,-9,-11,-12,-14,-15,-16,-18,-19,
-20,-22,-23,-25,-26,-27,-29,-30,-32,-33,-34,-36,-37,-39,-40,-41,-43,-44,-46,-47,-49,-50,-51,-53,-54,-56,-57,-59,-60,-61,-63,-64,-66,-67,-69,-70,-72,-73,-75,
-76,-78,-79,-80,-82,-83,-85,-86,-88,-89,-91,-92,-94,-95,-97,-98,-100,-101,-103,-104,-106,-107,-109,-110,-112,-114,-115,-117,-118,-120,-121,-123,-124,-126,-127,-129,-130,-132,-134
-135,-137,-138,-140,-141,-143,-145,-146,-148,-149,-151,-153,-154,-156,-157,-159,-161,-162,-164,-165,-167,-169,-170,-172,-174,-175,-177,-178,-180,-182,-183,-185,-187,-188,-190,-192,-193,-195,-197
-198,-200,-202,-203,-205,-207,-208,-210,-212,-213};


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  //Serial.begin(115200);  //used for debugging
  Serial2.begin(115200); //Main serial port - DL1
  Serial3.begin(115200); //Secondary   port - Dash Buttons

cli();//stop interrupts

//set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 137;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
sei();//allow interrupts

// Setup Output Pins
  pinMode(RelayOut1, OUTPUT); 
  pinMode(RelayOut2, OUTPUT); 
  pinMode(RelayOut3, OUTPUT);
  pinMode(RelayOut4, OUTPUT);
  pinMode(RelayOut5, OUTPUT);
  pinMode(RelayOut6, OUTPUT);  
  pinMode(BuzzerOut1, OUTPUT);
  
// Setup Input   Pins
  pinMode(BUTTONIN1,INPUT);  // Reset Distance
  pinMode(BUTTONIN2,INPUT);  // Increase Fuel 
  pinMode(BUTTONIN3,INPUT);  // Decrease Fuel
  pinMode(BUTTONIN4,INPUT);  // Fuel Multiplier
  pinMode(BUTTONIN5,INPUT);  // Go pro start
  pinMode(BUTTONIN6,INPUT);  // Reset power Delay
  pinMode(DIGITALIN1,INPUT);  //GoProInput
   
// Setup Interupt Pins
 attachInterrupt(5, FuelCount, RISING);
 
// Read Data Saved in EEPROM
pulseCountTotal = ReadDistanceEEPROM(distaneEEPROMAddres);
pulseCountFuel = ReadFuelCountEEPROM(fuelCountEEPROMAddres);
digitalWrite(RelayOut6, LOW);
} //End Setup
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// the loop routine runs over and over again forever:
void loop() {
  
  
//update the  Button debouncers
BounceButton1.update ( );
BounceButton2.update ( );
BounceButton3.update ( );
BounceButton4.update ( );
BounceButton5.update ( );
BounceButton6.update ( );

  
    // read the input on analog pin 0:
analogInput1 = analogRead(A0);
analogInput2 = analogRead(A2);
analogInput3 = analogRead(A4);
analogInput4 = analogRead(A6);
analogInput5 = analogRead(A12);
analogInput6 = analogRead(A10);
analogInput7 = analogRead(A8);
analogInput8 = analogRead(A14);
analogInput9 = analogRead(A15);

sendAnalogMessage(HeaderAnalogue9, analogInput1);   //Lambda 2
sendAnalogMessage(HeaderAnalogue10, analogInput2);  //Lambda 3
sendAnalogMessage(HeaderAnalogue11, analogInput3);  //Spare
sendAnalogMessage(HeaderAnalogue12, analogInput4);  //Spare
sendAnalogMessage(HeaderAnalogue13, analogInput5);  //Head Temp Front
sendAnalogMessage(HeaderAnalogue14, analogInput6);  //Head Temp Middle
sendAnalogMessage(HeaderAnalogue15, analogInput7);  //Head Temp back
sendAnalogMessage(HeaderAnalogue16, analogInput8);  //Spare
sendAnalogMessage(HeaderAnalogue17, analogInput9);  //Bat Voltage
//Dummy used for Fuel
//Dummy used for Fuel
//Dummy used for Fuel
sendFuelRateMessage(HeaderAnalogue21, TempDiff);  //Maximum Difference of Head Sent as Temp not as voltage
sendFuelRateMessage(HeaderAnalogue22, ThermoFanOn);  //Thermo Fan has been switched on by controller

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Re-Setable Odomoeter
if (pulseCountMem1 == 0){
  pulseCountMem1 = pulseCount1;
}
pulseCountDiff1 = (pulseCount1 - pulseCountMem1);
if (pulseCount1 > pulseCountMem1){        //check to make sure pulse count from DL1 hasn't reset
    pulseCountTotal = pulseCountTotal + pulseCountDiff1;
    saveFlag = 0;
    saveDelayCounter = 0;
  }
pulseCountMem1 = pulseCount1;

if ( BounceButton1.read()== LOW){    //check to see if reset button has been pressed
      pulseCountTotal = 0;
  }
 
distance1 = (pulseCountTotal/wheelSpeedTeeth)*rollingCircumfrance;
sendDistanceMessage(distance1);
//End of Re-Setable Odomoeter
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Fuel Flow Calculations
if ( BounceButton2.fallingEdge()== true){    //check to see if add fuel button has been pressed and has been released
      //digitalWrite(BuzzerOut1, HIGH);
      if ( BounceButton4.read()== LOW){
        pulseCountFuel = pulseCountFuel + (fuelPulsesPerLitre * 10);
      }else{
        pulseCountFuel = pulseCountFuel + fuelPulsesPerLitre;
      } 
   if( pulseCountFuel >= FuelPulseMax){
     pulseCountFuel = FuelPulseMax;
   }
  }
if ( BounceButton3.fallingEdge()== true){    //check to see if add fuel button has been pressed
      //digitalWrite(BuzzerOut1, HIGH);
      if ( BounceButton4.read()== LOW){
        pulseCountFuel = pulseCountFuel - (fuelPulsesPerLitre *10);
      }else{
        pulseCountFuel = pulseCountFuel - fuelPulsesPerLitre;
      } 
   if( pulseCountFuel <= 0){
     pulseCountFuel = 0;
   }      
  }
 
FuelRate =  rollingAverage(FuelRateArray, FuelRateArraySize, (fuelPulsesPerDistance/pulseCount2));
FuelRateCounter = FuelRateCounter + 1;
if (FuelRateCounter >= FuelRateArraySize){
  FuelAverage =  rollingAverage(FuelAverageArray, 20, FuelRate);
  FuelRateCounter = 0;
}

//Serial.println((fuelPulsesPerDistance/pulseCount2));

sendFuelCountMessage(HeaderAnalogue18, pulseCountFuel);  //pulsecount3
sendFuelRateMessage(HeaderAnalogue19, FuelRate);
sendFuelRateMessage(HeaderAnalogue20, FuelAverage);
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Power off Delay
if ((analogInput9 < 50) && (SOTimerStartFlag == false)){      //check if power has been switched off
  millisMemSOTimer = millis();
  SOTimerStartFlag = true;
  }
  
if (analogInput9 >= 50){                                      //Reset if power is switched back on
  digitalWrite(RelayOut6, HIGH);
  SOTimerStartFlag = false;
  }

if ( (BounceButton6.read()== LOW) && (SOTimerStartFlag == true)){    //Override delay timer, save data and switch off unit
  SaveDistanceEEPROM(distaneEEPROMAddres,pulseCountTotal);
  saveFuelCountMessageEEPROM(fuelCountEEPROMAddres,pulseCountFuel);
  digitalWrite(RelayOut6, LOW);
  }

if ((SOTimerStartFlag == true) && (millis() > (SOTimerDelay + millisMemSOTimer))){    //Wait for delay timer, save data and switch off unit
  SaveDistanceEEPROM(distaneEEPROMAddres,pulseCountTotal);
  saveFuelCountMessageEEPROM(fuelCountEEPROMAddres,pulseCountFuel);
  digitalWrite(RelayOut6, LOW);
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////
//Go Pro Control
if ( BounceButton5.risingEdge()== true && digitalRead(DIGITALIN1) == LOW && GoProButtonPressed == 0){ //if Go Pro Is Not Recording 
     digitalWrite(RelayOut4, HIGH);
     GoProButtonPressed = 1;
     GoProTimmer = millis() + GoProOnDelayMS;
  } 
if ( BounceButton5.risingEdge()== true  && digitalRead(DIGITALIN1) == HIGH && GoProButtonPressed == 0){ //if Go Pro Is Recording 
     digitalWrite(RelayOut4, HIGH);
     GoProButtonPressed = 1;
     GoProTimmer = millis() + GoProOffDelayMS;
  }
if(millis() >= GoProTimmer){
  digitalWrite(RelayOut4, LOW);
  GoProButtonPressed = 0;
  }
  
////////////////////////////////////////////////////////////////////////////////////////////////////////
//Difference in Head Temperatures and Thermo Fan
TempMax = min(analogInput5,analogInput6);
TempMax = min(TempMax,analogInput7);
//TempDiff = TempMax;
if (TempMax <= ThermoFanTempOn) {
  digitalWrite(RelayOut5, HIGH);
  ThermoFanOn = 1000;
  }
if ((TempMax >= ThermoFanTempOff)||(gpsSpeed > ThermoFanOnSpeed)) {
  digitalWrite(RelayOut5, LOW);
  ThermoFanOn = 0;
  }
TempMax = tempConvert[TempMax];
//TempDiff = TempMax;
TempMin = max(analogInput5,analogInput6);
TempMin = max(TempMin,analogInput7);
TempMin = tempConvert[TempMin];
TempDiff = TempMax - TempMin;
} //end loop


//**************************************************************************************************************
//
//                  Start of Interupt Area
//
//***************************************************************************************************************

ISR(TIMER2_COMPA_vect){//timer1 interrupt to perform Decode of incomming serial data at 115200 kbps
//noInterrupts();
/***************************************************/
/*************   serial comm From DL1   ************/
 
if (bytesInBuffer == 0){            //Check if our buffer is empty
  if (Serial2.available()>=1){      //Check if serial buffer is empty
    buffer[0] = Serial2.read();     //read first byte into our buffer   
    bytesInBuffer = 1;              //Update buffer counter
    //Serial.println(1); 
  }else{
    goto endOfSerial;
  }//end if to check if serial buffer is empty
}//end if to check if there are bytes in our buffer

if(buffer[0] > MAX_MESS){//check to see if 1st byte could be a message
  moveBytesDown();
  goto endOfSerial;
  //Serial.println(2); 
  }//end check of message number check
  
currentMessageLength = messageLengths[(buffer[0]-1)];      //get message length from array
  
if(currentMessageLength == 0 ){//check to see if 1st byte could be a message
  moveBytesDown();
  goto endOfSerial;
  //Serial.println(3); 
  }//end check of message number check

if (currentMessageLength > bytesInBuffer){    //check if we have enough bytes in our buffer
  bytesNeededInBuffer = currentMessageLength-bytesInBuffer;
    if (Serial2.available()>=bytesNeededInBuffer){ //check if there are enough bytes in serial array
      int bytesInBufferOld = bytesInBuffer;
      for(int i=bytesInBufferOld; i < currentMessageLength; i++){    //for loop to add bytes to end of our array
        buffer[i]= Serial2.read();                
        bytesInBuffer=bytesInBuffer + 1;
      }//end for loop
    }else{
      goto endOfSerial;
    }  
}     
      
calculatedCheckSum = 0;                      //reset checksum
for (int i=0; i<(currentMessageLength-1); i++){      //for loop to sum checksum value
    calculatedCheckSum = calculatedCheckSum + buffer[i];
  }//end if checksum for loop
  
if (calculatedCheckSum == buffer[currentMessageLength-1]){ //if message has been proven 
  if (buffer[0]==89) {                                    //check to see if we want to use it
    pulseCount1 = decodePulseCount();
    }
  if (buffer[0]==11) {                                    //check to see if we want to use it
    gpsSpeed = decodeSpeed();
    }    
                                                            //if(bytesInBuffer > currentMessageLength){    //Check if there are any unused bytes in our buffer
                                                               //bytesInBuffer = 0;
                                                             //   for(int i=0; i=(bytesInBuffer-currentMessageLength); i++){        //if so move them down to be used
                                                             //   buffer[i]=buffer[i+currentMessageLength];
                                                             //   //digitalWrite(BuzzerOut1, HIGH);
                                                             //  } 
                                                             //  bytesInBuffer = 0;//bytesInBuffer-currentMessageLength;
                                                             //  //currentMessageLength = 255;
                                                             // }else{
                                                             // bytesInBuffer = 0;              //get rid of message for next one
                                                             // }
    bytesInBuffer = 0;
  }else{
    moveBytesDown();
  }

endOfSerial:

if (Serial2.available()>60){      //Check if serial buffer is full
  while(Serial2.available()>0) trash = Serial2.read();    //if full, clear buffer to ensure full messages are recorded
  digitalWrite(BuzzerOut1, LOW);
// end of serial comm (DL-1)

}

serialReadCount = serialReadCount + 1;
if (serialReadCount > 7200){  //serialReadCount runs a 14400 hz   
/**************************************************************/
/*************   serial comm From Dash 3 Buttons   ************/
  
  serial_available2 = Serial3.available();
  if (serial_available2 >= 5) { //message 91 should have 5 bytes
    first_byte2 = Serial3.peek();
    // Confirm this matches the address of the dash 3 Buttons - message 91
    if (first_byte2 == 91){
          for (int j=0; j<5; j++) {     // read in the data packet
             buffer2[j] = Serial3.read();
             UnitControlChannel = decodeUnitControlChannel();
             if (UnitControlChannel == 0x01){ //start logging
                 if (digitalRead(DIGITALIN1) == LOW && GoProButtonPressed == 0){ //if Go Pro Is Not Recording 
                     digitalWrite(RelayOut4, HIGH);
                     GoProButtonPressed = 1;
                     GoProTimmer = millis() + GoProOnDelayMS;
                    }
               }
             if (UnitControlChannel == 0x02){ //Stop logging    
                if (digitalRead(DIGITALIN1) == HIGH && GoProButtonPressed == 0){ //if Go Pro Is Recording 
                   digitalWrite(RelayOut4, HIGH);
                   GoProButtonPressed = 1;
                   GoProTimmer = millis() + GoProOffDelayMS;
                }
             }          
           UnitControlChannel = 0;  
          }
          
         
    } 
 trash = Serial3.read(); // no match, throw it out and try the next byte 
 serialReadCount = 0; //reset serial count
  }// end of serial comm (Dash 3)
}//end of if statement for count
//interrupts();
}//end of timmer 0 


//////////////////////////////////
//Function to Count Fuel Pulses and determine frequency
void FuelCount (void){
//  noInterrupts();
   if( pulseCountFuel <= 0){
     pulseCountFuel = 0;
   }else{ 
  pulseCountFuel = pulseCountFuel - 1;
   }
pulseCount2 = pulseCountTotal - pulseCountMem2;  //determine odometer ticks between fuel count ticks
pulseCountMem2 = pulseCountTotal;
//  interrupts();
}

//**************************************************************************************************************
//
//                  Start of Function Area
//
//***************************************************************************************************************

//Function To move bytes down in Local Serial Buffer

void moveBytesDown(void){
    for (int i=0; i<=bytesInBuffer; i++){
    buffer[i] = buffer[i+1];
    }//end of for loop to move bytes down
  bytesInBuffer=bytesInBuffer-1;
  }
  
//Function to Save Distance Data to EEPROM
  void SaveDistanceEEPROM(int EepromAddress, unsigned long distanceinMM) {
  int byte1 = 0;
  int byte2 = 0;
  int byte3 = 0;
  int byte4 = 0;
  byte1 = lowByte(distanceinMM >> 24);
  byte2 = lowByte(distanceinMM >> 16);
  byte3 = lowByte(distanceinMM >> 8);
  byte4 = lowByte(distanceinMM);
  EEPROM.write(EepromAddress,byte1);
  EEPROM.write(EepromAddress + 1,byte2);
  EEPROM.write(EepromAddress + 2,byte3);
  EEPROM.write(EepromAddress + 3,byte4);
}

//Function to Read Distance Data From EEPROM
 unsigned long  ReadDistanceEEPROM(int EepromAddress) {
  int byte1 = EEPROM.read(EepromAddress);
  int byte2 = EEPROM.read(EepromAddress + 1);
  int byte3 =  EEPROM.read(EepromAddress + 2);
  int byte4 = EEPROM.read(EepromAddress + 3); 
  return  (byte1 * pow(2,24) + byte2 * pow(2,16) +  byte3 * pow(2,8) + byte4);
}

//Function to Save FuelCount Data to EEPROM
// (Header, Byte1, Byte2, Checksum)
void saveFuelCountMessageEEPROM(int EepromAddress,long int sensorValue) {
  int byte1 = 0;
  int byte2 = 0;
  //sensorValue = map(sensorValue, 0, FuelPulseMax, 0, 55000);
  //sensorValue = sensorValue/1.2;
  byte2 = lowByte(sensorValue);
  byte1 = lowByte(sensorValue>>8);
  EEPROM.write(EepromAddress,byte1);
  EEPROM.write(EepromAddress + 1,byte2);
}

//Function to Read Fuel Count Data From EEPROM
 unsigned long  ReadFuelCountEEPROM(int EepromAddress) {
  int byte1 = EEPROM.read(EepromAddress);
  int byte2 = EEPROM.read(EepromAddress + 1);
  return  (byte1 * pow(2,8) + byte2);
}

//**************************************************************************************************************
//         Functions to Send Coded Message Data to DL1 or Dash 3
//***************************************************************************************************************

//Function to send out Analoge message in RaceTech. Format
// (Header, Byte1, Byte2, Checksum)
void sendAnalogMessage(int header,int sensorValue) {
  int byte1 = 0;
  int byte2 = 0;
  sensorValue = map(sensorValue, 0, 1023, 0, 5000);
  byte2 = lowByte(sensorValue);
  byte1 = lowByte(sensorValue>>8);
  Serial2.write(header);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(header+byte1+byte2);
}

//Function to send out Fuel message in RaceTech. Format
// (Header, Byte1, Byte2, Checksum)
void sendFuelCountMessage(int header,long sensorValue) {
  int byte1 = 0;
  int byte2 = 0;
  sensorValue = (sensorValue*1000)/fuelPulsesPerLitre;
  byte2 = lowByte(sensorValue);
  byte1 = lowByte(sensorValue>>8);
  Serial2.write(header);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(header+byte1+byte2);
}

//Function to send out Feul message in RaceTech. Format
// (Header, Byte1, Byte2, Checksum)
void sendFuelRateMessage(int header,long int sensorValue) {
  int byte1 = 0;
  int byte2 = 0;
  //sensorValue = map(sensorValue, 0, FuelPulseMax, 0, 55000);
  sensorValue = sensorValue;
  byte2 = lowByte(sensorValue);
  byte1 = lowByte(sensorValue>>8);
  Serial2.write(header);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(header+byte1+byte2);
}

//Function to send out Distance in mm in RaceTech. Format
// (Header, Byte1, Byte2, Byte3, Byte4, Checksum)
void sendDistanceMessage(unsigned long distanceinMM) {
  int header = 78;
  int byte1 = 0;
  int byte2 = 0;
  int byte3 = 0;
  int byte4 = 0;
  byte1 = lowByte(distanceinMM >> 24);
  byte2 = lowByte(distanceinMM >> 16);
  byte3 = lowByte(distanceinMM >> 8);
  byte4 = lowByte(distanceinMM);
  Serial2.write(header);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(byte3);
  Serial2.write(byte4);
  Serial2.write(header+byte1+byte2+byte3+byte4);
}
/*
//Function to send out Distance in mm in RaceTech. Format
// (Header, Byte1, Byte2, Byte3, Byte4, Checksum)
void sendFuelCountMessage(int header,unsigned long PulseCount) {
  int byte1 = 0;
  int byte2 = 0;
  int byte3 = 0;
  byte1 = lowByte(PulseCount >> 16);
  byte2 = lowByte(PulseCount >> 8);
  byte3 = lowByte(PulseCount);
  Serial2.write(header);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(byte3);
  Serial2.write(header+byte1+byte2+byte3);
}
*/
//Function to send out Unit Control Channel in racetech Format
// (Header, datalogger MSB, Datalogger LSB, command, Checksum)
void sendUnitControlChannel(int header, int byte1, int byte2, int command) {
  Serial2.write(header);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(command);
  Serial2.write(header+byte1+byte2+command);
}

//**************************************************************************************************************
//         Functions to decode Messages from DL1 or Dash 3
//***************************************************************************************************************
/*
Speed data
Channel Number	11
Total Length	10 bytes
Channel	Data1	Data2	Data3	Data4	Data5	Data6	Data7	Data8	Checksum

Speed (m/s) = (Data1 * 2^24 + Data2 * 2^16 + Data3 * 2^8 + Data4)* 0.01
SpeedAcc (m/s) =( Data6 * 2^16 + Data7 * 2^8 + Data8)* 0.01
*/

// if this is too slow then we can get rid of the floats

int decodeSpeed() {
  byte checksum = 0;
  for (int k=0; k<9; k++) {
    checksum += buffer[k];
  }
  if (checksum == buffer[9]) {
    return (buffer[1] * pow(2,24) + buffer[2] * 65536 + buffer[3] * 256 + buffer[4]) * 0.036; // was 0.01, but need to multiple by 3.6 to convert to km/hr, so just saving a calculation here
  } else { // invalid checksum
    return gpsSpeed;
  }
}

/*
Time Stamp Data
Channel Number	09
Total Length	5 bytes
Channel	Data1	Data2	Data3	Checksum
*/

float decodeTimeStamp() {
  float result = 0;
  byte checksum = 0;
  for (int k=0; k<4; k++) {
    checksum += buffer[k];
  }
  if (checksum == buffer[4]) {
    result = (buffer[1] * pow(2,16) + buffer[2] * pow(2,8) + buffer[3]);
    return result;
  } else { // invalid checksum
    return -1;
  }
}
  
/*
Unit Control Channel
Channel Number 91
Total Length	5 bytes
Channel	Data1	logger Serial No. MSDB    logger Serial No. LSB  Command Checksum
*/

int decodeUnitControlChannel() {
  int result = 0;
  byte checksum = 0;
  for (int k=0; k<4; k++) {
    checksum += buffer2[k];
  }
  if (checksum == buffer2[4]) {
    sendUnitControlChannel(buffer2[0],buffer2[1],buffer2[2],buffer2[3]);
    result = buffer2[3];  //0x01 = Start Logging, 0x02=Stop Logging, 0x03 = Add Marker
    return result;
  } else { // invalid checksum
    return -1;
  }
}  

/*
Pulse Count
Channel Number 86
Total Length	5 bytes
Channel	Data1 Data2 Data3 Checksum
pulsecount = Data1* 2^16 + Data2 * 2^8 + Data3
*/

unsigned long decodePulseCount() {
  unsigned long result = 0;
  byte checksum = 0;
  for (int k=0; k<4; k++) {
    checksum += buffer[k];
  }
  if (checksum == buffer[4]) {
    result = (buffer[1] * pow(2,16) + buffer[2] * pow(2,8) + buffer[3]);
    return result;
  } else { // invalid checksum
    return pulseCount1;
  }
}  
