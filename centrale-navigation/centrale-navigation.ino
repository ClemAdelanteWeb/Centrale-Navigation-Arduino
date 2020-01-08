#include <math.h>
#include <Thread.h>;
#include <AltSoftSerial.h>

#define WindSpeedSensorPin 2 // The pin location of the anemometer sensor
#define WindDirectionSensorPin A2 // The pin location of the anemometer sensor
 
//wind direction

int VaneValue;// raw analog value from wind vane
int Direction;// translated 0 - 360 direction,
int CalDirection;// converted value with offset applied
int LastValue;
char nmeawinddirection[100];
byte checksum=0;
byte start_with = 0;
byte end_with = 0;
const int Offset = 53;
float WindDirectionAvrg; // speeed average 


// Wind speed declaration

volatile unsigned long pulses; // cup rotation counter used in interrupt routine
unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine

float WindSpeed; // speed miles per hour
float WindSpeedAvrg; // speeed average 
float WindSpeedMax;
float FX;
int maxSpeedShow = 0;

// Précédente valeur de millis()
unsigned long previousMillis = 0;

// Interval d'affichage du vent en ms
const unsigned long WIND_INTERVAL = 1000;

//Serial reception
AltSoftSerial Nmea2;

char c1, c2, c3, c4;
String Buffer1, Buffer2, Buffer3, Buffer4;
const unsigned int MaxNmeaSentenceLength = 82;

void setup() {
  Serial.begin(38400);
  Serial1.begin(4800); // NMEA 1 (la plus rapide) => GPS MLR
  Serial2.begin(9600); // GPS 
  Serial3.begin(38400); // AIS
  Nmea2.begin(4800); // Sondeur
  
  pinMode(WindSpeedSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WindSpeedSensorPin), rotation, FALLING);
  //Serial.println("Rotations\tMPH");
}

void loop() {

   // Récupére la valeur actuelle de millis()
  unsigned long currentMillis = millis();
  
  // Interval d'affichage du vent
  if(currentMillis - previousMillis >= WIND_INTERVAL) {
    
    // Garde en mémoire la valeur actuelle de millis()
    previousMillis = currentMillis;
  
  

    // Rotations = 0; // Set Rotations count to 0 ready for calculations
    
    
    // convert to mp/h using the formula V=P(2.25/T)
    // V = P(2.25/3) = P * 0.75
    Rotations = (float) pulses / 2; // 2 pulses per rotation
    
    //Rotations = Rotations/2; // toutes les 500ms
    FX = 0.4;
    
    
    WindSpeed = Rotations/0.795;// m/S
    //WindSpeed = round(WindSpeed*10)/10;
    
    if(WindSpeed > WindSpeedMax) {
    WindSpeedMax = WindSpeed;
    }
    
    //WindSpeed = (WindSpeed*3600)/1852; // mile / h = nds
    //Serial.print(Rotations); Serial.print(" Tr/s \t");
    
    WindSpeedAvrg = runningAverage(WindSpeed*10);
    WindSpeedAvrg = WindSpeedAvrg/10;
    
   
    
    // wind direction
     VaneValue = analogRead(WindDirectionSensorPin);
    Direction = map(VaneValue, 0, 1023, 0, 360);
    CalDirection = Direction + Offset;
    
    if(CalDirection > 360)
    CalDirection = CalDirection - 360;
    
    if(CalDirection < 0)
    CalDirection = CalDirection + 360;
    
    // Only update the display if change greater than 2 degrees.
    //if(abs(CalDirection - LastValue) > 5)
    //{
    // Serial.println(WindSpeedAvrg);

    String sentence;
    sentence = F("WIMWV,");
    sentence += (String) CalDirection;
    sentence += F(",R,");
    sentence += (String) WindSpeedAvrg;
    sentence += F(",N,A");

uint8_t payload[sentence.length()]; 
sentence.getBytes(payload, sentence.length()+1);


    //char tempSentenceChar = sentence;

    int checksum = 0;
    for (byte x = 0; x<sentence.length(); x++){ // XOR chars between '$' and '*'
      checksum = checksum ^ payload[x];
    }   


    String finalSentence;
    finalSentence = F("$");
    finalSentence += sentence;
    finalSentence += F("*");
    finalSentence += String(checksum, HEX);

    // sprintf(nmeaWindSentence, "$WIMWV,%d,R,%d,N,A", CalDirection, (int)(WindSpeedAvrg*100)%100);
     //Serial.print("$WIMWV,");
     //Serial.print(CalDirection);
     //Serial.print(",R,");
     //Serial.print(WindSpeedAvrg);
     //Serial.print(",N,A,,\n");
     Serial.println(finalSentence);

    
  pulses = 0;
  
  }

  // --------
  // SERIAL et NMEA Multiplexer

  // Serial 1
  if (Serial1.available()) {
    c1 = Serial1.read();
  
    if (Buffer1.length() < MaxNmeaSentenceLength) {
      Buffer1 += c1;
    }
    
    if (c1 == '\n') { 
      Serial.print(Buffer1);
      Buffer1 = "";
    }
  }

  // Serial 2
  if (Serial2.available()) {
    c2 = Serial2.read();
  
    if (Buffer2.length() < MaxNmeaSentenceLength) {
      Buffer2 += c2;
    }
    
    if (c2 == '\n') { 
      Serial.print(Buffer2);
      Buffer2 = "";
    }
  }

  // Serial 3
  if (Serial3.available()) {
    c3 = Serial3.read();
  
    if (Buffer3.length() < MaxNmeaSentenceLength) {
      Buffer3 += c3;
    }
    
    if (c3 == '\n') { 
      Serial.print(Buffer3);
      Buffer3 = "";
    }
  }

  // SoftSerial
  if (Nmea2.available()) {
    c4 = Nmea2.read();
  
    if (Buffer1.length() < MaxNmeaSentenceLength) {
      Buffer4 += c4;
    }
    
    if (c4 == '\n') { 
      Serial.print(Buffer4);
      Buffer4 = "";
    }
  }

}
void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
       frac = (val - int(val)) * precision;
   else
       frac = (int(val)- val ) * precision;
   Serial.print(frac,DEC) ;
} 
// This is the function that the interrupt calls to increment the rotation count
void rotation () {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    pulses++;
    ContactBounceTime = millis();
  }
} 


long runningAverage(int M) {
  #define LM_SIZE 6
  static int LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum / count;
}

uint8_t getChecksum(char *string)
{
  int XOR = 0;  
  for (int i = 0; i < strlen(string); i++) 
  {
    XOR = XOR ^ string[i];
  }
  return XOR;
}
