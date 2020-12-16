// This #include statement was automatically added by the Particle IDE.
#include <neopixel.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include <SPI.h>

  



SYSTEM_MODE(AUTOMATIC);
//SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED); ///start setup/loop before connecting to Inet
boolean connectToCloud = false;

const char version[] = "climate_bench_test_deployment__v0.3.3";


// NeoPixel COUNT, PIN and TYPE
#define NUMPIXELS 8
#define PIXEL_TYPE WS2812B

//setup for time related variables
time_t epoch;
unsigned long now;
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
unsigned long lastSync = millis();

///set pins for thermocouple serial connection
int thermoCLK = D0;
int thermoCS = D1;
int thermoDO = D2;
int tempInC = 0;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

int CS= D5;   //set pin for Digital pot chip select
#define potPin A4  //set pin for slidepot 
#define PIXEL_PIN D7 //set pin for NeoPixel strip

#define CurrentIn1 A0 //set pin for current sensor
#define Conn D6 //set pin connect button


// Define a variable for storing string values
String Temp_String;
String sampletostring; 
String temperature_String;
String strval;



// We need to define each variable separately so we can access them using the Photon Cloud API
// These will be passed by reference into the Particle.varible() function. This could be an
// array, but for readability it's better to define them individually.
int Current_A0 = 0;
int Current_A0_pre = 0;
int Current_A1 = 0;
int sample = 0; // temp variable for storing analog input values
int  offset = 5800; // Offset in mA //////////////////////////////////////////////////////////////////////////////////////////////
int  offset150 = 21549; // Offset in mA for 150A sensor///////////////////////////////////////////////////////////////////////////
String sensor;
double quality;


int interval = 5; ///interval to publish data to cloud
#define Post_MILLIS (interval* 1000)
unsigned long lastPost = millis();


byte address = 0x00;
int i;              // loop variable
int value;          // analog read of potentiometer
int display_value;  // number of NeoPixels to display out of NUMPIXELS
int med_value;  // scaling pot value
int pot;
int potval;
int col;
int amp;
int spd;

Adafruit_NeoPixel pixels (NUMPIXELS, PIXEL_PIN, PIXEL_TYPE);

//STARTUP( setup_pins() );
void setup() {
    Serial.begin(9600);
    SPI.begin();
    pinMode (CS, OUTPUT);
    pinMode(potPin, INPUT_PULLUP);  // set pull-up on analog pin 
    pixels.begin(); // This initializes the NeoPixel library.
    //attachInterrupt(Conn, connect, FALLING);
    Particle.variable("Interval", interval);
    Particle.variable("version", version);
    Particle.variable("sensor", sensor);
    Particle.variable("signal", quality);
    Particle.variable("mA_Input", &Current_A1, INT);
    Particle.variable("Surface_Temp", &tempInC, INT);
    Particle.function("publish", postinterval);
    Particle.variable("pot", &value, INT);
    
    
}

void loop() {
    if (Particle.connected())                    //Only time sync when Cloud connected otherwise it will block
    {
    time_t lastSyncTimestamp;
    unsigned long lastSync = Particle.timeSyncedLast(lastSyncTimestamp);
    if (millis() - lastSync >= ONE_DAY_MILLIS)     //More than one day since last time sync
    {
        unsigned long cur = millis();
        Particle.syncTime();               //Request time synchronization from Particle Device Cloud
        waitUntil(Particle.syncTimeDone);  //Wait until Photon receives time from Particle Device Cloud or connection to Particle Device Cloud is lost
        if (Particle.timeSyncedLast() >= cur)          //Check if synchronized successfully
        {
            //do some daily clean up operations here
        }
    }
    else
    {
    
    }
    }
    potRead();
    strip();

    
    pixels.show(); // This sends the updated pixel color to the hardware.
    
    sample = analogRead(CurrentIn1);    // Get a reading from this A/D port
    //delay(50);  // Short delay for A/D settling
    delay(50);
    //Serial.print("Analog reading A1");  // Local debut output
    // Serial.println(sample); // Local debut output
    Current_A1 = AD_To_Current(sample, 80, 2);  // Call our conversion function to get current
    //Serial.print("Current A1"); // Send locally for serial debug
    //Serial.println(Current_A1); // Send locally for serial debug
    //Temp_String = String(Current_A1, DEC);      // Convert the current value to a string
    //Particle.publish("FrontPlate", Temp_String);    // Send the current value to Particle Console for a heartbeat
    tempInC = thermocouple.readCelsius();
    //Temp_String = String(tempInC, DEC);      // Convert the current value to a string
    //Particle.publish("IntTemp", Temp_String);    // Send the current value to Particle Console for a heartbeat
    epoch = Time.now();
    
    sensor = publishData(Current_A1,tempInC,epoch);

    delay(50);
}


int digitalPotWrite(int value)
{
digitalWrite(CS, LOW);
SPI.transfer(address);
SPI.transfer(value);
digitalWrite(CS, HIGH);
}

void strip(){
        // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one
    
    for(i=0; i<display_value; i++){
    
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        if (med_value > 91)
        {
        pixels.setPixelColor(i, 0, 0, 100); //Red
        }
        else if (med_value >= 85 && med_value <= 100)
        {
        pixels.setPixelColor(i, 0, 100, 0); //Green
        }
        else if (med_value < 100)
        {
        pixels.setPixelColor(i, 100, 0, 0); //Blue
        }
        else
        {
        
        }
    }
    for(i=display_value; i<NUMPIXELS; i++) {
      pixels.setPixelColor(i, 0, 0, 0);    // turn off all pixels after value displayed
    }
}

int AD_To_Current(int AD_Reading, float sensitivity, float scaling)
{
    int My_Current = 0;
    int Current = 0;
    
    // Sensor is 1000mA/80mv, or 12.5mA/mV. But we have a 1:2 voltage divider on it.
    // So sensor is really 1000mA/(80/2) = 25mA/mV
    // Photon is 3300mV/4095bits
    // Calculation is (n-bits * millivolts/bits) * mA/mV * voltage_divider_ratio
    Current = (((float(AD_Reading) * 3300)/4095) * (1000/sensitivity) * scaling); ////Current before subtracting offset//////////////////////////////////
    My_Current = int(Current - offset);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    return My_Current;
}


String publishData (int Current_A1,int tempInC,time_t epoch){
    
    String sensor = String::format(
      "{"
        "\"mA_Input\":%i,"
        "\"Surface_Temp\":%i,"
        "\"epochtime\":%lu"
      "}",
      Current_A1,
      tempInC,
      epoch
    );
    
    if (millis() - lastPost > Post_MILLIS) {
        if (Particle.connected())                    //Only post  when Cloud connected otherwise it will save
        {
        Particle.publish("Sensor", sensor, 60, PRIVATE);
        lastPost = millis();
        }
        else///save post somewhere
        {
            
        }
    }
    return sensor;
    
}

void connect() {
    Particle.connect();
    Serial.print("connectcheck");
}

void setup_pins()
{
    pinMode (CS, OUTPUT);
    pinMode(potPin, INPUT_PULLUP);  // set pull-up on analog pin A2
    potRead();
}


void potRead(){
    
    // Read PIN value and scale from 0 to NUMPIXELS -1
    value = analogRead(potPin);
    Serial.print(value);
    //Particle.publish("PotVal", strval, 60, PRIVATE);
    Serial.print(", ");
    med_value = map(value, 0, 4085, 127, 0);
    Serial.print(med_value);
    Serial.print(", ");
    display_value = map(med_value, 127, 0, 1, 8);
    Serial.println(display_value);
    //pot = map(med_value, 0, 205, 0, 127);
    spd = map(med_value, 127,0, -400, 400);
    amp = abs(spd);
    //col = map(med_value, 0, 453, 0, 255); 
    digitalPotWrite(med_value);
}


int postinterval(String command){  ///particle publish interval
    // Ritual incantation to convert String into Int
    char inputStr[64];
    command.toCharArray(inputStr,64);
    int i = atoi(inputStr);
    
    interval = i;
    // Respond
    return 1;
}
