/*
 * Project Boron4BeesStockwaage20
 * Description: Boron4BeesStockwaage20 - API-Key und Scalefactor werden über die serielle Schnittstelle konfiguriert und im EEPROM gespeichert.
 * Author: Dieter Metzler
 * Date: 08.09.2020
 * Updated: 28.01.2021
 */


#include "HX711.h"
#include "Adafruit_DHT.h"
#include "cloud4bees.h"

// Using SEMI_AUTOMATIC mode to get the lowest possible data usage by
// registering functions and variables BEFORE connecting to the cloud.
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

char API_KEY[] = "ABCDEFGHIJKLMNOP";
char api_key_Buffer[18] = {0}; //Der API-Key benötigt 16 Byte + 1 Byte. Ein Byte ist Reserve.  
int api_key_address = 0; // EEPROM starting address
String key = "";
//String channel_key = "3LU2W6GL3ZYSELBD";
String channel_key = "";

static int count = 0;

boolean waitForSetup = 0;
Timer timer(1000, proceedAfter5sec);

//HX711 Wägezellenverstärker
#define DOUT  A0
#define CLK  A1

HX711 scale(DOUT, CLK);

//String strScalefactor = "";
//String strOffset = "";

float valueNull = 0;
float valueKnownWeight = 0;
float offset = 0;
float scalefactor = 1;

//float weight = 0;

float floatWeight = 0;
String stringWeight = "";
int const num_weight_readings = 5;
float weight_readings[num_weight_readings];


int scalefactorEepromAdress = 30;
int offsetEepromAdress = 35;
long t;



#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define DHTPIN D3     // what pin we're connected to

/// Uncomment whatever type you're using!
//#define DHTTYPE DHT11		// DHT 11 
#define DHTTYPE DHT22		// DHT 22 (AM2302)
//#define DHTTYPE3 DHT21		// DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);


float temperature = 0;
int const num_temperature_readings = 10;
float temperature_readings[num_temperature_readings];

float humidity = 0;
int const num_humidity_readings = 10;
float humidity_readings[num_humidity_readings];


FuelGauge fuel;
int const num_soc_readings = 5; //number of instantaneous scale readings to calculate the median
float readings[num_soc_readings]; // create arry to hold readings
float soc; // Variable to keep track of LiPo state-of-charge (SOC)
float absolutminimumSoC = 20.0;
float minimumSoC = 30.0;
String stringSOC = "";


long sleepTimeSecs = 3540;
long sleepADay = 86400;

PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below.

// Use primary serial over USB interface for logging output
SerialLogHandler logHandler;

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
  wait5Seconds();

  // Apply a custom power configuration
  SystemPowerConfiguration conf;

  conf.powerSourceMaxCurrent(550) 
  .powerSourceMinVoltage(4300) 
  .batteryChargeCurrent(850) 
  .batteryChargeVoltage(4210);

  int res = System.setPowerConfiguration(conf); 
  Log.info("setPowerConfiguration=%d", res);
  // returns SYSTEM_ERROR_NONE (0) in case of success
  // Settings are persisted, you normally wouldn't do this on every startup.

  setupSerial();
  wait5Seconds();
  setupScale();
}

void loop() {

      scale.power_up();
      dht.begin();
      PMICSetup(); 
      wait5Seconds();    
      readSoC();
      //veryLowBattery();
      //batteryCheck();
      readWeight();
      readDHT();

      connectToCellular();
      connectToParticle();
      
      sendValuesToCloud();
      System.sleep( {}, {}, sleepTimeSecs); 
      
    
}



void setupSerial() {
  Serial.begin(9600);
  Serial.setTimeout(5000);
}

void setupScale() {
  
  //Abfrage ob ein Setup der Stockwaage durchgeführt werden soll.
  //Wenn nach 5 Sekunden keine Eingabe erfolgt wird das Programm weiter ausgeführt.
  timer.start();
  Serial.println("Do you want to setup the Scale? y/n");
  while (waitForSetup == 0) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
        timer.stop();
        Serial.println("Setup of the Scale");
        setupApiKey();
        calibrate();
        waitForSetup = 1;
      }
      else if (inByte
       == 'n') {
        timer.stop();
        Serial.println("No Setup of the Scale");
        waitForSetup = 1;
      }
    }
  }
  timer.stop();
}

void setupApiKey(){
    //Der API-Key für den entsprechenden Datenkanal kann über die serielle Schnittstelle (z.B. Putty) eingegeben werden.
    //Der API-Schlüssel wird dann dauerhaft im EEPROM gespeichert.
    Serial.println("Geben Sie den API_Key Ihres Datenkanales ein:");
    boolean f = 0;
    while (f == 0) {
      if (Serial.available() > 0) {
        String s = Serial.readStringUntil('\r');
        if (s != 0) {
          Serial.print("API_Key: ");
          Serial.println(s);
          f = 1;
          key = s;
        }
        else {
          Serial.println("Invalid value");
        }
        
      }
  }
    
  Serial.println("Key: " + key);
  key.toCharArray(API_KEY, 18);  
  Serial.println("API_Key_String: " + String(API_KEY));  

  EEPROM.put(api_key_address, API_KEY); // writing string value
  api_key_address = 0; // reset address

    
}

void proceedAfter5sec()
{
    //Diese Methode wird benötigt, um nach 5 Sekunden mit dem Programm fort zu fahren, wenn keine Eingabe über die serielle Schnittstelle erfolgt.
    count++;
    Serial.println("Waiting for Setup: ");
    if (count < 5) {
        
        waitForSetup = 0;
      } else
      {
         waitForSetup = 1;
      }
      
}

void calibrate() {

  scale.set_scale(scalefactor); //Adjust to this calibration factor
  //valueNull = scale.get_units();
  //Ermittlung des Medians zur Verbesserung der Genauigkeit
  for (int i = 0; i < num_weight_readings; i++) {
    weight_readings[i] = scale.get_units();  // fill the array with instantaneous readings from the scale
    Serial.println("Weight_0: " + String(weight_readings[i]));
    delay(100);
  }
  delay(100);
  
  valueNull = median(weight_readings,num_weight_readings);  //calculate median 

  Serial.println("Value_0: " + String(valueNull));
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("It is assumed that the mcu was started with no load applied to the load cell.");
  Serial.println("Now, place your known mass on the loadcell,");
  Serial.println("then send the weight of this mass (i.e. 100.0) from serial monitor.");
  float knownMass = 0;
  boolean f = 0;
  while (f == 0) {
    scale.set_scale();
    if (Serial.available() > 0) {
      knownMass = Serial.parseFloat();
      if (knownMass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(knownMass);
        f = 1;
      }
      else {
        Serial.println("Invalid value");
      }
    }
  }
  
  //float calibrationFactor = scale.get_units() / knownMass;
  Serial.println("Value_0: " + String(valueNull));

  //valueKnownWeight = scale.get_units();
  //Ermittlung des Medians zur Verbesserung der Genauigkeit
  for (int i = 0; i < num_weight_readings; i++) {
    weight_readings[i] = scale.get_units();  // fill the array with instantaneous readings from the scale
    Serial.println("ValueKnownWeight: " + String(weight_readings[i]));
    delay(100);
  }
  delay(100);
  
  valueKnownWeight = median(weight_readings,num_weight_readings);  //calculate median 

  Serial.println("Value_x: " + String(valueKnownWeight));
  scalefactor = (valueKnownWeight - valueNull) / knownMass;
  Serial.println("Calculated Scalefactor: " + String(scalefactor));
  offset = valueNull / scalefactor ;
  Serial.println("Calculated Offset: " + String(offset));
  //scale.set_scale(calibrationFactor); //Adjust to this calibration factor
  //Serial.print("Calculated calibration value is: ");
  //Serial.print(calibrationFactor);
  Serial.print("Calculated scalefactor is: ");
  Serial.print(scalefactor);
  Serial.println(", use this in your project sketch");
  Serial.print("Calculated offset is: ");
  Serial.print(offset);
  Serial.println(", use this in your project sketch");
  f = 0;
  Serial.print("Save Scalefactor to EEPROM adress ");
  Serial.println(scalefactorEepromAdress);
  Serial.print("Save Offset to EEPROM adress ");
  Serial.println(offsetEepromAdress);
  Serial.println("? y/n");
  while (f == 0) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
        EEPROM.put(scalefactorEepromAdress, scalefactor);
        EEPROM.get(scalefactorEepromAdress, scalefactor);
        Serial.print("Scalefactor ");
        Serial.print(scalefactor);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(scalefactorEepromAdress);

        EEPROM.put(offsetEepromAdress, offset);
        EEPROM.get(offsetEepromAdress, offset);
        Serial.print("Offset ");
        Serial.print(offset);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(offsetEepromAdress);
        f = 1;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        f = 1;
      }
    }
  }
  Serial.println("End calibration");
  Serial.println("For manual edit, send 'c' from serial monitor");
  Serial.println("***");
}


void sendValuesToCloud()
{

          EEPROM.get(api_key_address, api_key_Buffer);
          Serial.println("API_Key(EEPROM): " + String(api_key_Buffer));
          delay(1000);

          channel_key = String(api_key_Buffer);
          Cloud4BeesLibrary::Cloud4Bees cloud4bees (channel_key);

            Serial.println("Weight: " + String(floatWeight));
            delay(1000);

            //Send Value to cloud4Bees
            //cloud4bees.setConnectionTimeout(5000);  //Timeoutfunktion (in sendValues()) funktioniert nicht!!!Notwendig?
            cloud4bees.recordValue(1, String(floatWeight, 2));
            cloud4bees.recordValue(2, String(temperature, 2));
            cloud4bees.recordValue(3, String(humidity, 2));
            cloud4bees.recordValue(4, String(soc, 2));
            cloud4bees.sendValues();
            delay(1000);
            Serial.println("Werte wurden an cloud4Bees gesendet...");

  }


void readWeight() {
  
  //float calibrationFactor = 0;
  EEPROM.get(scalefactorEepromAdress, scalefactor);
  Serial.println("Scalefactor: " + String(scalefactor));
  scale.set_scale(scalefactor); //Adjust to this calibration factor

  EEPROM.get(offsetEepromAdress, offset);
  Serial.println("Offset: " + String(offset));
  

  
  scale.set_scale(scalefactor);
  //scale.get_units(10) returns the medium of 10 measures
  //floatWeight = (scale.get_units(10) - offset);
  for (int i = 0; i < num_weight_readings; i++) {
    weight_readings[i] = scale.get_units() - offset;  // fill the array with instantaneous readings from the scale
    Serial.println("Weight: " + String(weight_readings[i]));
    delay(1000);
  }
  delay(1000);
  
  floatWeight = median(weight_readings,num_weight_readings);  //calculate median 
 
  stringWeight =  String(floatWeight, 2);
  Serial.println("Weight_Median: " + stringWeight);
  delay(100);

  scale.power_down();
}


void readDHT() {
  // Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a 
// very slow sensor)

  for (int i = 0; i < num_temperature_readings; i++) {
    temperature_readings[i] = dht.getTempCelcius();;  // fill the array with instantaneous readings from the scale
    Serial.println("Temperature: " + String(temperature_readings[i]));
    delay(1000);
  }
  delay(1000);
  
  temperature = median(temperature_readings,num_temperature_readings);  //calculate median 
  Serial.println("Temperature: " + String(temperature));


  for (int i = 0; i < num_humidity_readings; i++) {
    humidity_readings[i] = dht.getHumidity();  // fill the array with instantaneous readings from the scale
    Serial.println("Humidity: " + String(humidity_readings[i]));
    delay(1000);
  }
  delay(1000);
  
  humidity = median(humidity_readings,num_humidity_readings);  //calculate median 
  Serial.println("Humidity: " + String(humidity));
  
// Check if any reads failed and exit early (to try again).
	if (isnan(humidity) || isnan(temperature)) {
		Serial.println("Failed to read from DHT sensor!");
		return;
	}

// Compute heat index
// Must send in temp in Fahrenheit!
/*
	float hi = dht.getHeatIndex();
	float dp = dht.getDewPoint();
	float k = dht.getTempKelvin();
*/
}


void wait5Seconds() {
  for(int i=0;i<5;i++) {
  Serial.println("waiting " + String(5-i) + " seconds before we publish");
  delay(1000);
  }
}

void PMICSetup(){

  PMIC power(true);
  Log.info("Current PMIC settings:");
  Log.info("VIN Vmin: %u", power.getInputVoltageLimit());
  Log.info("VIN Imax: %u", power.getInputCurrentLimit());
  Log.info("Ichg: %u", power.getChargeCurrentValue());
  Log.info("Vchg: %u", power.getChargeVoltageValue());

  int powerSource = System.powerSource();
  int batteryState = System.batteryState();
  float batterySoc = System.batteryCharge();

  constexpr char const* batteryStates[] = {
  "unknown", "not charging", "charging",
  "charged", "discharging", "fault", "disconnected"
  };
  constexpr char const* powerSources[] = {
  "unknown", "vin", "usb host", "usb adapter",
  "usb otg", "battery"
  };

  Log.info("Power source: %s", powerSources[std::max(0, powerSource)]);
  Log.info("Battery state: %s", batteryStates[std::max(0, batteryState)]);
  Log.info("Battery charge: %f", batterySoc);

}


void readSoC() {
  //fuel.quickStart();
  delay(500);
  for (int i = 0; i < num_soc_readings; i++) {
  readings[i] = fuel.getSoC(); // fill the array with instantaneous readings from the scale
  Serial.println("SoC: " + String(readings[i]));
  delay(500);
  }
  delay(100);
  soc = median(readings,num_soc_readings); //calculate median 
  //soc = fuel.getSoC();
  stringSOC = String(soc);
  Serial.println("SoC_Median: " + String(soc));
}

void veryLowBattery() {
  if (soc != 0.0 && soc <= absolutminimumSoC) {
  Serial.println("Very low Battery");
  delay(500);
  System.sleep( {}, {}, sleepADay);
  //System.sleep(SLEEP_MODE_DEEP, sleepADay);
  } else {
  String stringAbsolutminimumSoC = String(absolutminimumSoC);
  Serial.println("SoC > " + stringAbsolutminimumSoC + "%");
  }
}

void batteryCheck() {
  if (soc != 0.0 && soc <= minimumSoC && !pmic.isPowerGood()) {
  Serial.println("Low Battery");
  //System.sleep(SLEEP_MODE_DEEP, sleepTimeSecs);
  System.sleep( {}, {}, sleepTimeSecs);
  } else {
  String stringMinimumSoC = String(minimumSoC);
  Serial.println("SoC > " + stringMinimumSoC + "%");
  }
}

void connectToCellular() {
  Cellular.off(); //Turning off the Cellular module will force it to go through a full re-connect to the Cellular network the next time it is turned on.
  delay(500);
  Cellular.on(); //Turns on the Cellular module. Useful when you've turned it off, and you changed your mind.
  Serial.println("Cellular on...");
  delay(500);
  Cellular.connect(); // This command turns on the Cellular Modem and tells it to connect to the cellular network.

  if (!waitFor(Cellular.ready, 300000)) { //If the cellular modem does not successfuly connect to the cellular network in 2 mins then go back to sleep via the sleep command below.
  System.sleep( {}, {}, sleepTimeSecs);  
  } else {
  Serial.println("Cellular connected...");
  }

}

void connectToParticle() {
  Particle.connect(); //Connects the device to the Cloud. This will automatically activate the cellular connection and attempt to connect to the Particle cloud if the device is not already connected to the cloud.

  if (!waitFor(Particle.connected, 300000)) { //If the cellular modem does not successfuly connect to the cellular network in 1 mins then go back to sleep via the sleep command below.
  Serial.printf("WARNING: connection failed...");
  System.sleep( {}, {}, sleepTimeSecs);  
  } else {
  Serial.println("Particle connected...");
  }
}



//Following functions are based on "https://github.com/dndubins/QuickStats", by David Dubins  
float median(float samples[],int m) //calculate the median
{
  //First bubble sort the values: https://en.wikipedia.org/wiki/Bubble_sort
  float sorted[m];   //Define and initialize sorted array.
  float temp=0.0;      //Temporary float for swapping elements
  
  for(int i=0;i<m;i++){
    sorted[i]=samples[i];
  }
  bubbleSort(sorted,m);  // Sort the values
  
  if (bitRead(m,0)==1) {  //If the last bit of a number is 1, it's odd. This is equivalent to "TRUE". Also use if m%2!=0.
    return sorted[m/2]; //If the number of data points is odd, return middle number.
  } else {    
    return (sorted[(m/2)-1]+sorted[m/2])/2; //If the number of data points is even, return avg of the middle two numbers.
  }
}

void bubbleSort(float A[],int len) {
  unsigned long newn;
  unsigned long n=len;
  float temp=0.0;
  do {
    newn=1;
    for(int p=1;p<len;p++){
      if(A[p-1]>A[p]){
        temp=A[p];           //swap places in array
        A[p]=A[p-1];
        A[p-1]=temp;
        newn=p;
      } //end if
    } //end for
    n=newn;
  } while(n>1);
}