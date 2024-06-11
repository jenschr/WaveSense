#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include "MPU6050.h"

#define SDA_PIN 0
#define SCL_PIN 1
#define ANALOG_PIN 2
#define BUZZER_PIN 3
#define SD_CS_PIN 4
#define SD_MISO_PIN 5
#define SD_CLK_PIN 6
#define SD_MOSI_PIN 7
#define LED_PIN 8

#define NUM_LEDS 1

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
File root;
RTC_DS3231 rtc;

MPU6050 accelgyro(0x69);
int16_t ax, ay, az;
int16_t gx, gy, gz;
int loopCounter = 0;

void setPixels(uint8_t r, uint8_t g, uint8_t b)
{
  for(int i=0;i<NUM_LEDS;i++)
  {
    pixels.setPixelColor(i, r, g, b);
  }
  pixels.show();
}

void scanI2c()
{
  byte error, address; //variable for error and I2C address
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void scanSD()
{
  
  Serial.println("initialization done.");

  root = SD.open("/");

  printDirectory(root, 0);

  Serial.println("done!");
}

void setupClock()
{
  // initializing the rtc
    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1) delay(10);
    }

    if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        Serial.println("RTC set time!");
    }

    //we don't need the 32K Pin, so disable it
    rtc.disable32K();

    char date[10] = "hh:mm:ss";
    rtc.now().toString(date);
    Serial.print(date);
    Serial.println(" - RTC initialized");
}

void testAudio()
{
  tone(BUZZER_PIN, 440, 500);
  delay(500);
  tone(BUZZER_PIN, 880, 500);
  delay(500);
  tone(BUZZER_PIN, 440, 500);
  delay(500);
  tone(BUZZER_PIN, 880, 500);
}

void readAccelerometer()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.println("");
    delay(100);
}

void updateTheLed()
{
  if( loopCounter % 50 == 0 )
  {
    int r = random(0,155);
    int g = random(0,155);
    int b = random(0,155);
    setPixels( r, g, b );
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pixels.begin();
  setPixels(0, 0, 0);

  SPI.begin(SD_CLK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN);
  Serial.println("SPI initialized");

  Wire.begin(SDA_PIN,SCL_PIN);

  delay(4000);
  scanI2c();

  delay(1000);
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  scanSD();

  delay(1000);
  setupClock();

  delay(1000);
  testAudio();

  delay(1000);
  Serial.println("Initializing MPU-6050");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop()
{
  readAccelerometer();
  updateTheLed();
  loopCounter++;
}
