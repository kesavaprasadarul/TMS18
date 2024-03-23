/*
  This example creates a BLE peripheral with a service that contains a
  couple of characteristics to test BLE connection.
  The yellow LED shows the BLE module is initialized.
  The green LED shows RSSI of zero. The more it blinks the worse the connection.

  The circuit:
  - Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include "Arduino.h"


//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_TEST_SERVICE               "9A48ECBA-2E92-082F-C079-9E75AAE428B1"
#define BLE_UUID_ACCELERATION               "2713"
#define BLE_UUID_COUNTER                    "1A3AC130-31EE-758A-BC50-54A61958EF81"
#define BLE_UUID_RESET_COUNTER              "FE4E19FF-B132-0099-5E94-3FFB2CF07940"
#define TOF_BL_BLOCKCONTENT_VALUE        "6A23EEBF-287E-419A-B2F0-31D18674A192"

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

BLEService tofCoreService( BLE_UUID_TEST_SERVICE );
// BLEFloatCharacteristic accelerationCharacteristic( BLE_UUID_ACCELERATION, BLERead | BLENotify );
// BLEUnsignedLongCharacteristic counterCharacteristic( BLE_UUID_COUNTER, BLERead | BLENotify );
BLEUnsignedLongCharacteristic tofRearLeftCharacteristic(TOF_BL_BLOCKCONTENT_VALUE, BLERead | BLENotify);
// BLEBoolCharacteristic resetCounterCharacteristic( BLE_UUID_RESET_COUNTER, BLEWriteWithoutResponse );

DFRobot_VL53L0X sensor;



const int BLE_LED_PIN = LED_BUILTIN;
const int RSSI_LED_PIN = LED_PWR;


void setup()
{
  Serial.begin( 9600 );
  while ( !Serial );

  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();

  pinMode( BLE_LED_PIN, OUTPUT );
  pinMode( RSSI_LED_PIN, OUTPUT );

  // if ( !IMU.begin() )
  // {
  //   Serial.println( "Failed to initialize IMU!" );
  //   while ( 1 );
  // }
  // Serial.print( "Accelerometer sample rate = " );
  // Serial.print( IMU.accelerationSampleRate() );
  // Serial.println( " Hz" );

  if( setupBleMode() )
  {
    digitalWrite( BLE_LED_PIN, HIGH );
  }
} // setup


bool PT1Messung(float &FiltVal, float Kanal, float Periode, float Tau){
  static float lastRun = 0;
  float FF = Tau / (Periode == 0 ? 0.00001 : Periode);
  float neuWert = Kanal;
  if (millis()-lastRun < Periode) return false;
  if (lastRun == 0) FiltVal = neuWert; 
  FiltVal= ((FiltVal * FF) + neuWert) / ((FF+1) == 0 ? 0.0001 : FF+1); 

  lastRun = millis();
  return true;
}

static float del2 = -16384.0;
static float del1 = -16384.0;
static float valueOld = 0.0f;

static float rawOld = 0.0f;
static float del1Old = 0.0f;

float CalculateStabilityFactor(float ValueIn, float Threshold, float expGain = 4)
{
  float SensorAngleThreshold = 900.0f;
  if(del1Old == 0)
  {
    if(rawOld == 0)
    {
      rawOld = abs(ValueIn);
      return 0;
    }

    del1Old = abs(ValueIn - rawOld);
    return 0;
  }

  del1 = abs(ValueIn - rawOld);
  rawOld = abs(ValueIn);
  del2 = abs(del1 - del1Old);
  del1Old = abs(del1);

  if(ValueIn > SensorAngleThreshold) return 0;
  //Serial.print("Gain Del1: "); Serial.print(del1);Serial.print("\t"); Serial.print("Gain Del2: "); Serial.println(del2);
  return exp((-del2)/expGain);

}


void loop()
{
  static unsigned long counter = 0;
  static long previousMillis = 0;

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  if ( central )
  {
    Serial.print( "Connected to central: " );
    Serial.println( central.address() );

    while ( central.connected() )
    {

      long interval = 20;
      unsigned long currentMillis = millis();
      if( currentMillis - previousMillis > interval )
      {
        previousMillis = currentMillis;

       // Serial.print( "Central RSSI: " );
       // Serial.println( central.rssi() );

        if( central.rssi() != 0 )
        {
          // digitalWrite( RSSI_LED_PIN, LOW );
          // float accelerationX, accelerationY, accelerationZ;
          // if ( IMU.accelerationAvailable() )
          // {
          //   IMU.readAcceleration( accelerationX, accelerationY, accelerationZ );
          //   accelerationCharacteristic.writeValue( accelerationX );
          // }

          // counter++;
          // counterCharacteristic.writeValue( counter );
          static float FiltVal;
          static float FiltDelVal;
          float SamplingPeriod_ms = 50;
          float ToFDistance = sensor.getDistance();
          float ToFFild = PT1Messung(FiltVal, ToFDistance, SamplingPeriod_ms, 60);
          float StabilityFactor = CalculateStabilityFactor(FiltVal, 250.0f, 4);

          int FiltValInt = (int)floor(FiltVal);
          int StabilityValueInt = (int)(StabilityFactor*100.0f);

          Serial.print(ToFDistance); Serial.print("\t"); Serial.print(StabilityValueInt); Serial.print("\t"); Serial.print(FiltValInt);Serial.print("\t"); Serial.print(StabilityValueInt | (FiltValInt << 16));
          Serial.println("");
          tofRearLeftCharacteristic.writeValue(StabilityValueInt | (FiltValInt << 16));
        }
        else
        {
          digitalWrite( RSSI_LED_PIN, HIGH );
        }
      } // intervall
    } // while connected

    Serial.print( F( "Disconnected from central: " ) );
    Serial.println( central.address() );
  } // if central
} // loop



bool setupBleMode()
{
  if ( !BLE.begin() )
  {
    return false;
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName( "Arduino Nano 33 BLE" );
  BLE.setLocalName( "Arduino Nano 33 BLE" );
  BLE.setAdvertisedService( tofCoreService );

  // BLE add characteristics
  tofCoreService.addCharacteristic( tofRearLeftCharacteristic );

  // add service
  BLE.addService( tofCoreService );

  // set the initial value for the characeristic:
  tofRearLeftCharacteristic.writeValue( 0.0 );

  // start advertising
  BLE.advertise();

  return true;
}