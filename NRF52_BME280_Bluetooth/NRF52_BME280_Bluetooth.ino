/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <avr/dtostrf.h>

#define SEALEVELPRESSURE_HPA (1013.25)

//#define USB_UART // enables Serial debugging

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void setup()
{
#ifdef USB_UART
    Serial.begin(115200);

  #if CFG_DEBUG
    // Blocking wait for connection when debug mode is enabled via IDE
    while (!Serial ) yield();
  #endif
    while(!Serial);    // time to get serial running

    Serial.println(F("BME280 test"));
#endif
  unsigned status;
  
  // default settings
  status = bme.begin(0x76); //or 0x77
  #ifdef USB_UART
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  
  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");
  #endif
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
  #ifdef USB_UART
  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
  #endif
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop()
{

    delay(60000); //per minute

    char buf1[64], buf2[64], buf3[64];

    //read Temperature
    char str_temp[6];
    float temp = bme.readTemperature();
    dtostrf(temp, 4, 2, str_temp);
    sprintf(buf1, "%s °C \n", str_temp);
    //Serial.println(buf);

    //read Pressure
    char str_pres[6];
    float pres = bme.readPressure() / 100.0F;
    dtostrf(pres, 4, 2, str_pres);
    sprintf(buf2, "%s hPa \n", str_pres);

    //read Humidity
    char str_humi[6];
    float humi = bme.readHumidity();
    dtostrf(humi, 4, 2, str_humi);
    sprintf(buf3, "%s %% \n", str_humi);
    
    //int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write(buf1); bleuart.write(buf2); bleuart.write(buf3);
  
    // Forward from BLEUART to HW Serial
    #ifdef USB_UART
    while ( bleuart.available() )
    {
      uint8_t ch;
      ch = (uint8_t) bleuart.read();
      Serial.write(ch);
    }
    #endif
      
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  #ifdef USB_UART
  Serial.print("Connected to ");
  Serial.println(central_name);
  #endif
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  #ifdef USB_UART
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  #endif
}
