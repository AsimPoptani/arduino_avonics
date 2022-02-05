#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



#include "IOSdcard.h"
#include "BMI088.h"
#include <BMP388_DEV.h>


#define SDCARDCS 33
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
TaskHandle_t ReadPressureTask;

#define BUFFERSIZE 100
File pressureFile;

struct Pressure
{
  int pressure;
  unsigned long time;
};

struct Pressure pressureBuffer1[BUFFERSIZE]={};
struct Pressure pressureBuffer2[BUFFERSIZE]={};

int pressureBufferCounter = 0;
int switchBuffer = 0;

void writePressure()
{


  pressureFile=SD.open("/pressure.csv",FILE_APPEND);

  // Loop through the switch buffer and write each value to the file
  if (pressureFile){
    printf("Opened pressure file");
    delay(1000);
  }
  
  for (int index = 0; index < BUFFERSIZE; index++)
  {
    struct Pressure (*pressureBuffer)[BUFFERSIZE];
    if (switchBuffer == 0)
    {
      pressureBuffer = &pressureBuffer1;
    }
    else
    {
      pressureBuffer = &pressureBuffer2;
    }
    // Create string to hold the values for writing
    char temp[300]={};
    sprintf(temp, "%d,%lu", (*pressureBuffer)[index].pressure, (*pressureBuffer)[index].time);
    printf("%s\n", temp);
    pressureFile.println(temp);
  }
  pressureFile.flush();
  // Close the file
  pressureFile.close();
}
void ReadPressure(void *pvParameters)
{
  while (1)
  {
    if (pressureBufferCounter == BUFFERSIZE)
    {
        // Switch the buffer
      switchBuffer = !switchBuffer;
      // Reset the counter
      pressureBufferCounter = 0;
      // Say we are writing to the buffer
      Serial.println("Writing to the buffer");
      writePressure();
    }
    // Read pressure
    int pressure = 0;
    // Read time
    int time = millis();
    // Add to buffer
    struct Pressure (*pressureBuffer)[BUFFERSIZE];
    if (switchBuffer == 0)
    {
      pressureBuffer = &pressureBuffer1;
    }
    else
    {
      pressureBuffer = &pressureBuffer2;
    }
    (*pressureBuffer)[pressureBufferCounter].pressure = pressure;
    (*pressureBuffer)[pressureBufferCounter].time = time;
    pressureBufferCounter++;
    printf("Adding one to the pressure buffer %d\n", pressureBufferCounter);
    
    delay(100);
  }
}
// BMP388_DEV bmp388; 
SPIClass *hspi;
RF24 radio(26, 15);
// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;
void setup()
{

  Serial.begin(115200);
  hspi = new SPIClass(HSPI);
  hspi->begin();
  Wire.begin();
  if (!radio.begin(hspi)) {
    Serial.println(F("radio hardware not responding!!"));
    while (1) {} // hold program in infinite loop to prevent subsequent errors
  }else{
    Serial.println(F("radio hardware OK"));
  }
  // if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //   Serial.println(F("SSD1306 allocation failed"));
  // }
  // if(bmp388.begin()){
  //   Serial.println("BMP388 is online");
  // }
  // else{
  //   Serial.println("BMP388 is not offline");
  // }                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  // bmp388.setTimeStandby(TIME_STANDBY_160MS);     // Set the standby time to 1280ms
  // bmp388.startNormalConversion();                 // Start NORMAL conversion mode
  // SPIClass spi = SPIClass(VSPI);
  // if (bmi088.isConnection()){
  //   bmi088.initialize();
  //   Serial.println("BMI088 connected");
  // }

  // if (!SD.begin(SDCARDCS))
  // {
  //   Serial.println("initialization failed!");
  //   return;
  // }
  // xTaskCreatePinnedToCore(ReadPressure, "Pressure", 10000, NULL, 1, &ReadPressureTask, 0);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening(); // put radio in RX mode
  }
}
void loop()
{
     unsigned long start_timer = micros();                    // start the timer
    bool report = radio.write(&payload, sizeof(float));      // transmit & save the report
    unsigned long end_timer = micros();                      // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));          // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);                 // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload);                               // print payload sent
      payload += 0.01;                                       // increment float payload
    } else {
      Serial.println(F("Transmission failed or timed out")); // payload was not delivered
    }

    // to make this example readable in the serial monitor
    delay(10);  // slow transmissions down by 1 second
  // display.display();
  // delay(2000); // Pause for 2 seconds
  // float temperature, pressure, altitude;
  // if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  // {
  //   printf("Temperature: %f\n", temperature);
  //   printf("Pressure: %f\n", pressure);
  //   printf("Altitude: %f\n", altitude);
  // }
  // float x,y,z=0;
  // bmi088.getAcceleration(&x,&y,&z);
  // printf("Acceleration: %f, %f, %f\n", x, y, z);
    

}
