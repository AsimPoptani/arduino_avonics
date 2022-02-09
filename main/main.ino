#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "IOSdcard.h"
#include "Buffer.hpp"


#include "BMI088.h"
#include <BMP388_DEV.h>

// PINS
#define SDCARDCS 33

// OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// Handle tasks
TaskHandle_t AccelAndGyroTask;
TaskHandle_t PressureTask;
TaskHandle_t DisplayTask;
TaskHandle_t RadioTask;

// Semaphores
SemaphoreHandle_t SDWriteSemaphore= xSemaphoreCreateMutex();
SemaphoreHandle_t I2CSemaphore= xSemaphoreCreateMutex();
SemaphoreHandle_t DisplaySemaphore= xSemaphoreCreateMutex();

// Files
File pressureFile;
File accelGyroFile;

// Buffer size for data
#define BUFFERSIZE 5



// Hold Pressure data
struct PressureStuct
{
  int pressure;
  int temperature;
  int altitude;
  unsigned long time;
  char * (*Convert)(PressureStuct *pressure);
};
// char * pressureToSting(Pressure *pressure)
// {
//     int returnSize=snprintf(NULL,0,"%d,%d,%d,%lu\n",pressure->pressure,pressure->temperature,pressure->altitude,pressure->time);
//     char * buffer=(char*)malloc(returnSize+1);
//     snprintf(buffer,returnSize+1,"%d,%d,%d,%lu\n",pressure->pressure,pressure->temperature,pressure->altitude,pressure->time);
//     return buffer;
// }
struct AccelerationAndGyroStruct
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  unsigned long time;
  char * (*Convert)(AccelerationAndGyroStruct *accelGyro);
};
char * accelGyroToSting(AccelerationAndGyroStruct *accelGyro)
{


    int returnSize=snprintf(NULL,0,"%f,%f,%f,%f,%f,%f,%lu\n",accelGyro->ax,accelGyro->ay,accelGyro->az,accelGyro->gx,accelGyro->gy,accelGyro->gz,accelGyro->time);
    char * buffer=(char*)malloc(returnSize+1);
    snprintf(buffer,returnSize+1,"%f,%f,%f,%f,%f,%f,%lu\n",accelGyro->ax,accelGyro->ay,accelGyro->az,accelGyro->gx,accelGyro->gy,accelGyro->gz,accelGyro->time);

    return buffer;
}

// BUFFERS
typedef Buffer<PressureStuct> PressureBuffer;
typedef Buffer<AccelerationAndGyroStruct> AccelAndGyroBuffer;
PressureBuffer pressureBuffer("/pressure.csv","pressure",BUFFERSIZE,&SDWriteSemaphore);
AccelAndGyroBuffer accelAndGyroBuffer("/accelAndGyro.csv","accel",BUFFERSIZE,&SDWriteSemaphore);
// pressureBuffer=new PressureBuffer("/pressure.csv","pressure",BUFFERSIZE,&SDWriteSemaphore,pressureFile);
// Devices
BMP388_DEV bmp388; 
SPIClass *hspi;
RF24 radio(26, 15);

// Buffer for pressure data
// struct Pressure pressureBuffer1[BUFFERSIZE]={};
// struct Pressure pressureBuffer2[BUFFERSIZE]={};

// Switchs pressure
// int pressureBufferCounter = 0;
// int pressureSwitchBuffer = 0;




// void writePressure()
// {


//   pressureFile=SD.open("/pressure.csv",FILE_APPEND);

//   // Loop through the switch buffer and write each value to the file
//   if (pressureFile){
//     printf("Opened pressure file");
//     delay(1000);
//   }
  
//   for (int index = 0; index < BUFFERSIZE; index++)
//   {
//     struct Pressure (*pressureBuffer)[BUFFERSIZE];
//     if (pressureSwitchBuffer == 0)
//     {
//       pressureBuffer = &pressureBuffer1;
//     }
//     else
//     {
//       pressureBuffer = &pressureBuffer2;
//     }
//     // Create string to hold the values for writing
//     char temp[300]={};
//     sprintf(temp, "%d,%lu", (*pressureBuffer)[index].pressure, (*pressureBuffer)[index].time);
//     printf("%s\n", temp);
//     pressureFile.println(temp);
//   }
//   pressureFile.flush();
//   // Close the file
//   pressureFile.close();
// }
// void ReadPressure(void *pvParameters)
// {
//   while (1)
//   {
//     if (pressureBufferCounter == BUFFERSIZE)
//     {
//         // Switch the buffer
//       pressureSwitchBuffer = !pressureSwitchBuffer;
//       // Reset the counter
//       pressureBufferCounter = 0;
//       // Say we are writing to the buffer
//       Serial.println("Writing to the buffer");
//       writePressure();
//     }
//     // Read pressure
//     int pressure = 0;
//     // Read time
//     int time = millis();
//     // Add to buffer
//     struct Pressure (*pressureBuffer)[BUFFERSIZE];
//     if (pressureSwitchBuffer == 0)
//     {
//       pressureBuffer = &pressureBuffer1;
//     }
//     else
//     {
//       pressureBuffer = &pressureBuffer2;
//     }
//     (*pressureBuffer)[pressureBufferCounter].pressure = pressure;
//     (*pressureBuffer)[pressureBufferCounter].time = time;
//     pressureBufferCounter++;
//     printf("Adding one to the pressure buffer %d\n", pressureBufferCounter);
    
//     delay(100);
//   }
// }

void Radio(void *pvParameters)
{
  // HSPI is the hardware SPI port for the radio
  hspi = new SPIClass(HSPI);
  hspi->begin();
  
  // ADDRESS

  uint8_t address[][6] = {"1Node", "2Node"};

  bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

  
  float payload = 0.0;
  if (!radio.begin(hspi)) {
    Serial.println(F("radio hardware not responding!!"));
    while (1) {} // hold program in infinite loop to prevent subsequent errors
  }else{
    Serial.println(F("radio hardware OK"));
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  radio.stopListening();  // put radio in TX mode

  while (1)
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
    delay(1000);
  }

}

void Display(void * pvParameters)
{
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
    }

    while (1)
    {
      display.clearDisplay();
      display.display();
      delay(500);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Hello World!");
      display.display();
      delay(1000);
    }
    
}

void Pressure(void *pvParameters)
{
  if(bmp388.begin()){
    Serial.println("BMP388 is online");
  }
  else{
    Serial.println("BMP388 is not offline");
  }                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.setTimeStandby(TIME_STANDBY_160MS);     // Set the standby time to 1280ms
  bmp388.startNormalConversion();                 // Start NORMAL conversion mode
  while (1)
  {
    float temperature, pressure, altitude;
    if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
    {
      // printf("Temperature: %f\n", temperature);
      // printf("Pressure: %f\n", pressure);
      // printf("Altitude: %f\n", altitude);
    }
    
  }
  

}

void AccelAndGyro(void *pvParameters)
{
  delay(1000);
  if (bmi088.isConnection()){
    bmi088.initialize();
    Serial.println("BMI088 connected");
  }
  else{
    Serial.println("BMI088 not connected");
    while (1) {}
  }
  while (1)
  {
      float x,y,z=0;
      bmi088.getAcceleration(&x,&y,&z);
      // printf("Acceleration: %f, %f, %f\n", x, y, z);
      float xg,yg,zg=0;
      bmi088.getGyroscope(&xg,&yg,&zg);
      // printf("Gyroscope: %f, %f, %f\n", xg, yg, zg);
      AccelerationAndGyroStruct accelAndGyro;
      accelAndGyro.ax = x;
      accelAndGyro.ay = y;
      accelAndGyro.az = z;
      accelAndGyro.gx = xg;
      accelAndGyro.gy = yg;
      accelAndGyro.gz = zg;
      accelAndGyro.time = millis();
      accelAndGyro.Convert=accelGyroToSting;
      // add to buffer
      accelAndGyroBuffer.Push(accelAndGyro);

      delay(100);
  }
  
}


void setup()
{

  Serial.begin(115200);
  
  Wire.begin(21,22);
  

  SPIClass spi = SPIClass(VSPI);
  pinMode(19,INPUT_PULLUP);
  spi.begin(18,19,23,SDCARDCS);

  if (!SD.begin(SDCARDCS,spi))
  {
    Serial.println("initialization failed!");
    return;
  }


  

  pressureBuffer.AddFile(&pressureFile,&SD);
  accelAndGyroBuffer.AddFile(&accelGyroFile,&SD);

  xTaskCreatePinnedToCore(AccelAndGyro, "Accel", 40000, NULL, 1, &AccelAndGyroTask, 0);
  xTaskCreatePinnedToCore(Pressure, "pressure", 20000, NULL, 1, &PressureTask, 0);
  xTaskCreatePinnedToCore(Display, "Display", 20000, NULL, 1, &DisplayTask, 0);
  
}
void loop()
{
    
  delay(1000);
    // to make this example readable in the serial monitor
    
  // display.display();
  // delay(2000); // Pause for 2 seconds
  

    

}
