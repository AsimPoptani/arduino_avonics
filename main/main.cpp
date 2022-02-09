#include <cstdio>
#include <cstdlib>
#include "Buffer.hpp"
struct Pressure
{
  int pressure;
  int temperature;
  int altitude;
  unsigned long time;
  char * (*Convert)(Pressure *pressure);
};

char * pressureToSting(Pressure *pressure)
{
    int returnSize=snprintf(NULL,0,"%d,%d,%d,%lu\n",pressure->pressure,pressure->temperature,pressure->altitude,pressure->time);
    char * buffer=(char*)malloc(returnSize+1);
    snprintf(buffer,returnSize+1,"%d,%d,%d,%lu\n",pressure->pressure,pressure->temperature,pressure->altitude,pressure->time);
    return buffer;
}
// Buffer<Pressure> pressureBuffer("/pressure.csv","pressure",BUFFERSIZE,&SDWriteSemaphore);

int main(){
    Pressure pressure;
    pressure.Convert=pressureToSting;
    pressure.pressure=1;
    pressure.temperature=2;
    pressure.altitude=3;
    pressure.time=4;
    
    char * buffer=pressure.Convert(&pressure);
    printf("%s\n",buffer);
    free(buffer);
}