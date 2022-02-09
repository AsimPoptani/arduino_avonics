#ifndef BUFFER_SDCARD
#define BUFFER_SDCARD

#include "IOSdcard.h"

template<class T>
struct Handle
{
    T* buffer;
    int bufferSize;
    const char * filename;
    SemaphoreHandle_t * SDCardWrite;
    SDFS *fs;
    File * file;

};

template <class T>
class Buffer
{
private:
    typedef Handle<T> handleType;
    handleType handle;
    static void BufferWrite(void * pv);
    static void BufferWrite2(void * pv);
    int bufferCounter;
    int bufferSwitch;
    int bufferSize;
    T *buffer1;
    T *buffer2;
    const char *name;
    SemaphoreHandle_t * SDCardWrite;
    SDFS *fs;
    File *file;

    
public:
    const char *fileName;
    Buffer(const char * fileName,const char *name, int bufferSize, SemaphoreHandle_t * SDCardWrite);
    ~Buffer();
    bool Push(T data);
    void AddFile(File *file,SDFS *fs);
};



template <class T>
Buffer<T>::Buffer(const char * fileName,const char *name ,int bufferSize, SemaphoreHandle_t * SDCardWrite) 
{
    this->name=name;
    this->bufferSize = bufferSize;
    this->buffer1= new T[this->bufferSize];
    this->buffer2= new T[this->bufferSize];
    this->bufferCounter = 0;
    this->bufferSwitch = 0;
    this->fileName = fileName;
    this->SDCardWrite=SDCardWrite;
}

template <class T>
void Buffer<T>::AddFile(File *file,SDFS *fs)
{
    this->file=file;
    this->fs=fs;


}

template <class T>
bool Buffer<T>::Push(T data)
{
    // printf("Adding data %s\n",data.Convert(&data));
    // If the buffer is full, write the data to the file and switch buffers
    if (bufferCounter == this->bufferSize)
    {
        printf("Buffer full, writing to file\n");

        handle.buffer = bufferSwitch?buffer2:buffer1;
        handle.bufferSize = this->bufferSize;
        handle.SDCardWrite = this->SDCardWrite;
        handle.filename = this->fileName;
        handle.file = this->file;
        handle.fs = this->fs;
        printf("Sending to Core 1\n");
        // xTaskCreatePinnedToCore(BufferWrite, this->name, 20000, &this->handle, 1, NULL,1);
        // Address of handle
        // printf("Address of Handle: %p\n",(void *)&handle);
        // BufferWrite(&handle);
        BufferWrite2(this);
        bufferCounter = 0;
        bufferSwitch = !bufferSwitch;

        return true;
    }
    // Otherwise append to the buffer
    if (bufferSwitch == 0)
    {
        buffer1[bufferCounter] = data;
    }
    else
    {
        buffer2[bufferCounter] = data;
    }
    bufferCounter++;
    return true;

}

template <class T>
void Buffer<T>::BufferWrite2(void* pv){
    typedef Buffer<T> BufferRef;
    BufferRef *buffer = (BufferRef *)pv;
    printf("Testing file print");
    File f22=buffer->fs->open("/test.txt","w",true);
    printf("Writing to test file");
    f22.println("testing 123!");
    printf("Closing File");
    f22.close();
}

template <class T>
void Buffer<T>::BufferWrite(void * pv)
{
    printf("Converting to string\n");
    // printf("Address of Handle: %p\n",(void *)&handle);
    typedef Handle<T> HandleType;
    HandleType handle2 = *(HandleType*)(pv);
    // printf("Name %s\n",handle2.filename);
    // HandleType handle = *handle2;
    // Take the semaphore
    printf("Setting Semaphore\n");
    // xSemaphoreTake(handle2.SDCardWrite, portMAX_DELAY);
    printf("Opening SD Card\n");


    // File f = SD.open(handle2.filename, FILE_APPEND);
    
    // handle2.file->
    printf("Writing to file %s\n",handle2.filename);
    for (int i=0;i<handle2.bufferSize;i++)
    {
        // Use that Convert
        char * result = handle2.buffer[i].Convert(&handle2.buffer[i]);
        // Write to the file
        printf("Writing %s\n",result);
        // handle2.file.print(result);
        // f.print(result);
        // Free the buffer
        free(result);
    }
    // flush
    printf("Flushing to file\n");
    // handle2.file.close();
    // Close the file
    // f.close();
    // Release the semaphore
    // xSemaphoreGive(handle2.SDCardWrite);
    // return true;

}


template <class T>
Buffer<T>::~Buffer()
{
}

#endif