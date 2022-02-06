// #include "IOSdcard.h"

template<class T>
struct Handle
{
    T* buffer;
    int bufferSize;
    int semaphore;

};

template <class T>
class Buffer
{
private:

    static void BufferWrite(void * pv);
    int bufferCounter;
    int bufferSwitch;
    int bufferSize;
    T *buffer1;
    T *buffer2;
    const char *name;

    
public:
    const char *fileName;
    Buffer(const char * fileName,const char *name, int bufferSize);
    ~Buffer();
    bool Push(T data);
};



template <class T>
Buffer<T>::Buffer(const char * fileName,const char *name ,int bufferSize) 
{
    this->name=name;
    this->bufferSize = bufferSize;
    this->buffer1= new T[this->bufferSize];
    this->buffer2= new T[this->bufferSize];
    this->bufferCounter = 0;
    this->bufferSwitch = 0;
    this->fileName = fileName;
}

template <class T>
bool Buffer<T>::Push(T data)
{
    // If the buffer is full, write the data to the file and switch buffers
    if (bufferCounter == this->bufferSize)
    {
        typedef Handle<T> handleType;
        handleType handle;
        handle.buffer = bufferSwitch?buffer2:buffer1;
        handle.bufferSize = this->bufferSize;
        handle.semaphore = 0;
        BufferWrite(&handle);
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
void Buffer<T>::BufferWrite(void * pv)
{
    typedef Handle<T> HandleType;
    HandleType handle = *(HandleType*)pv;

    for (int i=0;i<handle.bufferSize;i++)
    {
        handle.buffer[i].a=handle.buffer[i].a+1;
        printf("%d %d\n",handle.buffer[i].a,handle.buffer[i].b);
        
    }
    // return true;

}


template <class T>
Buffer<T>::~Buffer()
{
}

