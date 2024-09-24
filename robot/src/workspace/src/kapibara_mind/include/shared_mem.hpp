#pragma once

#include <cstring>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

template<typename Interface>
class SharedMemory
{
    private:

    int shmid; 
    int key;
    
    Interface* ptr;

    public:

    SharedMemory()
    {
        shmid=-1;
    }

    SharedMemory(const int& key,const size_t& size)
    : SharedMemory()
    {
        this->init(key,size);
    }

    void init(const int& key,const size_t& size)
    {
        if( (shmid = shmget(key, size , 0666|IPC_CREAT)) < 0 )
        {
            return;
        }

        this->ptr = (Interface *)shmat(shmid, (void*)0, 0);

        if( (char *)this->ptr == (char *) -1)
	    {
            return;
	    }

        this->key=key;
    }

    const int& getKey() const
    {
        return this->key;
    }

    Interface* getPtr()
    {
        return this->ptr;
    }

    void write(Interface* data,size_t size=sizeof(Interface),size_t offset=0)
    {
        memcpy(this->ptr+offset,data,size);
    }

    void read(Interface* output,size_t size=sizeof(Interface))
    {
        memcpy(output,this->ptr,size);
    }

    void clear()
    {
        shmdt(this->ptr); 

        shmctl(this->shmid, IPC_RMID, NULL);  
    }

    operator bool()
    {
        return shmid >= 0;
    }

    ~SharedMemory()
    {
        shmdt(this->ptr);

        shmctl(this->shmid, IPC_RMID, NULL);
    }
    

};