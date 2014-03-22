
#ifndef CTHREADS
#define CTHREADS

#include "util/inclstdint.h"

#include <pthread.h>
#include <unistd.h>

class CThread
{
  public:
    CThread();
    ~CThread();
    void startThreads();
    
    void StopThreads();
    void AsyncStopThreads();
    void JoinThreads();
    bool IsRunning();

  protected:
    pthread_t     thread_1;
    pthread_t     thread_2;
    volatile bool m_stop;
    volatile bool m_running;

    static void*  Thread_1(void* args);
    static void*  Thread_2(void* args);
    virtual void  socketProcess();
    virtual void  beamProcess();
};

#endif //CTHREADS

