#pragma once

#include <vector>

/*
  This particular library is for having multiple sensor threads for my
  NodeMCU monitoring system.  This system was prototyped in Lua using
  the NodeMCU firmware, but this quickly ran out of memory. Hopefully
  the firmware generated this way is more reliable.

  Main Idea:

   - There is one instance of ThreadManager, to which threads are
     added. The ThreadManager's handle() method must be called
     regularly to ensure that the threads are run. (I.e. it should be
     called in arduino's loop() function.)

   - Threads look a lot like an arduino program. They primarily have
     an init(), which is the constructor, and a loop() method, also
     called this. The loop method should return quickly: i.e. delay()
     statements are strictly forbidden. There [will be] a helper class
     to do things like that.

   - The loop() method of a thread returns objects which allow the
     ThreadManager to control the running of the thread.

   - These classes avoid doing threading using system calls and stack
     unravelling at all costs.
 */


/* Partial Declaration of the Thread Class */
class ThreadManager;

class Thread {
 protected:
  ThreadManager &mManager;
 public:
  Thread(ThreadManager &mgr);
  virtual void loop() = 0;
};

/**
   The thread manager is responsible for giving a pool of threads CPU
   time. This manager is non-preemptive, and requires the threads to
   co-operate and not hang.
 */
class ThreadManager {
 private:
  std::vector<Thread *> mThreads;

  public:

  void handle()
  {
    for(auto it = mThreads.begin(); it != mThreads.end(); it++)
      {
        delay(0);
        (*it)->loop();
      }
  }
  
  void add(Thread *thread)
  {
    mThreads.push_back(thread);
  }

};

/**
   A thread is a virtual object which is based around the idea of a
   single-purpose arduino sketch. It initialises in the constructor,
   and then has a loop method which is called (nearly) continuously.
 */

Thread::Thread(ThreadManager &mgr) : mManager(mgr)
  {mManager.add(this);}


#include <MQTT.h>
#include <ESP8266WiFi.h>

class DummyThread : public Thread {
  public:
    DummyThread (ThreadManager &tm):
      Thread(tm) {}

    virtual void loop () {}
};

/**
   Time to add some features for data collection.
   
   A DCThread with an MQTT object which the thread relies on. Also
   includes an option to run a polled function at a given
   period. Polling will not happen if there is no MQTT connection.
 */

class DCThread : public Thread {
 protected:
  // MQTT Connection
  MQTTClient mClient;
  WiFiClient mWiFi;
  const char * mTopic;

  // Timing for poll()
  unsigned int mPeriod;
  unsigned int mLastMillis;


 
 public:
  DCThread(
           ThreadManager &mgr,
           const char *SensorTopic,
           unsigned int period = 0
           )
   : Thread(mgr), mClient(), mTopic(SensorTopic),  mPeriod(period), mLastMillis(millis())
  {};
  
  void beginMQTT (
            const char *MQTTHost,
            const int MQTTPort
           )
    {
      mClient.begin(MQTTHost, MQTTPort, mWiFi);
    }

  void try_connect() {
    if (WiFi.status() != WL_CONNECTED){
      // If wifi isn't connected, yeild
      Serial.println("WiFi Disconnected");
      return;
    }
    Serial.println("Offline");
    String name = "MonitorNode: ";
    name += mTopic;
    mClient.connect(name.c_str());
  }

  const char * topic ()
  {return mTopic;}

  virtual void poll () {}
  
  virtual void loop() {
    // Let MQTT do its thing
    mClient.loop();
    // Try to connect if we're not
    if (!mClient.connected()) {
      try_connect();
      return;
    }
    // Wait for the period to call the poll() function
    if (mPeriod > 0) {
      if (millis() - mLastMillis > mPeriod){
        mLastMillis = millis();
        poll();
      }
    }
  }
};
