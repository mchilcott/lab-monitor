#pragma once

#include <vector>
#include <map>

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

  String mClientName;

 public:
  DCThread(
           ThreadManager &mgr,
           const char *SensorTopic,
           unsigned int period = 0
           )
   : Thread(mgr), mClient(), mTopic(SensorTopic),  mPeriod(period), mLastMillis(millis())
    {
      String name = "MonitorNode: ";
      name += mTopic;
      mClientName = name;
    };
  
    virtual void beginMQTT (
            const char *MQTTHost,
            const int MQTTPort
           )
    {
      mClient.begin(MQTTHost, MQTTPort, mWiFi);
    }

  bool try_connect() {
    if (WiFi.status() != WL_CONNECTED){
      // If wifi isn't connected, yeild
      Serial.println("WiFi Disconnected");
      return false;
    }
    Serial.println("Offline");
    String name = "MonitorNode: ";
    name += mTopic;
    return mClient.connect(name.c_str());
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

class CallbackBroker {
    private:
        static std::map<MQTTClient *, std::function<void(String &, String &)>> msCallbacks;
    
    
    public:
        static void callback_target(MQTTClient * client, char topic[], char bytes[], int length)
        {
            auto it = msCallbacks.find(client);
            
            if (it == msCallbacks.end())
            {
                Serial.println("ERROR: CallbackBroker failed to find the right client");
                return;
            }
            // create topic string
            String str_topic = String((const char *)topic);
            // create payload string
            String str_payload;
            if ((const char *) bytes != nullptr) 
                str_payload = String((const char *)bytes);
                
            (*it).second(str_topic, str_payload);
        }
        
        static void register_cb (MQTTClient *client, std::function<void(String &, String &)> func)
        {
            msCallbacks.insert(std::make_pair(client, func));
        }
        
};

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

class ControlThread : public DCThread {
 protected:
  bool mLastConnectionState;

 public:
  ControlThread(
           ThreadManager &mgr,
           const char *topic,
           unsigned int period = 0
           )
   : DCThread(mgr, topic, period), mLastConnectionState(false)
  {};
  
  virtual void beginMQTT (
            const char *MQTTHost,
            const int MQTTPort
           )
    {
        mClient.begin(MQTTHost, MQTTPort, mWiFi);
        CallbackBroker::register_cb(&mClient, [this](String &a, String &b){this->onMessage(a,b);});
        mClient.onMessageAdvanced(&CallbackBroker::callback_target);
        //mClient.onMessage(messageReceived);
    }

  virtual void poll () {};
  
  virtual void loop() {
    // Handle disconnects/reconnects
    bool connected = mClient.connected();

    if(connected != mLastConnectionState)
    {
      // There has been a change of state
      if(connected) onConnect();
      else onDisconnect();

      mLastConnectionState = connected;
    }

    // Usual thread loop
    DCThread::loop();
  }

  virtual void onDisconnect() {};

  virtual void onConnect() 
    {
      //mClient.subscribe(mTopic);
    };
  
  virtual void onMessage(String &topic, String &payload) {};
};


class DigitalOutput : public ControlThread {
  public:
    unsigned int mPin;
    bool mActiveHigh;

    DigitalOutput(
          ThreadManager &mgr,
          const char *topic = "control/default/digital",
          unsigned int default_state = LOW,
          bool active_high = true,
          bool open_drain = false,
          unsigned int pin = 16, // NodeMCU D0,
          unsigned int period = 0
    ): ControlThread(mgr, topic, period), mPin(pin), mActiveHigh(active_high)
    {
      pinMode(mPin, open_drain ? OUTPUT_OPEN_DRAIN : OUTPUT);
      digitalWrite(mPin, default_state);
    }

    virtual void onConnect() 
    {
      mClient.subscribe(mTopic);
    }

    virtual void onMessage(String &topic, String &payload)
    {
      if (topic == mTopic)
      {
        if (payload.equalsIgnoreCase("ON") || payload.equalsIgnoreCase("TRUE") || payload.equalsIgnoreCase("1"))
          digitalWrite(mPin, mActiveHigh ? HIGH : LOW);
        else if (payload.equalsIgnoreCase("OFF") || payload.equalsIgnoreCase("FALSE") || payload.equalsIgnoreCase("0"))
          digitalWrite(mPin, mActiveHigh ? LOW : HIGH);
        else if (payload.equalsIgnoreCase("HIGH"))
          digitalWrite(mPin, HIGH);
        else if (payload.equalsIgnoreCase("LOW"))
          digitalWrite(mPin, LOW);
        else
          Serial.println("DigitalOutput: Got invalid command.");
        
      }
    }
};


class HomieThread : public ControlThread {
    private:
        String mTopicBase;
        String mLabel;
        String mStatusTopic;
        String mNodeName;
    public:
    HomieThread(
           ThreadManager &mgr,
           String node_name,
           const char *basetopic = "status/",
           unsigned int period = 30000
    ) : mNodeName(node_name), ControlThread(mgr, basetopic, period)
    {
        mTopicBase = String(basetopic);
        
        mLabel = String(node_name);
        
        mTopicBase += String(ESP.getChipId(), HEX);
        mTopicBase += "/";
        mTopic = mTopicBase.c_str();
        
        mStatusTopic = mTopicBase + "$status";
        
        mClient.setWill(mStatusTopic.c_str(), "lost");
        
        String label =" Node:";
        label += String(ESP.getChipId(), HEX);
        label += ":";
        label += node_name;
        mClientName = label;
    }

    
    virtual void poll ()
    {
        mClient.publish(mTopicBase + "$name", mNodeName);
        mClient.publish(mTopicBase + "$localip", WiFi.localIP().toString());
        mClient.publish(mTopicBase + "$mac", WiFi.macAddress());
        mClient.publish(mTopicBase + "$stats/rssi", String() + WiFi.RSSI());
        mClient.publish(mTopicBase + "$stats/interval", String() + mPeriod);
    
        // Build a node string
        String node_str = "";
    
        //for (auto it = collectors.begin(); it != collectors.end(); ++it)
        //{
        //  node_str += (*it)->topic();
        //  node_str += ',';
        //}
    
        mClient.publish(mTopicBase + "$nodes", node_str + "[]");
        mClient.publish(mTopicBase + "$fw/name", "ArduinoMonitor");
        mClient.publish(mTopicBase + "$fw/version", __DATE__ " " __TIME__);
        mClient.publish(mTopicBase + "$implementation", "NodeMCU");
    }
    
    void publish_status (const char * new_status)
    {
        mClient.publish(mStatusTopic, new_status);
    }
};
