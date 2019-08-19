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
     statements are strictly forbidden. There [maybe will be] a helper class
     to do things like that?

   - The loop() method of a thread returns objects which allow the
     ThreadManager to control the running of the thread.

   - These classes avoid doing threading using system calls and stack
     unravelling at all costs.
 */


/* Forward Declaration to make everything work */
class ThreadManager;

/**
 * Prototype Thread Class
 * 
 * A thread is a virtual object which is based around the idea of a
 * single-purpose arduino sketch. It initialises in the constructor,
 * and then has a loop method which is called repeatedly.
 * 
 * This class contains a pure virtual function, and cannot be instantiated.
 */
class Thread {
 public:
  ThreadManager *mParent; //!< The Manager looking after this thread

  /**
   * Default Constructor
   */
  Thread() : mParent(nullptr)
    {}

  /**
   * The thread's main loop. Each call should run one iteration.
   * 
   * The call to this function should not include calls to delay() as 
   * this function should aim to complete as quickly as possible.
   */
  virtual void loop() = 0;
};

/**
 * Manage a list of threads. Each thread follows the same idea as the Arduino framework,
 * with some initial setup (in the constructor of the thread), and a loop() function, which is called regularly.
 * 
 * This manager performs Cooperative multitasking -- each thread will be given however much time it takes to run loop(), without being interrupted.
 * For a functional, responsive system, threads must take care to complete a call to loop() quickly. If threads take too long,
 * the watchdog timer on the ESP8266 will decide that the processor is stuck, and reset the whole system. Long execution times
 * can also interfere with WiFi performance.
 * 
 * Interrupts are serviced as usual, and interrupt the running threads.
 */
class ThreadManager {
 private:
  std::vector<Thread *> mThreads; //!< List of threads being managed

  public:
  /**
   * Call loop() on every thread. This function should be put in the main loop of the arduino framework.
   */
  void handle()
  {
    for(auto it = mThreads.begin(); it != mThreads.end(); it++)
      {
        delay(0);
        (*it)->loop();
      }
  }


  /**
   * Add a new thread to be managed by this manager
   */  
  void add(Thread *thread)
  {
    mThreads.push_back(thread);
    thread->mParent = this;
  }

};


/**
 * A valid thread which does nothing. For debugging/testing purposes.
 */
class DummyThread : public Thread {
  public:
    DummyThread ():
      Thread() {}

    virtual void loop () {}
};

#include <MQTT.h>
#include <ESP8266WiFi.h>

/**
 * A DCThread (Data Collection Thread) with an MQTT object which the thread relies on.
 * 
 * This thread has some knowlege of the network, and owns its own MQTT connection which it manages.
 * 
 * The thread also has a poll() function to be called at a configurable interval. (E.g. to collect a datapoint)
 */
class DCThread : public Thread {
 protected:
  // MQTT Connection
  MQTTClient mClient;
  WiFiClient mWiFi;
  const char * mTopic;
  const char * mUser;
  const char * mPass;

  // Timing for poll()
  unsigned int mPeriod;
  unsigned int mLastMillis;

  String mClientName;

  bool mInit;

 public:
   /**
   * Default Constructor. It is expected that this class will be inherited from, and that this constructor will only be used from a child class.
   * 
   * \param SensorTopic The MQTT topic associated with this data. This is used as an identifier for the thread. It doesn't *need* to correspond to a real measurement (depending on the subclass), but should provide a useful name, and be unique to the monitoring system.
   * 
   * \param period The period with which the poll() function should be called. In milliseconds. Set to 0 to disable.
   */
  DCThread(
           const char *SensorTopic,
           unsigned int period = 0
           )
   : Thread(), mClient(), mTopic(SensorTopic),  mPeriod(period), mLastMillis(millis()), mInit(false)
    {
      String name = "MonitorNode";
      name += String(ESP.getChipId(), HEX);
      
      name += ": ";
      name += mTopic;
      
      mClientName = name;
    };
  
  /**
   * Begin the MQTT instance. This should be called from the main program to give MQTT access details.
   */
  virtual void beginMQTT (
          const char *MQTTHost,
          const int MQTTPort,
          const char * MQTTUsername,
          const char * MQTTPassword
         )
  {
    mClient.begin(MQTTHost, MQTTPort, mWiFi);
    mUser = MQTTUsername;
    mPass = MQTTPassword;
  }

  /**
   * Attempt to form a connection to the MQTT server. This should be handled automatically.
   */
  bool try_connect() {
    if (WiFi.status() != WL_CONNECTED){
      // If wifi isn't connected, yeild
      Serial.println("WiFi Disconnected");
      return false;
    }
    bool success = mClient.connect(mClientName.c_str(), mUser, mPass);
    return success;
  }

  /**
   * Get the MQTT topic associated with this thread
   */
  const char * topic ()
  {return mTopic;}

  /**
   * The virtual poll() function to be implemented by subclasses.
   */
  virtual void poll () {}

  /**
   * An init() function to be implemented by subclasses. This 
   */
  virtual void init () {}
  
  /**
   * The main loop of the DCThead. This is virtual, so can be reimplemented by subclasses, but should be called in the subclass's main loop to ensure
   * that the MQTT connection and polling is properly managed.
   */
  virtual void loop() {
    // Let MQTT do its thing
    mClient.loop();
    // Try to connect if we're not
    if (!mClient.connected()) {
      try_connect();
      return;
    }

    if(!mInit)
      {
        mInit = true;
        init();
      }


    // Wait for the period to call the poll() function
    unsigned long currentMillis = millis();
    if (mPeriod > 0) {
      if (currentMillis - mLastMillis > mPeriod){
        mLastMillis = currentMillis;
        poll();
      }
    }
  }
};


/**
 * A broker to allow objects to collect MQTT messages to a std::function. This allows calling a non-static member function, which
 * was otherwise difficult with the MQTT library.
 * 
 * This class is not designed for the user to interact with, but to cause ControlThread to work nicely and without hackery.
 * 
 * \warning
 * The main cpp file should instantiate the callback map:
 \verbatim
 std::map<MQTTClient *, std::function<void(String &, String &)>> MQTTCallbackBroker::msCallbacks;
 \endverbatim
 */
class MQTTCallbackBroker {
    private:
        /**
         * A mapping between MQTTClients, and the callback function for messages received on that client.
         */
        static std::map<MQTTClient *, std::function<void(String &, String &)>> msCallbacks;
    
    
    public:
        /**
         * Each MQTT object should have this function as it's callback for proper operation. This
         * function looks up the std::function to be called in the map.
         * 
         * This function will be called by the MQTT library, not by any of our code.
         */
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
        
        /**
         * Register a new callback.
         * 
         * \param client The MQTTClient object which expects to be receiving messages
         * \param func std::function callback object. This will receive the topic and message sent to the client.
         */
        static void register_cb (MQTTClient *client, std::function<void(String &, String &)> func)
        {
            msCallbacks.insert(std::make_pair(client, func));
        }
        
};

/**
 * While DCThreads are for transmitting data, ControlThreads add receiving data capabilities. By
 * subscribing to MQTT topics, ControlThreads outline the ability for the nodes to have actions.
 */
class ControlThread : public DCThread {
 protected:
  bool mLastConnectionState;

 public:
  /**
   * \param topic A MQTT topic to be associated with this thread. This does not have to be the thread that is listened/transmitted to, but provides a way of naming the instance.
   * \param period poll() period as per DCThread.
   */
  ControlThread(
           const char *topic,
           unsigned int period = 0
           )
   : DCThread(topic, period), mLastConnectionState(false)
  {};
  
  virtual void beginMQTT (
          const char *MQTTHost,
          const int MQTTPort,
          const char * MQTTUsername,
          const char * MQTTPassword
         )
    {
        DCThread::beginMQTT(MQTTHost, MQTTPort, MQTTUsername, MQTTPassword);
        MQTTCallbackBroker::register_cb(&mClient, [this](String &a, String &b){this->onMessage(a,b);});
        mClient.onMessageAdvanced(&MQTTCallbackBroker::callback_target);
    }
  
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

  /**
   * Subscribe the MQTT client to a topic. This function should probably be called from onConnect().
   */
  void subscribe(const char *topic)
    {
      mClient.subscribe(topic);
    }

  /**
   * This function is called if the MQTTClient disconnects
   */
  virtual void onDisconnect() {};

  /**
   * This function is called when the MQTTClient connects. This is the appropriate place to
   * make subscriptions to topics.
   */
  virtual void onConnect() 
    {};
  
  /**
   * Called when a message is received on the MQTTClient.
   * 
   * \param topic MQTT Topic that message was received on
   * \param payload MQTT message received
   */
  virtual void onMessage(String &topic, String &payload) {};
};

/**
 * Thread which listens for MQTT commands to produce digital signals.
 * 
 * This is ideal for e.g. switching relays in response to control commands.
 * 
 * Note that calls to onConnect(), onMessage() are handled by the parent class, we
 * just need to implement their behaviour.
 */
class DigitalOutput : public ControlThread {
  public:
    unsigned int mPin;
    bool mActiveHigh;

    DigitalOutput(
          const char *topic = "control/default/digital",
          unsigned int default_state = LOW,
          bool active_high = true,
          bool open_drain = false,
          unsigned int pin = 16, // NodeMCU D0,
          unsigned int period = 0
    ): ControlThread(topic, period), mPin(pin), mActiveHigh(active_high)
    {
      pinMode(mPin, open_drain ? OUTPUT_OPEN_DRAIN : OUTPUT);
      digitalWrite(mPin, default_state);
    }

    /**
     * Subscribe to control MQTT topic when connected.
     */
    virtual void onConnect() 
    {
      subscribe(mTopic);
    }

    /**
     * 
     */
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

/**
 * Implement (parts) of the [Homie](https://homieiot.github.io/) convention.
 * 
 * This implementation is not complete, nor do we really try to be, but it does provide a convenient
 * way of learning about connected nodes. Completeness may be added in the future, but this is probably unlikely.
 * 
 * This thread sends out some information about the monitor node periodically. By listening to all subtopics of the base topic,
 * one can keep an eye on which nodes are running, what measurements they make, etc.
 */
class HomieThread : public ControlThread {
    private:
        String mTopicBase;
        String mLabel;
        String mStatusTopic;
        String mNodeName;
        std::vector<DCThread *> &mCollectors;
    public:
    HomieThread(
           String node_name,
           std::vector<DCThread *> &collectors,
           const char *basetopic = "status/",
           unsigned int period = 60000
    ) : mNodeName(node_name), ControlThread(basetopic, period), mCollectors(collectors)
    {
        mTopicBase = String(basetopic);
        
        mLabel = String(node_name);
        
        mTopicBase += String(ESP.getChipId(), HEX);
        mTopicBase += "/";
        mTopic = mTopicBase.c_str();
        
        mStatusTopic = mTopicBase + "$state";
        
        mClient.setWill(mStatusTopic.c_str(), "lost");
        
        String label =" Node:";
        label += String(ESP.getChipId(), HEX);
        label += ":";
        label += node_name;
        mClientName = label;
    }

    /**
     * Publish a complete list of status info. 
     */
    virtual void poll ()
    {
        // Publish some general status info
        mClient.publish(mTopicBase + "$name", mNodeName);
        mClient.publish(mTopicBase + "$type", "MonitoringNode");
        mClient.publish(mTopicBase + "$properties", "");

        mClient.publish(mTopicBase + "$hostname", wifi_station_get_hostname());


        mClient.publish(mTopicBase + "$localip", WiFi.localIP().toString());
        mClient.publish(mTopicBase + "$mac", WiFi.macAddress());
        mClient.publish(mTopicBase + "$stats/rssi", String() + WiFi.RSSI());
        mClient.publish(mTopicBase + "$stats/interval", String() + (mPeriod/1000.0));

        mClient.publish(mTopicBase + "$stats/uptime", String()+ (micros64()/1e6));
        mClient.publish(mTopicBase + "$stats/freeheap", String() = ESP.getFreeHeap());
        mClient.publish(mTopicBase + "$stats/heapfragment", String() = ESP.getHeapFragmentation());
    
        // Build a string with a list of the currently monitored signals.
        String node_str = "";
        for (auto it = mCollectors.begin(); it != mCollectors.end(); ++it)
        {
          node_str += (*it)->topic();
          node_str += ',';
        }
    
        mClient.publish(mTopicBase + "$nodes", node_str + "[]");
        mClient.publish(mTopicBase + "$fw/name", "ArduinoMonitor");
        mClient.publish(mTopicBase + "$fw/version", __DATE__ " " __TIME__);
        mClient.publish(mTopicBase + "$implementation", "NodeMCU");
    }
    
    /**
     * Publish a new device state (e.g. init, ready, alert).
     * 
     * See https://homieiot.github.io/specification/#device-behavior
     */
    void publish_status (const char * new_status)
    {
        mClient.publish(mStatusTopic, new_status);
    }
};
