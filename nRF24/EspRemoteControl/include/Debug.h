#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

/**
 * simple helper class for debug printing
 */
class Debug {
  
  public:

    /**
     * prints the supplied message in printf format to the serial interface
     * and sends to the MQTT Debug topic if MQTT is connected and enabled
     * @param format printf style format string
     * @param ... additional arguments as required by the format string
     */
    static void log(const char* format, ...);

    /**
     * enable or disable MQTT debug messages
     * @param enable true to enable, false to disable
     */
    static void enableMQTTDebug(bool enable) {
      mqttDebugEnabled = enable;
    }

  private:
    static bool mqttDebugEnabled;

};

#endif // DEBUG_PRINT_H
