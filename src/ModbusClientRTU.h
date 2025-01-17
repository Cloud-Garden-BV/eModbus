// =================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to eModbus
//               MIT license - see license.md for details
// =================================================================================================
#ifndef _MODBUS_CLIENT_RTU_H
#define _MODBUS_CLIENT_RTU_H

#include "options.h"

#if HAS_FREERTOS

#include "ModbusClient.h"
#include "HardwareSerial.h"
#include "RTUutils.h"
#include <queue>

#if USE_MUTEX
#include <mutex>                  // NOLINT
#endif
#include <vector>

using std::queue;
#if USE_MUTEX
using std::mutex;
using std::lock_guard;
#endif

#define DEFAULTTIMEOUT 2000

class ModbusClientRTU : public ModbusClient {
public:
  // Base addRequest must be present
  Error addRequest(ModbusMessage msg, uint32_t token);

template <typename... Args>
Error addRequest(uint32_t token, Args&&... args) {
  Error rc = SUCCESS;        // Return value

  // Create request, if valid
  ModbusMessage m;
  rc = m.setMessage(std::forward<Args>(args) ...);

  // Add it to the queue, if valid
  if (rc == SUCCESS) {
    // Queue add successful?
    if (!addToQueue(token, m)) {
      // No. Return error after deleting the allocated request.
      rc = REQUEST_QUEUE_FULL;
    }
  }
  return rc;
}

  // Constructor takes Serial reference and optional DE/RE pin and queue limit
  explicit ModbusClientRTU(HardwareSerial& serial, int8_t rtsPin = -1, uint16_t queueLimit = 100);

  // Destructor: clean up queue, task etc.
  ~ModbusClientRTU();

  void end();
  // begin: start worker task
  void begin(int coreID = -1);

  // Set default timeout value for interface
  void setTimeout(uint32_t TOV);
  void setRTSPinCallback( std::function<void( bool level)> func );

protected:
  struct RequestEntry {
    uint32_t token;
    ModbusMessage msg;
    RequestEntry(uint32_t t, ModbusMessage m) :
      token(t),
      msg(m) {}
  };

  // addToQueue: send freshly created request to queue
  bool addToQueue(uint32_t token, ModbusMessage msg);

  // handleConnection: worker task method
  static void handleConnection(ModbusClientRTU *instance);

  // receive: get response via Serial
  ModbusMessage receive(const ModbusMessage request);

  void isInstance() { return; }   // make class instantiable
  queue<RequestEntry> requests;   // Queue to hold requests to be processed
  #if USE_MUTEX
  mutex qLock;                    // Mutex to protect queue
  #endif
  HardwareSerial& MR_serial;      // Ptr to the serial interface used
  uint32_t MR_lastMicros;         // Microseconds since last bus activity
  uint32_t MR_interval;           // Modbus RTU bus quiet time
  int8_t MR_rtsPin;               // GPIO pin to toggle RS485 DE/RE line. -1 if none.
  std::function<void( bool level)> _RTSPinCB; // Callback reference for the RS485 RE/DE line.
  uint16_t MR_qLimit;             // Maximum number of requests to hold in the queue
  uint32_t MR_timeoutValue;       // Interface default timeout

};

#endif  // HAS_FREERTOS

#endif  // INCLUDE GUARD
