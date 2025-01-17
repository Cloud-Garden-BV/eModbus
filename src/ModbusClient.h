// =================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to eModbus
//               MIT license - see license.md for details
// =================================================================================================
#ifndef _MODBUS_CLIENT_H
#define _MODBUS_CLIENT_H

#include "options.h"

#include "ModbusMessage.h"
#include <functional>

#if HAS_FREERTOS
extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
}
#elif IS_LINUX
#include <pthread.h>
#endif

typedef void (*MBOnData) (ModbusMessage msg, uint32_t token);
typedef void (*MBOnError) (Modbus::Error errorCode, uint32_t token);

class ModbusClient {
public:
  bool onDataHandler(MBOnData handler);   // Accept onData handler 
  bool onErrorHandler(MBOnError handler); // Accept onError handler 
  uint32_t getMessageCount();              // Informative: return number of messages created
  // Virtual addRequest variant needed internally. All others done in the derived client classes by template!
  virtual Error addRequest(ModbusMessage msg, uint32_t token) = 0;
  
protected:
  ModbusClient();             // Default constructor
  virtual void isInstance() = 0;   // Make class abstract
  uint32_t messageCount;           // Number of requests generated. Used for transactionID in TCPhead
#if HAS_FREERTOS
  TaskHandle_t worker;             // Interface instance worker task
#elif IS_LINUX
  pthread_t worker;
#endif
  MBOnData onData;                 // Response data handler
  MBOnError onError;               // Error response handler
  static uint8_t instanceCounter; // Number of ModbusClients created
  std::function<void( bool level)> _RTSPinCB; // Callback reference for the RS485 RE/DE line.

};

#endif
