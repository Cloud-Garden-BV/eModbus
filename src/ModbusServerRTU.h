// =================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to eModbus
//               MIT license - see license.md for details
// =================================================================================================
#ifndef _MODBUS_SERVER_RTU_H
#define _MODBUS_SERVER_RTU_H

#include "options.h"

#if HAS_FREERTOS

#include <Arduino.h>
#include "HardwareSerial.h"
#include "ModbusServer.h"
#include "RTUutils.h"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
}

class ModbusServerRTU : public ModbusServer {
public:
  // Constructor
  ModbusServerRTU(HardwareSerial& serial, uint32_t timeout, int rtsPin = -1);

  // Destructor
  ~ModbusServerRTU();

  // start: create task with RTU server to accept requests
  bool start(int coreID = -1);

  // stop: kill server task
  bool stop();

  // If the RTS pin is external, then call the external function to set the RTS pin low/high.
  // Use this for example when the RE/DE pin is connected to an IO expander
  void          setRTSPinCallback( std::function<void( bool level)> func );
protected:
  // Prevent copy construction and assignment
  ModbusServerRTU(ModbusServerRTU& m) = delete;
  ModbusServerRTU& operator=(ModbusServerRTU& m) = delete;

  inline void isInstance() { }           // Make class instantiable

  static uint8_t instanceCounter;        // Number of RTU servers created (for task names)
  TaskHandle_t serverTask;               // task of the started server
  uint32_t serverTimeout;                // given timeout for receive. Does not really
                                         // matter for a server, but is needed in 
                                         // RTUutils. After timeout without any message
                                         // the server will pause ~1ms and start 
                                         // receive again.
  HardwareSerial& MSRserial;             // The serial interface to use
  uint32_t MSRinterval;                  // Bus quiet time between messages
  uint32_t MSRlastMicros;                // microsecond time stamp of last bus activity
  int8_t MSRrtsPin;                    // GPIO number of the RS485 module's RE/DE line
  std::function<void( bool level)> _RTSPinCB; // Callback reference for the RS485 RE/DE line.

  // serve: loop function for server task
  static void serve(ModbusServerRTU *myself);
};

#endif  // HAS_FREERTOS

#endif // INCLUDE GUARD
