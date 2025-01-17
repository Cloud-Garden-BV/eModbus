// =================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to eModbus
//               MIT license - see license.md for details
// =================================================================================================
#include "ModbusClientRTU.h"

#if HAS_FREERTOS

#undef LOCAL_LOG_LEVEL
// #define LOCAL_LOG_LEVEL LOG_LEVEL_VERBOSE
#include "Logging.h"

// Constructor takes Serial reference and optional DE/RE pin
ModbusClientRTU::ModbusClientRTU(HardwareSerial& serial, int8_t rtsPin, uint16_t queueLimit) :
  ModbusClient(),
  MR_serial(serial),
  MR_lastMicros(micros()),
  MR_interval(2000),
  MR_rtsPin(rtsPin),
  MR_qLimit(queueLimit),
  MR_timeoutValue(DEFAULTTIMEOUT) {
    // Switch serial FIFO buffer copy threshold to 1 byte (normally is 112!)
    RTUutils::UARTinit(serial, 1);
}
void ModbusClientRTU::setRTSPinCallback( std::function<void( bool level)> func ) {
  _RTSPinCB = func;
  // If valid, then set it low, since that's the "default"
  LOG_D("Set RTS pin callback.\n");
  if(_RTSPinCB !=NULL) {
        _RTSPinCB(LOW); // Set the external RE/DE pin low when starting
  }
}
// Destructor: clean up queue, task etc.
ModbusClientRTU::~ModbusClientRTU() {
  // Clean up queue
  {
    // Safely lock access
    LOCK_GUARD(lockGuard, qLock);
    // Get all queue entries one by one
    while (!requests.empty()) {
      // Remove front entry
      requests.pop();
    }
  }
  // Kill task
  vTaskDelete(worker);
  LOG_D("Worker task %d killed.\n", (uint32_t)worker);
}

// stop all related things
void ModbusClientRTU::end() {
  // Clean up queue
  {
    // Safely lock access
    LOCK_GUARD(lockGuard, qLock);
    // Get all queue entries one by one
    while (!requests.empty()) {
      // Remove front entry
      requests.pop();
    }
  }
  // Kill task
  vTaskDelete(worker);
  LOG_D("Worker task %d killed.\n", (uint32_t)worker);
}

// begin: start worker task
void ModbusClientRTU::begin(int coreID) {
  // Only start worker if HardwareSerial has been initialized!
  if (MR_serial.baudRate()) {
    // If rtsPin is >=0, the RS485 adapter needs send/receive toggle
    if (MR_rtsPin >= 0) {
      pinMode(MR_rtsPin, OUTPUT);
      digitalWrite(MR_rtsPin, LOW);
    }

    // silent interval is at least 3.5x character time
    MR_interval = 35000000UL / MR_serial.baudRate();  // 3.5 * 10 bits * 1000 µs * 1000 ms / baud

    // Create unique task name
    char taskName[12];
    snprintf(taskName, 12, "Modbus%02XRTU", instanceCounter);
    // Start task to handle the queue
    xTaskCreatePinnedToCore((TaskFunction_t)&handleConnection, taskName, 4096, this, 6, &worker, coreID >= 0 ? coreID : NULL);

    LOG_D("Worker task %d started. Interval=%d\n", (uint32_t)worker, MR_interval);
  } else {
    LOG_E("Worker task could not be started! HardwareSerial not initialized?\n");
  }
}

// setTimeOut: set/change the default interface timeout
void ModbusClientRTU::setTimeout(uint32_t TOV) {
  MR_timeoutValue = TOV;
  LOG_D("Timeout set to %d\n", TOV);
}

// Base addRequest taking a preformatted data buffer and length as parameters
Error ModbusClientRTU::addRequest(ModbusMessage msg, uint32_t token) {
  Error rc = SUCCESS;        // Return value

  LOG_D("request for %02X/%02X\n", msg.getServerID(), msg.getFunctionCode());

  // Add it to the queue, if valid
  if (msg) {
    // Queue add successful?
    if (!addToQueue(token, msg)) {
      // No. Return error after deleting the allocated request.
      rc = REQUEST_QUEUE_FULL;
    }
  }

  LOG_D("RC=%02X\n", rc);
  return rc;
}

// addToQueue: send freshly created request to queue
bool ModbusClientRTU::addToQueue(uint32_t token, ModbusMessage request) {
  bool rc = false;
  // Did we get one?
  if (request) {
    RequestEntry re(token, request);
    if (requests.size()<MR_qLimit) {
      // Yes. Safely lock queue and push request to queue
      rc = true;
      LOCK_GUARD(lockGuard, qLock);
      requests.push(re);
    }
    messageCount++;
  }

  LOG_D("RC=%02X\n", rc);
  return rc;
}

// handleConnection: worker task
// This was created in begin() to handle the queue entries
void ModbusClientRTU::handleConnection(ModbusClientRTU *instance) {
  // initially clean the serial buffer
  while (instance->MR_serial.available()) instance->MR_serial.read();
  delay(100);

  // Loop forever - or until task is killed
  while (1) {
    // Do we have a reuest in queue?
    if (!instance->requests.empty()) {
      // Yes. pull it.
      RequestEntry request = instance->requests.front();

      LOG_D("Pulled request from queue with baud %u and conf %u\n",request.msg.getBaudrate(), request.msg.getSerialConfig());
      // First check if the serial configuration is different from the current configuration, and change it if necessary
      RTUutils::handleSerialConfig(instance->MR_serial, instance->MR_interval, request.msg.getBaudrate(), request.msg.getSerialConfig());
      // Send it via Serial
      // if there is a RTS callback registered, then use that
      if(instance->_RTSPinCB!=NULL){
        RTUutils::send(instance->MR_serial, instance->MR_lastMicros, instance->MR_interval, instance->_RTSPinCB, request.msg);
      }else{ // otherwise, directly toggle the RTS pin.
        RTUutils::send(instance->MR_serial, instance->MR_lastMicros, instance->MR_interval, instance->MR_rtsPin, request.msg);
      }

      LOG_D("Request sent.\n");
      // HEXDUMP_V("Data", request.msg.data(), request.msg.size());

      // Get the response - if any
      ModbusMessage response = RTUutils::receive(instance->MR_serial, instance->MR_timeoutValue, instance->MR_lastMicros, instance->MR_interval);

      LOG_D("%s response (%d bytes) received.\n", response.size()>1 ? "Data" : "Error", response.size());
      HEXDUMP_V("Data", response.data(), response.size());

      // No error in receive()?
      if (response.size() > 1) {
        // No. Check message contents
        // Does the serverID match the requested?
        if (request.msg.getServerID() != response.getServerID()) {
          // No. Return error response
          response.setError(request.msg.getServerID(), request.msg.getFunctionCode(), SERVER_ID_MISMATCH);
        // ServerID ok, but does the FC match as well?
        } else if (request.msg.getFunctionCode() != (response.getFunctionCode() & 0x7F)) {
          // No. Return error response
          response.setError(request.msg.getServerID(), request.msg.getFunctionCode(), FC_MISMATCH);
        // Both serverID and FC are ok - how about the CRC?
        } else if (!RTUutils::validCRC(response)) {
          // CRC faulty - return error
          response.setError(request.msg.getServerID(), request.msg.getFunctionCode(), CRC_ERROR);
        // Everything seems okay
        } else {
          // Build response from received message (cut off CRC)
          response.resize(response.size() - 2);
        }
      } else {
        // No, we got an error code from receive()
        // Return it as error response
        response.setError(request.msg.getServerID(), request.msg.getFunctionCode(), static_cast<Error>(response[0]));
      }

      LOG_D("Response generated.\n");
      HEXDUMP_V("Response packet", response.data(), response.size());

      // Did we get a normal response?
      if (response.getError()==SUCCESS) {
        // Yes. Do we have an onData handler registered?
        if (instance->onData) {
          // Yes. call it
          instance->onData(response, request.token);
        }
      } else {
        // No, something went wrong. All we have is an error
        // Do we have an onError handler?
        if (instance->onError) {
          // Yes. Forward the error code to it
          instance->onError(response.getError(), request.token);
        }
      }
      // Clean-up time. 
      {
        // Safely lock the queue
        LOCK_GUARD(lockGuard, instance->qLock);
        // Remove the front queue entry
        instance->requests.pop();
      }
    } else {
      delay(1);
    }
  }
}

#endif  // HAS_FREERTOS
