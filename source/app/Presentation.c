////////////////////////////////////////////////////////////////////////////////
//  S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023, Sensirion AG
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// @file Presentation.c
///
/// Implementation of presentation controller
///

#include "Presentation.h"

#include "app_service/item_store/ItemStore.h"
#include "app_service/networking/ble/BleGatt.h"
#include "app_service/networking/ble/BleInterface.h"
#include "app_service/nvm/ProductionParameters.h"
#include "app_service/power_manager/BatteryMonitor.h"
#include "app_service/sensor/Sht4x.h"
#include "app_service/timer_server/TimerServer.h"
#include "app_service/user_button/Button.h"
#include "hal/Uart.h"
#include "stm32wbxx_ll_cortex.h"
#include "utility/AppDefines.h"
#include "utility/ErrorHandler.h"
#include "utility/log/Log.h"
#include "utility/scheduler/Message.h"
#include "utility/scheduler/MessageId.h"

#include <stdbool.h>
#include <string.h>

/// Defines the timeout in seconds when the pairing is aborted
#define PAIRING_TIMEOUT_S 30
/// Timer ID sensor readout trigger timer
static uint8_t _sht4xReadoutTimer;

/// The time in seconds between two successive TIME_ELAPSES messages
/// Initially this delta is set to one seconds.
static uint8_t _timeStepDeltaSeconds = 1;

/// The presentation controller manages application state
typedef struct _tPresentation_Controller {
  MessageListener_Listener_t listener;     ///< listens to the message bus
  BatteryMonitor_AppState_t batteryState;  ///< application state of battery
  bool bleOn;                              ///< BLE status
  float temperatureC;                      ///< Temperature in degrees celsius
  float humidity;                          ///< evaluated humidity
  uint64_t uptimeSeconds;                  ///< nr of seconds the system is up
  uint64_t uptimeSecondsSinceUserEvent;    ///< nr of seconds since last user
                                           ///< interaction
  uint32_t pairingCode;                    ///< pairing code received
} Presentation_Controller_t;

static void PublishAppTimeTickCb(void);
static bool AppBootStateCb(Message_Message_t* msg);
static bool AppShowVersionStateCb(Message_Message_t* msg);
static bool AppNormalOperationStateCb(Message_Message_t* msg);
static bool AppPairingStateCb(Message_Message_t* msg);
static bool EvalBatteryEventCb(Message_Message_t* msg);
static void LogRhtValues(Presentation_Controller_t* controller);
static void LogFirmwareVersion();
static bool HandleSystemStateChange(Message_Message_t* msg);
static void SetLogEnabled(bool enabled);
static void HandleNewSensorValues(Presentation_Controller_t* controller,
                                  Sht4x_SensorMessage_t* msg);
static void PublishReadoutIntervalIfChanged(uint8_t readoutIntervalSec);

/// presentation controller instance
Presentation_Controller_t _controller = {
    .listener = {.receiveMask = MESSAGE_BROKER_CATEGORY_TIME_INFORMATION |
                                MESSAGE_BROKER_CATEGORY_BLE_EVENT |
                                MESSAGE_BROKER_CATEGORY_BATTERY_EVENT |
                                MESSAGE_BROKER_CATEGORY_SYSTEM_STATE_CHANGE |
                                MESSAGE_BROKER_CATEGORY_BUTTON_EVENT |
                                MESSAGE_BROKER_CATEGORY_SENSOR_VALUE,
                 .currentMessageHandlerCb = AppBootStateCb},
    .bleOn = false,
    .batteryState = BATTERY_MONITOR_APP_STATE_NO_RESTRICTION};

MessageListener_Listener_t* Presentation_ControllerInstance() {
  return &_controller.listener;
}

void Presentation_setTimeStep(uint8_t timeStepSeconds) {
  _timeStepDeltaSeconds = timeStepSeconds;
  TimerServer_Stop(_sht4xReadoutTimer);
  TimerServer_Start(_sht4xReadoutTimer, _timeStepDeltaSeconds * 1000);
}

static bool AppBootStateCb(Message_Message_t* msg) {
  if (HandleSystemStateChange(msg)) {
    return true;
  }
  if (msg->header.category == MESSAGE_BROKER_CATEGORY_TIME_INFORMATION) {
    if (msg->header.id == MESSAGE_ID_TIME_INFO_TIME_ELAPSED) {
      _controller.uptimeSeconds += msg->header.parameter1;
      _controller.uptimeSecondsSinceUserEvent += msg->header.parameter1;
      if (_controller.uptimeSeconds > 1) {
        _controller.listener.currentMessageHandlerCb = AppShowVersionStateCb;
      }
      return true;
    }
  }
  return EvalBatteryEventCb(msg);
}

static bool AppShowVersionStateCb(Message_Message_t* msg) {
  if (msg->header.category == MESSAGE_BROKER_CATEGORY_TIME_INFORMATION) {
    if (msg->header.id == MESSAGE_ID_TIME_INFO_TIME_ELAPSED) {
      _controller.uptimeSeconds += msg->header.parameter1;
      _controller.uptimeSecondsSinceUserEvent += msg->header.parameter1;
      if (_controller.uptimeSeconds > 3) {
        _controller.listener.currentMessageHandlerCb =
            AppNormalOperationStateCb;
      }
      return true;
    }
  }
  if (HandleSystemStateChange(msg)) {
    return true;
  }
  return EvalBatteryEventCb(msg);
}

static bool AppNormalOperationStateCb(Message_Message_t* msg) {
  if ((msg->header.category == MESSAGE_BROKER_CATEGORY_SENSOR_VALUE) &&
      (msg->header.id == SHT4X_MESSAGE_ID_SENSOR_DATA) &&
      (msg->header.parameter1 != SHT4X_COMMAND_READ_SERIAL_NUMBER)) {
    HandleNewSensorValues(&_controller, (Sht4x_SensorMessage_t*)msg);
    return true;
  }
  if ((msg->header.category == MESSAGE_BROKER_CATEGORY_TIME_INFORMATION) &&
      (msg->header.id == MESSAGE_ID_TIME_INFO_TIME_ELAPSED)) {
    _controller.uptimeSeconds += msg->header.parameter1;
    _controller.uptimeSecondsSinceUserEvent += msg->header.parameter1;
    if (_controller.uptimeSecondsSinceUserEvent > 300) {
      PublishReadoutIntervalIfChanged(LONG_READOUT_INTERVAL_S);
    } else if (_controller.uptimeSecondsSinceUserEvent > 30) {
      PublishReadoutIntervalIfChanged(MEDIUM_READOUT_INTERVAL_S);
    }
    return true;
  }
  if (msg->header.category == MESSAGE_BROKER_CATEGORY_BUTTON_EVENT) {
    _controller.uptimeSecondsSinceUserEvent = 0;
    PublishReadoutIntervalIfChanged(SHORT_READOUT_INTERVAL_S);
    return true;
  }
  if (msg->header.category == MESSAGE_BROKER_CATEGORY_BLE_EVENT) {
    if (msg->header.id == BLE_INTERFACE_MSG_ID_DISCONNECT) {
      _controller.uptimeSecondsSinceUserEvent = 0;
      PublishReadoutIntervalIfChanged(SHORT_READOUT_INTERVAL_S);
    }
    if (msg->header.id == BLE_INTERFACE_MSG_ID_ASK_USER_ACCEPT_PAIRING) {
      BleInterface_Message_t* bleMessage = (BleInterface_Message_t*)msg;
      _controller.pairingCode = bleMessage->parameter.pairingCode;
      _controller.listener.currentMessageHandlerCb = AppPairingStateCb;
    }
    return true;
  }
  if (HandleSystemStateChange(msg)) {
    return true;
  }
  return EvalBatteryEventCb(msg);
}

static bool EvalBatteryEventCb(Message_Message_t* msg) {
  if ((msg->header.category == MESSAGE_BROKER_CATEGORY_BATTERY_EVENT) &&
      (msg->header.id == BATTERY_MONITOR_MESSAGE_ID_STATE_CHANGE)) {
    BatteryMonitor_Message_t* batteryMsg = (BatteryMonitor_Message_t*)msg;
    _controller.batteryState = batteryMsg->currentState;
    return true;
  }
  return false;
}

static bool HandleSystemStateChange(Message_Message_t* msg) {
  if (msg->header.category != MESSAGE_BROKER_CATEGORY_SYSTEM_STATE_CHANGE) {
    return false;
  }
  if (msg->header.id == MESSAGE_ID_BLE_SUBSYSTEM_OFF) {
    _controller.bleOn = false;
  }
  if (msg->header.id == MESSAGE_ID_BLE_SUBSYSTEM_ON) {
    _controller.bleOn = true;
  }
  if (msg->header.id == MESSAGE_ID_PERIPHERALS_INITIALIZED) {
    _controller.uptimeSeconds = 0;
    _controller.uptimeSecondsSinceUserEvent = 0;
    _sht4xReadoutTimer = TimerServer_CreateTimer(TIMER_SERVER_MODE_REPEATED,
                                                 PublishAppTimeTickCb);

    TimerServer_Start(_sht4xReadoutTimer, _timeStepDeltaSeconds * 1000);
    LogFirmwareVersion();
    return true;
  }
  if (msg->header.id == MESSAGE_ID_READOUT_INTERVAL_CHANGE) {
    Presentation_setTimeStep((uint8_t)msg->parameter2);
    return true;
  }
  if (msg->header.id == MESSAGE_ID_STATE_CHANGE_ERROR) {
    LOG_ERROR("Unrecoverable error %lu!!\n", (unsigned long)msg->parameter2);
    return true;
  }
  if (msg->header.id == MESSAGE_ID_DEVICE_SETTINGS_READ) {
    ItemStore_SystemConfig_t* cfg = (ItemStore_SystemConfig_t*)msg->parameter2;
    SetLogEnabled(cfg->isLogEnabled);
  }
  if (msg->header.id == MESSAGE_ID_DEVICE_SETTINGS_CHANGED &&
      msg->header.parameter1 ==
          SERVICE_REQUEST_MESSAGE_ID_SET_DEBUG_LOG_ENABLE) {
    SetLogEnabled((bool)msg->parameter2);
  }
  return true;
}

static bool AppPairingStateCb(Message_Message_t* msg) {
  if ((msg->header.category == MESSAGE_BROKER_CATEGORY_SENSOR_VALUE) &&
      (msg->header.id == SHT4X_MESSAGE_ID_SENSOR_DATA) &&
      (msg->header.parameter1 != SHT4X_COMMAND_READ_SERIAL_NUMBER)) {
    Sht4x_SensorMessage_t* shtMessage = (Sht4x_SensorMessage_t*)msg;
    _controller.humidity =
        Sht4x_TicksToHumidity(shtMessage->data.measurement.humidityTicks);
    _controller.temperatureC = Sht4x_TicksToTemperatureCelsius(
        shtMessage->data.measurement.temperatureTicks);
    LogRhtValues(&_controller);
    return true;
  }
  if ((msg->header.category == MESSAGE_BROKER_CATEGORY_TIME_INFORMATION) &&
      (msg->header.id == MESSAGE_ID_TIME_INFO_TIME_ELAPSED)) {
    _controller.uptimeSeconds += msg->header.parameter1;
    _controller.uptimeSecondsSinceUserEvent += msg->header.parameter1;
    return true;
  }
  if (msg->header.category == MESSAGE_BROKER_CATEGORY_BUTTON_EVENT) {
    Message_Message_t pairingOkMsg = {
        .header.category = MESSAGE_BROKER_CATEGORY_BLE_EVENT,
        .header.id = BLE_INTERFACE_MSG_ID_USER_ACCEPTED_PAIRING,
        .header.parameter1 = 0,
        .parameter2 = 0};
    BleInterface_PublishBleMessage(&pairingOkMsg);
    _controller.listener.currentMessageHandlerCb = AppNormalOperationStateCb;
    return true;
  }
  if (HandleSystemStateChange(msg)) {
    return true;
  }
  return EvalBatteryEventCb(msg);
}

static void SetLogEnabled(bool enabled) {
  Trace_TraceFunctionCb_t traceFun = Trace_DevNull;
  if (enabled) {
    traceFun = Uart_WriteBlocking;
  }
  Trace_RegisterTraceFunction(traceFun);
}

static void LogRhtValues(Presentation_Controller_t* controller) {
  int humidityInt = (int)controller->humidity;
  int humidityDec =
      (int)((controller->humidity - (float)humidityInt + 0.5f) * 100);

  int tempInt = (int)controller->temperatureC;
  int tempDec = (int)((controller->temperatureC - (float)tempInt + 0.5f) * 100);
  LOG_INFO(
      "SHT43 read out -> "
      "\tTemperature = %i.%i; Humidity = %i.%i\n",
      tempInt, tempDec, humidityInt, humidityDec);
}

static void LogFirmwareVersion(void) {
  const char* firmware_suffix = FIRMWARE_VERSION_DEVELOP ? "-develop" : "";
  LOG_INFO("Firmware Version: %i.%i.%i%s\n", FIRMWARE_VERSION_MAJOR,
           FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH, firmware_suffix);
}

static void PublishAppTimeTickCb(void) {
  static uint32_t _elapsedSeconds = 0;
  _elapsedSeconds += _timeStepDeltaSeconds;
  Message_Message_t message = {
      .header.id = MESSAGE_ID_TIME_INFO_TIME_ELAPSED,
      .header.category = MESSAGE_BROKER_CATEGORY_TIME_INFORMATION,
      .header.parameter1 = _timeStepDeltaSeconds,
      .parameter2 = _elapsedSeconds};

  Message_PublishAppMessage(&message);
}

static void HandleNewSensorValues(Presentation_Controller_t* controller,
                                  Sht4x_SensorMessage_t* msg) {
  controller->humidity =
      Sht4x_TicksToHumidity(msg->data.measurement.humidityTicks);

  controller->temperatureC =
      Sht4x_TicksToTemperatureCelsius(msg->data.measurement.temperatureTicks);
  LogRhtValues(controller);
}

static void PublishReadoutIntervalIfChanged(uint8_t readoutIntervalSec) {
  if (readoutIntervalSec == _timeStepDeltaSeconds) {
    return;
  }
  Message_Message_t msg = {
      .header.category = MESSAGE_BROKER_CATEGORY_SYSTEM_STATE_CHANGE,
      .header.id = MESSAGE_ID_READOUT_INTERVAL_CHANGE,
      .parameter2 = readoutIntervalSec};
  Message_PublishAppMessage(&msg);
}
