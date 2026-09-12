/*
 * arm_receiver.cpp
 *
 * Receives comma-separated arm joint angles over Bluetooth Classic SPP
 * from armprocessing.py running on a Mac. Called from main.cpp via
 * receiver_setup() / receiver_loop().
 *
 * Expected line format (matches armprocessing.py's --bluetooth output):
 *   left_elbow_angle,right_elbow_angle,left_shoulder_angle,right_shoulder_angle\n
 *
 * Requires: original ESP32 or FireBeetle (dual-mode WROOM-32 Bluetooth chip). This will NOT
 * compile for ESP32-S3/C3/C6 boards, which are BLE-only and lack Classic
 * Bluetooth/SPP support.
 *
 * Pairing (macOS): see Pairing Procedure in armprocessing_BL.py or README
 * 
 */

#include "arm_receiver.h"

BluetoothSerial SerialBT;

// Most recently received joint angles, in degrees.
float leftElbowAngle = 0.0f;
float rightElbowAngle = 0.0f;
float leftShoulderAngle = 0.0f;
float rightShoulderAngle = 0.0f;

// Timestamp of the last successfully parsed line, for detecting a stale
// or dropped Bluetooth link from within loop() if you need that later.
unsigned long lastUpdateMillis = 0;

String rxBuffer = "";

// Order: leftShoulder, leftElbow, rightShoulder, rightElbow.
static float latestAngles[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Parses one line of the form "a,b,c,d" into the four angle globals above.
// Returns false (and leaves the globals unchanged) if the line doesn't
// contain exactly four comma-separated numeric fields.
bool parseAngleLine(const String &line) {
  float values[4];
  int valueIndex = 0;
  int start = 0;

  for (int i = 0; i <= (int)line.length() && valueIndex < 4; i++) {
    if (i == (int)line.length() || line.charAt(i) == ',') {
      String token = line.substring(start, i);
      token.trim();
      if (token.length() == 0) {
        return false;  // empty field - malformed line
      }
      values[valueIndex++] = token.toFloat();
      start = i + 1;
    }
  }

  if (valueIndex != 4) {
    return false;
  }

  leftElbowAngle = values[0];
  rightElbowAngle = values[1];
  leftShoulderAngle = values[2];
  rightShoulderAngle = values[3];
  lastUpdateMillis = millis();
  return true;
}

// Flag for client disconnect, to reinitialize Bluetooth and prepare for reconnect
volatile bool btNeedsRestart = false;

// Fires on Bluetooth SPP events so you can see from the USB serial
// monitor what's happening at the RFCOMM/SPP level
void btEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_INIT_EVT:
      Serial.println("Bluetooth SPP initialized");
      break;
    case ESP_SPP_START_EVT:
      Serial.println("Bluetooth SPP server started -- ready to accept a connection");
      break;
    case ESP_SPP_SRV_OPEN_EVT:
      Serial.println("Bluetooth client connected");
      break;
    case ESP_SPP_CLOSE_EVT:
      Serial.println("Bluetooth client disconnected");
      // Classic BluetoothSerial on ESP32 has a long-standing bug where it
      // won't accept a new connection after the first disconnect unless the
      // stack is restarted (see arduino-esp32 issues #8202, #4967, #4915).
      // Flag it here; the actual restart happens in receiver_loop().
      btNeedsRestart = true;
      break;
    case ESP_SPP_DATA_IND_EVT:
      // Fires per incoming data packet -- left unlogged on purpose since
      // it would flood the monitor once actual streaming starts.
      break;
    default:
      // Catch-all so nothing happens silently while debugging
      Serial.printf("Bluetooth SPP event: %d (unhandled)\n", (int)event);
      break;
  }
}

// Restarts the Bluetooth SPP service so it will accept a new connection, 
// without rebooting the whole board (servos/other states stay intact)
void restartBluetoothServer() {
  Serial.println("Restarting Bluetooth service to allow a new connection...");
  SerialBT.end();
  delay(200);  // let the stack fully tear down before bringing it back up
  SerialBT.register_callback(btEventCallback);
  if (!SerialBT.begin("FireBeetle_ArmData")) {
    Serial.println("Bluetooth restart failed! If this keeps happening, "
                    "try forgetting the device in macOS Bluetooth settings "
                    "and re-pairing, or fall back to ESP.restart() here.");
  } else {
    Serial.println("Bluetooth ready again. Waiting for a new connection...");
  }
}

// Receiver setup function to be called in main's setup(), initializes Bluetooth link on board
void receiver_setup() {
  SerialBT.register_callback(btEventCallback);

  if (!SerialBT.begin("FireBeetle_ArmData")) {
    Serial.println("Bluetooth init failed! Check that Classic BT is available on this board.");
  } else {
    Serial.println("Bluetooth started. Pair with 'FireBeetle_ArmData' from Mac System Settings > Bluetooth.");
  }
}

// Receiver routine to be run in main's loop(), returns pointer to array of arm data
float* receiver_loop() {
  if (btNeedsRestart) {
    btNeedsRestart = false;
    restartBluetoothServer();
  }

  // Continuous Bluetooth stream
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n') {
      if (parseAngleLine(rxBuffer)) {
        Serial.printf("L-elbow: %.1f  L-shoulder: %.1f  R-shoulder: %.1f  R-elbow: %.1f \n",
                      leftElbowAngle, leftShoulderAngle, rightShoulderAngle, rightElbowAngle);

        latestAngles[0] = leftShoulderAngle;
        latestAngles[1] = leftElbowAngle;
        latestAngles[2] = rightShoulderAngle;
        latestAngles[3] = rightElbowAngle;

      } else {
        Serial.println("Malformed Bluetooth line, ignoring: " + rxBuffer);
      }
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
    }
  }

  return latestAngles;
}