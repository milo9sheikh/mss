/*
  Sensor monitor + Arduino Uno + SIM800L TCP sender
  (self-healing version, IMEI device identity + typed frames)

  === What this firmware does ===
  - Watches digital pins 2-5 (motion, door, remote lock, remote unlock)
    and sends an EVENT frame to the TCP server immediately.
  - Every 15 s sends a HEARTBEAT frame carrying device id + status.
  - Reads the modem IMEI with AT+CGSN and puts it in EVERY frame, so the
    server can tell devices apart.
  - After a heartbeat/status frame the socket stays open for a short
    window so the server can push a COMMAND; the device answers with a
    RESPONSE frame on the same socket.
  - GPRS is kept attached between sends (fast sends), with automatic
    full re-bring-up on failure and a soft-reset watchdog.

  === Frame format (same in both directions) ===

    [0xF0][0xF2][type][lenHi][lenLo][ data ... ][checksum][0xF4]

    header    2 bytes  0xF0 0xF2
    type      1 byte   see below
    length    2 bytes  big-endian, length of the data section
    data      N bytes  ALWAYS starts with the 15-byte ASCII IMEI, then a
                       type-specific body
    checksum  1 byte   XOR of type + lenHi + lenLo + every data byte
    trailer   1 byte   0xF4

  === Data types ===

    0x01 HEARTBEAT (device -> server)
         data = [IMEI 15][active][alarm]
    0x02 EVENT     (device -> server)
         data = [IMEI 15][event code]
           0x01 motion   0x02 door    0x03 active
           0x04 de-active 0x05 alm-on 0x06 alm-off
    0x03 RESPONSE  (device -> server)
         data = [IMEI 15][command ID][result][active][alarm]
           result: 0x00 = OK, 0x01 = unknown command
    0x04 STATUS    (device -> server, sent once at boot)
         data = [IMEI 15][active][alarm]
    0x05 COMMAND   (server -> device)
         data = [IMEI 15][command ID]
           0x01 activate     0x02 de-activate
           0x03 alarm on     0x04 alarm off
           0x05 get status

    active: 0x01 = Active,  0x00 = De-Active
    alarm : 0x01 = Alarm ON, 0x00 = Alarm OFF

  NOTE: the length/data can contain any byte value, including 0x1A.
  That is why AT+CIPSEND=<length> is used instead of the old Ctrl+Z
  terminator - Ctrl+Z inside a binary frame would cut it short.

  === Alarm logic ===
    1. Device ACTIVE + motion/door sensor triggers  -> alarm ON for 30 s,
       then it switches OFF by itself (event 0x05, then event 0x06).
       A new trigger while the alarm is ON restarts the 30 s.
    2. Device DE-ACTIVE + ANY change on the sensor pins -> alarm OFF
       (event 0x06 if it was ON).
    - Server commands 0x03 / 0x04 switch the alarm ON / OFF directly
      (a commanded ON has no 30 s limit).
    - At power-up the alarm is ON (ALARM_ON_AT_BOOT) until rule 2 or a
      server command switches it off.

  Wiring:
    Sensor inputs: pin2, pin3, pin4, pin5 (digital)
    Alarm output : pin6 (HIGH = alarm on; siren / relay driver)
    SIM800L TX  -> Arduino D7
    SIM800L RX  <- Arduino D8 via level shifter / resistor divider
    SIM800L GND -> Arduino GND

  SIM800L power:
    Use a dedicated stable 3.7-4.2 V supply capable of 2 A bursts.
    Do NOT power the SIM800L from the Arduino Uno 5 V pin.

  Open Arduino Serial Monitor at 115200 baud.
*/

#include <SoftwareSerial.h>

// ===== Sensor pins =====
const int pin2 = 2;
const int pin3 = 3;
const int pin4 = 4;
const int pin5 = 5;

const int ALARM_PIN = 6; // siren / relay output (HIGH = alarm ON)

int lastP2, lastP3, lastP4, lastP5;

// ===== SIM800L / TCP config =====
SoftwareSerial sim800(7, 8); // SoftwareSerial(RX, TX)

const unsigned long MODEM_BAUD = 9600;
const unsigned long DEBUG_BAUD = 115200;

// ===== EDIT THESE VALUES =====
const char APN[]      = "internet";
const char APN_USER[] = "";
const char APN_PASS[] = "";

const char SERVER[]   = "office.maktro.com";
const uint16_t PORT   = 5000;
// =============================

const unsigned long HEARTBEAT_INTERVAL = 15000UL; // 15 seconds
const unsigned long RX_WINDOW_MS       = 1500UL;  // how long to wait for a server command after a heartbeat/status
const unsigned long ALARM_DURATION_MS  = 30000UL; // sensor-triggered alarm stays ON this long (30 s)
const bool ALARM_ON_AT_BOOT            = true;    // alarm state at power-up ("for now": ON)
const byte MAX_CONSECUTIVE_FAILURES    = 4;       // soft-reset modem after this many failed cycles in a row

// ===== Frame constants =====
const byte FRAME_HEADER_1 = 0xF0;
const byte FRAME_HEADER_2 = 0xF2;
const byte FRAME_TRAILER  = 0xF4;

const byte IMEI_LEN         = 15;
const byte MAX_BODY_LEN     = 4;                           // longest body after the IMEI (response)
const byte FRAME_DATA_OFFSET = 5;                          // header(2) + type(1) + len(2)
const byte FRAME_MAX        = FRAME_DATA_OFFSET + IMEI_LEN + MAX_BODY_LEN + 2; // + checksum + trailer

// ===== Data types =====
const byte TYPE_HEARTBEAT = 0x01;
const byte TYPE_EVENT     = 0x02;
const byte TYPE_RESPONSE  = 0x03;
const byte TYPE_STATUS    = 0x04;
const byte TYPE_COMMAND   = 0x05; // server -> device

// ===== Event codes (TYPE_EVENT) =====
const byte EVT_MOTION     = 0x01;
const byte EVT_DOOR       = 0x02;
const byte EVT_ACTIVE     = 0x03;
const byte EVT_DEACTIVE   = 0x04;
const byte EVT_ALARM_ON   = 0x05;
const byte EVT_ALARM_OFF  = 0x06;

// ===== Command IDs (TYPE_COMMAND) =====
const byte CMD_ACTIVATE   = 0x01;
const byte CMD_DEACTIVATE = 0x02;
const byte CMD_ALARM_ON   = 0x03;
const byte CMD_ALARM_OFF  = 0x04;
const byte CMD_GET_STATUS = 0x05;

// ===== Response results (TYPE_RESPONSE) =====
const byte RESULT_OK          = 0x00;
const byte RESULT_UNKNOWN_CMD = 0x01;

// ===== Runtime state =====
char imei[IMEI_LEN + 1] = "";
bool imeiValid = false;

bool deviceActive = false; // Active / De-Active
bool alarmOn      = ALARM_ON_AT_BOOT; // Alarm ON / OFF
bool alarmTimed   = false; // true while a sensor-triggered alarm is counting its 30 s
unsigned long alarmStartMillis = 0;

char lastReply[220];
byte consecutiveFailures = 0;
unsigned long lastHeartbeatMillis = 0;
bool gprsReady = false; // true once GPRS is attached and kept live between sends

enum RxState : byte {
  RX_H1, RX_H2, RX_TYPE, RX_LEN_HI, RX_LEN_LO, RX_DATA, RX_CHECKSUM, RX_TRAILER
};

// ---------------------------------------------------------------
// Low-level modem helpers
// ---------------------------------------------------------------

void clearModemInput() {
  while (sim800.available()) {
    sim800.read();
  }
}

bool waitFor(const char *expected, unsigned long timeoutMs) {
  unsigned long started = millis();
  size_t position = 0;

  lastReply[0] = '\0';

  while (millis() - started < timeoutMs) {
    while (sim800.available()) {
      char c = sim800.read();

      // Print every SIM800L response in Serial Monitor
      Serial.write(c);

      if (position < sizeof(lastReply) - 1) {
        lastReply[position++] = c;
        lastReply[position] = '\0';
      }

      if (strstr(lastReply, expected) != NULL) {
        Serial.println(F("<< OK"));
        return true;
      }

      if (strstr(lastReply, "ERROR") != NULL ||
          strstr(lastReply, "FAIL") != NULL) {
        Serial.println(F("<< FAILED"));
        return false;
      }
    }
  }

  Serial.print(F("\n<< TIMEOUT. Expected: "));
  Serial.println(expected);
  return false;
}

bool sendAT(const char *command, const char *expected, unsigned long timeoutMs) {
  clearModemInput();

  Serial.print(F("\n>> "));
  Serial.println(command);

  sim800.println(command);
  return waitFor(expected, timeoutMs);
}

bool registeredOnNetwork() {
  if (!sendAT("AT+CREG?", "OK", 5000)) {
    return false;
  }

  // ,1 = registered on home network
  // ,5 = registered while roaming
  return strstr(lastReply, ",1") != NULL ||
         strstr(lastReply, ",5") != NULL;
}

bool waitForNetwork() {
  Serial.println(F("\n--- Checking mobile network ---"));

  for (byte attempt = 0; attempt < 30; attempt++) {
    if (registeredOnNetwork()) {
      Serial.println(F("Network registered."));
      return true;
    }

    Serial.println(F("Not registered. Retrying..."));
    delay(2000);
  }

  Serial.println(F("Network registration failed."));
  return false;
}

bool startGprs() {
  Serial.println(F("\n--- Starting GPRS (full clean bring-up) ---"));

  // Always tear down any previous IP session first. It is okay if this
  // "fails" because there was nothing to tear down.
  sendAT("AT+CIPSHUT", "SHUT OK", 15000);

  if (!sendAT("AT+CGATT=1", "OK", 30000)) {
    return false;
  }

  char command[110];

  snprintf(command, sizeof(command),
           "AT+CSTT=\"%s\",\"%s\",\"%s\"",
           APN, APN_USER, APN_PASS);

  if (!sendAT(command, "OK", 10000)) {
    return false;
  }

  if (!sendAT("AT+CIICR", "OK", 85000)) {
    return false;
  }

  Serial.println(F("\nGPRS IP address:"));

  // CIFSR returns an IP address, for example: 10.123.45.67
  if (!sendAT("AT+CIFSR", ".", 10000)) {
    return false;
  }

  return true;
}

// ---------------------------------------------------------------
// Device identity (IMEI)
// ---------------------------------------------------------------

// Reads the modem IMEI with AT+CGSN. The reply looks like:
//   \r\n867858035727471\r\n\r\nOK\r\n
// so we look for a run of 15 digits in the reply.
bool readImei() {
  Serial.println(F("\n--- Reading device IMEI (AT+CGSN) ---"));

  for (byte attempt = 0; attempt < 5; attempt++) {
    if (sendAT("AT+CGSN", "OK", 3000)) {
      byte run = 0;

      for (const char *p = lastReply; *p != '\0'; p++) {
        if (*p >= '0' && *p <= '9') {
          run++;
          if (run == IMEI_LEN) {
            memcpy(imei, p - (IMEI_LEN - 1), IMEI_LEN);
            imei[IMEI_LEN] = '\0';
            imeiValid = true;

            Serial.print(F("\nDevice IMEI: "));
            Serial.println(imei);
            return true;
          }
        } else {
          run = 0;
        }
      }
    }

    delay(500);
  }

  Serial.println(F("\nCould not read IMEI."));
  return false;
}

// ---------------------------------------------------------------
// Frame building / sending
// ---------------------------------------------------------------

void printHexByte(byte b) {
  Serial.print(F("0x"));
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
  Serial.print(' ');
}

void printTypeName(byte type) {
  switch (type) {
    case TYPE_HEARTBEAT: Serial.print(F("HEARTBEAT")); break;
    case TYPE_EVENT:     Serial.print(F("EVENT"));     break;
    case TYPE_RESPONSE:  Serial.print(F("RESPONSE"));  break;
    case TYPE_STATUS:    Serial.print(F("STATUS"));    break;
    default:             Serial.print(F("UNKNOWN"));   break;
  }
}

void printEventName(byte code) {
  switch (code) {
    case EVT_MOTION:    Serial.print(F("motion"));    break;
    case EVT_DOOR:      Serial.print(F("door"));      break;
    case EVT_ACTIVE:    Serial.print(F("active"));    break;
    case EVT_DEACTIVE:  Serial.print(F("de-active")); break;
    case EVT_ALARM_ON:  Serial.print(F("alarm-on"));  break;
    case EVT_ALARM_OFF: Serial.print(F("alarm-off")); break;
    default:            Serial.print(F("unknown"));   break;
  }
}

void printStatusText(byte active, byte alarm) {
  if (active) Serial.print(F("Active"));
  else        Serial.print(F("De-Active"));

  Serial.print(F(", "));

  if (alarm) Serial.print(F("Alarm ON"));
  else       Serial.print(F("Alarm OFF"));
}

// Shows in the Serial Monitor exactly what is being sent to the server:
// the raw bytes as hex, then each field decoded.
void debugPrintFrame(const byte *frame, uint16_t n) {
  byte type = frame[2];
  const byte *body = &frame[FRAME_DATA_OFFSET + IMEI_LEN];

  Serial.println(F("\n========== SENDING TO SERVER =========="));

  Serial.print(F("Raw hex  : "));
  for (uint16_t i = 0; i < n; i++) {
    printHexByte(frame[i]);
  }
  Serial.println();

  Serial.print(F("Size     : "));
  Serial.print(n);
  Serial.println(F(" bytes"));

  Serial.print(F("Type     : "));
  printHexByte(type);
  printTypeName(type);
  Serial.println();

  Serial.print(F("Length   : "));
  Serial.println(((uint16_t)frame[3] << 8) | frame[4]);

  Serial.print(F("Device ID: "));
  for (byte i = 0; i < IMEI_LEN; i++) {
    Serial.write((char)frame[FRAME_DATA_OFFSET + i]);
  }
  Serial.println();

  Serial.print(F("Data     : "));
  if (type == TYPE_HEARTBEAT || type == TYPE_STATUS) {
    printStatusText(body[0], body[1]);
  } else if (type == TYPE_EVENT) {
    printHexByte(body[0]);
    printEventName(body[0]);
  } else if (type == TYPE_RESPONSE) {
    Serial.print(F("command "));
    printHexByte(body[0]);
    Serial.print(F(", result "));
    if (body[1] == RESULT_OK) Serial.print(F("OK"));
    else                      Serial.print(F("UNKNOWN COMMAND"));
    Serial.print(F(", "));
    printStatusText(body[2], body[3]);
  }
  Serial.println();

  Serial.print(F("Checksum : "));
  printHexByte(frame[n - 2]);
  Serial.println();

  Serial.print(F("Trailer  : "));
  printHexByte(frame[n - 1]);
  Serial.println();

  Serial.println(F("======================================="));
}

// Builds one frame [header][type][len][IMEI + body][checksum][trailer]
// and sends it on the ALREADY-OPEN TCP connection.
// Uses AT+CIPSEND=<n> (exact byte count) so binary bytes such as 0x1A
// inside the frame cannot terminate the send early.
bool writeFrame(byte type, const byte *body, byte bodyLen) {
  if (bodyLen > MAX_BODY_LEN) {
    return false;
  }

  byte frame[FRAME_MAX];
  uint16_t dataLen = IMEI_LEN + bodyLen;
  byte lenHi = (byte)(dataLen >> 8);
  byte lenLo = (byte)(dataLen & 0xFF);
  uint16_t n = 0;

  frame[n++] = FRAME_HEADER_1;
  frame[n++] = FRAME_HEADER_2;
  frame[n++] = type;
  frame[n++] = lenHi;
  frame[n++] = lenLo;

  memcpy(&frame[n], imei, IMEI_LEN);
  n += IMEI_LEN;

  if (bodyLen > 0) {
    memcpy(&frame[n], body, bodyLen);
    n += bodyLen;
  }

  byte checksum = type ^ lenHi ^ lenLo;
  for (uint16_t i = FRAME_DATA_OFFSET; i < FRAME_DATA_OFFSET + dataLen; i++) {
    checksum ^= frame[i];
  }

  frame[n++] = checksum;
  frame[n++] = FRAME_TRAILER;

  debugPrintFrame(frame, n);

  char command[24];
  snprintf(command, sizeof(command), "AT+CIPSEND=%u", (unsigned int)n);

  if (!sendAT(command, ">", 10000)) {
    return false;
  }

  sim800.write(frame, n);

  return waitFor("SEND OK", 30000);
}

// ---------------------------------------------------------------
// Device state + server commands
// ---------------------------------------------------------------

void applyAlarm(bool on) {
  alarmOn = on;
  alarmTimed = false; // any direct change cancels a running 30 s timer
  digitalWrite(ALARM_PIN, on ? HIGH : LOW);
}

// Sensor-triggered alarm: ON for ALARM_DURATION_MS (restarts if already ON).
// Returns true if the alarm was OFF and has just switched ON.
bool triggerTimedAlarm() {
  bool changed = !alarmOn;

  applyAlarm(true);
  alarmTimed = true; // arm the 30 s timer after applyAlarm() cleared it
  alarmStartMillis = millis();

  return changed;
}

void fillStatus(byte *body) {
  body[0] = deviceActive ? 0x01 : 0x00;
  body[1] = alarmOn ? 0x01 : 0x00;
}

// Runs a command. Returns the result code. If the command changed device
// state, eventCode is set to the matching event (0 = nothing changed).
byte executeCommand(byte cmd, byte &eventCode) {
  eventCode = 0;

  switch (cmd) {
    case CMD_ACTIVATE:
      if (!deviceActive) {
        deviceActive = true;
        eventCode = EVT_ACTIVE;
      }
      return RESULT_OK;

    case CMD_DEACTIVATE:
      if (deviceActive) {
        deviceActive = false;
        eventCode = EVT_DEACTIVE;
      }
      return RESULT_OK;

    case CMD_ALARM_ON:
      if (!alarmOn) {
        applyAlarm(true);
        eventCode = EVT_ALARM_ON;
      }
      return RESULT_OK;

    case CMD_ALARM_OFF:
      if (alarmOn) {
        applyAlarm(false);
        eventCode = EVT_ALARM_OFF;
      }
      return RESULT_OK;

    case CMD_GET_STATUS:
      return RESULT_OK;

    default:
      return RESULT_UNKNOWN_CMD;
  }
}

// A complete, checksum-verified frame arrived from the server while the
// socket is still open. Executes it and answers on the same socket.
void handleInboundFrame(byte type, const byte *data, uint16_t len) {
  if (type != TYPE_COMMAND || len < (uint16_t)(IMEI_LEN + 1)) {
    Serial.println(F("\nIgnoring inbound frame (not a command)."));
    return;
  }

  if (memcmp(data, imei, IMEI_LEN) != 0) {
    Serial.println(F("\nCommand is for another device; ignored."));
    return;
  }

  byte cmd = data[IMEI_LEN];
  byte eventCode = 0;
  byte result = executeCommand(cmd, eventCode);

  Serial.print(F("\nCommand received: "));
  printHexByte(cmd);
  Serial.println();

  byte body[4];
  body[0] = cmd;
  body[1] = result;
  fillStatus(&body[2]);

  writeFrame(TYPE_RESPONSE, body, 4);

  // If the command changed state, also report it as an event.
  if (eventCode != 0) {
    writeFrame(TYPE_EVENT, &eventCode, 1);
  }
}

// Listens on the open socket for one server frame, for up to windowMs.
void listenForCommand(unsigned long windowMs) {
  Serial.println(F("\n--- Listening for server command ---"));

  RxState state = RX_H1;
  byte type = 0;
  byte checksum = 0;
  uint16_t len = 0;
  uint16_t got = 0;
  byte data[IMEI_LEN + MAX_BODY_LEN];

  unsigned long started = millis();

  while (millis() - started < windowMs) {
    while (sim800.available()) {
      byte b = sim800.read();

      switch (state) {
        case RX_H1:
          if (b == FRAME_HEADER_1) state = RX_H2;
          break;

        case RX_H2:
          if (b == FRAME_HEADER_2)      state = RX_TYPE;
          else if (b == FRAME_HEADER_1) state = RX_H2;
          else                          state = RX_H1;
          break;

        case RX_TYPE:
          type = b;
          checksum = b;
          state = RX_LEN_HI;
          break;

        case RX_LEN_HI:
          len = (uint16_t)b << 8;
          checksum ^= b;
          state = RX_LEN_LO;
          break;

        case RX_LEN_LO:
          len |= b;
          checksum ^= b;
          got = 0;
          if (len > sizeof(data)) {
            state = RX_H1;              // too long for us, resync
          } else if (len == 0) {
            state = RX_CHECKSUM;
          } else {
            state = RX_DATA;
          }
          break;

        case RX_DATA:
          data[got++] = b;
          checksum ^= b;
          if (got >= len) state = RX_CHECKSUM;
          break;

        case RX_CHECKSUM:
          if (b == checksum) {
            state = RX_TRAILER;
          } else {
            Serial.println(F("\nInbound frame: bad checksum."));
            state = RX_H1;
          }
          break;

        case RX_TRAILER:
          state = RX_H1;
          if (b == FRAME_TRAILER) {
            handleInboundFrame(type, data, len);
            return;
          }
          break;
      }
    }
  }
}

// ---------------------------------------------------------------
// TCP session / send logic
// ---------------------------------------------------------------

// Opens the socket, sends one frame, optionally waits for a server
// command, then closes.
bool sendMessage(byte type, const byte *body, byte bodyLen, bool listen, byte extraEvent) {
  Serial.println(F("\n--- Connecting to server ---"));

  char command[110];

  snprintf(command, sizeof(command),
           "AT+CIPSTART=\"TCP\",\"%s\",\"%u\"",
           SERVER, PORT);

  if (!sendAT(command, "CONNECT OK", 30000)) {
    sendAT("AT+CIPSTATUS", "OK", 5000); // printed to Serial Monitor for debugging
    sendAT("AT+CIPCLOSE", "CLOSE OK", 10000);
    return false;
  }

  bool success = writeFrame(type, body, bodyLen);

  // Optional second EVENT frame on the same connection (e.g. the alarm
  // event that goes with a sensor event) - no second connect needed.
  if (success && extraEvent != 0) {
    success = writeFrame(TYPE_EVENT, &extraEvent, 1);
  }

  if (success && listen) {
    listenForCommand(RX_WINDOW_MS);
  }

  // Close connection; ignored if the server has already closed it.
  sendAT("AT+CIPCLOSE", "CLOSE OK", 10000);

  return success;
}

void modemSoftReset() {
  Serial.println(F("\n=== Too many failed cycles: soft-resetting modem (AT+CFUN=1,1) ==="));

  sendAT("AT+CFUN=1,1", "OK", 10000);
  delay(8000); // let the radio stack reboot

  for (byte attempt = 0; attempt < 10; attempt++) {
    if (sendAT("AT", "OK", 2000)) break;
    delay(1000);
  }

  sendAT("ATE0", "OK", 2000);
  sendAT("AT+CPIN?", "OK", 5000);
  sendAT("AT+CSQ", "OK", 5000);
  sendAT("AT+CSCLK=0", "OK", 5000); // must be reasserted, CFUN=1,1 reboots the module
}

// Does a full registration check + GPRS bring-up, and marks GPRS as
// "ready" (kept attached) on success.
bool bringUpGprs() {
  if (!registeredOnNetwork()) {
    Serial.println(F("Not registered on the mobile network yet."));
    return false;
  }

  if (!startGprs()) {
    Serial.println(F("GPRS bring-up failed."));
    gprsReady = false;
    return false;
  }

  gprsReady = true;
  return true;
}

// Sends as fast as possible: if GPRS is already attached, just does the
// quick CIPSTART/CIPSEND/CIPCLOSE. Falls back to a full GPRS re-bring-up
// if GPRS wasn't ready or the quick send failed.
// Nothing is ever sent without a valid IMEI.
bool sendData(byte type, const byte *body, byte bodyLen, bool listen, byte extraEvent) {
  if (!imeiValid && !readImei()) {
    Serial.println(F("No IMEI yet - not sending."));
    return false;
  }

  bool success = false;

  if (gprsReady) {
    success = sendMessage(type, body, bodyLen, listen, extraEvent);
    if (!success) {
      Serial.println(F("Quick send failed; GPRS session likely dropped."));
      gprsReady = false;
    }
  }

  if (!success) {
    if (bringUpGprs()) {
      success = sendMessage(type, body, bodyLen, listen, extraEvent);
      if (!success) {
        gprsReady = false;
      }
    }
  }

  return success;
}

// Wraps sendData() with the consecutive-failure watchdog / soft-reset logic.
bool sendWithWatchdog(byte type, const byte *body, byte bodyLen, bool listen, byte extraEvent) {
  bool ok = sendData(type, body, bodyLen, listen, extraEvent);

  if (ok) {
    consecutiveFailures = 0;
  } else {
    consecutiveFailures++;
    Serial.print(F("Consecutive failed sends: "));
    Serial.println(consecutiveFailures);

    if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      modemSoftReset();
      gprsReady = false; // force a fresh bring-up after a modem reset
      consecutiveFailures = 0;
    }
  }

  return ok;
}

// Convenience senders --------------------------------------------

// Events are sent immediately and do not wait for a server command,
// so they stay fast. A second event (0 = none) rides on the same connection.
void sendEvents(byte first, byte second) {
  sendWithWatchdog(TYPE_EVENT, &first, 1, false, second);
}

// Heartbeat = device id (added by writeFrame) + status.
void sendHeartbeat() {
  byte body[2];
  fillStatus(body);
  sendWithWatchdog(TYPE_HEARTBEAT, body, 2, true, 0);
}

void sendStatus() {
  byte body[2];
  fillStatus(body);
  sendWithWatchdog(TYPE_STATUS, body, 2, true, 0);
}

// ---------------------------------------------------------------
// Sensor handling
// ---------------------------------------------------------------

void checkSensors() {
  int p2 = digitalRead(pin2);
  int p3 = digitalRead(pin3);
  int p4 = digitalRead(pin4);
  int p5 = digitalRead(pin5);

  bool changed = (p2 != lastP2) || (p3 != lastP3) ||
                 (p4 != lastP4) || (p5 != lastP5);

  if (changed) {
    const char *name = NULL;
    byte eventCode = 0; // what the sensors/remote did
    byte alarmEvent = 0; // alarm change caused by the alarm logic

    if (p5 == 1 && p2 == 0 && p3 == 0 && p4 == 0) {
      name = "remote lock -> active";
      eventCode = EVT_ACTIVE;
      deviceActive = true;
    } else if (p4 == 1 && p2 == 0 && p3 == 0 && p5 == 0) {
      name = "remote unlock -> de-active";
      eventCode = EVT_DEACTIVE;
      deviceActive = false;
    } else if (p2 == 1 && p5 == 1 && p3 == 0 && p4 == 0) {
      name = "motion sensor";
      eventCode = EVT_MOTION;
    } else if (p3 == 1 && p4 == 1 && p5 == 1 && p2 == 0) {
      name = "door sensor";
      eventCode = EVT_DOOR;
    }

    if (name != NULL) {
      Serial.println(name);
    }

    // Rule 1: device ACTIVE + motion/door trigger -> alarm ON for 30 s.
    if (deviceActive && (eventCode == EVT_MOTION || eventCode == EVT_DOOR)) {
      if (triggerTimedAlarm()) {
        alarmEvent = EVT_ALARM_ON;
        Serial.println(F("Alarm ON (30 s)"));
      } else {
        Serial.println(F("Alarm already ON - 30 s restarted"));
      }
    }
    // Rule 2: device DE-ACTIVE + ANY sensor change -> alarm OFF.
    else if (!deviceActive && alarmOn) {
      applyAlarm(false);
      alarmEvent = EVT_ALARM_OFF;
      Serial.println(F("Alarm OFF (device de-active)"));
    }

    // Report: the sensor event first, then the alarm event, on one connection.
    if (eventCode != 0) {
      sendEvents(eventCode, alarmEvent);
    } else if (alarmEvent != 0) {
      sendEvents(alarmEvent, 0);
    }

    lastP2 = p2;
    lastP3 = p3;
    lastP4 = p4;
    lastP5 = p5;
  }
}

// Switches a sensor-triggered alarm OFF when its 30 s are over.
void checkAlarmTimer() {
  if (alarmTimed && alarmOn &&
      millis() - alarmStartMillis >= ALARM_DURATION_MS) {
    applyAlarm(false);
    Serial.println(F("\nAlarm OFF (30 s elapsed)"));
    sendEvents(EVT_ALARM_OFF, 0);
  }
}

// ---------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------

void setup() {
  Serial.begin(DEBUG_BAUD);

  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  pinMode(pin5, INPUT);

  pinMode(ALARM_PIN, OUTPUT);
  applyAlarm(alarmOn); // alarm defaults to ON at boot

  lastP2 = digitalRead(pin2);
  lastP3 = digitalRead(pin3);
  lastP4 = digitalRead(pin4);
  lastP5 = digitalRead(pin5);

  sim800.begin(MODEM_BAUD);

  Serial.println(F("\n\nSensor + SIM800L TCP sender"));
  Serial.println(F("Starting modem..."));

  delay(5000);

  bool modemFound = false;

  for (byte attempt = 0; attempt < 5; attempt++) {
    if (sendAT("AT", "OK", 2000)) {
      modemFound = true;
      break;
    }
    delay(1000);
  }

  if (!modemFound) {
    Serial.println(F("\nSIM800L not found."));
    Serial.println(F("Check power, GND, D7/D8 wiring, and 9600 baud rate."));

    while (true) {
      delay(1000);
    }
  }

  sendAT("ATE0", "OK", 2000);       // Disable command echo
  sendAT("AT+CPIN?", "OK", 5000);   // SIM status
  sendAT("AT+CSQ", "OK", 5000);     // Signal quality
  sendAT("AT+CSCLK=0", "OK", 5000); // Disable UART sleep mode - without this the
                                    // module stops answering after ~15s of idle

  readImei();                       // Device id used in every frame

  waitForNetwork();

  // Bring GPRS up once at boot so the very first event can do a quick send.
  bringUpGprs();

  // Tell the server we are online, with our current status.
  sendStatus();

  lastHeartbeatMillis = millis();
}

void loop() {
  // Poll sensors on every pass so an event triggers an immediate send.
  checkSensors();
  checkAlarmTimer();

  // Periodic heartbeat: device id + status, every 15 s, independent of
  // sensor activity.
  if (millis() - lastHeartbeatMillis >= HEARTBEAT_INTERVAL) {
    Serial.println(F("\n--- Heartbeat ---"));
    sendHeartbeat();
    lastHeartbeatMillis = millis();
  }

  delay(20);
}
