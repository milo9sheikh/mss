/*
  Merged farmware: Sensor monitor + Arduino Uno + SIM800L TCP sender
  (self-healing version)

  === What this is ===
  Combines two original sketches:
    1) Digital pin sensor monitor (pins 2-5: remote lock/unlock,
       motion sensor, door sensor) that used to just Serial.println()
       the event name.
    2) SIM800L TCP "heartbeat" sender that sent the fixed text "Test"
       to a TCP server every 15 seconds.

  === What changed ===
  Whenever a sensor event fires (motion sensor / door sensor / remote
  lock / remote unlock), instead of only printing to Serial, that exact
  event string is now sent to the TCP server IMMEDIATELY.

  The periodic 15-second heartbeat still runs on its own timer and still
  sends "Test" - unchanged. Sensor events are separate, immediate sends
  and do not wait for the heartbeat timer.

  IMPORTANT DESIGN NOTE ON SPEED:
  The original heartbeat sketch tore down and fully rebuilt the GPRS
  context (AT+CIPSHUT, AT+CGATT, AT+CSTT, AT+CIICR) on every single send.
  That full bring-up genuinely takes several seconds to ~15+ seconds on
  real hardware - so doing it for every sensor event as well made events
  look like they were "waiting 15 seconds" even though nothing was
  actually delaying them on purpose.

  To make sensor/remote events fast, GPRS is now kept attached
  persistently once it comes up:
    - On boot (and after any failure), a full GPRS bring-up is done once.
    - Every send after that (sensor event OR heartbeat) just does a quick
      AT+CIPSTART / AT+CIPSEND / AT+CIPCLOSE on the already-live GPRS
      context - no re-attach, so it's fast.
    - If a quick send fails (session dropped, etc.), the code
      automatically falls back to a full GPRS re-bring-up and retries
      once.
  The UART-sleep disable and the consecutive-failure soft-reset watchdog
  are unchanged.

  Wiring:
    Sensor inputs: pin2, pin3, pin4, pin5 (digital, as in the original
                   sensor sketch)
    SIM800L TX  -> Arduino D7
    SIM800L RX  <- Arduino D8 via level shifter / resistor divider
    SIM800L GND -> Arduino GND

  SIM800L power:
    Use a dedicated stable 3.7-4.2 V supply capable of 2 A bursts.
    Do NOT power the SIM800L from the Arduino Uno 5 V pin.

  Open Arduino Serial Monitor at 115200 baud.
*/

#include <SoftwareSerial.h>

// ===== Sensor pins (unchanged from original sensor sketch) =====
const int pin2 = 2;
const int pin3 = 3;
const int pin4 = 4;
const int pin5 = 5;

int lastP2, lastP3, lastP4, lastP5;

// ===== SIM800L / TCP config (unchanged from original heartbeat sketch) =====
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

// ===== Binary protocol config =====
// Packet layout: [HEADER 1B][TYPE 1B][LENGTH 2B big-endian][DATA 0-2B][CHECKSUM 1B][TRAILER 1B]
// CHECKSUM = XOR of TYPE byte, both LENGTH bytes, and all DATA bytes.
const uint8_t PKT_HEADER  = 0xAA;
const uint8_t PKT_TRAILER = 0xF4;

const uint8_t TYPE_HEARTBEAT    = 0x00;
const uint8_t TYPE_SENSOR_EVENT = 0x01;

// Sensor event codes (sent as the 1-byte DATA payload)
const uint8_t EVT_REMOTE_LOCK   = 0x01;
const uint8_t EVT_REMOTE_UNLOCK = 0x02;
const uint8_t EVT_MOTION_SENSOR = 0x03;
const uint8_t EVT_DOOR_SENSOR   = 0x04;

const uint8_t MAX_PACKET_LEN = 8; // header+type+len(2)+data(max 2)+checksum+trailer
const uint8_t MAX_HEX_LEN    = MAX_PACKET_LEN * 2 + 1; // 2 ASCII chars per byte + null terminator

// Converts raw packet bytes into an ASCII hex string (uppercase, no separators),
// e.g. {0xAA, 0x01} -> "AA01". outHex must be at least (2*len + 1) bytes.
// Returns the string length (2*len), not counting the null terminator.
uint8_t toHexString(const uint8_t *packet, uint8_t len, char *outHex) {
  const char hexDigits[] = "0123456789ABCDEF";

  for (uint8_t i = 0; i < len; i++) {
    outHex[i * 2]     = hexDigits[(packet[i] >> 4) & 0x0F];
    outHex[i * 2 + 1] = hexDigits[packet[i] & 0x0F];
  }
  outHex[len * 2] = '\0';

  return len * 2;
}

const unsigned long HEARTBEAT_INTERVAL = 15000UL; // 15 seconds
const byte MAX_CONSECUTIVE_FAILURES    = 4;        // soft-reset modem after this many full failed cycles in a row

char lastReply[220];
byte consecutiveFailures = 0;
unsigned long lastHeartbeatMillis = 0;
bool gprsReady = false; // true once GPRS is attached and kept live between sends

// ---------------------------------------------------------------
// Low-level modem helpers (unchanged from original heartbeat sketch)
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

// Builds a binary packet into outBuf and returns its total length.
// outBuf must be at least MAX_PACKET_LEN bytes. data may be NULL if dataLen is 0.
uint8_t buildPacket(uint8_t type, const uint8_t *data, uint8_t dataLen, uint8_t *outBuf) {
  uint8_t i = 0;

  outBuf[i++] = PKT_HEADER;
  outBuf[i++] = type;
  outBuf[i++] = (dataLen >> 8) & 0xFF; // length high byte
  outBuf[i++] = dataLen & 0xFF;        // length low byte

  uint8_t checksum = type ^ outBuf[2] ^ outBuf[3];

  for (uint8_t d = 0; d < dataLen; d++) {
    outBuf[i++] = data[d];
    checksum ^= data[d];
  }

  outBuf[i++] = checksum;
  outBuf[i++] = PKT_TRAILER;

  return i; // total packet length
}

// Sends a raw binary packet over TCP using AT+CIPSEND=<length>, which tells
// the modem exactly how many bytes to expect - no Ctrl+Z terminator needed.
// This matters for binary data: Ctrl+Z (0x1A) could otherwise appear as a
// legitimate data byte inside the packet and end the send early.
bool sendMessage(const uint8_t *packet, uint8_t len) {
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

  snprintf(command, sizeof(command), "AT+CIPSEND=%u", len);

  if (!sendAT(command, ">", 10000)) {
    sendAT("AT+CIPCLOSE", "CLOSE OK", 10000);
    return false;
  }

  Serial.print(F("\n>> Sending TCP payload ("));
  Serial.print(len);
  Serial.print(F(" bytes): "));
  for (uint8_t b = 0; b < len; b++) {
    Serial.write(packet[b]);
  }
  Serial.println();

  sim800.write(packet, len);
  // No Ctrl+Z: AT+CIPSEND=<n> auto-sends once <n> bytes have been received.

  bool success = waitFor("SEND OK", 30000);

  if (success) {
    Serial.println(F("Packet sent."));
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

// Sends payload as fast as possible: if GPRS is already attached, just
// does the quick CIPSTART/CIPSEND/CIPCLOSE. Only falls back to a full
// GPRS re-bring-up if GPRS wasn't ready yet or the quick send failed
// (e.g. the session silently dropped).
bool sendData(const uint8_t *packet, uint8_t len) {
  bool success = false;

  if (gprsReady) {
    success = sendMessage(packet, len);
    if (!success) {
      Serial.println(F("Quick send failed; GPRS session likely dropped."));
      gprsReady = false;
    }
  }

  if (!success) {
    if (bringUpGprs()) {
      success = sendMessage(packet, len);
      if (!success) {
        gprsReady = false;
      }
    }
  }

  return success;
}

// Wraps sendData() with the consecutive-failure watchdog / soft-reset
// logic, same as the original heartbeat loop had. Used for both sensor
// events and the periodic heartbeat.
bool sendWithWatchdog(const uint8_t *packet, uint8_t len) {
  bool ok = sendData(packet, len);

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

// ---------------------------------------------------------------
// Sensor handling (unchanged decision logic from original sensor sketch,
// except each matched event is now also sent over TCP)
// ---------------------------------------------------------------

void checkSensors() {
  int p2 = digitalRead(pin2);
  int p3 = digitalRead(pin3);
  int p4 = digitalRead(pin4);
  int p5 = digitalRead(pin5);

  bool changed = (p2 != lastP2) || (p3 != lastP3) ||
                 (p4 != lastP4) || (p5 != lastP5);

  if (changed) {
    const char *event = NULL;
    uint8_t eventCode = 0;

    if (p5 == 1 && p2 == 0 && p3 == 0 && p4 == 0) {
      event = "remote lock";
      eventCode = EVT_REMOTE_LOCK;
    } else if (p4 == 1 && p2 == 0 && p3 == 0 && p5 == 0) {
      event = "remote unlock";
      eventCode = EVT_REMOTE_UNLOCK;
    } else if (p2 == 1 && p5 == 1 && p3 == 0 && p4 == 0) {
      event = "motion sensor";
      eventCode = EVT_MOTION_SENSOR;
    } else if (p3 == 1 && p4 == 1 && p5 == 1 && p2 == 0) {
      event = "door sensor";
      eventCode = EVT_DOOR_SENSOR;
    }

    if (event != NULL) {
      Serial.println(event);

      uint8_t packet[MAX_PACKET_LEN];
      uint8_t data[1] = { eventCode };
      uint8_t packetLen = buildPacket(TYPE_SENSOR_EVENT, data, 1, packet);

      char hexPacket[MAX_HEX_LEN];
      uint8_t hexLen = toHexString(packet, packetLen, hexPacket);

      sendWithWatchdog((const uint8_t *)hexPacket, hexLen); // send this event's data to the TCP server, immediately
    }

    lastP2 = p2;
    lastP3 = p3;
    lastP4 = p4;
    lastP5 = p5;
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

  sendAT("ATE0", "OK", 2000);      // Disable command echo
  sendAT("AT+CPIN?", "OK", 5000);  // SIM status
  sendAT("AT+CSQ", "OK", 5000);    // Signal quality
  sendAT("AT+CSCLK=0", "OK", 5000); // Disable UART sleep mode - without this the
                                     // module stops answering after ~15s of idle

  waitForNetwork();

  // Bring GPRS up once at boot so the very first sensor event doesn't
  // have to pay the full bring-up cost - it can just do a quick send.
  bringUpGprs();

  lastHeartbeatMillis = millis();
}

void loop() {
  // Check sensors on every pass (fast, non-blocking) so an event can
  // trigger an immediate TCP send without waiting for the heartbeat timer.
  checkSensors();

  // Periodic keepalive heartbeat, still sends "Test" every 15 seconds,
  // independent of any sensor activity. Non-blocking wait so sensors
  // keep being polled while we wait for the interval to elapse.
  if (millis() - lastHeartbeatMillis >= HEARTBEAT_INTERVAL) {
    Serial.println(F("\n--- Heartbeat ---"));

    uint8_t packet[MAX_PACKET_LEN];
    uint8_t packetLen = buildPacket(TYPE_HEARTBEAT, NULL, 0, packet);

    char hexPacket[MAX_HEX_LEN];
    uint8_t hexLen = toHexString(packet, packetLen, hexPacket);

    sendWithWatchdog((const uint8_t *)hexPacket, hexLen);
    lastHeartbeatMillis = millis();
  }

  delay(20);
}
