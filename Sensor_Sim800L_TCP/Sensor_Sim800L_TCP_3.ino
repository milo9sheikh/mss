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

  === v3 additions: device state, alarm siren, EEPROM ===
  - deviceActive : remote lock  -> ACTIVE, remote unlock -> DE-ACTIVE
                   (physical remote inputs AND server commands both work)
  - alarmEnabled : alarm ON / OFF, set by a server command (default ON)
  - If deviceActive AND alarmEnabled AND motion/door sensor fires,
    the siren output (D6) is driven HIGH for 30 seconds.
    The 30 s timer is serviced inside the blocking modem waits too, so
    a slow TCP send can't stretch the siren time.
    Remote unlock or alarm OFF silences a ringing siren immediately.
  - Both states are saved in EEPROM (only rewritten when they change)
    and restored at boot.
  - SERVER COMMANDS (text, sent by the server over the same TCP socket):
        ACTIVE#     -> same as remote lock
        DEACTIVE#   -> same as remote unlock
        ALM,1,1#    -> alarm ON
        ALM,1,0#    -> alarm OFF
    '#' or a line ending both end a command; case doesn't matter.
    After every command the device replies with a status packet.
  - To receive commands the TCP connection is now KEPT OPEN between sends
    (no CIPSTART/CIPCLOSE per packet). If the server closes it or a send
    fails, the device reconnects automatically.
  - New TCP packet TYPE_STATUS (0x02), DATA = [active][alarm]
    (1 = active / alarm ON, 0 = de-active / alarm OFF). Sent:
      * once at boot (after GPRS is up)
      * right after every remote lock / unlock and every alarm toggle
      * every STATUS_INTERVAL as a resync
  - Existing packets (heartbeat, sensor events) are unchanged.

  Extra wiring:
    D6 -> siren/buzzer driver (transistor or relay module, NOT the siren
          directly from the pin)
*/

#include <SoftwareSerial.h>
#include <EEPROM.h>

// ===== Sensor pins (unchanged from original sensor sketch) =====
const int pin2 = 2;
const int pin3 = 3;
const int pin4 = 4;
const int pin5 = 5;

int lastP2, lastP3, lastP4, lastP5;

// ===== NEW: siren output =====
const uint8_t ALARM_PIN     = 6;  // siren driver, HIGH = siren ON
const unsigned long ALARM_DURATION   = 30000UL; // siren rings for 30 seconds
const unsigned long SENSOR_SETTLE_MS = 50;      // wait for the 4 input lines to settle before decoding

// ===== NEW: device state (persisted in EEPROM) =====
const int     EEPROM_ADDR_MAGIC  = 0;
const int     EEPROM_ADDR_ACTIVE = 1;
const int     EEPROM_ADDR_ALARM  = 2;
const uint8_t EEPROM_MAGIC       = 0xA5; // marks EEPROM as initialised by this firmware

bool deviceActive = false; // true = active (remote locked), false = de-active (remote unlocked)
bool alarmEnabled = true;  // true = alarm ON, false = alarm OFF

bool alarmSounding = false;
unsigned long alarmStartMillis = 0;

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
const uint8_t TYPE_STATUS       = 0x02; // DATA = [deviceActive 1/0][alarmEnabled 1/0]

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
const unsigned long STATUS_INTERVAL    = 60000UL; // periodic status resync (also sent on every state change)
const byte MAX_CONSECUTIVE_FAILURES    = 4;        // soft-reset modem after this many full failed cycles in a row

char lastReply[220];
byte consecutiveFailures = 0;
unsigned long lastHeartbeatMillis = 0;
unsigned long lastStatusMillis = 0;
bool gprsReady = false; // true once GPRS is attached and kept live between sends
bool tcpConnected = false; // true while the TCP socket to the server is open (kept open between sends)

// ===== NEW: server command receiver =====
const uint8_t CMD_ACTIVE    = 1;
const uint8_t CMD_DEACTIVE  = 2;
const uint8_t CMD_ALARM_ON  = 3;
const uint8_t CMD_ALARM_OFF = 4;

const uint8_t RX_BUF_SIZE = 24;
const unsigned long RX_IDLE_FLUSH_MS = 300; // treat a pause in incoming text as end of a command

char rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;
unsigned long lastRxMillis = 0;

const uint8_t CMD_QUEUE_SIZE = 4;
uint8_t cmdQueue[CMD_QUEUE_SIZE];
uint8_t cmdCount = 0;

// ---------------------------------------------------------------
// NEW: EEPROM state storage
// ---------------------------------------------------------------

// EEPROM.update() only writes a byte if it actually changed, so calling
// this on every state change doesn't wear the EEPROM needlessly.
void saveState() {
  EEPROM.update(EEPROM_ADDR_MAGIC,  EEPROM_MAGIC);
  EEPROM.update(EEPROM_ADDR_ACTIVE, deviceActive ? 1 : 0);
  EEPROM.update(EEPROM_ADDR_ALARM,  alarmEnabled ? 1 : 0);
}

void loadState() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) {
    // First boot / blank EEPROM: start de-active with alarm ON.
    deviceActive = false;
    alarmEnabled = true;
    saveState();
    Serial.println(F("EEPROM empty: defaults stored (DE-ACTIVE, alarm ON)"));
    return;
  }

  deviceActive = (EEPROM.read(EEPROM_ADDR_ACTIVE) == 1);
  alarmEnabled = (EEPROM.read(EEPROM_ADDR_ALARM)  == 1);
}

// ---------------------------------------------------------------
// NEW: siren control (non-blocking, millis() based)
// ---------------------------------------------------------------

void alarmStart() {
  if (alarmSounding) return; // already ringing; don't restart the 30 s timer

  alarmSounding = true;
  alarmStartMillis = millis();
  digitalWrite(ALARM_PIN, HIGH);
  Serial.println(F("ALARM: siren ON (30 s)"));
}

void alarmStop() {
  if (!alarmSounding) return;

  alarmSounding = false;
  digitalWrite(ALARM_PIN, LOW);
  Serial.println(F("ALARM: siren OFF"));
}

// Turns the siren off once ALARM_DURATION has passed. Called from loop()
// AND from inside the blocking modem waits, so a long TCP send can't
// keep the siren ringing past 30 s.
void alarmService() {
  if (alarmSounding && (millis() - alarmStartMillis >= ALARM_DURATION)) {
    alarmStop();
  }
}

// delay() replacement that keeps the siren timer running.
void serviceDelay(unsigned long ms) {
  unsigned long started = millis();

  while (millis() - started < ms) {
    alarmService();
    delay(1);
  }
}

// ---------------------------------------------------------------
// NEW: receiving server commands
//
// Every character coming from the modem is passed to rxFeed(), no matter
// where it was read (loop, waitFor, clearModemInput). rxFeed() only
// RECORDS a command in a small queue; the command is executed later from
// loop() by processServerCommands(). This avoids starting a TCP send from
// inside another TCP send.
// ---------------------------------------------------------------

void queueCommand(uint8_t cmd) {
  if (cmdCount < CMD_QUEUE_SIZE) {
    cmdQueue[cmdCount++] = cmd;
  } else {
    Serial.println(F("\nCommand queue full, command dropped"));
  }
}

bool rxEndsWith(const char *token) {
  uint8_t tokenLen = strlen(token);

  if (rxLen < tokenLen) {
    return false;
  }

  return memcmp(rxBuf + rxLen - tokenLen, token, tokenLen) == 0;
}

// Looks at the text collected since the last terminator.
// NOTE: DEACTIVE must be checked before ACTIVE, because "DEACTIVE" ends with "ACTIVE".
void rxEvaluate() {
  if (rxLen == 0) {
    return;
  }

  if (rxEndsWith("DEACTIVE")) {
    queueCommand(CMD_DEACTIVE);
  } else if (rxEndsWith("ACTIVE")) {
    queueCommand(CMD_ACTIVE);
  } else if (rxEndsWith("ALM,1,1")) {
    queueCommand(CMD_ALARM_ON);
  } else if (rxEndsWith("ALM,1,0")) {
    queueCommand(CMD_ALARM_OFF);
  } else if (rxEndsWith("CLOSED")) {
    tcpConnected = false;               // server closed the socket
  } else if (rxEndsWith("+PDP: DEACT")) {
    tcpConnected = false;               // GPRS context dropped by the network
    gprsReady = false;
  }

  rxLen = 0;
}

void rxFeed(char c) {
  lastRxMillis = millis();

  // '#' or a line ending finishes a command
  if (c == '#' || c == '\r' || c == '\n') {
    rxEvaluate();
    return;
  }

  if (c >= 'a' && c <= 'z') {
    c = c - 'a' + 'A'; // commands are case-insensitive
  }

  if (rxLen >= RX_BUF_SIZE - 1) {
    memmove(rxBuf, rxBuf + 1, rxLen - 1); // buffer full: drop the oldest char
    rxLen--;
  }

  rxBuf[rxLen++] = c;
}

// Reads whatever the modem sent on its own (server commands, "CLOSED", ...)
void pollModem() {
  while (sim800.available()) {
    char c = sim800.read();
    Serial.write(c);
    rxFeed(c);
  }

  // A command without '#' or line ending: evaluate it after a short pause.
  if (rxLen > 0 && (millis() - lastRxMillis >= RX_IDLE_FLUSH_MS)) {
    rxEvaluate();
  }
}

// ---------------------------------------------------------------
// Low-level modem helpers (unchanged from original heartbeat sketch,
// except alarmService() / serviceDelay() so the siren timer keeps running)
// ---------------------------------------------------------------

void clearModemInput() {
  // Leftover modem text is no longer thrown away: a server command could be
  // in there. Feed it to the command parser instead.
  while (sim800.available()) {
    char c = sim800.read();
    Serial.write(c);
    rxFeed(c);
  }
}

bool waitFor(const char *expected, unsigned long timeoutMs) {
  unsigned long started = millis();
  size_t position = 0;

  lastReply[0] = '\0';

  while (millis() - started < timeoutMs) {
    alarmService(); // keep the 30 s siren timer accurate during long modem waits

    while (sim800.available()) {
      char c = sim800.read();

      // Print every SIM800L response in Serial Monitor
      Serial.write(c);

      rxFeed(c); // NEW: a server command may arrive while we wait for a reply

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
    serviceDelay(2000);
  }

  Serial.println(F("Network registration failed."));
  return false;
}

bool startGprs() {
  Serial.println(F("\n--- Starting GPRS (full clean bring-up) ---"));

  // Always tear down any previous IP session first. It is okay if this
  // "fails" because there was nothing to tear down.
  sendAT("AT+CIPSHUT", "SHUT OK", 15000);
  tcpConnected = false; // CIPSHUT closes any open socket

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

// Opens the TCP connection to the server. The connection is then KEPT OPEN
// (no CIPCLOSE after every packet) so the server can push commands to us.
bool tcpConnect() {
  Serial.println(F("\n--- Connecting to server ---"));

  char command[110];

  snprintf(command, sizeof(command),
           "AT+CIPSTART=\"TCP\",\"%s\",\"%u\"",
           SERVER, PORT);

  if (!sendAT(command, "CONNECT OK", 30000)) {
    sendAT("AT+CIPSTATUS", "OK", 5000); // printed to Serial Monitor for debugging
    sendAT("AT+CIPCLOSE", "CLOSE OK", 10000);
    tcpConnected = false;
    return false;
  }

  tcpConnected = true;
  return true;
}

void tcpClose() {
  // Ignored by the modem if the server has already closed it.
  sendAT("AT+CIPCLOSE", "CLOSE OK", 10000);
  tcpConnected = false;
}

// Sends a raw binary packet over the ALREADY OPEN connection using
// AT+CIPSEND=<length>, which tells the modem exactly how many bytes to
// expect - no Ctrl+Z terminator needed. This matters for binary data:
// Ctrl+Z (0x1A) could otherwise appear as a legitimate data byte inside
// the packet and end the send early.
bool sendOnOpenConnection(const uint8_t *packet, uint8_t len) {
  char command[24];

  snprintf(command, sizeof(command), "AT+CIPSEND=%u", len);

  if (!sendAT(command, ">", 10000)) {
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

  return success;
}

// Sends one packet. Connects first if the socket isn't open. If the send
// fails on a socket that WAS open (server closed it, silent drop, ...),
// closes it, reconnects once and retries - all without redoing GPRS.
bool sendMessage(const uint8_t *packet, uint8_t len) {
  bool wasOpen = tcpConnected;

  if (!wasOpen && !tcpConnect()) {
    return false;
  }

  if (sendOnOpenConnection(packet, len)) {
    return true;
  }

  tcpClose();

  if (!wasOpen) {
    return false; // a brand-new connection failed too: let the caller escalate
  }

  Serial.println(F("Send on open connection failed; reconnecting..."));

  if (!tcpConnect()) {
    return false;
  }

  if (sendOnOpenConnection(packet, len)) {
    return true;
  }

  tcpClose();
  return false;
}

void modemSoftReset() {
  Serial.println(F("\n=== Too many failed cycles: soft-resetting modem (AT+CFUN=1,1) ==="));

  sendAT("AT+CFUN=1,1", "OK", 10000);
  tcpConnected = false;
  serviceDelay(8000); // let the radio stack reboot

  for (byte attempt = 0; attempt < 10; attempt++) {
    if (sendAT("AT", "OK", 2000)) break;
    serviceDelay(1000);
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
// NEW: packet helper + status reporting + state changes
// ---------------------------------------------------------------

// Builds a packet, hex-encodes it and sends it (with the watchdog).
// Used for heartbeat, sensor events and status packets.
bool sendPacket(uint8_t type, const uint8_t *data, uint8_t dataLen) {
  uint8_t packet[MAX_PACKET_LEN];
  uint8_t packetLen = buildPacket(type, data, dataLen, packet);

  char hexPacket[MAX_HEX_LEN];
  uint8_t hexLen = toHexString(packet, packetLen, hexPacket);

  return sendWithWatchdog((const uint8_t *)hexPacket, hexLen);
}

// Tells the server the current state: [deviceActive][alarmEnabled]
bool sendStatus() {
  Serial.print(F("\n--- Status: "));
  Serial.print(deviceActive ? F("ACTIVE") : F("DE-ACTIVE"));
  Serial.print(F(", alarm "));
  Serial.println(alarmEnabled ? F("ON") : F("OFF"));

  uint8_t data[2];
  data[0] = deviceActive ? 1 : 0;
  data[1] = alarmEnabled ? 1 : 0;

  bool ok = sendPacket(TYPE_STATUS, data, 2);
  lastStatusMillis = millis();
  return ok;
}

void setDeviceActive(bool active) {
  deviceActive = active;
  saveState();

  if (!active) {
    alarmStop(); // remote unlock silences a ringing siren
  }
}

void setAlarmEnabled(bool enabled) {
  alarmEnabled = enabled;
  saveState();

  if (!enabled) {
    alarmStop(); // alarm OFF silences a ringing siren
  }
}

// Applies commands received from the server (queued by rxFeed()), then
// replies with ONE status packet that shows the resulting state.
void processServerCommands() {
  if (cmdCount == 0) {
    return;
  }

  while (cmdCount > 0) {
    uint8_t cmd = cmdQueue[0];

    for (uint8_t i = 1; i < cmdCount; i++) {
      cmdQueue[i - 1] = cmdQueue[i];
    }
    cmdCount--;

    switch (cmd) {
      case CMD_ACTIVE:
        Serial.println(F("\nServer command: ACTIVE (lock)"));
        setDeviceActive(true);
        break;
      case CMD_DEACTIVE:
        Serial.println(F("\nServer command: DEACTIVE (unlock)"));
        setDeviceActive(false);
        break;
      case CMD_ALARM_ON:
        Serial.println(F("\nServer command: alarm ON"));
        setAlarmEnabled(true);
        break;
      case CMD_ALARM_OFF:
        Serial.println(F("\nServer command: alarm OFF"));
        setAlarmEnabled(false);
        break;
    }
  }

  sendStatus(); // acknowledge (also sent if the state didn't change)
}

// ---------------------------------------------------------------
// Sensor handling (decision logic unchanged from original sensor sketch;
// each matched event is sent over TCP, and now also updates the device
// state / triggers the siren)
// ---------------------------------------------------------------

bool sensorInputsChanged(int p2, int p3, int p4, int p5) {
  return (p2 != lastP2) || (p3 != lastP3) ||
         (p4 != lastP4) || (p5 != lastP5);
}

void checkSensors() {
  int p2 = digitalRead(pin2);
  int p3 = digitalRead(pin3);
  int p4 = digitalRead(pin4);
  int p5 = digitalRead(pin5);

  if (!sensorInputsChanged(p2, p3, p4, p5)) {
    return;
  }

  // The 4 lines may not switch at exactly the same instant. Wait a moment
  // and re-read so a half-changed pattern (e.g. only D4 rising while the
  // door pattern is still forming) is never mistaken for "remote unlock".
  delay(SENSOR_SETTLE_MS);

  p2 = digitalRead(pin2);
  p3 = digitalRead(pin3);
  p4 = digitalRead(pin4);
  p5 = digitalRead(pin5);

  if (!sensorInputsChanged(p2, p3, p4, p5)) {
    return; // it was only a glitch
  }

  lastP2 = p2;
  lastP3 = p3;
  lastP4 = p4;
  lastP5 = p5;

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

  if (event == NULL) {
    return;
  }

  Serial.println(event);

  // 1) Local action FIRST (state + siren), so the siren doesn't wait for
  //    the TCP send to finish.
  if (eventCode == EVT_REMOTE_LOCK) {
    setDeviceActive(true);
  } else if (eventCode == EVT_REMOTE_UNLOCK) {
    setDeviceActive(false);
  } else { // motion or door sensor
    if (deviceActive && alarmEnabled) {
      alarmStart();
    } else {
      Serial.println(F("Sensor triggered but no alarm (device de-active or alarm OFF)"));
    }
  }

  // 2) Send the event to the server immediately (unchanged behaviour).
  uint8_t data[1] = { eventCode };
  sendPacket(TYPE_SENSOR_EVENT, data, 1);

  // 3) After a lock/unlock, also report the new state.
  if (eventCode == EVT_REMOTE_LOCK || eventCode == EVT_REMOTE_UNLOCK) {
    sendStatus();
  }
}

// ---------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------

void setup() {
  // Siren OFF first, before anything else can delay us.
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

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

  loadState(); // restore ACTIVE/DE-ACTIVE and alarm ON/OFF from EEPROM

  Serial.println(F("\n\nSensor + SIM800L TCP sender"));
  Serial.print(F("Restored state: "));
  Serial.print(deviceActive ? F("ACTIVE") : F("DE-ACTIVE"));
  Serial.print(F(", alarm "));
  Serial.println(alarmEnabled ? F("ON") : F("OFF"));
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

  // Tell the server the restored state right after boot.
  sendStatus();

  lastHeartbeatMillis = millis();
  lastStatusMillis = millis();
}

void loop() {
  // Check sensors on every pass (fast, non-blocking) so an event can
  // trigger an immediate TCP send without waiting for the heartbeat timer.
  checkSensors();

  pollModem();             // NEW: read anything the server sent (commands)
  processServerCommands(); // NEW: apply ACTIVE#/DEACTIVE#/ALM,1,x# commands
  alarmService();          // NEW: switch siren off after 30 s

  // Periodic keepalive heartbeat, still sends "Test" every 15 seconds,
  // independent of any sensor activity. Non-blocking wait so sensors
  // keep being polled while we wait for the interval to elapse.
  if (millis() - lastHeartbeatMillis >= HEARTBEAT_INTERVAL) {
    Serial.println(F("\n--- Heartbeat ---"));

    sendPacket(TYPE_HEARTBEAT, NULL, 0);
    lastHeartbeatMillis = millis();
  }

  // NEW: periodic status resync (in case a status packet was lost)
  if (millis() - lastStatusMillis >= STATUS_INTERVAL) {
    sendStatus();
  }

  delay(20);
}
