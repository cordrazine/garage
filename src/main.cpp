#include <Homie.h>
#include <DHT.h>

#define FW_NAME "garage"
#define FW_VERSION "0.0.1"

//Pin allocation definition
const int PIN_TRIGGER = 16;    // D0 on NodeMCU, Trigger push button
const int PIN_RELAY = 5;    // D1 on NodeMCU, output relay for controlling the garage door
const int PIN_BUZZ = 14;    // D5 on NodeMCU, output buzzer
const int PIN_SD_OPEN = 2;    // D4 on NodeMCU, DoorSensor open contact (NC)
const int PIN_SD_CLOSE = 4;    // D2 on NodeMCU, DoorSensor close contact (NC)

// Constants
const int TIME_OUT = 10000; // 10s
const int TIME_RELAY_PULSE = 1000; // 1s
const int TIME_BUZZ_PULSE = 1000; // 1s
const int TIME_DEBOUNCE = 50; // 50ms
enum Command_enum {CMD_IDLE, CMD_OPEN, CMD_CLOSE, CMD_STOP};

// variables
// Door sensors and Trigger push button
int lastTriggerValue = 1;          // The last readout Trigger value
int lastSDOpenValue = 1;           // The last readout of the DoorSensor Open
int lastSDCloseValue = 1;          // The last readout of the DoorSensor Close

int lastCommandValue = CMD_IDLE;    // The last Command received via MQTT_DISCONNECTED

// Time-out
bool TimeOutActivated = false;
unsigned long startTimeOut = 0;

// Birth and last Will message
const char* birthMessage = "online";
const char* lwtMessage = "offline";       // not used for the moment

// state machine definitions
enum State_enum {ST_CLOSED, ST_OPENING, ST_OPEN, ST_CLOSING, ST_OPENING_STOPPED, ST_CLOSING_STOPPED};
uint8_t state = ST_CLOSED;   // reset and default state

// Homie nodes & debounce
HomieNode statusNode("status", "status");
HomieNode commandNode("command", "trigger");
HomieNode triggerNode("trigger", "trigger");
HomieNode SDOpenNode("SDOpen", "SDOpen");
HomieNode SDCloseNode("SDClose", "SDClose");
Bounce debouncerTrigger = Bounce(); // Bounce is built into Homie, so you can use it without including it first
Bounce debouncerSDOpen = Bounce(); // Bounce is built into Homie, so you can use it without including it first
Bounce debouncerSDClose = Bounce(); // Bounce is built into Homie, so you can use it without including it first

// Relay and buzzer handler
enum Relay_enum {RELAY_IDLE, RELAY_BUZZ, RELAY_RELAY};
bool RelayStartCommand = false;
bool RelayPulseActive = false;
unsigned long RelayPulseStart = 0;
uint8_t RelayState = RELAY_IDLE;   // reset and default state

void relayBuzzHandler (void){
  // handles the organisation of the buzzer and Relay
  // Use RelayStartCommand as input for the start of buzzer and relay
  // When not in idle hte RelayPulseActive flag is true

    switch(RelayState) {
      case RELAY_IDLE:
        if (RelayStartCommand == true){
          RelayStartCommand = false;  // reset command
          RelayPulseStart = millis();     // record the starting time
          digitalWrite(PIN_BUZZ, HIGH);
          RelayPulseActive = true;
          Homie.getLogger() << "RelayBuzzHandler: Buzzer started" << endl;
          RelayState = RELAY_BUZZ;
        }
      break;

      case RELAY_BUZZ:
        if (millis() - RelayPulseStart >= TIME_BUZZ_PULSE) {
          digitalWrite(PIN_BUZZ, LOW);
          digitalWrite(PIN_RELAY, HIGH);
          RelayPulseStart = millis();     // record the starting time
          Homie.getLogger() << "RelayBuzzHandler: Relay started" << endl;
          RelayState = RELAY_RELAY;
        }
      break;

      case RELAY_RELAY:
        if (millis() - RelayPulseStart >= TIME_RELAY_PULSE) {
          digitalWrite(PIN_RELAY, LOW);
          RelayPulseActive = false;
          Homie.getLogger() << "RelayBuzzHandler: Idle" << endl;
          RelayState = RELAY_IDLE;
        }
      break;

      default:
        RelayState = RELAY_IDLE;
      break;
    }
}

void state_machine_garagedoor(void) {
// This is the main Finite-State-Machine. This is a one-hot FSM.
// Based on: Doorsensors (SDxx), Trigger, MQTT command (CMD_xxx) the stat is set.
// before transition, a Relay pulse is generated
  switch(state) {
    case ST_CLOSED:
      // default State. No detection for SensorDoorClose since we suppose to be here alreary
      if (RelayPulseActive == false && (!lastTriggerValue  || lastCommandValue == CMD_OPEN)) {
        lastCommandValue = CMD_IDLE;    // reset command
        RelayStartCommand = true;       // Start the buzzer / relay sequence
        startTimeOut = millis();        // record time for time-out
        statusNode.setProperty("status").send("OPENING");
        Homie.getLogger() << "ST_CLOSED: Valid Trigger / command received. DoorState --> OPENING" << endl;
        state = ST_OPENING;
      }
      if(!lastSDOpenValue){        // In case after reset, faulty situation etc
        statusNode.setProperty("status").send("OPEN");
        Homie.getLogger() << "ST_CLOSED: Detected OpenSensor active. DoorState --> OPEN" << endl;
        state = ST_OPEN;
      }
    break;

    case ST_OPENING:
      if (RelayPulseActive == false && (!lastTriggerValue  || lastCommandValue == CMD_STOP)) {
        lastCommandValue = CMD_IDLE;    // reset command
        RelayStartCommand = true;       // Start the buzzer / relay sequence
        startTimeOut = millis();        // record time for time-out
        statusNode.setProperty("status").send("OPENING_STOPPED");
        Homie.getLogger() << "ST_OPENING: Valid Trigger / command received. DoorState --> OPENING_STOPPED" << endl;
        state = ST_OPENING_STOPPED;
      }
      if (millis() - startTimeOut >= TIME_OUT) {
        statusNode.setProperty("time-out").send("true");
        statusNode.setProperty("status").send("OPENING_STOPPED");
        Homie.getLogger() << "ST_OPENING: Time-out detected. Doorstate --> OPENING_STOPPED" << endl;
        state = ST_OPENING_STOPPED;
      }
      if(!lastSDOpenValue){
        Homie.getLogger() << "ST_OPENING: SDOpenValue detected. DoorState --> OPEN" << endl;
        statusNode.setProperty("status").send("OPEN");
        state = ST_OPEN;
      }
    break;

    case ST_OPEN:
      if (RelayPulseActive == false && (!lastTriggerValue  || lastCommandValue == CMD_CLOSE)) {
        lastCommandValue = CMD_IDLE;    // reset command
        RelayStartCommand = true;       // Start the buzzer / relay sequence
        startTimeOut = millis();        // record time for time-out
        statusNode.setProperty("status").send("CLOSING");
        Homie.getLogger() << "ST_OPEN: Valid Trigger detected. DoorState --> CLOSING" << endl;
        state = ST_CLOSING;
      }
      if(!lastSDCloseValue){        // In case after reset, faulty situation etc
        statusNode.setProperty("status").send("CLOSED");
        Homie.getLogger() << "ST_OPEN: Detected CloseSensor active. DoorState --> CLOSED" << endl;
        state = ST_CLOSED;;
      }
    break;

    case ST_CLOSING:
      if (RelayPulseActive == false && (!lastTriggerValue  || lastCommandValue == CMD_STOP)) {
        lastCommandValue = CMD_IDLE;    // reset command
        RelayStartCommand = true;       // Start the buzzer / relay sequence
        startTimeOut = millis();        // record time for time-out
        statusNode.setProperty("status").send("CLOSING_STOPPED");
        Homie.getLogger() << "ST_CLOSING: Valid Trigger / command received. DoorState --> CLOSING_STOPPED" << endl;
        state = ST_CLOSING_STOPPED;
      }
      if (millis() - startTimeOut >= TIME_OUT) {
        statusNode.setProperty("time-out").send("true");
        statusNode.setProperty("status").send("CLOSING_STOPPED");
        Homie.getLogger() << "ST_CLOSING: Time-out detected. Doorstate --> CLOSING_STOPPED" << endl;
        state = ST_CLOSING_STOPPED;
      }
      if(!lastSDCloseValue){
        Homie.getLogger() << "ST_CLOSING: SDCloseValue detected. DoorState --> CLOSED" << endl;
        statusNode.setProperty("status").send("CLOSED");
        state = ST_CLOSED;
      }
    break;

    case ST_OPENING_STOPPED:
      if (RelayPulseActive == false && (!lastTriggerValue  || lastCommandValue > CMD_IDLE)) {
        lastCommandValue = CMD_IDLE;    // reset command
        RelayStartCommand = true;       // Start the buzzer / relay sequence
        TimeOutActivated = false;
        statusNode.setProperty("time-out").send("false");
        startTimeOut = millis();        // record time for time-out
        statusNode.setProperty("status").send("CLOSING");
        Homie.getLogger() << "ST_OPENING_STOPPPED: Trigger received. DoorState --> CLOSING" << endl;
        state = ST_CLOSING;
      }
    break;

    case ST_CLOSING_STOPPED:
      if (RelayPulseActive == false && (!lastTriggerValue  || lastCommandValue > CMD_IDLE)) {
        lastCommandValue = CMD_IDLE;    // reset command
        RelayStartCommand = true;       // Start the buzzer / relay sequence
        TimeOutActivated = false;
        statusNode.setProperty("time-out").send("false");
        startTimeOut = millis();        // record time for time-out
        statusNode.setProperty("status").send("OPENING");
        Homie.getLogger() << "ST_CLOSING_STOPPPED: Trigger received. DoorState --> OPENING" << endl;
        state = ST_OPENING;
      }
    break;

    default:
      state = ST_CLOSED;
      lastCommandValue = CMD_IDLE;    // reset command
      statusNode.setProperty("status").send("CLOSED");
      break;
  }
}

void LoopHandler() {
  // handles the Door Sensors and the Trigger button, and trigger pulse
  int triggerValue = debouncerTrigger.read();
  if (triggerValue != lastTriggerValue) {
    // The trigger pin has an internal pull up and is pulled to GND when the button is pressed. So the trigger is an inverted trigger
     Homie.getLogger() << "Trigger is now " << (triggerValue ? "false" : "true") << endl;
     triggerNode.setProperty("active").send(triggerValue ? "false" : "true");
     lastTriggerValue = triggerValue;
  }

  int SDOpenValue = debouncerSDOpen.read();
  if (SDOpenValue != lastSDOpenValue) {
    // The trigger pin has an internal pull up and is pulled to GND when the button is pressed. So the trigger is an inverted trigger
     Homie.getLogger() << "SensorDoorOpen (handler) is now " << (SDOpenValue ? "false" : "true") << endl;
     SDOpenNode.setProperty("active").send(SDOpenValue ? "false" : "true");
     lastSDOpenValue = SDOpenValue;
  }

  int SDCloseValue = debouncerSDClose.read();
  if (SDCloseValue != lastSDCloseValue) {
    // The trigger pin has an internal pull up and is pulled to GND when the button is pressed. So the trigger is an inverted trigger
    Homie.getLogger() << "SensorDoorClose (handler) is now " << (SDCloseValue ? "false" : "true") << endl;
    SDCloseNode.setProperty("active").send(SDCloseValue ? "false" : "true");
    lastSDCloseValue = SDCloseValue;
  }

}

bool commandHandler(const HomieRange& range, const String& value) {
  if (value != "OPEN" && value != "CLOSE" && value != "STOP") return false;
    if (value == "OPEN") lastCommandValue = CMD_OPEN;
    if (value == "CLOSE") lastCommandValue = CMD_CLOSE;
    if (value == "STOP") lastCommandValue = CMD_STOP;
    Homie.getLogger() << "MQTT Command " << value << " received."<< endl;
    commandNode.setProperty("command").send(value);
    return true;
  }

void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::STANDALONE_MODE:
      Serial << "Standalone mode started" << endl;
      break;
    case HomieEventType::CONFIGURATION_MODE:
      Serial << "Configuration mode started" << endl;
      break;
    case HomieEventType::NORMAL_MODE:
      Serial << "Normal mode started" << endl;
      break;
    case HomieEventType::OTA_STARTED:
      Serial << "OTA started" << endl;
      break;
    case HomieEventType::OTA_PROGRESS:
      Serial << "OTA progress, " << event.sizeDone << "/" << event.sizeTotal << endl;
      break;
    case HomieEventType::OTA_FAILED:
      Serial << "OTA failed" << endl;
      break;
    case HomieEventType::OTA_SUCCESSFUL:
      Serial << "OTA successful" << endl;
      break;
    case HomieEventType::ABOUT_TO_RESET:
      Serial << "About to reset" << endl;
      break;
    case HomieEventType::WIFI_CONNECTED:
      Serial << "Wi-Fi connected, IP: " << event.ip << ", gateway: " << event.gateway << ", mask: " << event.mask << endl;
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      Serial << "Wi-Fi disconnected, reason: " << (int8_t)event.wifiReason << endl;
      break;
    case HomieEventType::MQTT_READY:
      Serial << "MQTT connected" << endl;
      statusNode.setProperty("availability").send(birthMessage);
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      break;
    case HomieEventType::MQTT_PACKET_ACKNOWLEDGED:
      //Serial << "MQTT packet acknowledged, packetId: " << event.packetId << endl;
      break;
    case HomieEventType::READY_TO_SLEEP:
      Serial << "Ready to sleep" << endl;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  Homie.onEvent(onHomieEvent);
  pinMode(PIN_TRIGGER, INPUT);
  digitalWrite(PIN_TRIGGER, HIGH);   // Since set as input, this will enable the internal pull up
  debouncerTrigger.attach(PIN_TRIGGER);
  debouncerTrigger.interval(TIME_DEBOUNCE);

  pinMode(PIN_SD_OPEN, INPUT);
  digitalWrite(PIN_SD_OPEN, HIGH);   // Since set as input, this will enable the internal pull up
  debouncerSDOpen.attach(PIN_SD_OPEN);
  debouncerSDOpen.interval(TIME_DEBOUNCE);

  pinMode(PIN_SD_CLOSE, INPUT);
  digitalWrite(PIN_SD_CLOSE, HIGH);   // Since set as input, this will enable the internal pull up
  debouncerSDClose.attach(PIN_SD_CLOSE);
  debouncerSDClose.interval(TIME_DEBOUNCE);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  Homie_setFirmware(FW_NAME, FW_VERSION);
  Homie.setLoopFunction(LoopHandler);
  commandNode.advertise("command").settable(commandHandler);
  statusNode.advertise("status");
  statusNode.advertise("time-out");
  statusNode.advertise("availability");
  triggerNode.advertise("active");
  SDOpenNode.advertise("active");
  SDCloseNode.advertise("active");
  lastCommandValue = CMD_IDLE;
  Homie.setup();
}

void loop() {
  Homie.loop();
  debouncerTrigger.update();
  debouncerSDOpen.update();
  debouncerSDClose.update();
  state_machine_garagedoor();
  relayBuzzHandler();
}
