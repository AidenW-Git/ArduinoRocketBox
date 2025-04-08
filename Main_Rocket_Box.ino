#include <DIYables_4Digit7Segment_74HC595.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiS3.h>
#include <WiFiServer.h>

// Pin definitions
#define SCLK1 1
#define RCLK1 2
#define DIO1 3
#define SCLK2 4
#define RCLK2 5
#define DIO2 6
#define pressureSensorPin A5
#define batteryVoltagePin A4
#define leftSolenoidPin 10
#define rightSolenoidPin 8
#define decorativeLedPin 9
#define launchTriggerPin 13
#define LED_PIN 12
#define LED_COUNT 38

// Display and LED strip objects
DIYables_4Digit7Segment_74HC595 display1(SCLK1, RCLK1, DIO1);
DIYables_4Digit7Segment_74HC595 display2(SCLK2, RCLK2, DIO2);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

// Battery voltage calibration
const float fullArduinoVoltage = 4.7;
const float deadArduinoVoltage = 3.0;

// Display flashing variables
unsigned long displayFlashTimer = 0;
bool displayState = false;

// Sensor and battery variables
float percentage = 0.0;
float psi = 0.0;
float latestArduinoVoltage = 0.0;

// Launch variables
unsigned long launchTimer = 0;
unsigned long solenoidTimer = 0;
bool buttonPressed = false;
bool armedOveride = false;

// LED variables
unsigned long redFlashTimer = 0;
bool redFlashState = true;
unsigned long greenFlashTimer = 0;
bool greenFlashState = true;
unsigned long decorativeRainbowTimer = 0;
uint16_t decorativeRainbowHue = 0;
int redFadeValue = 0;
int redFadeDirection = 1;

// Configuration
const int transitionPoint = 16;
const unsigned long launchDuration = 2000;
const unsigned long solenoidDuration = 500;

// State machine for launcher
enum LauncherState {
  IDLE,
  ARMED,
  LAUNCHING,
  DEAD
};
LauncherState launcherState = IDLE;

// WiFi AP credentials
char ssid[] = "LBCRocketBox";

WiFiServer server(80);

// Function prototypes
float readBatteryPercentage();
void handleLaunchSequence();
void updateLEDs();
uint32_t Wheel(byte WheelPos);
void updateSolenoids();
void handleClient(WiFiClient client);
void printWiFiStatus();
void updatePressureDisplay();
void updateBatteryDisplay();

void setup() {
  Serial.begin(9600);
  Serial.println("Serial started");  //Added serial print to verify serial is working.

  Serial.println("Access Point Web Server");

  pinMode(leftSolenoidPin, OUTPUT);
  pinMode(rightSolenoidPin, OUTPUT);
  pinMode(decorativeLedPin, OUTPUT);
  pinMode(launchTriggerPin, INPUT_PULLUP);

  digitalWrite(leftSolenoidPin, LOW);
  digitalWrite(rightSolenoidPin, LOW);
  digitalWrite(decorativeLedPin, HIGH);

  display1.clear();
  display2.clear();

  strip.begin();
  strip.show();

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  if (WiFi.beginAP(ssid) == WL_AP_LISTENING) {
    Serial.println("AP started");
  } else {
    Serial.println("AP failed");
    Serial.print("Status code: ");
    Serial.println(WiFi.beginAP(ssid));
  }
  server.begin();
  if (digitalRead(launchTriggerPin) == LOW) {
    armedOveride = true;
    launcherState = ARMED;
    for (int i = 0; i <= LED_COUNT; i++) {
      strip.setPixelColor(i, 0, 255, 0);
    }
    strip.show();
    delay(2000);
  }
}

void loop() {
  handleLaunchSequence();
  display1.loop();
  display2.loop();
  handleClient(server.available());
  display1.loop();
  display2.loop();
  updateLEDs();
  updateSensors();
  updateSolenoids();
}

void updateSensors() {
  static unsigned long sensorTimer = 0;
  static unsigned long batteryTimer = 0;
  static unsigned long battReadTimer = 0;
  unsigned long currentTime = millis();

  if (currentTime - battReadTimer >= 2000) {
    battReadTimer = currentTime;
    readBatteryPercentage();
  }

  if (currentTime - sensorTimer >= 2000) {
    sensorTimer = currentTime;
    updatePressureDisplay();
  }

  if (currentTime - batteryTimer >= 2000) {
    batteryTimer = currentTime;
    updateBatteryDisplay();
  }
}

float readBatteryPercentage() {
  float sum = 0.0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(batteryVoltagePin);
  }
  latestArduinoVoltage = (sum / 10.0) * (5.0 / 1023.0);
  percentage = ((latestArduinoVoltage - deadArduinoVoltage) / (fullArduinoVoltage - deadArduinoVoltage)) * 100.0;
  return constrain(percentage, 0, 100);
}

void updatePressureDisplay() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pressureSensorPin);
  }
  int sensorValue = sum / 10;
  float voltage = sensorValue * (5.0 / 1023.0);
  psi = (voltage - 0.45) * (100.0 / (4.5 - 0.45));
  psi = round(psi * 10.0) / 10.0;

  if (psi > 99.9) {
    psi = 99.9;
  }

  display2.clear();
  if (psi < 0) {
    display2.setChar(2, SegChars::DASH);
    display2.setNumber(3, abs((int)(psi * 10)) / 10);
    display2.setDot(3);
    display2.setNumber(4, abs((int)(psi * 10)) % 10);
  } else {
    int integerPart = (int)psi;
    int decimalPart = (int)(psi * 10) % 10;
    display2.setNumber(4, decimalPart);
    display2.setDot(3);
    display2.setNumber(3, integerPart % 10);
    if (integerPart >= 10) {
      display2.setNumber(2, integerPart / 10);
    }
  }
  display2.show();
}

void updateBatteryDisplay() {
  readBatteryPercentage();
  int displayPercentage = (int)percentage;
  bool triggerWarning = false;
  display1.clear();
  display1.setNumber(2, displayPercentage / 100);
  display1.setNumber(3, (displayPercentage / 10) % 10);
  display1.setNumber(4, displayPercentage % 10);
  display1.show();
  if (percentage <= 10) {
    launcherState = DEAD;
    triggerWarning = true;
  }
  else if (armedOveride == true && triggerWarning) {
    launcherState = ARMED;
    triggerWarning = false;
  }

  else if (triggerWarning) {
    launcherState = IDLE;
    triggerWarning = false;
  }

}

void handleLaunchSequence() {
  unsigned long currentTime = millis();

  if (digitalRead(launchTriggerPin) == LOW && !buttonPressed && launcherState == ARMED) {
    if (launcherState == ARMED || launcherState == IDLE) {
      launcherState = LAUNCHING;
      launchTimer = currentTime;
      solenoidTimer = currentTime;
      buttonPressed = true;

    }
  }

  if (launcherState == LAUNCHING && (currentTime - launchTimer >= launchDuration) && armedOveride == false) {
    launcherState = IDLE;
  } else if (launcherState == LAUNCHING && (currentTime - launchTimer >= launchDuration) && armedOveride == true) {
    launcherState = ARMED;
  }

  if (digitalRead(launchTriggerPin) == HIGH) {
    buttonPressed = false;
  }
}

void updateSolenoids() {
  unsigned long currentTime = millis();
  if (launcherState == LAUNCHING && (currentTime - solenoidTimer < solenoidDuration)) {
    digitalWrite(leftSolenoidPin, HIGH);
    digitalWrite(rightSolenoidPin, HIGH);
  } else {
    digitalWrite(leftSolenoidPin, LOW);
    digitalWrite(rightSolenoidPin, LOW);
  }
}

void updateLEDs() {
  unsigned long currentTime = millis();

  // LAUNCHING State Instructions

  if (launcherState == LAUNCHING) {
    if (currentTime - redFlashTimer >= 100) {
      redFlashTimer = currentTime;
      redFlashState = !redFlashState;
    }
    for (int i = 0; i < LED_COUNT; i++) {
      if (redFlashState) {
        strip.setPixelColor(i, 255, 0, 0);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }
    strip.show();
    return;
  }

  // ARMED State Instructions

  if (launcherState == ARMED) {

    if (currentTime - greenFlashTimer >= 400) {
      greenFlashTimer = currentTime;
      greenFlashState = !greenFlashState;
    }
    for (int i = 0; i < transitionPoint; i++) {
      if (greenFlashState) {
        strip.setPixelColor(i, 0, 255, 0);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }

    for (int i = transitionPoint; i < LED_COUNT; i++) {
      if (currentTime - decorativeRainbowTimer >= 12) {
        strip.setPixelColor(i, Wheel((i * 256 / (transitionPoint + 1) + decorativeRainbowHue) & 255));
      }
    }
    if (currentTime - decorativeRainbowTimer >= 12) {
      decorativeRainbowTimer = currentTime;
      decorativeRainbowHue++;
      if (decorativeRainbowHue >= 256) {
        decorativeRainbowHue = 0;
      }
    }
    strip.show();
    return;
  }

  // IDLE State Instructions

  if (launcherState == IDLE) {

    for (int i = 0; i <= 8; i++) {
      strip.setPixelColor(i, 255, 0, 0);
    }
    for (int i = 9; i < LED_COUNT; i++) {
      if (currentTime - decorativeRainbowTimer >= 12) {
        strip.setPixelColor(i, Wheel((i * 256 / (transitionPoint + 1) + decorativeRainbowHue) & 255));
      }
    }

    strip.show();
  }

  // DEAD State Instructions

  if (launcherState == DEAD) {

    if (currentTime - redFlashTimer >= 30) {
      redFlashTimer = currentTime;
      redFadeValue += redFadeDirection * 5;
      if (redFadeValue >= 255) {
        redFadeValue = 255;
        redFadeDirection = -1;
      } else if (redFadeValue <= 0) {
        redFadeValue = 0;
        redFadeDirection = 1;
      }
    }
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, redFadeValue, 0, 0);
    }

    strip.show();
    return;
  }

  redFadeValue = 0;
  redFadeDirection = 1;
  redFlashTimer = 0;

  greenFlashTimer = 0;

  if (currentTime - decorativeRainbowTimer >= 12) {
    decorativeRainbowTimer = currentTime;
    decorativeRainbowHue++;
    if (decorativeRainbowHue >= 256) {
      decorativeRainbowHue = 0;
    }
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void handleClient(WiFiClient client) {
  static enum { WAIT_FOR_REQUEST,
                HANDLE_REQUEST } state = WAIT_FOR_REQUEST;
  static String request = "";
  static unsigned long timeout = 0;
  String response = "";

  if (client) {
    if (state == WAIT_FOR_REQUEST) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (request.endsWith("\r\n")) {
          state = HANDLE_REQUEST;
        }
      } else if (timeout == 0) {
        timeout = millis() + 5000;
      } else if (millis() > timeout) {
        request = "";
        client.stop();
        state = WAIT_FOR_REQUEST;
        timeout = 0;
      }
    } else if (state == HANDLE_REQUEST) {
      bool armChanged = false;  // Track if arm state changed
      bool fired = false;

      if (request.indexOf("/arm") != -1) {
        if (launcherState == ARMED) {
          launcherState = IDLE;
        } else {
          launcherState = ARMED;
        }
        armChanged = true;  // Arm state changed
      }
      if (request.indexOf("/fire") != -1 && launcherState == ARMED) {
        //Simulate launchTriggerPin press
        if (launcherState == ARMED || launcherState == IDLE) {
          launcherState = LAUNCHING;
          launchTimer = millis();
          solenoidTimer = millis();
          buttonPressed = true;
          fired = true; // Indicate that a fire command was received
        }
      }

      response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
      response += "<html><head><title>Air Launcher Control</title>";
      response += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      response += "<style>";
      response += "body { font-family: sans-serif; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; background-color: #f0f0f0; }";
      response += ".container { text-align: center; }";
      response += ".arm { font-size: 2em; padding: 20px 40px; margin: 10px; border: none; border-radius: 10px; cursor: pointer; transition: transform 0.2s, background-color 0.3s; }";
      response += ".arm.armed { background-color: #FF9800; color: white; }";    //armed color
      response += ".arm.disarmed { background-color: #4CAF50; color: white; }"; //disarmed color
      response += ".arm:active { transform: scale(0.95); }";
      response += ".fire { font-size: 2em; padding: 20px 40px; margin: 10px; border: none; border-radius: 10px; cursor: pointer; background-color: #f44336; color: white; transition: transform 0.2s; }";
      response += ".fire:active { transform: scale(0.95); }";
      response += "</style>";
      response += "</head><body><div class=\"container\">";
      response += "<h1>Air Launcher Control</h1>";
      response += "<button class=\"arm " + String(launcherState == ARMED ? "armed" : "disarmed") + "\" id=\"armButton\">Toggle Arm</button><br>";
      response += "<button class=\"fire\" id=\"fireButton\">Fire!</button>";
      response += "</div><script>";
      response += "const armButton = document.getElementById('armButton');";
      response += "const fireButton = document.getElementById('fireButton');";
      response += "armButton.addEventListener('click', function() {";
      response += "  fetch('/arm');";
      response += "  this.classList.toggle('armed');";
      response += "  this.classList.toggle('disarmed');";
      response += "  if ('vibrate' in navigator) { navigator.vibrate(200); }";
      response += "});";
      response += "fireButton.addEventListener('click', function() {";
      response += "  fetch('/fire');";
      response += "  if ('vibrate' in navigator) { navigator.vibrate(500); }";
      response += "});";
      response += "function updateButtonStates() {";
      response += "  fetch('/state')"; // You'll need to create an endpoint to get the current state
      response += "    .then(response => response.json())";
      response += "    .then(data => {";
      response += "      if (data.launcherState === 'ARMED') {";
      response += "        armButton.classList.add('armed');";
      response += "        armButton.classList.remove('disarmed');";
      response += "      } else {";
      response += "        armButton.classList.add('disarmed');";
      response += "        armButton.classList.remove('armed');";
      response += "      }";
      response += "    });";
      response += "}";
      response += "// Periodically update the button states (e.g., every second)";
      response += "setInterval(updateButtonStates, 1000);";
      response += "</script></body></html>";

      client.print(response);
      client.stop();
      request = "";
      state = WAIT_FOR_REQUEST;
      timeout = 0;
    }
  } else {
    request = "";
    state = WAIT_FOR_REQUEST;
    timeout = 0;
  }
}
