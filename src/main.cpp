#define DEBUG true

// Define which LED library to use in the code
#define LED_LIB_FASTLED 0x01
#define LED_LIB_ADAFRUITNEOPIXEL 0x02
#define LED_LIB LED_LIB_FASTLED

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h>

#include <RotaryEncoder.h>
#include <Servo.h>

#if LED_LIB == LED_LIB_FASTLED
  #define FASTLED_ALLOW_INTERRUPTS 0

  #include <FastLED.h>
#elif LED_LIB == LED_LIB_ADAFRUITNEOPIXEL
  #include <Adafruit_NeoPixel.h>
#else
  #error "No valid LED library selected!"
#endif

#define PIN_IN1 D5
#define PIN_IN2 D6
#define PIN_SERVO D7
#define PIN_BUTTON D4
#define PIN_LED D3

Servo myservo;

RotaryEncoder *encoder = nullptr;

int rotaryPos = 0;
int servoPos = 0;

// Button debounce delay in milliseconds
#define DEBOUNCE_DELAY 50
#define ROTARY_DEBOUNCE_DELAY 1000

int lastButtonState = LOW;
int lastDebounceTime = 0;

bool rotaryStore = false;
int rotaryStoreDebounceTime = 0;

// Do some color or brighness change
bool doColorChange = true;

// Servo open/closed values for min/max rotation
// TODO: check this values
// #define SERVO_OPEN 530 // min position = 0°
#define SERVO_OPEN 2080
#define SERVO_CLOSED 2530 // max position = 180°

// Petal animation timing variables
int frameDuration = 3000;         // Number of milliseconds for complete movement
int frameElapsed = 0;             // How much of the frame has gone by, will range from 0 to frameDuration
unsigned long previousMillis = 0; // The last time we ran the position interpolation
unsigned long currentMillis = 0;  // Current time, we will update this continuosly
int interval = 0;

// Üetal animation status variables
int movementDirection = 0; // 0 = stopped, 1 opening, -1 closing

unsigned long targetTimer = 0;

// LED animation status variables
#define BRIGHTNESS_START 0
#define BRIGHTNESS_END 255

#define NUM_LEDS 32

#if LED_LIB == LED_LIB_FASTLED
  #define CHIPSET NEOPIXEL
  #define COLOR_ORDER RGB // Not required for NeoPixel

  // #define CHIPSET WS2812
  // #define COLOR_ORDER GRB

  CRGB leds[NUM_LEDS];
#elif LED_LIB == LED_LIB_ADAFRUITNEOPIXEL
  // Parameter 1 = number of pixels in strip
  // Parameter 2 = Arduino pin number (most are valid)
  // Parameter 3 = pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
#endif

#define FRAMES_PER_SECOND 60

// Define hostname and OTA settings
#define HOSTNAME "ESP-NightLight"
#define OTA_PORT 8266
#define OTA_PASSWORD "<PWD>"

// Prepare a struct to store some settings into the EEPROM
struct {
  int servoPosition = SERVO_CLOSED;
  float_t brightness = BRIGHTNESS_START;
  byte wheelPosition = 0;
  bool flowerGoalState = false;
  uint8_t brightnessMax = 50;
  uint16_t timer = 0;
} settings;

#define SETTINGS_ADRESS 0

// MQTT settings
// TODO: make this configurable via WiFiManager?
const char *mqtt_server = "<SERVER>";
const uint16_t mqtt_port = 1883;
const char *device_id = "esp8266-nightlamp";
const char *mqtt_user = "<USER>";
const char *mqtt_pass = "<PWD>";
// MQTT topics
const char *mqtt_topic_brightness = "esp/nightlamp/brightness";
const char *mqtt_topic_timer = "esp/nightlamp/timer";
const char *mqtt_topic_color = "esp/nightlamp/color";
const char *mqtt_topic_toggle = "esp/nightlamp/toggle";

char message_buff[100];
long lastReconnectAttempt = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);


/**
 * The interrupt service routine will be called on any change of one of the input signals.
 */
IRAM_ATTR void checkPosition()
{
  // just call tick() to check the state.
  encoder->tick();
}

/**
 * Store current settings into the EEPROM
 */
void storeSettings() {
  EEPROM.put(SETTINGS_ADRESS, settings);
  EEPROM.commit();
}

/**
 * Input a value 0 to 255 to get a color value.
 * The colours are a transiti
 * 
 * on r - g - b - back to r.
 */
#if LED_LIB == LED_LIB_FASTLED
CRGB Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;

  if (WheelPos < 85)
  {
    return CRGB((255 - WheelPos * 3), 0, (WheelPos * 3));
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return CRGB(0, (WheelPos * 3), (255 - WheelPos * 3));
  }
  else
  {
    WheelPos -= 170;
    return CRGB((WheelPos * 3), (255 - WheelPos * 3), 0);
  }
}
#elif LED_LIB == LED_LIB_ADAFRUITNEOPIXEL
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;

  if (WheelPos < 85)
  {
    return strip.Color((255 - WheelPos * 3), 0, (WheelPos * 3));
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(0, (WheelPos * 3), (255 - WheelPos * 3));
  }
  else
  {
    WheelPos -= 170;
    return strip.Color((WheelPos * 3), (255 - WheelPos * 3), 0);
  }
}
#endif

/**
 * Apply colours from the wheel to the pixels
 */
void setWheel(byte WheelPos, float brightness)
{
  // Convert brightness [0,1.0] to [0,255] for the LED strip
  settings.brightness = BRIGHTNESS_START + (settings.brightnessMax - BRIGHTNESS_START) * brightness;

#if LED_LIB == LED_LIB_FASTLED
  FastLED.setBrightness(settings.brightness);

  CRGB color = Wheel(WheelPos);
  CRGB color2 = Wheel(128 - WheelPos);

  for (uint16_t i = 0; i < NUM_LEDS / 2; i++)
  {
    leds[i] = color;
  }
  for (uint16_t i = NUM_LEDS / 2; i < NUM_LEDS; i++)
  {
    leds[i] = color2;
  }
  // for (uint16_t i = 0; i < NUM_LEDS; i++)
  // {
  //   leds[i] = color;
  // }

  FastLED.show();
  delayMicroseconds(100);
#elif LED_LIB == LED_LIB_ADAFRUITNEOPIXEL
  strip.setBrightness(settings.brightness);

  uint32_t color = Wheel(WheelPos);

  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    strip.setPixelColor(i, color);
  }

  strip.show();
#endif
}

void prepareTargetTimer() {
#ifdef DEBUG
  Serial.println("--- prepareTargetTimer ---");
  Serial.print("settings.timer: "); Serial.println(settings.timer);
  Serial.print("settings.flowerGoalState: "); Serial.println(settings.flowerGoalState);
#endif

  if (settings.timer > 0 && settings.flowerGoalState) {
    targetTimer = millis() + settings.timer * 1000;
  }
}

/**
 * Update the flower: open/close it and adjust the LEDs (dim up/down)
 */
void updateFlower()
{
  unsigned long currentMillis = millis();
  interval = currentMillis - previousMillis;
  previousMillis = currentMillis;

  frameElapsed += movementDirection * interval;
  float frameElapsedRatio = float(frameElapsed) / float(frameDuration);
  volatile float brightness = frameElapsedRatio;

  if (brightness <= 0) {
    brightness = 0;
  }

  if (frameElapsed < 0)
  {
    movementDirection = 0;
    frameElapsed = 0;

#ifdef DEBUG
    Serial.println("closed");
#endif

    brightness = 0.0;
    settings.brightness = BRIGHTNESS_START;
    settings.servoPosition = SERVO_OPEN;
    doColorChange = false;

    // Unset target timer
    targetTimer = 0;

    storeSettings();
  }
  if (frameElapsed > frameDuration)
  {
    movementDirection = 0;
    frameElapsed = frameDuration;

#ifdef DEBUG
    Serial.println("opened");
#endif

    brightness = 1.0;
    settings.brightness = settings.brightnessMax;
    settings.servoPosition = SERVO_CLOSED;
    doColorChange = false;

    prepareTargetTimer();

    storeSettings();
  }

  if (movementDirection != 0)
  {
    // Determine new position/brightness by interpolation between endpoints
    // int newServoMicros = (SERVO_CLOSED + int(frameElapsedRatio * (SERVO_OPEN - SERVO_CLOSED)));
    int newServoMicros = (SERVO_OPEN + int(frameElapsedRatio * (SERVO_CLOSED - SERVO_OPEN)));

    myservo.write(newServoMicros);
  }

  // Trigger LED color/brightness change only if the color or brightness has been changed.
  // This should reduce flickering further.
  if (doColorChange) {
    setWheel(settings.wheelPosition, brightness);
  }
}

/**
 * MQTT callback handler on incomming publish
 */
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG
  Serial.print("\nMessage arrived [");
  Serial.print(topic);
  Serial.println("] ");

  Serial.print("length: ");
  Serial.println(length);
#endif

  unsigned int i;
  for (i = 0; i < length; i++)
  {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';

  String msgString = String(message_buff);
#ifdef DEBUG
  Serial.println("payload: \"" + msgString + "\"");
#endif

  if (strcmp(topic, mqtt_topic_brightness) == 0)
  { 
    uint8_t convertedNumber = msgString.toInt();

    if (convertedNumber > 0) {
      settings.brightnessMax = convertedNumber;

      setWheel(settings.wheelPosition, settings.brightness);
      storeSettings();
    }
  }
  else if (strcmp(topic, mqtt_topic_timer) == 0)
  { 
    settings.timer = msgString.toInt();

    prepareTargetTimer();
    storeSettings();
  }
  else if (strcmp(topic, mqtt_topic_color) == 0)
  { 
    settings.wheelPosition = msgString.toInt();

    setWheel(settings.wheelPosition, settings.brightness);
    storeSettings();
  }
  else if (strcmp(topic, mqtt_topic_toggle) == 0)
  { 
    settings.flowerGoalState = !settings.flowerGoalState;

    if (settings.flowerGoalState)
    {
      movementDirection = 1;
      doColorChange = true;
    }
    else
    {
      movementDirection = -1;
      doColorChange = true;
    }

    storeSettings();
  }
}

/**
 * MQTT reconnect code (non blocking)
 */
bool reconnect()
{
  if (mqttClient.connect(device_id, mqtt_user, mqtt_pass))
  {
    mqttClient.subscribe(mqtt_topic_brightness);
    mqttClient.subscribe(mqtt_topic_timer);
    mqttClient.subscribe(mqtt_topic_color);
    mqttClient.subscribe(mqtt_topic_toggle);
  }
  
  return mqttClient.connected();
}

/*** SETUP ***/

void setupLed() {
#if LED_LIB == LED_LIB_FASTLED
#if (CHIPSET == NEOPIXEL)
  FastLED.addLeds<CHIPSET, PIN_LED>(leds, NUM_LEDS);
#else
  FastLED.addLeds<CHIPSET, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS);
#endif
  //.setCorrection(TypicalLEDStrip);

  // FastLED.setMaxRefreshRate(60);

  FastLED.setBrightness(settings.brightness);

  // CRGB color = Wheel(settings.wheelPosition);
  CRGB color = CRGB::Black;

  // Init LEDs with black
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = color;
  }

  FastLED.show();
  // delayMicroseconds(100);
#elif LED_LIB == LED_LIB_ADAFRUITNEOPIXEL
  strip.begin();
  strip.clear();
  strip.setBrightness(settings.brightness);
  strip.show();
#endif
}

void setupWifi() {
  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Reset saved settings
  // wifiManager.resetSettings();

  // Sets timeout until configuration portal gets turned off. Useful to make it all retry or go to sleep in seconds.
  wifiManager.setTimeout(180);

#ifdef DEBUG
  wifiManager.setDebugOutput(true);
#else
  wifiManager.setDebugOutput(false);
#endif

  // Set WiFi DNS hostname
  WiFi.setHostname(HOSTNAME);

  // Fetches ssid and pass from eeprom and tries to connect. If it does not connect it starts an access point with the specified name and goes into a blocking loop awaiting configuration.
  if (!wifiManager.autoConnect(HOSTNAME))
  {
#ifdef DEBUG
    Serial.println("failed to connect and hit timeout");
#endif
    delay(3000);
    // Reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

#ifdef DEBUG
  Serial.println("WiFi connected!");
#endif
}

void setupOta() {
  // Port defaults to 8266
  ArduinoOTA.setPort(OTA_PORT);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTA_PASSWORD);

#ifdef DEBUG
  ArduinoOTA.onStart([]()
                     { Serial.println("Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         Serial.println("Auth Failed");
                       else if (error == OTA_BEGIN_ERROR)
                         Serial.println("Begin Failed");
                       else if (error == OTA_CONNECT_ERROR)
                         Serial.println("Connect Failed");
                       else if (error == OTA_RECEIVE_ERROR)
                         Serial.println("Receive Failed");
                       else if (error == OTA_END_ERROR)
                         Serial.println("End Failed");
                     });
#endif
  ArduinoOTA.begin();
}

void setupMqtt() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
}

void setupRotaryEncoder() {
  // Setup the rotary encoder functionality
  encoder = new RotaryEncoder(
    PIN_IN1,
    PIN_IN2,
    // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
    // RotaryEncoder::LatchMode::FOUR3
    // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
    // RotaryEncoder::LatchMode::FOUR0
    // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
    RotaryEncoder::LatchMode::TWO03
  );

  // Register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  // Setting up rotary encoder push button
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

void setupServo() {
  // Setup the servo
  myservo.attach(PIN_SERVO, SERVO_OPEN, SERVO_CLOSED, settings.servoPosition);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  /*** EEPROM ***/
  EEPROM.begin(sizeof(settings));
  EEPROM.get(0, settings);

  if (settings.flowerGoalState) {
    frameElapsed = frameDuration;
  } else {
    frameElapsed = 0;
  }

  // NOTICE: Weird hack! The EEPROM might store non-boolean values (non equal to "0" and "1")
  settings.flowerGoalState = (settings.flowerGoalState ? 1 : 0);
  settings.brightnessMax = (settings.brightnessMax < 0 || settings.brightnessMax > 255 ? BRIGHTNESS_END : settings.brightnessMax);

  prepareTargetTimer();

#ifdef DEBUG
  Serial.println("--- settings ---");
  Serial.print("servo position: "); Serial.println(settings.servoPosition);
  Serial.print("brightness: "); Serial.println(settings.brightness);
  Serial.print("wheel position: "); Serial.println(settings.wheelPosition);
  Serial.print("flower goal state: "); Serial.println(settings.flowerGoalState);
  Serial.print("brightness max: "); Serial.println(settings.brightnessMax);
  Serial.print("timer: "); Serial.println(settings.timer);
#endif

  /*** LED strip ***/
  setupLed();

  /*** WiFi manager ***/
  setupWifi();

  /*** Prepare MQTT ***/
  setupMqtt();
  
  /*** OTA ***/
  setupOta();

  /*** Rotary encoder ***/
  setupRotaryEncoder();
  
  /*** Servo ***/
  setupServo();
}

/*** LOOP ***/

void loop()
{
  if (!mqttClient.connected()) {
    long now = millis();

    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;

      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    mqttClient.loop();
  }

  int newPos = encoder->getPosition();
  if (rotaryPos != newPos)
  {
    int direction = (int) encoder->getDirection();

#ifdef DEBUG
    Serial.println("--- rotary encoder ---");
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println(direction);
#endif
    rotaryPos = newPos;
    settings.wheelPosition += direction;

    rotaryStore = true;
    rotaryStoreDebounceTime = millis();

    doColorChange = true;
  }

  // Read button state and debounce
  int buttonState = digitalRead(PIN_BUTTON);

  if (buttonState != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    if (buttonState != lastButtonState)
    {
      lastButtonState = buttonState;
    }

    if (buttonState == LOW)
    {
      if (movementDirection == 0)
      {
        settings.flowerGoalState = !settings.flowerGoalState;
      }

      if (settings.flowerGoalState)
      {
        movementDirection = 1;
        doColorChange = true;
      }
      else
      {
        movementDirection = -1;
        doColorChange = true;
      }

#ifdef DEBUG
      Serial.print("movement direction:");
      Serial.println(movementDirection);
#endif
    }

#ifdef DEBUG
    Serial.println("Push button pushed");
#endif
  }

  // Check if timer is enabled at all (> 0)
  if (settings.timer > 0) {

    // If yes, the nightlight should stop when the timer is over
    if (targetTimer > 0 && millis() > targetTimer) {
#ifdef DEBUG
      Serial.println("--- target timer reached ---");
      Serial.print("targetTimer: "); Serial.println(targetTimer);
      Serial.print("settings.timer * 1000: "); Serial.println((int)(settings.timer * 1000));
      Serial.print("millis() + settings.timer * 1000: "); Serial.println((int)(millis() + settings.timer * 1000));
#endif
      settings.flowerGoalState = 0;
      movementDirection = -1;
      doColorChange = true;

      // Unset target timer
      targetTimer = 0;
    }
  }

  // Update flower color and brightness
  updateFlower();

  // Store updated rotary settings
  if (rotaryStore && (millis() - rotaryStoreDebounceTime) > ROTARY_DEBOUNCE_DELAY) {
    rotaryStore = false;
    doColorChange = false;

#ifdef DEBUG
    Serial.println("store wheel position");
#endif

    storeSettings();
  }
}
