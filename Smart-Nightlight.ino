#include <Adafruit_NeoPixel.h>

// ===== PIN SETUP =====
#define LED_PIN       5      // DIN of 8x8 ARGB LED board
#define NUM_LEDS      64     // 8 x 8

#define PIR_PIN       6      // SR501 PIR sensor
#define LDR_DIGITAL   7      // digital photoresistor output
#define LDR_ANALOG    A3     // not used now

// ===== LED STRIP OBJECT =====
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== STATE =====
int fingerCount = 0;   // 1-5 from Pi

// ===== HELPERS =====
void fillAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void clearAll() {
  fillAll(0, 0, 0);
}

// Read serial from Pi: digits '1'..'5'
void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= '1' && c <= '5') {
      fingerCount = c - '0';
    }
  }
}

// Map finger count to a color
void applyFingerColor() {
  uint8_t r = 0, g = 0, b = 0;

  switch (fingerCount) {
    case 1: // blue
      r = 0;
      g = 0;
      b = 255;
      break;
    case 2: // white
      r = 255;
      g = 255;
      b = 255;
      break;
    case 3: // purple
      r = 128;
      g = 0;
      b = 128;
      break;
    case 4: // green
      r = 0;
      g = 255;
      b = 0;
      break;
    case 5: // yellow
      r = 255;
      g = 255;
      b = 0;
      break;
    default:
      r = g = b = 0;
      break;
  }

  fillAll(r, g, b);
}

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(LDR_DIGITAL, INPUT);

  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(40);
  clearAll();
}

void loop() {
  // 1) Update finger count from Pi
  readSerialCommand();

  // 2) Read sensors
  int pirState   = digitalRead(PIR_PIN);      // HIGH = human present
  int ldrDigital = digitalRead(LDR_DIGITAL);  // depends on wiring

  // 3) Brightness logic (FIXED):
  //    Light detected (flashlight ON sensor)  -> brightness = 5
  //    No flashlight / normal room           -> brightness = 40
  //
  // In your wiring: flashlight = LOW, normal = HIGH
  uint8_t brightness;
  if (ldrDigital == LOW) {
    // flashlight on sensor
    brightness = 5;
  } else {
    // no flashlight
    brightness = 40;
  }

  // 4) PIR logic:
  if (pirState == LOW) {
    // no human -> LEDs off
    strip.setBrightness(0);
    clearAll();
  } else {
    // human present -> last finger color with chosen brightness
    strip.setBrightness(brightness);
    applyFingerColor();
  }

  delay(30);
}
