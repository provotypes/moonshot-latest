// 1/25/2022 Weston Day
// start for color patterns     for Team 6844, Provotypes 2022 FRC robot
// derived from Adafruit example sketch, strandtest_wheel, a NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//        connect GROUND (-) first, then +, then data.
#include <Adafruit_NeoPixel.h>

// Arduino pin driving the NeoPixels data line: 6 == D6
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 30

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

uint8_t brightness = 75;

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  strip.begin();
  strip.setBrightness(brightness);
  strip.show(); // Initialize all pixels to 'off'

  Serial.begin(9600);
}

uint32_t red = strip.Color(255, 0, 0);
uint32_t orange = strip.Color(255, 127, 0);
uint32_t yellow = strip.Color(255, 255, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t off = strip.Color(0, 0, 0);
uint8_t powerLvl = 0;


void colorSolid(uint32_t c, uint8_t wait=50) {
  if (powerLvl == 1) {
    strip.setPixelColor(0, orange);
  }
  for(uint16_t i=powerLvl; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

void colorBlink(uint32_t c1, uint32_t c2, uint8_t wait=100) {
  colorSolid(c1, wait);
  colorSolid(c2, wait);
  colorSolid(c1, wait);
  colorSolid(c2, wait);
  colorSolid(c1, wait);
  colorSolid(c2, wait);
}

void colorAlternate(uint32_t c1, uint32_t c2) {
  byte m = 0;
  byte p = 0;
  for (uint16_t j=0; j<8; j++) {
    if (powerLvl == 1) {
      strip.setPixelColor(0, orange);
    }
    for (uint16_t i=powerLvl; i<strip.numPixels(); i++) {
      if (m==0) {
        if (p==0) {
          p = 1;
          strip.setPixelColor(i, c1);
        } else {
          p = 0;
          strip.setPixelColor(i, c2);
        }
      } else {
        if (p==0) {
          p = 1;
          strip.setPixelColor(i, c2);
        } else {
          p = 0;
          strip.setPixelColor(i, c1);
        }
      }
    }
    if (m == 0) {
      m = 1;
    } else {
      m = 0;
    }
    strip.show();
    delay(50);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

void randomized(uint8_t r, uint8_t g, uint8_t b, uint8_t range) {
  for (uint8_t i = 0; i <= strip.numPixels(); i++) {
    uint8_t _r = 0;
    uint8_t _g = 0;
    uint8_t _b = 0;
    //if (r != 0) {
    _r = min(max(0, random(r - range, r + range)), 255);
    //}
    //if (g != 0) {
    _g = min(max(0, random(g - range, g + range)), 255);
    //}
    //if (b != 0) {
    _b = min(max(0, random(g - range, g + range)), 255);
    //}
    strip.setPixelColor(i, strip.Color(_r, _g, _b));
    //strip.setBrightness(min(max(0, random(brightness - range, brightness + range)), 255));
  }
  strip.show();
}

uint8_t state = 0;
uint8_t mode = 1;
uint8_t analog = 1;
unsigned long duration = 0;
// solder wire to arduino port A2!

void check_digital() {

  duration = pulseIn(2, HIGH);
  Serial.println(duration);
  if (int(duration) <= 25 && int(duration) >= 15) {
    state = 1;
  } else if (int(duration) <= 50 && int(duration) >= 40) {
    state = 2;
  } else if (int(duration) <= 100 && int(duration) >= 90) {
    state = 3;
  } else if (int(duration) <= 200 && int(duration) >= 190) {
    state = 4;
  } else {
    state = 0;
  }
}

void check_analog() {
  uint16_t value = analogRead(2);

  Serial.println(value);

  uint8_t v = 26;
  if (0 <= value && value <= v) { state = 0; }
  else if ((v)+2 <= value && value <= v*2) { state = 1; }
  else if ((v*2)+2 <= value && value <= v*3) { state = 2; }
  else if ((v*3)+2 <= value && value <= v*4) { state = 3; }
  else if ((v*4)+2 <= value && value <= v*5) { state = 4; }
  else if ((v*5)+2 <= value && value <= v*6) { state = 5; }
  else if ((v*6)+2 <= value && value <= v*7) { state = 6; }
  else if ((v*7)+2 <= value && value <= v*8) { state = 7; }
  else if ((v*8)+2 <= value && value <= v*9) { state = 8; }
  else if ((v*9)+2 <= value && value <= v*10) { state = 9; }
  else if ((v*10)+2 <= value && value <= v*11) { state = 10; }
  else if ((v*11)+2 <= value && value <= v*12) { state = 11; }
  else if ((v*12)+2 <= value && value <= v*13) { state = 12; }
  else if ((v*13)+2 <= value && value <= v*14) { state = 13; }
  else if ((v*14)+2 <= value && value <= v*15) { state = 14; }
  else if ((v*15)+2 <= value && value <= v*16) { state = 15; }
}



void loop() {


  // state setting
  if (analog == 0) {
    check_digital();
  } else {
    check_analog();
  }

  switch (mode) {
    case 0: {
      // colorSolid(off, 0);
      if (state == 1) {
        strip.setPixelColor(min(int(duration), strip.numPixels() - 1), off);
      }
      duration = pulseIn(2, HIGH);
      Serial.println(duration);
      strip.setPixelColor(min(int(duration), strip.numPixels() - 1), green);
      strip.show();
      break;
    }
    case 1: {
      switch (state) {
        case 0:  { colorAlternate(green, off); break; }
        case 1:  { colorAlternate(red, off); break; }
        case 2:  { colorAlternate(blue, off); break; }
        case 3:  { colorAlternate(yellow, off); break; }
        case 4:  { colorBlink(green, off); break; }
        case 5:  { colorBlink(red, off); break; }
        case 6:  { colorBlink(blue, off); break; }
        case 7:  { colorBlink(yellow, off); break; }
        case 8:  { colorSolid(green); break; }
        case 9:  { colorSolid(red); break; }
        case 10: { colorSolid(blue); break; }
        case 11: { colorSolid(yellow); break; }
        case 12: { colorAlternate(blue, green); break; }
        case 13: { colorAlternate(blue, red); break; }
        case 14: { colorAlternate(green, red); break; }
        case 15: { randomized(-5, 250, 0, 5); }
      }
    }
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}



//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}



// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
