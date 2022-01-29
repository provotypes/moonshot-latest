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



void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
}

uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t off = strip.Color(0, 0, 0);
uint32_t yellow = strip.Color(255, 255, 0);



void colorSolid(uint32_t c, uint8_t wait=50) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
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

void colorAlternate(uint32_t c1, uint32_t c2, uint8_t wait=100) {
  byte m = 0;
  byte p = 0;
  for (uint16_t j=0; j<2; j++) {
    for (uint16_t i=0; i<strip.numPixels(); i++) {
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


uint8_t state = 9;

void loop() {

  if (state == 0) {
    colorAlternate(green, off);
  } else if (state == 1) {
    colorBlink(red, off);
  } else if (state == 2) {
    colorBlink(green, off);
  } else if (state == 3) {
    colorBlink(blue, off);
  } else if (state == 4) {
    colorBlink(yellow, off);
  } else if (state == 5) {
    colorSolid(red);
  } else if (state == 6) {
    colorSolid(green);
  } else if (state == 7) {
    colorSolid(blue);
  } else if (state == 8) {
    colorSolid(yellow);
  } else if (state == 9) {
    rainbow(50);
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
