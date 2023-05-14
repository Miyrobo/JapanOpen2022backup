#include "Arduino.h"
#include "mysetup.h"
#include "math.h"

#include <Wire.h>
#include <EEPROM.h>

//ディスプレイ
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Pixy2.h>

//mp3
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "IO-Kit.h"

#include "FastLED.h"                  //NeoPixel
#define NUM_LEDS 16
#define neopixel_pin 23
#define BRIGHTNESS  10               //明るさ (1~255)
CRGB leds[NUM_LEDS];

#define timer_MAX 50