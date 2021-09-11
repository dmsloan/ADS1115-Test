//+--------------------------------------------------------------------------
// This is to test the ADS1115 Analog to digital converter
// 
//
// File:        ADS1115 Test
//  
// Description:
//  How to read the ADS1115 and how to read a shunt.
//  The shunt used is 100mv/amp.
//  Found if the shunt does not share a common ground with the ADS1115
//  the readings are not steady. You must ground the low side of the shunt
//  with the ground on the ADS1115
//  
// History:     March-03-2021     Derek      Created
//
//---------------------------------------------------------------------------

#include <Arduino.h>          // Arduino Framework <> searches the libraries paths
#include <Wire.h>             // TWI/I2C library for Arduino & Wiring
#include <U8g2lib.h>          // For text on the little on-chip OLED
#include <Adafruit_ADS1015.h> // For the ADS1115 analog to digital converter 

const String sketchName = "ADS1115 Test";

// For the heltec_wifi_lora_32 CLOCK 15 DATA 4 RESET 16
// For the wemos lolin32 #define CLOCK 4 DATA 5 RESET 16

// The active board is declared in platformio.ini. The define is 
// declared in pins_arduino.h.

#if defined(ARDUINO_HELTEC_WIFI_LORA_32) //# is evaluated at time of compile
  #define OLED_CLOCK SCL_OLED            // Pins for OLED display
  #define OLED_DATA SDA_OLED
  #define OLED_RESET RST_OLED
//  #define LED_PIN 23                     //Output pin for the WS2812B led strip. Dave recomends pin 5 but it is being used by LoRa on my board
#elif defined(WIFI_LoRa_32_V2) //# is evaluated at time of compile
  #define OLED_CLOCK SCL_OLED            // Pins for OLED display
  #define OLED_DATA SDA_OLED
  #define OLED_RESET RST_OLED
//  #define LED_PIN 23                     //Output pin for the WS2812B led strip. Dave recomends pin 5 but it is being used by LoRa on my board
#elif defined(ARDUINO_LOLIN32)
  #define OLED_CLOCK 4              // Pins for OLED display
  #define OLED_DATA 5
  #define OLED_RESET 16
//  #define LED_PIN 5 //Output pin for the WS2812B led strip.
#else
  // #define OLED_CLOCK 15              // Pins for OLED display
  // #define OLED_DATA 4
  // #define OLED_RESET 16
  // #define LED_PIN 5 //Output pin for the WS2812B led strip.
#endif

//clock and data got swapped around to use hardware I2C instead of software
U8G2_SSD1306_128X64_NONAME_F_SW_I2C g_oled(U8G2_R2, OLED_CLOCK, OLED_DATA, OLED_RESET); // uses Software I2C and results in a framerate of 5 FPS
// The following line that useds I2C hardware does not appear to work with other devices on the I2C
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C g_oled(U8G2_R2, OLED_RESET, OLED_CLOCK, OLED_DATA); // uses Hardware I2C and results in a framerate of 26 FPS 
int g_linehight = 0;

//Adafruit_ADS1015 ads1015;  	// Construct an ads1015 at the default address: 0x48
Adafruit_ADS1115 ads1115(0x48);	// construct an ads1115 at address 0x48

const float scaleValue = 0.0078125; //0.0078125mV per division
const int ampsPerMv = 1;
int led = LED_BUILTIN;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;     // will store last time OLED was updated
const long interval = 1000;           // interval at which to update the OLED and send info to serial

// Tracks a weighted average in order to smooth out the values that it is given. 
// as the simple reciprocal of the amount of taken specified by the caller.
// it takes about ten readings to sabilize.
double weightedAverage (double amps){
  static double weightedAverage;
  weightedAverage = (weightedAverage * 0.9) + (amps * 0.1); // the two multipliers must equal one
  return weightedAverage;
}

void setup(void)
{
  pinMode(led, OUTPUT);

  g_oled.begin();
  g_oled.clear(); //sets curser at 0,0. Text draws from the bottom up so you will see nothing.
  g_oled.setFont(u8g2_font_profont15_tf);
  g_linehight = g_oled.getFontAscent() - g_oled.getFontDescent(); // Decent is a negative number so we add it to the total
  g_oled.drawRFrame(0,0,g_oled.getWidth(),g_oled.getHeight(),7);  // Draw a boarder around the display
  g_oled.setCursor(3,g_linehight * 6 + 2);
  g_oled.sendBuffer();
  g_oled.setFont(u8g2_font_inb19_mn);

  ads1115.begin();  // Initialize ads1115
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
   ads1115.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads1115.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads1115.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads1115.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads1115.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads1115.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  Serial.begin(115200);
  while (!Serial){};
  Serial.println(sketchName);
  
  Serial.println("Getting single-ended readings from AIN0, 1");
  Serial.println("Getting differential reading from AIN0_1");
  Serial.println("+/- 0.256V  1 bit = 0.0078125mV");
}

void loop(void)
{
  int16_t adc0, adc1, adc2, adc3, diff_0_1;
  float diffMv_0_1, amps, ampsWt;
   unsigned long currentMillis = millis();

  adc0 = ads1115.readADC_SingleEnded(0);
  adc1 = ads1115.readADC_SingleEnded(1);
  adc2 = ads1115.readADC_SingleEnded(2);
  adc3 = ads1115.readADC_SingleEnded(3);
  diff_0_1 = ads1115.readADC_Differential_0_1();

  diffMv_0_1 = ((float)diff_0_1 * 256.0) / 32768.0;//100mv shunt do the math here instead of having to use decimal numbers, not sure if it matters
  amps = diffMv_0_1;
  ampsWt = weightedAverage(amps); //apply weighted average to the scaled value

  // diffMv_0_1 =  (float)diff_0_1 * scaleValue;  //convert value read to mv
  // amps = diffMv_0_1 / (float)ampsPerMv;  //convet mv to amps, scale the reading
  // ampsWt = amperage(amps); //apply weighted average to the scaled value
	if (currentMillis - previousMillis >= interval) {
		previousMillis = currentMillis;
		Serial.print("AIN0: "); Serial.println(adc0);
		Serial.print("AIN1: "); Serial.println(adc1);
		Serial.print("AIN2: "); Serial.println(adc2);
		Serial.print("AIN3: "); Serial.println(adc3);
	//  Serial.print("Diff_0_1: "); Serial.println(diff_0_1, 15);
		Serial.print("diff_0_1: "); Serial.print(diff_0_1); Serial.println();
		Serial.print("diffMv_0_1: "); Serial.printf("%.3lf", diffMv_0_1); Serial.println();
		Serial.print("ampsWt: "); Serial.printf("%10.3lf", ampsWt); Serial.println();
		Serial.print("The gain is: "); Serial.println(ads1115.getGain());
		Serial.print("scaleValue is: "); Serial.println(scaleValue, 7);
		Serial.print("ampsPerMv is: "); Serial.println(ampsPerMv);
		Serial.println();
		
		//g_oled.print(aShunt, 4 );
		g_oled.setCursor(18,g_linehight * 2 + 2);
		g_oled.printf("%05.1lf", amps);       // send the amps to the OLED
		//g_oled.printf("+%05d", diff_0_1);
		g_oled.setCursor(20,g_linehight * 4 + 2);
		g_oled.printf("%05.1lf", ampsWt);     // send the amps weighted average to the OLED
		g_oled.sendBuffer();                  // Print it out to the OLED

		digitalWrite(led, !digitalRead(led));
	//	delay(100);
	}
delay(10);
}