//Arduino FFD LCD Code
//Rev a1.5 3/21/14

//supports Adafruit 2.8" LCD touch screen and Arduino Analog Shield.
//Provides a selectable spectrum up to 31.5kHz, crude amplitude and
//peak frequency measurements.

//required libraries for LCD display
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

//libraries for touch screen interface
#include <Wire.h>      // this is needed even though we aren't using it directly
#include <Adafruit_STMPE610.h>


#define ILI9341_GRAY   0xCCCC //add grey to predefined LCD colors as we want pretty gridlines.


// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000


#define offset 64 //offset constant Touch for button vertical positions.
#define textLeft 258 //set left border of controls area


// The STMPE610 uses hardware SPI on the shield, and #8
#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

//LCD Control Pins. For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC); // Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//required library for analog input
#include "analogShield.h"

//switches for FHT (FFT) library options.
#define LOG_OUT 1 // use the log output function
#define LIN_OUT8 1 // use the 8 bit linear output output function
#define FHT_N 256 // set to 256 point fht
#define TOT_BINS 128

#include <FHT.h> // include the library, must happen after the #defines.

//program constants.
boolean linLog = false; //Switch for logarithmic / linear display mode. false is log magnitude
int span = 30; //nominal span in kHz of the current display.
int spanDelay = 0; //timer constant to set sample rate.
float spanScale = 31.5; //actual nyquist rate of samples
int max_ampl = 0; //storage variable for the amplitude of the input signal.
boolean max_is_neg = false; //flag to determine if the input signal peaks positive or negative. Not displayed.
unsigned int updateCount = 0; //counter so we can do certain things (like calculate voltage) every few display updates. It rolls over a lot.
boolean gridOn = true; //flag to set the grid on or off.
boolean first_flag = true; //flag to indicate if we have to do a full (slow) screen redraw instead of a delta (fast) screen update.

const int w = tft.width(), h = tft.height(); //the height and width for the LCD are global variables.

void setup() {
  Serial.begin(9600);
  //start the LCD and set it up.
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  //draw the controls.
  userText();
  
  if (!ts.begin()) { //initialize the touchscreen.
    Serial.println("Couldn't start touchscreen controller");
    while (1);
  }
  Serial.println("Touchscreen started");
  
  //disable some interrupts so that we have less jitter.
  TIMSK0 = 0; // turn off timer0 for lower jitter
}

void loop() {
  while(1) { //not leaving the loop function reduces jitter
    cli();  // UDRE interrupt slows this way down on arduino1.0, so we stop all interrupts.
    if(span == 30) //fastest frequency range, no sample delay
    {
      for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
        fht_input[i] = analog.signedRead(0); // put real data into bins
      }
    }
    else //any other frequency span.
    {
      for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
        fht_input[i] = analog.signedRead(0); // put real data into bins
        delayMicroseconds(spanDelay); //lower the sample rate by waiting.
      }      
    }
    
    if(updateCount % 20 == 0) //calculate the voltage, but do so only every 20th update, for readability.
    {
      //find max amplitude.
      max_ampl = 0;
      for (int i = 2 ; i < FHT_N ; i++) 
      {
          if (abs(fht_input[i]) > max_ampl) //check for a new peak.
          {
            max_ampl = abs(fht_input[i]);
            if(fht_input[i] < 0) //keep the sign too.
            {
              max_is_neg = true;
            }
          }
      }
    }    
    
    //now, the fourier functions!
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht  
    fht_run(); // process the data in the fht
    
    if(linLog) //decide if output is going to be linear or logarithmic.
    {
      fht_mag_lin8(); // take the output of the fht
    }
    else
    {
      fht_mag_log(); // take the output of the fht
    }
    sei(); //start interrupts back up and build our display
    
    SPI.setDataMode(SPI_MODE0); //correct SPI Mode for the LCD

    if(updateCount % 20 == 0) //if the voltage has been recalculated, update the display
    {
      updateVoltage();
    }
    
    if(linLog) //draw the display in the right mode
    {
      drawBins(fht_lin_out8);  //draw the screen.
    }
    else
    {
      drawBins(fht_log_out);  //draw the screen.
    }
    
    touchParse(); //check for touch inputs and handle them.

    
    updateCount++;  //we did an update
    
    SPI.setDataMode(SPI_MODE3); //return to correct SPI mode for analog.read();
  }
}

void updateVoltage() //updates the peak voltage display on the LCD
{
    float ampl = max_ampl * 5.0 / 37000.0; //scale to +/-5V. Not 32768 because the Analog shield and filter have a small gain. The constant can be tuned.
//    if(max_is_neg) //add sign
//    {
//      ampl = -ampl;
//    }
    
    tft.fillRect(textLeft, offset + 74, 64, 10, ILI9341_BLUE);
    tft.setCursor(textLeft, offset + 76);
    tft.print(ampl);
    tft.print(" V");  
}

boolean touchDown = false; //flag to see if there was a touch event ongoing last time touchParse() was executed.
void touchParse() //parse touch input
{
  SPI.setClockDivider(SPI_CLOCK_DIV16); //set up SPI for touch screen
  SPI.setDataMode(SPI_MODE0);

  if (ts.bufferEmpty()) {  //no ongoing touch. Clear buffer and return.
    touchDown = false; //indicate that there was no ongoing touch this event.
    SPI.setClockDivider(SPI_CLOCK_DIV2); 
    return;
  }
  if(!touchDown) //if touch is ongoing, but touch wasn't ongoing last time. Stops press and hold.
  {
    // Retrieve a point twice. Because the buffer is unreliable.
    TS_Point p = ts.getPoint();
    while(!ts.bufferEmpty())
    {
      p = ts.getPoint();      
    }

    
    // Scale from ~0->4000 to tft.width using the calibration #'s
    p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
    p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
    touchDown = true;
    buttonPress(p); //button press parser.

  }
  while(!ts.bufferEmpty()) //again, clear that pesky buffer.
  {
    TS_Point p = ts.getPoint();      
  }
  SPI.setClockDivider(SPI_CLOCK_DIV2); //fix the SPI clock for the other peripherals.
}

void buttonPress(TS_Point p) //checks if button press is in the valid range.
{
  int span_color = ILI9341_BLACK;
  if((p.y > 180) && (p.x < (285  - offset)) && (p.x > (230 - offset))) //check if the touch event was inside the Span button's perimeter
  {
      switch (span) { //set up next span scale
        case 30:
          span = 1; //nominal kHz
          spanDelay = 450; //timer constant
          spanScale = 1.06; //actual cutoff for accurate frequency calibration
          span_color = ILI9341_RED; //change the outline color of the buttons, because I like colors.
          break;
        case 20:
          span = 30;
          spanDelay = 0;
          spanScale = 31.5;
          span_color = ILI9341_WHITE;
          break;
        case 10:
          span = 20;
          spanDelay = 9;
          spanScale = 20.29;
          span_color = ILI9341_GREEN;
          break;
        case 5:
          span = 10;
          spanDelay = 33;
          spanScale = 10.27;
          span_color = ILI9341_YELLOW;
          break;
        case 2:
          span = 5;
          spanDelay = 82;
          spanScale = 5.12;
          span_color = ILI9341_CYAN;
          break;
        default:
          span = 2;
          spanDelay = 210;
          spanScale = 2.21;
          span_color = ILI9341_MAGENTA;
          break;
      }
           
      //redraw the span button with the new label.
      tft.fillRect(textLeft - 2 , 32 + offset, 64, 30, ILI9341_BLUE);
      tft.drawRect(textLeft - 2 , 32 + offset, 64, 30, span_color);
      tft.setCursor(textLeft, 42 + offset);
      tft.print("Span ");
      tft.print(span);
      tft.print("k");
      tft.fillRect(0, 0, textLeft-2, 10, ILI9341_BLACK);
      first_flag = true; //we meed to update the whole display, since the span has changed.
      
  }
  if((p.y > 180) && (p.x < (215 - offset)) && (p.x > (145 - offset))) //check if the touch event was inside the Lin / Log button's perimeter
  {
    first_flag = true;

    tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(1);
    userText();
    
    linLog = !linLog;
    //redraw the button with the new label.
    span_color = ILI9341_WHITE;
    if(linLog)
    {
      span_color = ILI9341_GREEN;
    }
    //Draw Lin / Log Button itself
    tft.fillRect(textLeft - 2 , 84 + offset, 64, 30, ILI9341_BLUE);
    tft.drawRect(textLeft - 2 , 84 + offset, 64, 30, span_color);
    tft.setCursor(textLeft, 90 + offset);
    if(linLog) //print the appropriate mode
    {
      tft.print("Lin");      
    }
    else
    {
      tft.print("Log");
    }
    tft.setCursor(textLeft, 100 + offset);
    tft.print("Magnitude");  

  
  }
  if(p.y < 180) //check if the user tapped the main display, and if so, toggle the grid.
  {
    first_flag = true; //we have to redraw the bins completely..
    gridOn = !gridOn; //switch grid mode.
    if(!gridOn) //if turning the grid off, we have to fully blank the screen.
    {
      tft.fillRect(textLeft, 0, 20, 10, ILI9341_BLUE);
      tft.fillRect(0, 0, textLeft-2, h, ILI9341_BLACK);
    }
    
  }
}

void userText() //writes labels so they don't have to be written repeatedly by the draw function.
{
  //draw control box
  tft.fillRect(textLeft - 2 , 0, 64, h, ILI9341_BLUE);
  //draw frequency info
  tft.setCursor(textLeft, offset);
  tft.setTextColor(ILI9341_WHITE);  
  tft.setTextSize(1);
  tft.print("Largest");
  tft.setCursor(textLeft, offset + 10);
  tft.print("Component: ");

  //frequency span button
  tft.drawRect(textLeft - 2 , offset + 32, 64, 30, ILI9341_WHITE);
  tft.setCursor(textLeft, offset + 42);
  tft.print("Span ");
  tft.print(span);
  tft.print(" k");
  
  //voltage indicator
  tft.setCursor(textLeft, offset + 66);
  tft.print("Peak Ampl.");
  
  //Lin / Log Button
  tft.drawRect(textLeft - 2 , offset + 84, 64, 30, ILI9341_WHITE);
  tft.setCursor(textLeft, offset + 90);
  tft.println("Log");
  tft.setCursor(textLeft, offset + 100);
  tft.print("Magnitude");
  //draw frequency indexes.
  if(gridOn)
  {
    drawGridFreq();
  }
}

byte last_fft[FHT_N/2]; //buffer to store the last frame of data for sample updates
int oldMaxBinIndex = 0; //counter to indicate which is our peak bin for later display
void drawBins( byte *bins ) //draw the actual FFT.
{
  int maxBin = 0; //figure our the max bin.
  int maxBinIndex = 0;
  if(linLog) //a scale for linear mode so it looks like there's some signal.
  {
    for(int y = 0; y < TOT_BINS; y++)
    {
      bins[y] <<=2;
      bins[y] += 2;
    }
  }

  //now draw lines with the bins.
  if(first_flag) //draw a full screen update if the flag is set.
  {
    if(gridOn)
    {
      for(int i = 0; i < h; i++) //draw the horizontal gridlines as needed.
      {
        if(i%31 == 0)
        {
          tft.drawFastHLine(0, i, 256, ILI9341_GRAY);
        }
      }
 
    }
    
    for(int y = 0; y < TOT_BINS; y++) //full FFT output here, as vertical lines.
    {
      int top = w - constrain(bins[y], 0, 240);
      last_fft[y] = bins[y];
      tft.drawFastVLine(2*y, 0, top, ILI9341_BLACK);
      tft.drawFastVLine(2*y, top, w, ILI9341_CYAN);
    }
    
    if(gridOn) //draw the frequency markers on the grid if it's on.
    {
      drawGridFreq();
    }
    first_flag = false; //we can delta update untit some user process sets the flag again.
  }
  else //Otherwise, do a fast update. This is fast because after the first screen update is only draws deltas.
  {
    for(int y = 0; y < TOT_BINS; y++) //as above, calculate the peak.
    {

      if((bins[y] > maxBin) && (y >= 2)) //find center frequency
      {
        maxBin = bins[y];
        maxBinIndex = y;
      }

      //calculate bin delta changes and draw them
      int top = w - constrain(bins[y], 0, 240); //stop the bins from rolling off the screen.
      int bot = w - constrain(last_fft[y], 0, 240);
      if(last_fft[y] > bins[y]) //bin got smaller
      {
        tft.drawFastVLine(2*y, bot, last_fft[y] - bins[y], ILI9341_BLACK);
      }
      else if(last_fft[y] < bins[y]) //bin got bigger
      {
        tft.drawFastVLine(2*y, top, bins[y]-last_fft[y], ILI9341_CYAN);
      }
      last_fft[y] = bins[y];      
    }    
  }
  
  //check if the maximum frequency bin has changed. If so, update the meter onscreen.
  if(maxBinIndex != oldMaxBinIndex)
  {
    tft.fillRect(textLeft, 20 + offset, 64, 10, ILI9341_BLUE);
    float maxFreq = float(maxBinIndex) / 128.00;
    maxFreq = maxFreq * spanScale;
    tft.setCursor(textLeft, offset + 20);
    tft.print(maxFreq);
    tft.print("kHz");
    oldMaxBinIndex = maxBinIndex;
  }
  
  if(gridOn) //draw grid vertical axis markers as needed.
  {
    if(!linLog)
    {
      drawGriddB();
    }
    else
    {
      drawGridLin();
    }
  }
}

void drawGridFreq() //draws the frequency count and vertical lines
{
  for(int i = 0; i < w; i++) //iterate over the screen and draw numbers as needed
  {
    if(i%64 == 1)
    {
      if(i != 1)
      {
        tft.drawFastVLine(i, 0, 240, ILI9341_GRAY); //vertical lines, skip the first column.
      }
      if((span == 30)) //30kHz is big enough that we can do integer kHz and look decent
      {
        int dBMarker = round((float)i * (spanScale/256.0));
        if(i == 1)
        {
          dBMarker = 0;
        }
        tft.setCursor(i+2, 2); //offset the numbers from the vertical lines a tad.
        tft.print(dBMarker);
        if(i == 1)
        {
          tft.print( " kHz"); //only the first number gets a kHz
        }
      }
      else //smaller spans need floating point scales for clarity.
      {
        float dBMarker = (float)i * (spanScale/256.0);
        if(i == 1) //set zero point.
        {
          dBMarker = 0.0;
        }
        tft.setCursor(i+2, 2);
        char numBuf[5];
  
        dtostrf(dBMarker,4,1,numBuf);
        tft.print(numBuf);   
        if(i == 1)
        {
          tft.print( " kHz");
        }
      }
    }
  }
}
void drawGriddB() //draws dB scale markers
{
  for(int i = 0; i < h; i++)
  {
    if(i%31 == 0)
    {
      //tft.drawFastHLine(0, i, 256, ILI9341_GRAY);
      int dBMarker = -i/31 * 12 + 12; //start at +12dB
      if(dBMarker == 0) //write the dB scale with a correctly set offset so the digits are right aligned.
      {      
        tft.setCursor(textLeft-8, i + 2);
      }
      else if (dBMarker > 0)
      {
        tft.setCursor(textLeft-14, i + 2);
      }
      else if(dBMarker < -10)
      {      
        tft.setCursor(textLeft-20, i + 2);
      }
      else
      {
        tft.setCursor(textLeft-14, i + 2);        
      }

      tft.print(dBMarker);
      if(dBMarker == 12) //put dB at the top of the display.
      {
        tft.print(" dB");
      }
    }
  }
}

void drawGridLin() //draws linear scale markers
{
  for(int i = 0; i < h; i++)
  {
    if(i%31 == 0)
    {
      //tft.drawFastHLine(0, i, 256, ILI9341_GRAY);
      float linMarker = 5 - 0.64 * i/31;
      tft.setCursor(textLeft-26, i + 2);
      char numBuf[5];
  
      dtostrf(linMarker,4,1,numBuf); //create a string so we're limited to one decimal place
      tft.print(numBuf); //display
      if(linMarker == 5.0)
      {
        tft.print(" V"); //first element gets a 'V'
      }
    }
  }
}
