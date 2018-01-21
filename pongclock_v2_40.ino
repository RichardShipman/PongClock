/***********************************************************************
 * LED Pong Clock by Nick Hall
 * v2.27 Dec 2010 
 * v2.40 Jan 2011 Richard Shipman modifications - invader mode, dimming.
 * Uses 2x Sure 2416 LED modules, arduino and DS1307 clock chip.
 * Distributed under the terms of the GPL.
 *             
 * Holtek HT1632 LED driver chip code:
 * As implemented on the Sure Electronics DE-DP016 display board
 * (16*24 dot matrix LED module.)
 * Nov, 2008 by Bill Westfield ("WestfW")
 * Copyrighted and distributed under the terms of the Berkely license
 * (copy freely, but include this notice of original author.)
 ***********************************************************************/

#include "ht1632c.h"                     // Holtek LED driver by WestFW - updated to HT1632C by Nick Hall
#include <avr/pgmspace.h>                // Enable data to be stored in Flash Mem as well as SRAM              
#include "Font.h"                        // Font library
//#include <WProgram.h>                    // DS1307 clock
#include <Wire.h>                        // DS1307 clock
#include <DS1307.h>                      // DS1307 clock library by mattt on the arduino forum with various revisons from other users
#include <Button.h>                      // Button library by Alexander Brevig

#define ASSERT(condition)                // Nothing
#define X_MAX 47                         // Matrix X max LED coordinate (for 2 displays placed next to each other)
#define Y_MAX 15                         // Matrix Y max LED coordinate (for 2 displays placed next to each other)
#define NUM_DISPLAYS 2                   // Num displays for shadow ram data allocation
#define FADEDELAY 40                     // Time to fade display to black
#define SHOWCLOCK 5000                   // How long to show clocktype for
#define MAX_CLOCK_MODE 6                 // Number of clock modes
#define BAT1_X 2                         // Pong left bat x pos (this is where the ball collision occurs, the bat is drawn 1 behind these coords)
#define BAT2_X 45                        // Pong right bat x pos (this is where the ball collision occurs, the bat is drawn 1 behind these coords)
#define plot(x,y,v)  ht1632_plot(x,y,v)  // Plot LED
#define cls          ht1632_clear        // Clear display

static const byte ht1632_data = 10;      // Data pin for sure module
static const byte ht1632_wrclk = 11;     // Write clock pin for sure module
static const byte ht1632_cs[2] = {4,5};  // Chip_selects one for each sure module.
										 // Remember to set the DIP switches on the modules too.

Button buttonA = Button(2,PULLUP);       // Setup button A (using button library)
Button buttonB = Button(3,PULLUP);       // Setup button B (using button library)

int rtc[7];                              // Holds real time clock output
int random_mode = 1;
int mode_time_up;                        // Holds hour where clock mode will next change if in random mode
int mode_changed = 0;                    // Flag if mode changed.
int clock_mode = 4;                      // Default clock mode (1 = pong)

// Some of these should probably be stored in EEPROM and be changed through a settings interface.
byte current_brightness=15;              // Display brightness setting, to allow for nighttime dimming.
byte dim_setting=1;                      // The night-time dimmed setting for the display.
byte bright_setting=15;                  // The day-time brightness setting.
byte dim_hour=0;
byte bright_hour=7;

// ****** SET-UP ******

void setup ()  
{
  Serial.begin(9600);                    // DS1307 clock chip setup
  ht1632_setup();                        // Setup display (uses flow chart from page 17 of sure datasheet)
  randomSeed(analogRead(1));             // Setup random number generator
  printver();                            // Display clock software version on led screen
  gettime();                             // Get the time
  mode_time_up = rtc[2];                 // Set first time to change clock mode
}


// ****** MAIN ******

void loop ()
{
  if (random_mode){  
    gettime();
    //set counter to change clock type every 3 or 4 hours
    if (mode_time_up == rtc[2]) {
      mode_time_up = rtc[2] + random (2,4);   //set next time to change - add 2 to 3 hours
      if (mode_time_up >= 24) { 
        mode_time_up = random (1,2); 
      }   //if time is over 24, set to 0 + random
      clock_mode = random(0,MAX_CLOCK_MODE - 1);     //pick new random mode
    }
  }

  //reset clock type clock_mode
  switch (clock_mode){
  case 0: 
    normal_clock(); 
    break; 
  case 1: 
    pong(); 
    break;
  case 2: 
    word_clock(); 
    break;
  case 3: 
    jumble();
    break; 
  case 4:
    invaders();
    break;
  case 6: 
    set_time(); 
    break;
  }

  //if the mode hasn't changed, show the date
  if (mode_changed == 0) { 
    fade_down();
    display_date(); 
    fade_down();
  } 
  else {
    //the mode has changed, so don't bother showing the date, just go to the new mode.
    mode_changed = 0; //reset mdoe flag.
  }
}


// ****** SUB-ROUTINES ******

//Get the time from the DS1307 chip.
void gettime()
{
  RTC.get(rtc,true);
  if (rtc[DS1307_MIN]==0) 
  {
    check_brightness();
  }
}

/*
 *  Check what time it is and if it's time we should be changing brightness
 *  Usually will only make a change on the hour.
 *
 */
void check_brightness()
{
  byte new_setting;
  if (bright_hour > dim_hour) {
    if (rtc[DS1307_HR] <= bright_hour && rtc[DS1307_HR] >= dim_hour) 
    {
      new_setting = dim_setting;
    }
    else {
      new_setting = bright_setting;
    }
  } 
  else
  {
    if (rtc[DS1307_HR] >= dim_hour || rtc[DS1307_HR] <= bright_hour)
    {
      new_setting = dim_setting;
    }
    else {
      new_setting = bright_setting;
    }
  }
  if (new_setting != current_brightness)
  {
    fade_to_new(new_setting);
  }  
}

/*
 *  Change the brightness settings by a simple fade from current_brightness
 *  to the new brightness setting either up or down.
 *
 */
void fade_to_new(char new_setting)
{
  if (new_setting != current_brightness)
  {
    char dimmer_step;
    if (new_setting < current_brightness) {
      dimmer_step = -1;
    } else {
      dimmer_step = 1;
    }  
      
    for (char intensity = current_brightness; intensity != new_setting; intensity += dimmer_step) {
      ht1632_sendcmd(0, HT1632_CMD_PWM + intensity); //send intensity commands using CS0 for display 0
      ht1632_sendcmd(1, HT1632_CMD_PWM + intensity); //send intensity commands using CS0 for display 1
      delay(FADEDELAY);
    }
    current_brightness=new_setting;
  }
}

/*
 * ht1632_chipselect / ht1632_chipfree
 * Select or de-select a particular ht1632 chip. De-selecting a chip ends the commands being sent to a chip.
 * CD pins are active-low; writing 0 to the pin selects the chip.
 */
void ht1632_chipselect(byte chipno)
{
  DEBUGPRINT("\nHT1632(%d) ", chipno);
  digitalWrite(chipno, 0);
}

void ht1632_chipfree(byte chipno)
{
  DEBUGPRINT(" [done %d]", chipno);
  digitalWrite(chipno, 1);
}

/*
 * ht1632_writebits
 * Write bits (up to 8) to h1632 on pins ht1632_data, ht1632_wrclk Chip is assumed to already be chip-selected
 * Bits are shifted out from MSB to LSB, with the first bit sent being (bits & firstbit), shifted till firsbit is zero.
 */
void ht1632_writebits (byte bits, byte firstbit)
{
  DEBUGPRINT(" ");
  while (firstbit) {
    DEBUGPRINT((bits&firstbit ? "1" : "0"));
    digitalWrite(ht1632_wrclk, LOW);
    if (bits & firstbit) {
      digitalWrite(ht1632_data, HIGH);
    } 
    else {
      digitalWrite(ht1632_data, LOW);
    }
    digitalWrite(ht1632_wrclk, HIGH);
    firstbit >>= 1;
  }
}

/*
 * ht1632_sendcmd
 * Send a command to the ht1632 chip. A command consists of a 3-bit "CMD" ID, an 8bit command, and one "don't care bit".
 *   Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx Free
 */
static void ht1632_sendcmd (byte d, byte command)
{
  ht1632_chipselect(ht1632_cs[d]);        // Select chip
  ht1632_writebits(HT1632_ID_CMD, 1<<2);  // send 3 bits of id: COMMMAND
  ht1632_writebits(command, 1<<7);        // send the actual command
  ht1632_writebits(0, 1);         	  // one extra dont-care bit in commands.
  ht1632_chipfree(ht1632_cs[d]);          //done
}

/*
 * ht1632_senddata
 * send a nibble (4 bits) of data to a particular memory location of the
 * ht1632.  The command has 3 bit ID, 7 bits of address, and 4 bits of data.
 *    Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 Free
 * Note that the address is sent MSB first, while the data is sent LSB first!
 * This means that somewhere a bit reversal will have to be done to get
 * zero-based addressing of words and dots within words.
 */
static void ht1632_senddata (byte d, byte address, byte data)
{
  ht1632_chipselect(ht1632_cs[d]);      // Select chip
  ht1632_writebits(HT1632_ID_WR, 1<<2); // Send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6);      // Send address
  ht1632_writebits(data, 1<<3);         // Send 4 bits of data
  ht1632_chipfree(ht1632_cs[d]);        // Done.
}

/*
 * ht1632_setup
 * setup the ht1632 chips
 */
void ht1632_setup()
{
  for (byte d=0; d<NUM_DISPLAYS; d++) {
    pinMode(ht1632_cs[d], OUTPUT);

    digitalWrite(ht1632_cs[d], HIGH);  // Unselect (active low)

    pinMode(ht1632_wrclk, OUTPUT);
    pinMode(ht1632_data, OUTPUT);

    ht1632_sendcmd(d, HT1632_CMD_SYSON);    // System on 
    ht1632_sendcmd(d, HT1632_CMD_LEDON);    // LEDs on 
    ht1632_sendcmd(d, HT1632_CMD_COMS01);   // NMOS Output 24 row x 24 Com mode

    for (byte i=0; i<128; i++)
      ht1632_senddata(d, i, 0);  // clear the display!
  }
}

/*
 * we keep a copy of the display controller contents so that we can know which bits are on without having to (slowly) read the device.
 * Note that we only use the low four bits of the shadow ram, since we're shadowing 4-bit memory.  This makes things faster, and we
 * use the other half for a "snapshot" when we want to plot new data based on older data...
 */
byte ht1632_shadowram[NUM_DISPLAYS * 96];  // our copy of the display's RAM

/*
 * plot a point on the display, with the upper left hand corner being (0,0).
 * Note that Y increases going "downward" in contrast with most mathematical coordiate systems, but in common with many displays
 * No error checking; bad things may happen if arguments are out of bounds!  (The ASSERTS compile to nothing by default
 */
void ht1632_plot (char x, char y, char val)
{

  char addr, bitval;

  ASSERT(x >= 0);
  ASSERT(x <= X_MAX);
  ASSERT(y >= 0);
  ASSERT(y <= y_MAX);

  if (x<0 || x> X_MAX || y<0 || y> Y_MAX )
  {
    return;
  }

  byte d;
  //select display depending on plot values passed in
  if (x >= 0 && x <=23 ) {
    d = 0;
  }  
  if (x >=24 && x <=47) {
    d = 1;
    x = x-24; 
  }   

  /*
   * The 4 bits in a single memory word go DOWN, with the LSB (first transmitted) bit being on top.  However, writebits()
   * sends the MSB first, so we have to do a sort of bit-reversal somewhere.  Here, this is done by shifting the single bit in
   * the opposite direction from what you might expect.
   */

  bitval = 8>>(y&3);  // compute which bit will need set

  addr = (x<<2) + (y>>2);  // compute which memory word this is in 

  if (val) {  // Modify the shadow memory
    ht1632_shadowram[(d * 96)  + addr] |= bitval;
  } 
  else {
    ht1632_shadowram[(d * 96) + addr] &= ~bitval;
  }
  // Now copy the new memory value to the display
  ht1632_senddata(d, addr, ht1632_shadowram[(d * 96) + addr]);
}

/*
 * get_shadowram
 * return the value of a pixel from the shadow ram.
 */
byte get_shadowram(byte x, byte y)
{
  byte addr, bitval, d;

  //select display depending on plot values passed in
  if (x >= 0 && x <=23 ) {
    d = 0;
  }  
  if (x >=24 && x <=47) {
    d = 1;
    x = x-24; 
  }  

  bitval = 8>>(y&3);  // compute which bit will need set
  addr = (x<<2) + (y>>2);       // compute which memory word this is in 
  return (0 != (ht1632_shadowram[(d * 96) + addr] & bitval));
}

/*
 * snapshot_shadowram
 * Copy the shadow ram into the snapshot ram (the upper bits)
 * This gives us a separate copy so we can plot new data while
 * still having a copy of the old data.  snapshotram is NOT
 * updated by the plot functions (except "clear")
 */
void snapshot_shadowram()
{
  for (byte i=0; i< sizeof ht1632_shadowram; i++) {
    ht1632_shadowram[i] = (ht1632_shadowram[i] & 0x0F) | ht1632_shadowram[i] << 4;  // Use the upper bits
  }

}

/*
 * get_snapshotram
 * get a pixel value from the snapshot ram (instead of
 * the actual displayed (shadow) memory
 */
byte get_snapshotram(byte x, byte y)
{

  byte addr, bitval;
  byte d = 0;

  //select display depending on plot values passed in 
  if (x >=24 && x <=47) {
    d = 1;
    x = x-24; 
  }  

  bitval = 128>>(y&3);  // user upper bits!
  addr = (x<<2) + (y>>2);   // compute which memory word this is in 
  if (ht1632_shadowram[(d * 96) + addr] & bitval)
    return 1;
  return 0;
}

/*
 * ht1632_clear
 * clear the display, and the shadow memory, and the snapshot
 * memory.  This uses the "write multiple words" capability of
 * the chipset by writing all 96 words of memory without raising
 * the chipselect signal.
 */
void ht1632_clear()
{
  char i;
  for(byte d=0; d<NUM_DISPLAYS; d++)
  {
    ht1632_chipselect(ht1632_cs[d]);  // Select chip
    ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
    ht1632_writebits(0, 1<<6); // Send address
    for (i = 0; i < 96/2; i++) // Clear entire display
      ht1632_writebits(0, 1<<7); // send 8 bits of data
    ht1632_chipfree(ht1632_cs[d]); // done
    for (i=0; i < 96; i++)
      ht1632_shadowram[96*d + i] = 0;
  }
}

/* ht1632_putchar
 * Copy a 5x7 character glyph from the myfont data structure to display memory, with its upper left at the given coordinate
 * This is unoptimized and simply uses plot() to draw each dot.
 */
void ht1632_putchar(byte x, byte y, char c)
{
  byte dots;
  //  if (c >= 'A' && c <= 'Z' || (c >= 'a' && c <= 'z') ) {
  //    c &= 0x1F;   // A-Z maps to 1-26
  //  } 
  if (c >= 'A' && c <= 'Z' ) {
    c &= 0x1F;   // A-Z maps to 1-26
  } 
  else if (c >= 'a' && c <= 'z') {
    c = (c - 'a') + 41;   // A-Z maps to 41-67
  } 
  else if (c >= '0' && c <= '9') {
    c = (c - '0') + 31;
  } 
  else if (c == ' ') {
    c = 0; // space
  }
  else if (c == '.') {
    c = 27; // full stop
  }
  else if (c == '\'') {
    c = 28; // single quote mark
  }  
  else if (c == ':') {
    c = 29; // clock_mode selector arrow
  }
  else if (c == '>') {
    c = 30; // clock_mode selector arrow
  }
  else if (c >= -80 && c<= -67) {
    c *= -1;
  }

  for (char col=0; col< 5; col++) {
    dots = pgm_read_byte_near(&myfont[c][col]);
    for (char row=0; row < 7; row++) {
      if (dots & (64>>row))   	     // only 7 rows.
        plot(x+col, y+row, 1);
      else 
        plot(x+col, y+row, 0);
    }
  }
}

/* ht1632_putbigchar
 * Copy a 10x14 character glyph from the myfont data structure to display memory, with its upper left at the given coordinate
 * This is unoptimized and simply uses plot() to draw each dot.
 */
void ht1632_putbigchar(byte x, byte y, char c)
{
  byte dots;
  if (c >= 'A' && c <= 'Z' || (c >= 'a' && c <= 'z') ) {
    return;   //return, as the 10x14 font contains only numeric characters 
  } 
  if (c >= '0' && c <= '9') {
    c = (c - '0');
    c &= 0x1F;
  } 

  for (char col=0; col< 10; col++) {
    dots = pgm_read_byte_near(&mybigfont[c][col]);
    for (char row=0; row < 8; row++) {
      if (dots & (128>>row))   	   
        plot(x+col, y+row, 1);
      else 
        plot(x+col, y+row, 0);
    }

    dots = pgm_read_byte_near(&mybigfont[c][col+10]);
    for (char row=0; row < 8; row++) {
      if (dots & (128>>row))   	   
        plot(x+col, y+row+8, 1);
      else 
        plot(x+col, y+row+8, 0);
    } 
  }  
}

/* ht1632_puttinychar
 * Copy a 3x5 character glyph from the myfont data structure to display memory, with its upper left at the given coordinate
 * This is unoptimized and simply uses plot() to draw each dot.
 */
void ht1632_puttinychar(byte x, byte y, char c)
{
  byte dots;
  if (c >= 'A' && c <= 'Z' || (c >= 'a' && c <= 'z') ) {
    c &= 0x1F;   // A-Z maps to 1-26
  } 
  else if (c >= '0' && c <= '9') {
    c = (c - '0') + 31;
  } 
  else if (c == ' ') {
    c = 0; // space
  }
  else if (c == '.') {
    c = 27; // full stop
  }
  else if (c == '\'') {
    c = 28; // single quote mark
  } 
  else if (c == '!') {
    c = 29; // single quote mark
  }  
  else if (c == '?') {
    c = 30; // single quote mark
  }

  for (char col=0; col< 3; col++) {
    dots = pgm_read_byte_near(&mytinyfont[c][col]);
    for (char row=0; row < 5; row++) {
      if (dots & (16>>row))   	   
        plot(x+col, y+row, 1);
      else 
        plot(x+col, y+row, 0);
    }
  }  
}



/* normal_clock()
 * show the time in 10x14 characters and update it whilst the loop runs
 */
void normal_clock()
{
  cls();
  byte hours = rtc[DS1307_HR];
  byte mins = rtc[DS1307_MIN];

  //loop to display the clock for a set duration of SHOWCLOCK
  for (int show = 0; show < SHOWCLOCK ; show++) {

    gettime(); //get the time from the clock chip

    flash_seconds();
    
    if (button_checks())
    {
      return;
    }

    //update the clock if this is the first run of the show clock loop, or if the time has changed from 
    //what we had stored in mins and hours vars.
    if ( show == 0 || (mins != rtc[DS1307_MIN] ) ) {  

      //udate mins and hours with the new time
      mins = rtc[DS1307_MIN];
      hours = rtc[DS1307_HR];

      char buffer[3];

      itoa(hours,buffer,10);
      //fix - as otherwise if num has leading zero, e.g. "03" hours, itoa coverts this to chars with space "3 ". 
      if (hours < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }


      //update the display
      ht1632_putbigchar(0,  1, buffer[0]);
      ht1632_putbigchar(12, 1, buffer[1]);

      plot (23,4,1); //top point
      plot (23,5,1);
      plot (24,4,1);
      plot (24,5,1);
      plot (23,10,1); //bottom point
      plot (23,11,1);
      plot (24,10,1);
      plot (24,11,1);

      itoa (mins, buffer, 10);
      if (mins < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }

      ht1632_putbigchar(26, 1, buffer[0]);
      ht1632_putbigchar(38, 1, buffer[1]);  
    }
    delay(50);
  }
  fade_down();
}


/* invaders()
 * show the time and a space invader and update it whilst the loop runs
 */
void invaders()
{
  cls();
  byte hours = rtc[DS1307_HR];
  byte mins = rtc[DS1307_MIN];

  //loop to display the clock for a set duration of SHOWCLOCK
  for (int show = 0; show < SHOWCLOCK ; show++) {

    gettime(); //get the time from the clock chip

    flash_seconds();

    if (button_checks())
    {
      return;
    }

    //update the clock if this is the first run of the show clock loop, or if the time has changed from 
    //what we had stored in mins and hours vars.
    if ( show == 0 || (mins != rtc[1] ) ) {  

      //udate mins and hours with the new time
      mins = rtc[1];
      hours = rtc[2];

      char buffer[3];

      itoa(hours,buffer,10);
      //fix - as otherwise if num has leading zero, e.g. "03" hours, itoa coverts this to chars with space "3 ". 
      if (hours < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }

      //update the display
      ht1632_putchar(13,  1, buffer[0]);
      ht1632_putchar(19, 1, buffer[1]);

      plot (25,3,1); //top point
      plot (25,5,1); //bottom point

      itoa (mins, buffer, 10);
      if (mins < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }

      ht1632_putchar(27, 1, buffer[0]);
      ht1632_putchar(33, 1, buffer[1]);  
    }
    
    if (scroll_invader (9,-11,49,random(1,4))) {
      return;
    }
    if (scroll_invader (9,49,-11,random(1,4))) {
      return;
    }

    delay(50);
  }
  fade_down();
}

/*
 * display_date
 * print the day of week, date and month 
 */
void display_date()
{

  cls();
  //read the date from the DS1307
  //it returns the month number, day number, and a number representing the day of week - 1 for Tue, 2 for Wed 3 for Thu etc.
  byte dow = rtc[DS1307_DOW] - 1; //we  take one off the value the DS1307 generates, as our array of days is 0-6 and the DS1307 outputs  1-7.
  byte date = rtc[DS1307_DATE];
  byte month = rtc[DS1307_MTH] - 1; 

  //array of day and month names to print on the display. Some are shortened as we only have 8 characters across to play with 
  char daynames[7][9]={
    "Tuesday", "Wed", "Thursday", "Friday", "Saturday", "Sunday", "Monday"      };
  char monthnames[12][9]={
    "January", "February", "March", "April", "May", "June", "July", "August", "Sept", "October", "November", "December"      };

  //call the flashing cursor effect for one blink at x,y pos 0,0, height 5, width 7, repeats 1
  flashing_cursor(0,0,5,7,1);

  //print the day name
  int i = 0;
  while(daynames[dow][i])
  {
    flashing_cursor(i*6,0,5,7,0);
    ht1632_putchar(i*6 , 0, daynames[dow][i]); 
    i++;

    //check for button press and exit if there is one.
    if(buttonA.uniquePress() || buttonB.uniquePress()){
      return;
    }
  }

  //pause at the end of the line with a flashing cursor if there is space to print it.
  //if there is no space left, dont print the cursor, just wait.
  if (i*6 < 48){
    flashing_cursor(i*6,0,5,7,1);  
  } 
  else {
    delay(300);
  }

  //flash the cursor on the next line  
  flashing_cursor(0,8,5,7,0);

  //print the date on the next line: First convert the date number to chars so we can print it with ht1632_putchar
  char buffer[3];
  itoa(date,buffer,10);

  //then work out date 2 letter suffix - eg st, nd, rd, th etc
  char suffix[4][3]={
    "st", "nd", "rd", "th"        };
  byte s = 3; 
  if(date == 1 || date == 21 || date == 31) {
    s = 0;
  } 
  else if (date == 2 || date == 22) {
    s = 1;
  } 
  else if (date == 3 || date == 23) {
    s = 2;
  } 

  //print the 1st date number
  ht1632_putchar(0, 8, buffer[0]);

  //if date is under 10 - then we only have 1 digit so set positions of sufix etc one character nearer
  byte suffixposx = 6;

  //if date over 9 then print second number and set xpos of suffix to be 1 char further away
  if (date > 9){
    suffixposx = 12;
    flashing_cursor(6,8,5,7,0); 
    ht1632_putchar(6, 8, buffer[1]);
  }

  //print the 2 suffix characters
  flashing_cursor(suffixposx, 8,5,7,0);
  ht1632_putchar(suffixposx, 8, suffix[s][0]); 
  //delay(70);

  flashing_cursor(suffixposx+6,8,5,7,0);
  ht1632_putchar(suffixposx+6, 8, suffix[s][1]); 
  //delay(70);

  //blink cursor after 
  flashing_cursor(suffixposx + 12,8,5,7,1);  

  //replace day name with date on top line - effectively scroll the bottom line up by 8 pixels
  cls();   
  ht1632_putchar(0, 0, buffer[0]);  //date first digit
  ht1632_putchar(6, 0, buffer[1]);  //date second digit - this may be blank and overwritten if the date is a single number
  ht1632_putchar(suffixposx, 0, suffix[s][0]);   //date suffix
  ht1632_putchar(suffixposx+6, 0, suffix[s][1]);   //date suffix


  //flash the cursor for a second for effect
  flashing_cursor(suffixposx + 12,0,5,7,0);  

  //print the month name on the bottom row
  i = 0;
  while(monthnames[month][i])
  {  
    flashing_cursor(i*6,8,5,7,0);
    ht1632_putchar(i*6, 8, monthnames[month][i]); 
    i++; 

    //check for button press and exit if there is one.
    if(buttonA.uniquePress() || buttonB.uniquePress()){
      return;
    }
  }

  //blink the cursor at end if enough space after the month name, otherwise juts wait a while
  if (i*6 < 48){
    flashing_cursor(i*6,8,5,7,2);  
  } 
  else {
    delay(1000);
  }
  delay(3000);
}


void display_invader (byte xpos, byte ypos, byte invader_type, boolean wiggle)
{
  // base invaders are at: -67, -71, -75 wiggle is -2 from each
  char invader = -63 - (invader_type*4);
  if (wiggle) {
    invader -= 2;
  }

  ht1632_putchar(xpos,ypos, invader);
  ht1632_putchar(xpos+5,ypos, invader-1);

}

boolean scroll_invader (byte ypos, char xstart, char xend, byte invader_type)
{
  boolean wiggle=false;
  char xstep;
  if (xstart < xend) {
    xstep=1;
  } 
  else {
    xstep = -1;
  }
  for (char i = xstart; i != xend; i += xstep)
  {
    display_invader(i, ypos, invader_type, wiggle);
    wiggle = (!wiggle);
    delay(100);
    if (button_checks()) 
    {
      return true;
    }
    if (abs(i-xend)>1)
    {
      for (byte y = ypos; y < ypos+7; y++) {
        plot (i,y,0);
        plot (i+9,y,0);
      }
    }
    gettime();
    flash_seconds();
  }
  return false;
}


/*
 * flashing_cursor
 * print a flashing_cursor at xpos, ypos and flash it repeats times 
 */
void flashing_cursor(byte xpos, byte ypos, byte cursor_width, byte cursor_height, byte repeats)
{
  for (byte r = 0; r <= repeats; r++) {    
    for (byte x = 0; x <= cursor_width; x++) {
      for (byte y = 0; y <= cursor_height; y++) {
        plot(x+xpos, y+ypos, 1);
      }
    }

    if (repeats > 0) {
      delay(400);
    } 
    else {
      delay(70);
    }

    for (byte x = 0; x <= cursor_width; x++) {
      for (byte y = 0; y <= cursor_height; y++) {
        plot(x+xpos, y+ypos, 0);
      }
    }   
    //if cursor set to repeat, wait a while
    if (repeats > 0) {
      delay(400); 
    }
  }
}

/*
 * fade_down
 * fade the display to black
 */
void fade_down() {
  char intensity;
  for (intensity=current_brightness-1; intensity >= 0; intensity--) {
    ht1632_sendcmd(0, HT1632_CMD_PWM + intensity); //send intensity commands using CS0 for display 0
    ht1632_sendcmd(1, HT1632_CMD_PWM + intensity); //send intensity commands using CS0 for display 1
    delay(FADEDELAY);
  }
  //clear the display and set it to full brightness again so we're ready to plot new stuff
  cls();
  ht1632_sendcmd(0, HT1632_CMD_PWM + current_brightness);
  ht1632_sendcmd(1, HT1632_CMD_PWM + current_brightness);
}


/*
 * fade_up
 * fade the display up to full brightness
 */
void fade_up() {
  char intensity;
  for ( intensity=0; intensity < current_brightness; intensity++) {
    ht1632_sendcmd(0, HT1632_CMD_PWM + intensity); //send intensity commands using CS0 for display 0
    ht1632_sendcmd(1, HT1632_CMD_PWM + intensity); //send intensity commands using CS0 for display 1
    delay(FADEDELAY);
  }
}


/*
 * button_delay
 * like regular delay but can be quit by a button press
 */
void button_delay(int wait) {
  int i = 0;
  while ( i < wait){
    //check if a button is pressed, if it is, quit waiting
    if(buttonA.uniquePress() || buttonB.uniquePress()){
      return;
    }
    //else wait a moment
    delay (1);
    i++;
  }
}


/*
 * pong
 * play pong - using the time as the score
 */
void pong(){

  float ballpos_x, ballpos_y;
  byte erase_x = 10;  //holds ball old pos so we can erase it, set to blank area of screen initially.
  byte erase_y = 10;
  float ballvel_x, ballvel_y;
  int bat1_y = 5;  //bat starting y positions
  int bat2_y = 5;  
  int bat1_target_y = 5;  //bat targets for bats to move to
  int bat2_target_y = 5;
  byte bat1_update = 1;  //flags - set to update bat position
  byte bat2_update = 1;
  byte bat1miss, bat2miss; //flags set on the minute or hour that trigger the bats to miss the ball, thus upping the score to match the time.
  byte restart = 1;   //game restart flag - set to 1 initially to setup 1st game

  //setup
  cls();

  byte i = 0;
  char intro[11] = "Play Pong!";
  while(intro[i]) {
    ht1632_puttinychar((i*4)+6, 4, intro[i]); 
    i++;
  }
  delay(1500);

  cls();
  gettime();

  //draw pitch centre line
  for (byte i = 0; i <16; i++) {
    if ( i % 2 == 0 ) { //plot point if an even number
      plot(24, i, 1); 
    }
  } 


  //main pong game loop 
  for(int i=0; i< SHOWCLOCK; i++) {

    flash_seconds();

    if (button_checks())
    {
      return;
    }

    //if restart flag is 1, setup a new game
    if (restart) {

      //erase ball pos
      plot (erase_x, erase_y, 0);

      //update score / time
      byte mins = rtc[DS1307_MIN];
      byte hours = rtc[DS1307_HR];

      char buffer[3];

      itoa(hours,buffer,10);
      //fix - as otherwise if num has leading zero, e.g. "03" hours, itoa coverts this to chars with space "3 ". 
      if (hours < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }
      ht1632_puttinychar(14, 0, buffer[0] );
      ht1632_puttinychar(18, 0, buffer[1]);


      itoa(mins,buffer,10); 
      if (mins < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      } 
      ht1632_puttinychar(28, 0, buffer[0]);
      ht1632_puttinychar(32, 0, buffer[1]);  

      //set ball start pos
      ballpos_x = 23;
      ballpos_y = random (4,12);

      //pick random ball direction
      if (random(0,2) > 0) {
        ballvel_x = 1; 
      } 
      else {
        ballvel_x = -1;
      }
      if (random(0,2) > 0) {
        ballvel_y = 0.5; 
      } 
      else {
        ballvel_y = -0.5;
      }
      //draw bats in initial positions
      bat1miss = 0; 
      bat2miss = 0;
      //reset game restart flag
      restart = 0;

      //short wait
      delay(1500);
    }

    //get the time from the rtc
    gettime();

    //if coming up to the minute: secs = 59 and mins < 59, flag bat 2 (right side) to miss the return so we inc the minutes score
    if (rtc[DS1307_SEC] == 59 && rtc[DS1307_MIN] < 59){
      bat1miss = 1;
    }
    // if coming up to the hour: secs = 59  and mins = 59, flag bat 1 (left side) to miss the return, so we inc the hours score.
    if (rtc[DS1307_SEC] == 59 && rtc[DS1307_MIN] == 59){
      bat2miss = 1;
    }


    //AI - we run 2 sets of 'AI' for each bat to work out where to go to hit the ball back 

    //very basic AI...
    // For each bat, First just tell the bat to move to the height of the ball when we get to a random location.
    //for bat1
    if (ballpos_x == random(30,41)){// && ballvel_x < 0) {
      bat1_target_y = ballpos_y;
    }
    //for bat2
    if (ballpos_x == random(8,19)){//  && ballvel_x > 0) {
      bat2_target_y = ballpos_y;
    }

    //when the ball is closer to the left bat, run the ball maths to find out where the ball will land
    if (ballpos_x == 23 && ballvel_x < 0) {

      byte end_ball_y = pong_get_ball_endpoint(ballpos_x, ballpos_y, ballvel_x, ballvel_y);

      //if the miss flag is set,  then the bat needs to miss the ball when it gets to end_ball_y
      if (bat1miss == 1){
        bat1miss = 0;
        if ( end_ball_y > 8){
          bat1_target_y = random (0,3); 
        } 
        else {
          bat1_target_y = 8 + random (0,3);              
        }      
      } 
      //if the miss flag isn't set,  set bat target to ball end point with some randomness so its not always hitting top of bat
      else {
        bat1_target_y = end_ball_y - random (0, 6);        
        //check not less than 0
        if (bat1_target_y < 0){
          bat1_target_y = 0;
        }
        if (bat1_target_y > 10){
          bat1_target_y = 10;
        } 
      }
    }


    //right bat AI
    //if positive velocity then predict for right bat - first just match ball height

    //when the ball is closer to the right bat, run the ball maths to find out where it will land
    if (ballpos_x == 25 && ballvel_x > 0) {

      byte end_ball_y = pong_get_ball_endpoint(ballpos_x, ballpos_y, ballvel_x, ballvel_y);

      //if flag set to miss, move bat out way of ball
      if (bat2miss == 1){
        bat2miss = 0;
        //if ball end point above 8 then move bat down, else move it up- so either way it misses
        if (end_ball_y > 8){
          bat2_target_y = random (0,3); 
        } 
        else {
          bat2_target_y = 8 + random (0,3);
        }      
      } 
      else {
        //set bat target to ball end point with some randomness 
        bat2_target_y =  end_ball_y - random (0,6);
        //ensure target between 0 and 15
        if (bat2_target_y < 0){
          bat2_target_y = 0;
        } 
        if (bat2_target_y > 10){
          bat2_target_y = 10;
        } 
      }
    }


    //move bat 1 towards target    
    //if bat y greater than target y move down until hit 0 (dont go any further or bat will move off screen)
    if (bat1_y > bat1_target_y && bat1_y > 0 ) {
      bat1_y--;
      bat1_update = 1;
    }

    //if bat y less than target y move up until hit 10 (as bat is 6)
    if (bat1_y < bat1_target_y && bat1_y < 10) {
      bat1_y++;
      bat1_update = 1;
    }

    //draw bat 1
    if (bat1_update){
      for (byte i = 0; i < 16; i++){
        if (i - bat1_y < 6 &&  i - bat1_y > -1){
          plot(BAT1_X-1, i , 1);
          plot(BAT1_X-2, i , 1);
        } 
        else {
          plot(BAT1_X-1, i , 0);
          plot(BAT1_X-2, i , 0);
        }
      } 
    }


    //move bat 2 towards target (dont go any further or bat will move off screen)

    //if bat y greater than target y move down until hit 0
    if (bat2_y > bat2_target_y && bat2_y > 0 ) {
      bat2_y--;
      bat2_update = 1;
    }

    //if bat y less than target y move up until hit max of 10 (as bat is 6)
    if (bat2_y < bat2_target_y && bat2_y < 10) {
      bat2_y++;
      bat2_update = 1;
    }

    //draw bat2
    if (bat2_update){
      for (byte i = 0; i < 16; i++){
        if (  i - bat2_y < 6 && i - bat2_y > -1){
          plot(BAT2_X+1, i , 1);
          plot(BAT2_X+2, i , 1);
        } 
        else {
          plot(BAT2_X+1, i , 0);
          plot(BAT2_X+2, i , 0);
        }
      } 
    }

    //update the ball position using the velocity
    ballpos_x =  ballpos_x + ballvel_x;
    ballpos_y =  ballpos_y + ballvel_y;

    //check ball collision with top and bottom of screen and reverse the y velocity if either is hit
    if (ballpos_y <= 0 ){
      ballvel_y = ballvel_y * -1;
      ballpos_y = 0; //make sure value goes no less that 0
    }

    if (ballpos_y >= 15){
      ballvel_y = ballvel_y * -1;
      ballpos_y = 15; //make sure value goes no more than 15
    }

    //check for ball collision with bat1. check ballx is same as batx
    //and also check if bally lies within width of bat i.e. baty to baty + 6. We can use the exp if(a < b && b < c) 
    if ((int)ballpos_x == BAT1_X && (bat1_y <= (int)ballpos_y && (int)ballpos_y <= bat1_y + 5) ) { 

      //random if bat flicks ball to return it - and therefor changes ball velocity
      if(!random(0,3)) { //not true = no flick - just straight rebound and no change to ball y vel
        ballvel_x = ballvel_x * -1;
      } 
      else {
        bat1_update = 1;
        byte flick;  //0 = up, 1 = down.

        if (bat1_y > 1 || bat1_y < 8){
          flick = random(0,2);   //pick a random dir to flick - up or down
        }

        //if bat 1 or 2 away from top only flick down
        if (bat1_y <=1 ){
          flick = 0;   //move bat down 1 or 2 pixels 
        } 
        //if bat 1 or 2 away from bottom only flick up
        if (bat1_y >=  8 ){
          flick = 1;  //move bat up 1 or 2 pixels 
        }

        switch (flick) {
          //flick up
        case 0:
          bat1_target_y = bat1_target_y + random(1,3);
          ballvel_x = ballvel_x * -1;
          if (ballvel_y < 2) {
            ballvel_y = ballvel_y + 0.2;
          }
          break;

          //flick down
        case 1:   
          bat1_target_y = bat1_target_y - random(1,3);
          ballvel_x = ballvel_x * -1;
          if (ballvel_y > 0.2) {
            ballvel_y = ballvel_y - 0.2;
          }
          break;
        }
      }
    }


    //check for ball collision with bat2. check ballx is same as batx
    //and also check if bally lies within width of bat i.e. baty to baty + 6. We can use the exp if(a < b && b < c) 
    if ((int)ballpos_x == BAT2_X && (bat2_y <= (int)ballpos_y && (int)ballpos_y <= bat2_y + 5) ) { 

      //random if bat flicks ball to return it - and therefor changes ball velocity
      if(!random(0,3)) {
        ballvel_x = ballvel_x * -1;    //not true = no flick - just straight rebound and no change to ball y vel
      } 
      else {
        bat1_update = 1;
        byte flick;  //0 = up, 1 = down.

        if (bat2_y > 1 || bat2_y < 8){
          flick = random(0,2);   //pick a random dir to flick - up or down
        }
        //if bat 1 or 2 away from top only flick down
        if (bat2_y <= 1 ){
          flick = 0;  //move bat up 1 or 2 pixels 
        } 
        //if bat 1 or 2 away from bottom only flick up
        if (bat2_y >=  8 ){
          flick = 1;   //move bat down 1 or 2 pixels 
        }

        switch (flick) {
          //flick up
        case 0:
          bat2_target_y = bat2_target_y + random(1,3);
          ballvel_x = ballvel_x * -1;
          if (ballvel_y < 2) {
            ballvel_y = ballvel_y + 0.2;
          }
          break;

          //flick down
        case 1:   
          bat2_target_y = bat2_target_y - random(1,3);
          ballvel_x = ballvel_x * -1;
          if (ballvel_y > 0.2) {
            ballvel_y = ballvel_y - 0.2;
          }
          break;
        }
      }
    }

    //plot the ball on the screen
    byte plot_x = (int)(ballpos_x + 0.5f);
    byte plot_y = (int)(ballpos_y + 0.5f);

    //take a snapshot of all the led states
    snapshot_shadowram();

    //if the led at the ball pos is on already, dont bother printing the ball.
    if (get_shadowram(plot_x, plot_y)){
      //erase old point, but don't update the erase positions, so next loop the same point will be erased 
      //rather than this point which shouldn't be
      plot (erase_x, erase_y, 0);   
    } 
    else {
      //else plot the ball and erase the old position
      plot (plot_x, plot_y, 1);     
      plot (erase_x, erase_y, 0); 
      //reset erase to new pos
      erase_x = plot_x; 
      erase_y = plot_y;
    }

    //check if a bat missed the ball. if it did, reset the game.
    if ((int)ballpos_x == 0 ||(int) ballpos_x == 47){
      restart = 1; 
    } 
    delay(35);
  } 
  fade_down();
}


byte pong_get_ball_endpoint(float tempballpos_x, float  tempballpos_y, float  tempballvel_x, float tempballvel_y) {

  //run prediction until ball hits bat
  while (tempballpos_x > BAT1_X && tempballpos_x < BAT2_X  ){     
    tempballpos_x = tempballpos_x + tempballvel_x;
    tempballpos_y = tempballpos_y + tempballvel_y;
    //check for collisions with top / bottom
    if (tempballpos_y <= 0 || tempballpos_y >= 15){
      tempballvel_y = tempballvel_y * -1;
    }    
  }  
  return tempballpos_y; 
}


//display software version number
void printver(){

  byte i = 0;
  char ver[16] = "Clock V2.40";

  while(ver[i]) {
    ht1632_puttinychar((i*4) + 1, 1, ver[i]);
    i++;
  }
  delay(1000);

  scroll_invader (9,-11,49,random(1,4));

  delay(1000);
  cls();
}

boolean button_checks()
{
  if(buttonA.uniquePress()){
    switch_mode();
    return true;
  }
  if(buttonB.uniquePress()){
//    display_date();
//    fade_down();
    return true;
  }
  return false;
}

//print a clock using words rather than numbers
void word_clock() {

  cls();

  char numbers[19][10]   = { 
    "one", "two", "three", "four","five","six","seven","eight","nine","ten",
    "eleven","twelve", "thirteen","fourteen","fifteen","sixteen","seventeen","eighteen","nineteen"    };              
  char numberstens[5][7] = { 
    "ten","twenty","thirty","forty","fifty"     };

  byte hours_y, mins_y; //hours and mins and positions for hours and mins lines  

  byte hours = rtc[DS1307_HR];
  byte mins  = rtc[DS1307_MIN];

  //loop to display the clock for a set duration of SHOWCLOCK
  for (int show = 0; show < SHOWCLOCK ; show++) {

    gettime(); //get the time from the clock chip

    if (button_checks())
    {
      return;
    }

    flash_seconds();

    //print the time if it has changed or if we have just come into the subroutine
    if ( show == 0 || mins != rtc[DS1307_MIN] ) {  

      //reset these for comparison next time
      mins = rtc[DS1307_MIN];   
      hours = rtc[DS1307_HR];

      //make hours into 12 hour format
      if (hours > 12){ 
        hours = hours - 12; 
      }
      if (hours == 0){ 
        hours = 12; 
      } 

      //split mins value up into two separate digits 
      int minsdigit = mins % 10;
      byte minsdigitten = (mins / 10) % 10;

      char str_top[12];
      char str_bot[12];

      //if mins <= 10 , then top line has to read "minsdigti past" and bottom line reads hours
      if (mins < 10) {     
        strcpy (str_top,numbers[minsdigit - 1]);
        strcat (str_top," PAST");
        strcpy (str_bot,numbers[hours - 1]);
      }
      //if mins = 10, cant use minsdigit as above, so soecial case to print 10 past /n hour.
      if (mins == 10) {     
        strcpy (str_top,numbers[9]);
        strcat (str_top," PAST");
        strcpy (str_bot,numbers[hours - 1]);
      }

      //if time is not on the hour - i.e. both mins digits are not zero, 
      //then make top line read "hours" and bottom line ready "minstens mins" e.g. "three /n twenty one"
      else if (minsdigitten != 0 && minsdigit != 0  ) {

        strcpy (str_top,numbers[hours - 1]); 

        //if mins is in the teens, use teens from the numbers array for the bottom line, e.g. "three /n fifteen"
        if (mins >= 11 && mins <= 19) {
          strcpy (str_bot, numbers[mins - 1]);

          //else bottom line reads "minstens mins" e.g. "three \n twenty three"
        } 
        else {     
          strcpy (str_bot, numberstens[minsdigitten - 1]);
          strcat (str_bot, " "); 
          strcat (str_bot, numbers[minsdigit - 1]); 
        }
      }
      // if mins digit is zero, don't print it. read read "hours" "minstens" e.g. "three /n twenty"
      else if (minsdigitten != 0 && minsdigit == 0  ) {
        strcpy (str_top, numbers[hours - 1]);     
        strcpy (str_bot, numberstens[minsdigitten - 1]);
      }

      //if both mins are zero, i.e. it is on the hour, the top line reads "hours" and bottom line reads "o'clock"
      else if (minsdigitten == 0 && minsdigit == 0  ) {
        strcpy (str_top,numbers[hours - 1]);     
        strcpy (str_bot, "O'Clock");
      }

      //work out offset to center top line on display. 
      byte len = 0;
      while(str_top[len]) { 
        len++; 
      }; //get length of message
      byte offset_top = (X_MAX - ((len - 1)*4)) / 2; //

      //work out offset to center bottom line on display. 
      len = 0;
      while(str_bot[len]) { 
        len++; 
      }; //get length of message
      byte offset_bot = (X_MAX - ((len - 1)*4)) / 2; //

      fade_down();

      //plot hours line
      byte i = 0;
      while(str_top[i]) {
        ht1632_puttinychar((i*4) + offset_top, 2, str_top[i]); 
        i++;
      }

      i = 0;
      while(str_bot[i]) {
        ht1632_puttinychar((i*4) + offset_bot, 9, str_bot[i]); 
        i++;
      }
    }
    delay (50); 
  }
  fade_down();
}

void flash_seconds() {
      //flash led for seconds on arduino
    if ( (rtc[DS1307_SEC] % 2) == 0) { 
      digitalWrite(13,HIGH);
    }
    else{ 
      digitalWrite(13,LOW); 
    }
}

//show time and date and use a random jumble of letters transition each time the time changes.
void jumble() {

  char days[7][4] = {
    "TUE", "WED", "THU", "FRI", "SAT", "SUN","MON"      }; //DS1307 outputs 1-7
  char allchars[37] = {
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"      };
  char endchar[16];
  byte counter[16];
  byte mins = rtc[DS1307_MIN];
  byte seq[16];

  cls();

  for (int show = 0; show < SHOWCLOCK ; show++) {

    gettime();

    flash_seconds();
    
    if (button_checks())
    {
      return;
    }

    if ( show == 0 || mins != rtc[DS1307_MIN]  ) {  

      //fill an arry with 0-15 and randomize the order so we can plot letters in a jumbled pattern rather than sequentially
      for (int i=0; i<16; i++) {
        seq[i] = i;  // fill the array in order
      }
      //randomise array of numbers 
      for (int i=0; i<(16-1); i++) {
        int r = i + (rand() % (16-i)); // Random remaining position.
        int temp = seq[i]; 
        seq[i] = seq[r]; 
        seq[r] = temp;
      }


      //reset these for comparison next time
      mins = rtc[DS1307_MIN];
      byte hours = rtc[DS1307_HR];   
      byte dow   = rtc[DS1307_DOW] - 1; // the DS1307 outputs 1 - 7. 
      byte date  = rtc[DS1307_DATE];

      byte alldone = 0;

      //set counters to 50
      for(byte c=0; c<16 ; c++) {
        counter[c] = 3 + random (0,20);
      }

      //set final characters
      char buffer[3];
      itoa(hours,buffer,10);

      //fix - as otherwise if num has leading zero, e.g. "03" hours, itoa coverts this to chars with space "3 ". 
      if (hours < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }

      endchar[0] = buffer[0];
      endchar[1] = buffer[1];
      endchar[2] = ':';

      itoa (mins, buffer, 10);
      if (mins < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }

      endchar[3] = buffer[0];
      endchar[4] = buffer[1];

      itoa (date, buffer, 10);
      if (date < 10) {
        buffer[1] = buffer[0];
        buffer[0] = '0';
      }

      //then work out date 2 letter suffix - eg st, nd, rd, th etc
      char suffix[4][3]={
        "st", "nd", "rd", "th"                  };
      byte s = 3; 
      if(date == 1 || date == 21 || date == 31) {
        s = 0;
      } 
      else if (date == 2 || date == 22) {
        s = 1;
      } 
      else if (date == 3 || date == 23) {
        s = 2;
      }
      //set topline
      endchar[5] = ' ';
      endchar[6] = ' ';
      endchar[7] = ' ';

      //set bottom line
      endchar[8] = days[dow][0];
      endchar[9] = days[dow][1];
      endchar[10] = days[dow][2];
      endchar[11] = ' ';
      endchar[12] = buffer[0];
      endchar[13] = buffer[1];
      endchar[14] = suffix[s][0];
      endchar[15] = suffix[s][1];

      byte x = 0;
      byte y = 0;

      //until all counters are 0
      while (alldone < 16){

        //for each char    
        for(byte c=0; c<16 ; c++) {

          if (seq[c] < 8) { 
            x = 0;
            y = 0; 
          } 
          else {
            x = 8;
            y = 8;   
          }

          //if counter > 1 then put random char
          if (counter[ seq[c] ] > 1) {
            ht1632_putchar( ( seq[c] -x) * 6, y, allchars[random(0,36)]); //random
            counter[ seq[c] ]--;
          }

          //if counter == 1 then put final char 
          if (counter[ seq[c] ] == 1) {
            ht1632_putchar( (seq[c]-x) * 6, y, endchar[seq[c]]); //final char
            counter[seq[c]] = 0;
            alldone++;
          } 

          //if counter == 0 then just pause to keep update rate the same
          if (counter[seq[c]] == 0) {
            delay(4);
          }

          if(buttonA.uniquePress()){
            switch_mode();
            return;      
          }
        }
      }
    }
    delay(50);
  } //showclock
  fade_down();
}

//print menu to change the mode
void switch_mode() {

  char* modes[] = {
//    "Numbers", "Pong", "Words", "Invader", "Random", "Set Clk"       };
    "Numbers", "Pong", "Words", "Jumble", "Invader", "Random", "Set Clk"       };

  byte next_clock_mode;
  byte firstrun = 1;

  mode_changed = 1; //set this flag so we don't show the date when we get out the menu, we just go straigh to the new mode.

  //loop waiting for button (timeout after X loops to return to mode X)
  for(int count=0; count< 40 ; count++) {

    //if user hits button, change the clock_mode
    if(buttonA.uniquePress() || firstrun == 1){

      count = 0;
      cls();

      if (firstrun == 0) { 
        clock_mode++; 
      } 
      if (clock_mode > MAX_CLOCK_MODE) { 
        clock_mode = 0; 
      }

      //print arrown and current clock_mode name on line one and print next clock_mode name on line two
      char str_top[9];
      char str_bot[9];

      strcpy (str_top, ">");
      strcat (str_top, modes[clock_mode]);

      next_clock_mode = clock_mode + 1;
      if (next_clock_mode > MAX_CLOCK_MODE) { 
        next_clock_mode = 0; 
      }

      strcpy (str_bot, " ");
      strcat (str_bot, modes[next_clock_mode]);

      byte i = 0;
      while(str_top[i]) {
        ht1632_putchar(i*6, 0, str_top[i]); 
        i++;
      }

      i = 0;
      while(str_bot[i]) {
        ht1632_putchar(i*6, 8, str_bot[i]); 
        i++;
      }
      firstrun = 0;
    }
    delay(50); 
  }
  //if random clock_mode set, set next hour to change clock type 
  if (clock_mode == MAX_CLOCK_MODE - 1 ){ 
    random_mode = 1;  
    mode_time_up = rtc[DS1307_HR]; 
    clock_mode = 0;
  } 
  else {
    random_mode = 0;
  } 
}

//set time and date routine
void set_time() {

  cls();

  //fill settings with current clock values read from clock
  gettime();
  byte set_min   = rtc[DS1307_MIN];
  byte set_hr    = rtc[DS1307_HR];
  byte set_dow   = 0;// rtc[3] -1; //the DS1307 outputs 1-7.   /// FIXME!!!!!   xxxxxxxxxxxx
  byte set_date  = rtc[DS1307_DATE];
  byte set_mnth  = rtc[DS1307_MTH];
  byte set_yr    = rtc[DS1307_YR]; //not sure about this   

  //Set function - we pass in: which 'set' message to show at top, current value, reset value, and rollover limit.
  set_min  = set_value(0, set_min, 0, 59);
  set_hr   = set_value(1, set_hr, 0, 23);
  set_dow  = set_value_dow(set_dow);
  set_date = set_value(4, set_date, 1, 31);
  set_mnth = set_value(3, set_mnth, 1, 12);
  set_yr   = set_value(2, set_yr, 10, 99);

  //write the changes to the clock chip
  RTC.stop();
  RTC.set(DS1307_SEC,0);
  RTC.set(DS1307_MIN,set_min);
  RTC.set(DS1307_HR,set_hr);
  RTC.set(DS1307_DOW,(set_dow + 1)); //add one as DS1307 expects va1-7
  RTC.set(DS1307_DATE,set_date);
  RTC.set(DS1307_MTH,set_mnth);
  RTC.set(DS1307_YR,set_yr);
  RTC.start();

  //reset clock mode from 'set clock'
  cls();
  clock_mode = 0;  
}

//used to set min, hr, date, month, year values. pass 
//message = which 'set' message to print, 
//current value = current value of property we are setting
//reset_value = what to reset value to if to rolls over. E.g. mins roll from 60 to 0, months from 12 to 1
//rollover limit = when value rolls over

byte set_value(byte message, byte current_value, byte reset_value, byte rollover_limit){

  cls();
  char messages[6][17]   = {
    "SET MINS", "SET HOUR", "SET YEAR", "SET MNTH", "SET DAY", "DAY NAME"      };

  //Print "set xyz" top line
  byte i = 0;
  while(messages[message][i])
  {
    ht1632_putchar(i*6 , 0, messages[message][i]); 
    i++;
  }

  //print digits bottom line
  char buffer[3];
  itoa(current_value,buffer,10);
  ht1632_putchar(0 , 8, buffer[0]); 
  ht1632_putchar(6 , 8, buffer[1]); 

  delay(300);
  //wait for button input
  while (!buttonA.uniquePress()) {

    while (buttonB.isPressed()){

      if(current_value < rollover_limit) { 
        current_value++;
      } 
      else {
        current_value = reset_value;
      }
      //print the new value
      itoa(current_value, buffer ,10);
      ht1632_putchar(0 , 8, buffer[0]); 
      ht1632_putchar(6 , 8, buffer[1]);
      delay(150);
    }
  }
  return current_value;
}


byte set_value_dow(byte current_value){

  cls();
  char message[9] = {
    "DAY NAME"      };
  char days[7][9] = {
    "Tuesday ", "Weds    ", "Thursday", "Friday  ", "Saturday", "Sunday  ", "Monday  "      };

  //Print "set xyz" top line
  byte i = 0;
  while(message[i])
  {
    ht1632_putchar(i*6 , 0, message[i]); 
    i++;
  }
  i = 0;
  //Print current day set
  while(days[current_value][i]){
    ht1632_putchar(i*6 , 8, days[current_value][i]); 
    i++;
  } 

  //wait for button input
  while (!buttonA.uniquePress()) {
    while (buttonB.isPressed()) {

      if(current_value == 6) { 
        current_value = 0;
      } 
      else {
        current_value++;
      }
      //print the new value 
      i = 0;
      while(days[current_value][i]){
        ht1632_putchar(i*6 , 8, days[current_value][i]); 
        i++;
      }
      delay(150);
    }
  }
  return current_value;
}




