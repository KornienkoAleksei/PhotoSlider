#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "avr/pgmspace.h"					//library for using progmem

//digital output
#define af_on 9
#define shutter 10
#define lin_step 4
#define lin_dir 3
#define rot_step 6
#define rot_dir 5
#define ender 11
#define rot_off 13
//analog input
#define Uakb 7
#define camera_shot 0

//buttons
#define BUTTON_NONE 0		//no button pressed
#define BUTTON_RIGHT 7		//right button pressed
#define BUTTON_LEFT 2		//left button pressed        
#define BUTTON_BACK 12		//back button pressed           
#define BUTTON_SELECT 8		//select button pressed     

#define bt_delay_long 250	//delay short press
#define bt_delay_short 50	//delay between long press
#define bt_noise 20			//button noise

//slider lenght in driver steps
#define lenght_step_all 4415

#define linear_gear 49 //number of teeth in linear gear = steps number per revolution(steps driver) * 10 / (teeth number linear gear * tooth lenght (mm))
#define rotation_gear 2.833 //number of teeth in reduction: steps number per revolution(steps driver) * reduction rates / 3600


// define for draw
#define draw_0 4 //label start position
#define draw_1 4
#define draw_2_dir 1
#define draw_2_len 7
#define draw_3 1
#define draw_4_int 1
#define draw_4_time 10
#define draw_5 3
#define draw_6 5

#define draw_st_current_frame 0
#define draw_st_current_time 0
#define draw_st_all_time 10

//battery settings
#define akb_1 323
#define akb_2 357
#define akb_3 425
#define akb_4 459

#define ender_dead_zone 20 //ender rollback limit



LiquidCrystal_I2C lcd(0x3F, 16, 2);  //my display address is 0x3F for model 0x27

//Menu
const char MenuName_0[] PROGMEM = "    master      ";
const char MenuName_1[] PROGMEM = "    frames      ";
const char MenuName_2[] PROGMEM = "    lenght      ";
const char MenuName_3[] PROGMEM = "   rotation     ";
const char MenuName_4[] PROGMEM = "   interval     ";
const char MenuName_5[] PROGMEM = "     delay      ";
const char MenuName_6[] PROGMEM = "      home      ";
const char MenuName_7[] PROGMEM = "   test slide   ";
const char MenuName_8[] PROGMEM = "      run       ";
const char MenuName_9[] PROGMEM = "                ";
const char MenuName_10[] PROGMEM = "END press key :)";

const char* const MenuNames[] PROGMEM = { MenuName_0, MenuName_1, MenuName_2, MenuName_3, MenuName_4,
MenuName_5, MenuName_6, MenuName_7, MenuName_8, MenuName_9, MenuName_10};

//command description slider_mode
/*	* 1 - timelapse master Camera
	* 2 - timelapse master Slider
	* 3 - timelapse master offline
	*/

const char MenuNameSec_0[] PROGMEM = "        ";
const char MenuNameSec_1[] PROGMEM = "camera  ";
const char MenuNameSec_2[] PROGMEM = "slider  ";
const char MenuNameSec_3[] PROGMEM = "offline ";
const char MenuNameSec_4[] PROGMEM = " full   ";
const char MenuNameSec_5[] PROGMEM = "     . \xdf";
const char MenuNameSec_6[] PROGMEM = " h  m   ";
const char MenuNameSec_7[] PROGMEM = "sec     ";
const char MenuNameSec_8[] PROGMEM = " ok?    ";

const char* const MenuNamesSec[] PROGMEM = { MenuNameSec_0, MenuNameSec_1, MenuNameSec_2, MenuNameSec_3,
MenuNameSec_4, MenuNameSec_5, MenuNameSec_6, MenuNameSec_7, MenuNameSec_8 };

char menubuffer[17]; //buffer to avoid memory leaks
char menubuffersec[9]; //buffer to avoid memory leaks

byte MenuNowPos = 0; //current menu position 0 element is dummy in nature
boolean MenuEdit = false; //edit menu:false - view; true - edit
boolean MenuEdit1 = false; // submenu editing

byte MenuEditPos = 0; //position in character when editing
byte MenuDrawPos; //menu drawing position

//new symbols for displaying battery level
byte Akb5[8] = //first symbol HIGH
{
	B00110,
	B01111,
	B01111,
	B01111,
	B01111,
	B01111,
	B01111,
	B00000
};
byte Akb4[8] = //second symbol HIGH
{
	B00110,
	B01111,
	B01001,
	B01111,
	B01111,
	B01111,
	B01111,
	B00000
};
byte Akb3[8] = //third symbol HIGH
{
	B00110,
	B01111,
	B01001,
	B01001,
	B01111,
	B01111,
	B01111,
	B00000
};
byte Akb2[8] = //first symbol LOW
{
	B00110,
	B01111,
	B01001,
	B01001,
	B01001,
	B01111,
	B01111,
	B00000
};
byte Akb1[8] = //second symbol LOW
{
	B00110,
	B01111,
	B01001,
	B01001,
	B01001,
	B01001,
	B01111,
	B00000
};
byte sel[8] =  // select
{
	B10000,
	B11000,
	B11100,
	B11110,
	B11110,
	B11100,
	B11000,
	B10000
};

//movement variables
int all_frames; //target number of frames
int current_frame; //current frame

int lin_speed_delay = 3500; //delays between linear drive steps
int rot_speed_delay = 2000;	//delays between rotation drive steps

boolean full_lenght = true; //priority long trips, when true is maximum length
unsigned int lenght_step; //steps per frame

int slide_lenght; //slide length in steps
boolean lenght_direction; //direction of movement: thue - from step driver, false - to step driver
int Start_Delay = 1; //start delay up to 60 sec.

int rotation_step = 0; //target turn in steps
int rotation_step_home;
boolean rotation_direction = true; //direction of rotation 

boolean end_rot_step;				//end of rotation for each frame
boolean end_lin_step;				//end of movement for each frame

unsigned int rot_current_step; //current rotation step for step drive
unsigned int lin_current_step; //current linear step for step drive

//steps timers
unsigned long timer_lin;
unsigned long timer_rot;
unsigned long timer_step;

//frames timers
unsigned long timer_interval;
boolean rot_step_HIGH; //output state
boolean lin_step_HIGH; //output state


unsigned long interval_long;

boolean shot_end; //shutter flag
boolean ender_stop; //ender flag

//menu drawing
int all_lenght; //sliding length target for menu
int all_rotation; //rotation target for menu (deg.)

int interval; //delay between frames
int slider_mode; //working mode

int temp_1; //temporary variable 1
int temp_2; //temporary variable 2
unsigned long long_temp1; //temporary variable 1 long
unsigned long long_temp2; //temporary variable 2 long

int Uslide; //battery voltage

//variables for buttons
boolean press_left = false; //flag for button press
boolean press_right = false; //flag for button press
boolean press_select = false; //flag for button press
boolean press_back = false; //flag for button press
boolean press_left_prev = false; //flag for button press
boolean press_right_prev = false; //flag for button press
boolean press_select_prev = false; //flag for button press
boolean press_back_prev = false; //flag for button press
boolean bt_timer_1 = false; //delay first push
unsigned long timer_1 = 0; //buttons timer

//button click detection
byte getPressedButton() {
	//digitalread + flag arrangement
	if (!digitalRead(BUTTON_RIGHT))
	{
		press_right = false;
		press_right_prev = false;
	}	else
	{
		delay(bt_noise);
		if (digitalRead(BUTTON_RIGHT))
		{
			press_right = true;
		}
	}
	if (!digitalRead(BUTTON_LEFT))
	{
		press_left = false;
		press_left_prev = false;
	} else
	{
		delay(bt_noise);
		if (digitalRead(BUTTON_LEFT))
		{
			press_left = true;
		}
	}
	if (!digitalRead(BUTTON_BACK))
	{
		press_back = false;
		press_back_prev = false;
	} else
	{
		delay(bt_noise);
		if (digitalRead(BUTTON_BACK))
		{
			press_back = true;
		}
	}
	if (!digitalRead(BUTTON_SELECT))
	{
		press_select = false;
		press_select_prev = false;
	} else
	{
		delay(bt_noise);
		if (digitalRead(BUTTON_SELECT))
		{
			press_select = true;
		}
	}
	//reset timer when no button is pressed
	if (!press_back && !press_left && !press_right && !press_select)
	{
		bt_timer_1 = false;
	}
	//description of each button
	if (press_right)
	{
		if (!press_right_prev)
		{
			bt_timer_1 = true;
			timer_1 = millis();
			press_right_prev = true;
			return BUTTON_RIGHT;
		}
		else
		{
			if (bt_timer_1 && (millis() - timer_1)> bt_delay_long)
			{
				bt_timer_1 = false;
				timer_1 = millis();
				return BUTTON_RIGHT;
			}
			if (!bt_timer_1 && (millis() - timer_1) > bt_delay_short)
			{
				timer_1 = millis();
				return BUTTON_RIGHT;
			}
		}
	}
	if (press_left)
	{
		if (!press_left_prev)
		{
			bt_timer_1 = true;
			timer_1 = millis();
			press_left_prev = true;
			return BUTTON_LEFT;
		}
		else
		{
			if (bt_timer_1 && (millis() - timer_1) > bt_delay_long)
			{
				bt_timer_1 = false;
				timer_1 = millis();
				return BUTTON_LEFT;
			}
			if (!bt_timer_1 && (millis() - timer_1) > bt_delay_short)
			{
				timer_1 = millis();
				return BUTTON_LEFT;
			}
		}
	}
	if (press_back)
	{
		if (!press_back_prev)
		{
			press_back_prev = true;
			return BUTTON_BACK;
		}
	}
	if (press_select)
	{
		if (!press_select_prev)
		{
			press_select_prev = true;
			return BUTTON_SELECT;
		}
	}
	return BUTTON_NONE;
}

//battery status drawing
void DrawAkb() {
	Uslide = analogRead(Uakb);
	lcd.setCursor(15, 0);
	if (Uslide < akb_1)
	{
		lcd.print("\1");
	}
	else if (Uslide < akb_2)
	{
		lcd.print("\2");
	}
	else if (Uslide < akb_3)
	{
		lcd.print("\3");
	}
	else if (Uslide < akb_4)
	{
		lcd.print("\4");
	}
	else 
	{
		lcd.print("\5");
	}
}

//drawing 16 characters from memory: cursor, string, label
void DrawProgmem16(byte pg_cursor16, byte pg_string16, byte pg_sumbl16) {
	lcd.setCursor(pg_cursor16, pg_string16);
	strcpy_P(menubuffer, (char*)pgm_read_word(&(MenuNames[pg_sumbl16])));
	lcd.print(menubuffer);
}

//drawing 8 characters from memory: cursor, string, label
void DrawProgmem8(byte pg_cursor8, byte pg_string8, byte pg_sumbl8) {
	lcd.setCursor(pg_cursor8, pg_string8);
	strcpy_P(menubuffersec, (char*)pgm_read_word(&(MenuNamesSec[pg_sumbl8])));
	lcd.print(menubuffersec);
}

//drawing time: cursor, string, time
void DrawTime_H(byte dt_cursor, byte dt_string, unsigned long dt_time) {
	DrawProgmem8((dt_cursor+1), dt_string, 6);
	if (dt_time < 600)
	{
		lcd.setCursor((dt_cursor + 4), dt_string);
		lcd.print(dt_time / 60);
		lcd.setCursor((dt_cursor + 2), dt_string);
		lcd.print(" ");
	}
	else if (dt_time < 3600)
	{
		lcd.setCursor((dt_cursor + 3), dt_string);
		lcd.print(dt_time / 60);
		lcd.setCursor((dt_cursor + 2), dt_string);
		lcd.print(" ");
	}
	else if (dt_time < 36000)
	{
		lcd.setCursor((dt_cursor + 1), dt_string);
		lcd.print(dt_time / 3600);
		if ((dt_time % 3600) / 60 < 10) { lcd.setCursor((dt_cursor + 4), dt_string); }
		else { lcd.setCursor((dt_cursor + 3), dt_string); }
		lcd.print((dt_time % 3600) / 60);
	}
	else if (dt_time < 360000)
	{
		lcd.setCursor((dt_cursor), dt_string);
		lcd.print(dt_time / 3600);
		if ((dt_time % 3600) / 60 < 10) { lcd.setCursor((dt_cursor + 4), dt_string); }
		else { lcd.setCursor((dt_cursor + 3), dt_string); }
		lcd.print((dt_time % 3600) / 60);
	}
	else
	{
		lcd.setCursor(dt_cursor, dt_string);
		lcd.print("--/--");
	}
}

//line cleaning
void DrawMenuClean(byte MenuCleanString ) {
	lcd.setCursor(0, MenuCleanString);
	strcpy_P(menubuffer, (char*)pgm_read_word(&(MenuNames[9])));
	lcd.print(menubuffer);
}

void DrawMenu() {
	//browsing/scrolling through the menu
	if (!MenuEdit)
	{
		DrawProgmem16(0, 0, MenuNowPos);
		DrawMenuClean(1);
		//master
		if (MenuNowPos == 0)
		{
			DrawProgmem8(draw_0, 1, slider_mode);
		}
		// frames
		if (MenuNowPos == 1)
		{
			if (all_frames < 10) 
			{
				lcd.setCursor(draw_1, 1);
				lcd.print("   ");
				lcd.setCursor((draw_1 + 3), 1); 
			}
			else if (all_frames < 100) { 
				lcd.setCursor(draw_1, 1);
				lcd.print("  ");
				lcd.setCursor((draw_1 + 2), 1); 
			}
			else if (all_frames < 1000) { 
				lcd.setCursor(draw_1, 1);
				lcd.print(" "); 
				lcd.setCursor((draw_1 + 1), 1); 
			}
			else if (all_frames < 10000) {  lcd.setCursor(draw_1, 1); 			}
			else { lcd.setCursor((draw_1 - 1), 1); }
			lcd.print(all_frames);
		}
		//lenght
		if (MenuNowPos == 2)
		{
			lcd.setCursor(draw_2_dir, 1);
			if (!lenght_direction) { lcd.print("\x7e\x7e\x7e"); }
			else { lcd.print("\x7f\x7f\x7f"); }
			if (!full_lenght) {
				all_lenght = slide_lenght / linear_gear;
				lcd.setCursor(draw_2_len, 1);
				lcd.print(all_lenght);
			}
			if (full_lenght) { DrawProgmem8(draw_2_len, 1, 4); }
		}
		//rotation
		if (MenuNowPos == 3)
		{
			DrawProgmem8(draw_3, 1, 5);
			long_temp2 = all_frames * rotation_step / rotation_gear;
			
			if (long_temp2 < 100) {
				if (!rotation_direction)
				{
					lcd.setCursor((draw_3 + 3), 1);
					lcd.print("\x7f");
				}
				else
				{
					lcd.setCursor((draw_3 + 3), 1);
					lcd.print("\x7e");
				}
				lcd.setCursor((draw_3 + 4), 1); 
			}
			else if (long_temp2 < 1000) { 
				if (!rotation_direction)
				{
					lcd.setCursor((draw_3 + 2), 1);
					lcd.print("\x7f");
				}
				else
				{
					lcd.setCursor((draw_3 + 2), 1);
					lcd.print("\x7e");
				}
				lcd.setCursor((draw_3 + 3), 1);
			}
			else if (long_temp2 >= 1000) { 
				if (!rotation_direction)
				{
					lcd.setCursor((draw_3 + 1), 1);
					lcd.print("\x7f");
				}
				else
				{
					lcd.setCursor((draw_3 + 1), 1);
					lcd.print("\x7e");
				}
				lcd.setCursor((draw_3 + 2), 1);
			}
			lcd.print(long_temp2 / 10);
			lcd.setCursor((draw_3 + 6), 1);
			lcd.print(long_temp2 % 10);
		}
		//interval
		if (MenuNowPos == 4)
		{
			lcd.setCursor(draw_4_int, 1);
			lcd.print(interval);
			if (interval < 10)
			{
				lcd.setCursor((draw_4_int+1), 1);
			} else if (interval < 100)
			{
				lcd.setCursor((draw_4_int + 2), 1);
			} else 	{
				lcd.setCursor((draw_4_int + 3), 1);
			}
			lcd.print("sec");
			long_temp1 = interval * all_frames;
			DrawTime_H(draw_4_time, 1, long_temp1);
		}
		//delay
		if (MenuNowPos == 5)
		{
			if (Start_Delay < 10){lcd.setCursor((draw_5 + 2), 1);}
			else if (Start_Delay < 100)			{				lcd.setCursor((draw_5 + 1), 1);			}
			else {				lcd.setCursor((draw_5 + 0), 1);			}
			lcd.print(Start_Delay);
			DrawProgmem8((draw_5 + 4), 1, 7);
		}
		DrawAkb();
	}
	if (MenuEdit)
	{
		if (MenuNowPos == 0)
		{
			DrawProgmem8(draw_0, 1, slider_mode);
		}
		//frames
		if (MenuNowPos == 1)
		{
			if (MenuEdit) {
				if (all_frames < 10)
				{
					lcd.setCursor(draw_1, 1);
					lcd.print("   ");
					lcd.setCursor((draw_1 + 3), 1);
				}
				else if (all_frames < 100) {
					lcd.setCursor(draw_1, 1);
					lcd.print("  ");
					lcd.setCursor((draw_1 + 2), 1);
				}
				else if (all_frames < 1000) {
					lcd.setCursor(draw_1, 1);
					lcd.print(" ");
					lcd.setCursor((draw_1 + 1), 1);
				}
				else if (all_frames < 10000) { lcd.setCursor(draw_1, 1); }
				else { lcd.setCursor((draw_1 - 1), 1); }
				lcd.print(all_frames);
				lcd.setCursor((draw_1 + 3 - MenuEditPos), 1);
			}
		}
		//lenght
		if (MenuNowPos == 2) {
				if (MenuEditPos == 1)
				{
					lcd.setCursor(draw_2_dir, 1);
					if (!lenght_direction) { lcd.print("\x7e\x7e\x7e"); }
					else { lcd.print("\x7f\x7f\x7f"); }
					lcd.setCursor((draw_2_dir-1), 1);
				}
				if (MenuEditPos == 2)
				{
					if (!full_lenght) {
						all_lenght = slide_lenght / linear_gear;
						DrawProgmem8(draw_2_len, 1, 0);
						lcd.setCursor(draw_2_len, 1);
						lcd.print(all_lenght);
					}
					if (full_lenght) { DrawProgmem8(draw_2_len, 1, 4); }
					lcd.setCursor((draw_2_len-1), 1);
				}

		}
		//rotation
		if (MenuNowPos == 3)
		{
			long_temp2 = all_frames * rotation_step / rotation_gear;

			if (long_temp2 < 100) {
				lcd.setCursor((draw_3 + 1), 1);
				lcd.print("   ");
				if (!rotation_direction)
				{
					lcd.setCursor((draw_3 + 3), 1);
					lcd.print("\x7f");
				}
				else
				{
					lcd.setCursor((draw_3 + 3), 1);
					lcd.print("\x7e");
				}
				lcd.setCursor((draw_3 + 4), 1);
			}
			else if (long_temp2 < 1000) {
				lcd.setCursor((draw_3 + 1), 1);
				lcd.print(" ");
				if (!rotation_direction)
				{
					lcd.setCursor((draw_3 + 2), 1);
					lcd.print("\x7f");
				}
				else
				{
					lcd.setCursor((draw_3 + 2), 1);
					lcd.print("\x7e");
				}
				lcd.setCursor((draw_3 + 3), 1);
			}
			else if (long_temp2 >= 1000) {
				if (!rotation_direction)
				{
					lcd.setCursor((draw_3 + 1), 1);
					lcd.print("\x7f");
				}
				else
				{
					lcd.setCursor((draw_3 + 1), 1);
					lcd.print("\x7e");
				}
				lcd.setCursor((draw_3 + 2), 1);
			}
			lcd.print(long_temp2 / 10);
			lcd.setCursor((draw_3 + 6), 1);
			lcd.print(long_temp2 % 10);
		}
		//interval
		if (MenuNowPos == 4)
		{
			lcd.setCursor(draw_4_int, 1);
			lcd.print(interval);
			if (interval < 10)
			{
				lcd.setCursor((draw_4_int + 1), 1);
			}
			else if (interval < 100)
			{
				lcd.setCursor((draw_4_int + 2), 1);
			}
			else {
				lcd.setCursor((draw_4_int + 3), 1);
			}
			lcd.print("sec");
			long_temp1 = interval * all_frames;
			DrawTime_H(draw_4_time, 1, long_temp1);
		}
		//delay
		if (MenuNowPos == 5)
		{
			lcd.setCursor(draw_5, 1);
			lcd.print("   ");
			if (Start_Delay < 10) { lcd.setCursor((draw_5 + 2), 1); }
			else if (Start_Delay < 100) { lcd.setCursor((draw_5 + 1), 1); }
			else { lcd.setCursor((draw_5 + 0), 1); }
			lcd.print(Start_Delay);
		}
		// home = test slide = run
		if (MenuNowPos == 6 || MenuNowPos == 7 || MenuNowPos == 8)		
		{			
			DrawProgmem8(draw_6, 1, 8);		
		}
	}
}

void DrawState() {
	DrawMenuClean(0);
	DrawMenuClean(1);
	DrawAkb();
	lcd.setCursor(draw_st_current_frame, 0);
	lcd.print(current_frame);
	lcd.print("/");
	lcd.print(all_frames);
	long_temp1 = interval * current_frame;
	DrawTime_H(draw_st_current_time, 1, long_temp1);
	long_temp1 = interval * all_frames;
	DrawTime_H(draw_st_all_time, 1, long_temp1);
}

void run_right() {
	if (!MenuEdit)
	{
		if (MenuNowPos == 8)
		{
			MenuNowPos = 0;
		}
		else if ((slider_mode == 2 && MenuNowPos == 4) || ((slider_mode == 1 && MenuNowPos == 3)))
		{
			MenuNowPos = 6;
		}
		else
		{
			MenuNowPos++;
		}
	}
	else
	{
		if (MenuNowPos == 0)
		{
			if (slider_mode == 3)
			{
				slider_mode = 1;
			}
			else
			{
				slider_mode++;
			}
		}
		if (MenuNowPos == 1)
		{
			if (!MenuEdit1)
			{
				if (MenuEditPos > 0)
				{
					MenuEditPos--;
				}
				else
				{
					MenuEditPos = 0;
				}
			}
			else
			{
				all_frames += pow(10, MenuEditPos);
			}
		}
		if (MenuNowPos == 2)
		{
			if (!MenuEdit1)
			{
				MenuEditPos = 2;
				lcd.setCursor((draw_2_dir - 1), 1);
				lcd.print(" ");
				lcd.setCursor((draw_2_len - 1), 1);
				lcd.print("\6");
			}
			else
			{
				if (MenuEditPos == 1)
				{
					lenght_direction = false;
				}
				if (MenuEditPos == 2)
				{
					if (!full_lenght)
					{
						lenght_step++;
						slide_lenght = lenght_step * all_frames;
						if (slide_lenght> lenght_step_all)
						{
							full_lenght = true;
							slide_lenght = lenght_step_all;
							lenght_step--;
						}
					}
				}
			}
		}
		if (MenuNowPos == 3)
		{
			if (rotation_direction)
			{
				rotation_step++;
				long_temp2 = all_frames * rotation_step / rotation_gear;
				if (long_temp2 >=10000)
				{
					rotation_step--;
				}
			}
			else
			{
				if (rotation_step < 0)
				{
					rotation_step = 0;
					rotation_direction = true;
				}
				else if (rotation_step > 0)
				{
					rotation_step--;
				}
				else
				{
					rotation_step = 1;
					rotation_direction = true;
				}
			}
		}
		if (MenuNowPos == 4)
		{
			interval++;
		}
		if (MenuNowPos == 5)
		{
			Start_Delay++;
		}
	}
	DrawMenu();
}

void run_left() {
	if (!MenuEdit)
	{
		if (MenuNowPos == 0)
		{
			MenuNowPos = 8;
		}
		else if ( MenuNowPos == 6 && slider_mode == 2 )
		{
			MenuNowPos = 4;
		}
		else if (MenuNowPos == 6 && slider_mode == 1)
		{
			MenuNowPos = 3;
		}
		else
		{
			MenuNowPos--;
		}
	}
	else
	{
		if (MenuNowPos == 0)
		{
			if (slider_mode == 1)
			{
				slider_mode = 3;
			}
			else
			{
				slider_mode--;
			}
		}
		if (MenuNowPos == 1)
		{
			if (!MenuEdit1)
			{
				if (MenuEditPos < 3)
				{
					MenuEditPos++;
				}
				else
				{
					MenuEditPos = 3;
				}
			}
			else
			{
				if ((all_frames/ pow(10, MenuEditPos))>= 1)
				{
					all_frames -= pow(10, MenuEditPos);
				}
			}

		}
		if (MenuNowPos == 2)
		{
			if (!MenuEdit1)
			{
				MenuEditPos = 1;
				lcd.setCursor((draw_2_dir - 1), 1);
				lcd.print("\6");
				lcd.setCursor((draw_2_len - 1), 1);
				lcd.print(" ");
			}
			else
			{
				if (MenuEditPos == 1)
				{
					lenght_direction = true;
				}
				if (MenuEditPos == 2)
				{
					if (full_lenght)
					{
						lenght_step = lenght_step_all /all_frames;
						lenght_step--;
						slide_lenght = lenght_step * all_frames;
						full_lenght = false;
					}
					else if (lenght_step > 1)
					{
						lenght_step--;
						slide_lenght = lenght_step * all_frames;
					}
					else
					{
						lenght_step = 0;
						slide_lenght = 0;
					}

				}
			}
		}
		if (MenuNowPos == 3)
		{
			if (!rotation_direction)
			{
				rotation_step++;
				long_temp2 = all_frames * rotation_step / rotation_gear;
				if (long_temp2 >= 10000)
				{
					rotation_step--;
				}
			}
			else
			{
				if (rotation_step < 0)
				{
					rotation_step = 0;
					rotation_direction = false;
				}
				else if (rotation_step > 0)
				{
					rotation_step--;
				}
				else
				{
					rotation_step = 1;
					rotation_direction = false;
				}
			}
		}
		if (MenuNowPos == 4)
		{
			if (interval>1)
			{
				interval--;
			}
			else
			{
				interval = 1;
			}
		}
		if (MenuNowPos == 5)
		{
			if (Start_Delay > 1)
			{
				Start_Delay--;
			}
			else
			{
				Start_Delay = 1;
			}
		}

	}
	DrawMenu();
}

void run_back() {
	if (MenuNowPos == 0)
	{
		if (MenuEdit)
		{
			MenuEdit = false;
			lcd.setCursor((draw_0 - 1), 1);
			lcd.print(" ");
		}
	}
	if (MenuNowPos == 1)
	{
		if (MenuEdit1)
		{
			MenuEdit1 = false;
			MenuEdit = true;
			lcd.cursor_off();
			lcd.blink_on();
		}
		else if (MenuEdit)
		{
			MenuEdit = false;
			lcd.blink_off();
			//calculation of slip parameters
			if (full_lenght)
			{
				lenght_step = lenght_step_all / all_frames;
			}
			else
			{
				lenght_step = slide_lenght / all_frames;
			}

		}
	}
	if (MenuNowPos == 2)
	{
		if (MenuEdit1)
		{
			MenuEdit1 = false;
			MenuEdit = true;
			lcd.blink_off();
		}
		else if (MenuEdit)
		{
			MenuEdit = false;
		}

	}
	if (MenuNowPos == 3)
	{
		if (MenuEdit)
		{
			MenuEdit = false;
			lcd.setCursor((draw_3 - 1), 1);
			lcd.print(" ");
		}
	}
	if (MenuNowPos == 4)
	{
		if (MenuEdit)
		{
			MenuEdit = false;
			lcd.setCursor((draw_4_int - 1), 1);
			lcd.print(" ");
		}
	}
	if (MenuNowPos == 5)
	{
		if (MenuEdit)
		{
			MenuEdit = false;
			lcd.setCursor((draw_5 - 1), 1);
			lcd.print(" ");
		}
	}
	if (MenuNowPos == 6 || MenuNowPos == 7 || MenuNowPos == 8)
	{
		if (MenuEdit) {
			MenuEdit = false;
		}
	}
	DrawMenu();
}

void run_select() {
	if (MenuNowPos == 0)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			lcd.setCursor((draw_0 - 1), 1);
			lcd.print("\6");
		}
	}
	if (MenuNowPos == 1)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			MenuEdit1 = false;
			lcd.setCursor((draw_1 - 1), 1);
			lcd.print("\6");
			MenuEditPos = 1;
			lcd.blink_on();
		}
		else
		{
			if (!MenuEdit1)
			{
				MenuEdit1 = true;
				lcd.blink_off();
				lcd.cursor_on();
			}
		}
	}
	if (MenuNowPos == 2)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			MenuEdit1 = false;
			MenuEditPos = 1;
			lcd.setCursor((draw_2_dir - 1), 1);
			lcd.print("\6");
		}
		else
		{
			MenuEdit1 = true;
			lcd.blink_on();
		}
	}
	if (MenuNowPos == 3)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			lcd.setCursor((draw_3), 1);
			lcd.print("\6");
		}

	}
	if (MenuNowPos == 4)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			lcd.setCursor((draw_4_int-1), 1);
			lcd.print("\6");
		}
	}
	if (MenuNowPos == 5)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			lcd.setCursor((draw_5 - 1), 1);
			lcd.print("\6");
		}
	}
	if (MenuNowPos == 6)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			MenuEdit1 = false;
		}
		else if (!MenuEdit1)
		{
			MenuEdit = false;
			MenuEdit1 = false;
			auto_home();
		}

	}
	if (MenuNowPos == 7)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			MenuEdit1 = false;
		}
		else if (!MenuEdit1)
		{
			MenuEdit = false;
			MenuEdit1 = false;
			test_slide();
		}

	}
	if (MenuNowPos == 8)
	{
		if (!MenuEdit)
		{
			MenuEdit = true;
			MenuEdit1 = false;
		}
		else if (!MenuEdit1)
		{
			MenuEdit = false;
			MenuEdit1 = false;
			if (slider_mode == 1)			{				slide_mode_1();			}
			if (slider_mode == 2)			{				slide_mode_2();			}
			if (slider_mode == 3)			{				slide_mode_3();			}
		}
	}
	DrawMenu();
}

void frame() {
	digitalWrite(af_on, HIGH);
	delay(250);
	digitalWrite(shutter, HIGH);
	delay(250);
	digitalWrite(shutter, LOW);
	delay(50);
	digitalWrite(af_on, LOW);
}

void step_drive() {
	//direction setting
	digitalWrite(rot_dir, !rotation_direction ? HIGH : LOW);
	digitalWrite(lin_dir, lenght_direction ? HIGH : LOW);
	digitalWrite(rot_off, LOW);
	delayMicroseconds(5);
	end_lin_step = false;
	end_rot_step = false;
	timer_lin = micros();
	timer_rot = micros();
	lin_current_step = 0;
	rot_current_step = 0;
	do
	{
		timer_step = micros();
		if (timer_step < lin_speed_delay || timer_step < rot_speed_delay)
		{
			timer_lin = 1;
			timer_rot = 1;
		}

		if (!ender_stop)
		{
			if (digitalRead(ender))
			{
				lenght_direction = !lenght_direction;
				digitalWrite(lin_dir, lenght_direction ? HIGH : LOW);
				delayMicroseconds(50);
				timer_step = 0;
				while (timer_step < ender_dead_zone)
				{
					delayMicroseconds(lin_speed_delay);
					digitalWrite(lin_step, HIGH);
					delayMicroseconds(lin_speed_delay);
					digitalWrite(lin_step, LOW);
					timer_step++;
				}
				lenght_direction = !lenght_direction;
				digitalWrite(lin_dir, lenght_direction ? HIGH : LOW);
				ender_stop = true;
				end_lin_step = true;
			} 
			else if (!end_lin_step)
			{
				if ((timer_step - timer_lin) > lin_speed_delay)
				{
					if (!lin_step_HIGH)
					{
						digitalWrite(lin_step, HIGH);
						lin_step_HIGH = true;
						timer_lin = micros();
					}
					else
					{
						digitalWrite(lin_step, LOW);
						lin_step_HIGH = false;
						lin_current_step++;
						timer_lin = micros();
					}
				}
				if (lin_current_step >= lenght_step)
				{
					end_lin_step = true;
				}
			}
		}
		else
		{
			end_lin_step = true;
		}
		if (!end_rot_step)
		{
			if ((timer_step - timer_rot) > rot_speed_delay)
			{
				if (!rot_step_HIGH)
				{
					digitalWrite(rot_step, HIGH);
					rot_step_HIGH = true;
					timer_rot = micros();
				}
				else
				{
					digitalWrite(rot_step, LOW);
					rot_step_HIGH = false;
					rot_current_step++;
					timer_rot = micros();
				}
			}
			if (rot_current_step >= rotation_step)
			{
				end_rot_step = true;
			}
		}
	} while (!end_lin_step || !end_rot_step);
	digitalWrite(rot_off, HIGH);
}

void auto_home() {
	DrawProgmem16(0, 1, 8);
	rotation_step_home = rotation_step;
	lenght_step = lenght_step_all;
	rotation_step = 0;
	lenght_direction = !lenght_direction;
	step_drive();
	rotation_step = rotation_step_home;
	lenght_direction = !lenght_direction;
	if (full_lenght) { lenght_step = lenght_step_all / all_frames;	}
	else { lenght_step = slide_lenght / all_frames;	}
	ender_stop = false;
}

void test_slide() {
	DrawProgmem16(0, 1, 8);
	//current_frame
	if (full_lenght) { lenght_step = lenght_step_all / all_frames; }
	else { lenght_step = slide_lenght / all_frames; }
	long_temp2 = lenght_step * lin_speed_delay / rotation_step;
	if (long_temp2 < 1000)
	{
		rot_speed_delay = 1000;
		long_temp2 = rotation_step * rot_speed_delay / lenght_step;
		if (long_temp2 > 1500)
		{
			if (long_temp2 < 2000)
			{
				lin_speed_delay = long_temp2;
			}
			else
			{
				lin_speed_delay = 2000;
			}
		}
		else
		{
			lin_speed_delay = 1500;
		}
	}
	else
	{
		rot_speed_delay = long_temp2;
	}
	for (current_frame = 0; current_frame < all_frames; current_frame++)
	{
		step_drive();
	}
	ender_stop = false;
	delay(5000);
	lenght_direction = !lenght_direction;
	rotation_direction = !rotation_direction;
	for (current_frame = 0; current_frame < all_frames; current_frame++)
	{
		step_drive();
	}
	lenght_direction = !lenght_direction;
	rotation_direction = !rotation_direction;
	lin_speed_delay = 1750; //delays between steps
	rot_speed_delay = 1750;
}
 
//timelapse camera
void slide_mode_1() {
	DrawState();
	if (full_lenght) { lenght_step = lenght_step_all / all_frames; }
	else { lenght_step = slide_lenght / all_frames; }
	long_temp2 = lenght_step * lin_speed_delay / rotation_step;
	if (long_temp2 < 1000)
	{
		rot_speed_delay = 1000;
	}
	else
	{
		rot_speed_delay = long_temp2;
	}
	current_frame = 1;
	while (current_frame < all_frames)
	{
		Serial.println("1");
		shot_end = false;
		//waiting for voltage drop
		while (!shot_end)
		{
			Serial.println(analogRead(camera_shot));
			if (analogRead(camera_shot) < 300)
			{
				delay(50);
				Serial.println("3");
				if (analogRead(camera_shot) < 300)
				{
					Serial.println("4");
					shot_end = true;
				}
			}
		}
		delay(100);
		shot_end = false;
		//waiting for voltage increase
		while (!shot_end)
		{
			if (analogRead(camera_shot) > 400)
			{
				delay(50);
				if (analogRead(camera_shot) > 400)
				{
					shot_end = true;
				}
			}
		}
		delay(500);
		step_drive();
		if (ender_stop) { lenght_step = 0; }
		current_frame++;
		DrawState();
	}
	DrawProgmem16(0, 0, 9);
	DrawProgmem16(0, 1, 10);
	while (getPressedButton() == BUTTON_NONE)	{ delay(5); }
}

//timelapse slider
void slide_mode_2() {
	DrawState();
	if (full_lenght) { lenght_step = lenght_step_all / all_frames; }
	else { lenght_step = slide_lenght / all_frames; }
	long_temp2 = lenght_step * lin_speed_delay / rotation_step;
	if (long_temp2 < 1000)
	{
		rot_speed_delay = 1000;
	}
	else
	{
		rot_speed_delay = long_temp2;
	}
	current_frame = 1;
	interval_long = interval * 1000;
	delay(5000);
	timer_interval = millis();
	frame();
	while ((millis() - timer_interval) < interval_long)
	{
		delay(50);
	}
	while (current_frame < all_frames)
	{
		timer_interval = millis();
		step_drive();
		if (ender_stop) { lenght_step = 0; } //stop on trailer
		delay(1000);
		frame();
		current_frame++;
		DrawState();
		while ((millis() - timer_interval) < interval_long)
		{
			delay(50);
		}
	}
	DrawProgmem16(0, 0, 9);
	DrawProgmem16(0, 1, 10);
	while (getPressedButton() == BUTTON_NONE) { delay(5); }
}

//timelapse offline
void slide_mode_3() {
	DrawState();
	interval_long = Start_Delay * 1000;
	timer_interval = millis();
	while ((millis() - timer_interval) < interval_long)
	{
		delay(5);
	}
	if (full_lenght) { lenght_step = lenght_step_all / all_frames; }
	else { lenght_step = slide_lenght / all_frames; }
	long_temp2 = lenght_step * lin_speed_delay / rotation_step;
	if (long_temp2 < 1000)
	{
		rot_speed_delay = 1000;
	}
	else
	{
		rot_speed_delay = long_temp2;
	}
	current_frame = 1;
	interval_long = interval * 1000;
	while (current_frame < all_frames)
	{
		timer_interval = millis();
		step_drive();
		if (ender_stop) { lenght_step = 0; } //stop on trailer
		current_frame++;
		DrawState();
		while ((millis() - timer_interval) < interval_long) { delay(5); }
	}
	DrawProgmem16(0, 0, 9);
	DrawProgmem16(0, 1, 10);
	while (getPressedButton() == BUTTON_NONE) { delay(5); }
}

void setup() {
	lcd.begin();                 //initialise lcd
	lcd.backlight();             //turn on backlight
	//drawing special characters
	lcd.createChar(1, Akb1);
	lcd.createChar(2, Akb2);
	lcd.createChar(3, Akb3);
	lcd.createChar(4, Akb4);
	lcd.createChar(5, Akb5);
	lcd.createChar(6, sel);
	//set up outputs
	pinMode(lin_step, OUTPUT);
	pinMode(lin_dir, OUTPUT);
	pinMode(rot_step, OUTPUT);
	pinMode(rot_dir, OUTPUT);
	pinMode(af_on, OUTPUT);
	pinMode(shutter, OUTPUT);
	pinMode(rot_off, OUTPUT);
	//fixing
	digitalWrite(shutter, LOW);
	digitalWrite(af_on, LOW);
	digitalWrite(rot_off, LOW);
	//setting variables
	MenuNowPos = 0;
	slider_mode = 1;
	interval = 20;
	all_frames = 240; 
	current_frame = 0;
	lenght_step = lenght_step_all / all_frames;
	DrawMenu();
	ender_stop = false;
}

void loop() {
	switch (getPressedButton())
	{
	case (BUTTON_RIGHT):
		run_right();
		break;
	case (BUTTON_LEFT):
		run_left();
		break;
	case (BUTTON_BACK):
		run_back();
		break;
	case (BUTTON_SELECT):
		run_select();
		break;
	}
}
