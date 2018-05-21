#ifndef speedometer_h
#define speedometer_h

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <Wire.h> // Arduino's I2C library
#include <Keypad.h>

#define LCD05_I2C_ADDRESS byte((0xC6)>>1)
#define LCD05_I2C_INIT_DELAY 100 // in milliseconds

// LCD05's command related definitions
#define COMMAND_REGISTER byte(0x00)
#define FIFO_AVAILABLE_LENGTH_REGISTER byte(0x00)
#define LCD_STYLE_16X2 byte(5)

// LCD05's command codes
#define CURSOR_HOME             byte(1)
#define SET_CURSOR              byte(2) // specify position with a byte in the interval 0-32/80
#define SET_CURSOR_COORDS       byte(3) // specify position with two bytes, line and column
#define HIDE_CURSOR             byte(4)
#define SHOW_UNDERLINE_CURSOR   byte(5)
#define SHOW_BLINKING_CURSOR    byte(6)
#define BACKSPACE               byte(8)
#define HORIZONTAL_TAB          byte(9) // advances cursor a tab space
#define SMART_LINE_FEED         byte(10) // moves the cursor to the next line in the same column
#define VERTICAL_TAB            byte(11) // moves the cursor to the previous line in the same column
#define CLEAR_SCREEN            byte(12)
#define CARRIAGE_RETURN         byte(13)
#define CLEAR_COLUMN            byte(17)
#define TAB_SET                 byte(18) // specify tab size with a byte in the interval 1-10
#define BACKLIGHT_ON            byte(19)
#define BACKLIGHT_OFF           byte(20) // this is the default
#define DISABLE_STARTUP_MESSAGE byte(21)
#define ENABLE_STARTUP_MESSAGE  byte(22)
#define SAVE_AS_STARTUP_SCREEN  byte(23)
#define SET_DISPLAY_TYPE        byte(24) // followed by the type, which is byte 5 for a 16x2 LCD style
#define CHANGE_ADDRESS          byte(25) // see LCD05 specification
#define CUSTOM_CHAR_GENERATOR   byte(27) // see LCD05 specification
#define DOUBLE_KEYPAD_SCAN_RATE byte(28)
#define NORMAL_KEYPAD_SCAN_RATE byte(29)
#define CONTRAST_SET            byte(30) // specify contrast level with a byte in the interval 0-255
#define BRIGHTNESS_SET          byte(31) // specify brightness level with a byte in the interval 0-255


#define MAX_VELOCITY_TIMEOUT 1500
#define newTimeStampSize 12
#define MEDIAN_WINDOW 3
#define TICK_TIME 0.5e-6
#define SLEEP_WINDOW 15
#define LCD_WINDOW 1
#define BUTTON_WINDOW 0.10

#define RADIUS 0.3
#define WHEEL_LENGTH 1.885
#define RADIANS 2*(360/32)*(M_PI/180)

#define secPerYear 31536000
#define secPerDay 3600*24
#define secPerHour 3600
#define secPerMin 60

class speedometer
{
  public:
    speedometer();
	void getTime();
	void goToSleep();
	void lcdUpdate();
	void configTime();
	void init();
	void modeSelectKP(boolean wait);
	void backLight();
	bool getLCDUpdateFlag();
	bool getSleepFlag();
	bool getTimeConfigFlag();
	uint8_t getVelocity();
	float getDistance();
	
  private:
	void _medianFilter();
	
	//LCD commands
	inline void _write_command(byte command);
	void _set_display_type(byte address, byte type);
	void _clear_screen(byte address);
	void _cursor_home(byte address);
	void _set_cursor(byte address, byte pos);
	void _set_cursor_coords(byte address, byte line, byte column);
	void _backlight_on(byte address);
	void _backlight_off(byte address);
	bool _ascii_chars(byte address, byte* bytes, int length);
	byte _read_fifo_length(byte address);
	
	void _sendf(byte address,float st, int decimales);
	void _sendu8(byte address, char* format, uint8_t st);
	void _sendc(byte address, char* format, char st);
	void _sendi(byte address, char* format, int st);
	
	void _setTime();
	
	volatile bool _printLCD;
	volatile bool _sleep;
	bool _timeConfig;
	volatile uint8_t _velocity;
	volatile float _distRec;
	
	//Keypad(3x4)
	const byte _ROWS = 4; // Four rows
	const byte _COLS = 3; // Three columns
	char _keys[ROWS][COLS] = {
		{'*','0','#'},
		{'7','8','9'},
		{'4','5','6'},
		{'1','2','3'}
	};
	byte _rowPins[ROWS] = { 10, 9, 7, 6 };
	byte _colPins[COLS] = { 13, 12, 11 };
	Keypad _kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
	char _key;
	
	volatile float _W,_timeBetTicks;

	volatile byte _velocityTimeout;
	volatile float velocityArray[3] = {0,0,0};
	volatile uint8_t pos;
	
	bool _backLight, _printOpt;
	volatile uint64_t _debounceAux;
	volatile bool debounce;
	volatile bool _wakeUp;

	volatile uint64_t _sleepTimeout;
	volatile uint64_t _showLCD;
	
	volatile uint8_t _nSectVuelt;
	
	const int _INPUT_CAPTURE_PIN=2;
	const int _INPUT_CAPTURE_PIN_BIT=2;
	volatile bool _input_capture_pin_status=false;
	
	volatile uint64_t _timer1_overflows=0;
	volatile uint8_t _currentTimeStamp=0;
	volatile uint64_t _timestamps[2];
	volatile uint64_t _real_time; //#epoch
	volatile uint64_t _real_timeBuffer;
	volatile uint64_t _inputReal_timeBuffer;
	uint8_t _clockTimeStamp[6] = {0,1,1,0,0,0};
	uint8_t _inputTimeStamp[6] = {0,0,0,0,0,0};
	uint8_t _months[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	char _newTimeStamp[12];
	
	uint32_t _secPerMonths[12] = {2678400,2419200,2678400,2592000,2678400,2592000,2678400,2678400,2592000,2678400,2592000,2678400};
	


};
#endif