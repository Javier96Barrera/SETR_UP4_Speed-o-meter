#include speedometer.h

//PUBLIC FUNCTIONS
speedometer::speedometer();

void speedometer::getTime(){

  TIMSK1 &= ~(1<<(TOIE1));
  _real_timeBuffer = _real_time;
  TIMSK1 |= (1<<(TOIE1));

  _real_timeBuffer *= TICK_TIME;

  _clockTimeStamp[0] = _real_timeBuffer/secPerYear;
  _real_timeBuffer = _real_timeBuffer%secPerYear;
  int i;
  for(i=0; i<12;i++){
    if(_real_timeBuffer>=_secPerMonths[i])_real_timeBuffer-=_secPerMonths[i];
    else break;  
  }
  
  if(_real_timeBuffer>0){
    _clockTimeStamp[1] = i+1;
    if(_real_timeBuffer>=_secPerDay){
      _clockTimeStamp[2] = _real_timeBuffer/secPerDay;
      _real_timeBuffer = _real_timeBuffer%secPerDay;
    }
    if(_real_timeBuffer>=secPerHour){
      _clockTimeStamp[3] = _real_timeBuffer/secPerHour;
      _real_timeBuffer = _real_timeBuffer%secPerHour;
    }
    if(_real_timeBuffer>=secPerMin){
      _clockTimeStamp[4] = _real_timeBuffer/secPerMin;
      _clockTimeStamp[5] = _real_timeBuffer%secPerMin;
    }else{
      _clockTimeStamp[5] = _real_timeBuffer;
    }
  }
}
  

void speedometer::goToSleep(){
  EEPROM.put(0,distRec);
  Serial.flush();
  cli();
  _sleep=false;
  TIMSK1 &= ~(1<<(ICIE1));
  PCMSK0 |= (1<<PCINT0);
  PCICR |= (1<<(PCIE0)); //Pinchange interrupt
  PCIFR |= (1<<(PCIF0));
  sei();
  SMCR |= (1<<(SM1)) | (1<<(SE));
  sleep_cpu();
  cli();
  SMCR &= ~(1<<(SE));  
  PCICR &= ~(1<<(PCIE0)); //Pinchange interrupt
  PCIFR |= (1<<(PCIF0));
  TIMSK1 |= (1<<(ICIE1));
  _wakeUp = true;
  sei();  
}

void speedometer::lcdUpdate(){  

  if(printOpt){
    _clear_screen(LCD05_I2C_ADDRESS);
    _sendi(LCD05_I2C_ADDRESS," Speed = %i km/h",_velocity);
    _set_cursor_coords(LCD05_I2C_ADDRESS, 2,1);
    _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Dist ="),6);
    _sendf(LCD05_I2C_ADDRESS,_distRec,3);
    _set_cursor_coords(LCD05_I2C_ADDRESS, 2,13);
    _ascii_chars(LCD05_I2C_ADDRESS,(byte*)(" km"),3);
  }else{
    getTime();
    _clear_screen(LCD05_I2C_ADDRESS);
    _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Date: "),6);
    if(_clockTimeStamp[2] > 9){ sendu8(LCD05_I2C_ADDRESS,"%u/",_clockTimeStamp[2]);
    }else _sendu8(LCD05_I2C_ADDRESS,"0%u/",_clockTimeStamp[2]);
    if(_clockTimeStamp[1] > 9){ sendu8(LCD05_I2C_ADDRESS,"%u/",_clockTimeStamp[1]);
    }else _sendu8(LCD05_I2C_ADDRESS,"0%u/",_clockTimeStamp[1]);
    if(_clockTimeStamp[0] > 9){ sendu8(LCD05_I2C_ADDRESS,"%u",_clockTimeStamp[0]);
    }else _sendu8(LCD05_I2C_ADDRESS,"0%u",_clockTimeStamp[0]);
    _set_cursor_coords(LCD05_I2C_ADDRESS, 2,1);
    _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Time: "),6);
    if(_clockTimeStamp[3] > 9){ sendu8(LCD05_I2C_ADDRESS,"%u:",_clockTimeStamp[3]);
    }else _sendu8(LCD05_I2C_ADDRESS,"0%u:",_clockTimeStamp[3]);
    if(_clockTimeStamp[4] > 9){ sendu8(LCD05_I2C_ADDRESS,"%u:",_clockTimeStamp[4]);
    }else _sendu8(LCD05_I2C_ADDRESS,"0%u:",_clockTimeStamp[4]);
    if(_clockTimeStamp[5] > 9){ sendu8(LCD05_I2C_ADDRESS,"%u",_clockTimeStamp[5]);
    }else _sendu8(LCD05_I2C_ADDRESS,"0%u",_clockTimeStamp[5]);
  }
  TIMSK1 &= ~(1<<(TOIE1));
  _printLCD = false;
  TIMSK1 |= (1<<(TOIE1));
}

void speedometer::configTime(){
    int p;
    _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Date: "),6);
    _set_cursor_coords(LCD05_I2C_ADDRESS, 2,1);
    _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Time: "),6);
    _set_cursor_coords(LCD05_I2C_ADDRESS, 1,7);
    for(p = 0; p<=_newTimeStampSize; p++){
      
      _key = _kpd.waitForKey();
      if(_key == '*')  // Check for a valid key.
      {
        if(p == 0){
          _timeConfig = false;
          break;
        }
        _clear_screen(LCD05_I2C_ADDRESS);
        _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Date: "),6);
        _set_cursor_coords(LCD05_I2C_ADDRESS, 2,1);
        _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("Time: "),6);
        _set_cursor_coords(LCD05_I2C_ADDRESS, 1,7);
        p=-1;
      }else if(_key == '#' && p==newTimeStampSize){
        _timeConfig = false;
        _setTime();
        break;
        //flag de terminado para guardar en timestamp
      }else if(p < _newTimeStampSize && _key != '#'){
        if(p==2 || p==4){
          _ascii_chars(LCD05_I2C_ADDRESS,(byte*)("/"),1);
        }else if(p == 6){
          _set_cursor_coords(LCD05_I2C_ADDRESS, 2,7);
          
        }else if(p==8 || p==10){
          _ascii_chars(LCD05_I2C_ADDRESS,(byte*)(":"),1);
        }
        _sendc(LCD05_I2C_ADDRESS,"%c",_key);
        _newTimeStamp[p] = _key;
      }else p--;
    }
    TIMSK1 |= (1<<(ICIE1)) | (1<<(TOIE1));  
}

 
// the setup routine runs once when you press reset:
void speedometer::init() {

  DDRD = 0b00000000;
  PORTD = 0b00110000;
  
  
  _sleep, _debounce, _wakeUp,_printLCD,_timeConfig= false;
  _printOpt = true;
  _sleepTimeout, _W, _velocity, _nSectVuelt, _pos = 0;
  _velocityTimeout = MAX_VELOCITY_TIMEOUT;
  _distRec = EEPROM.get(0,_distRec);
  
  Wire.begin();
  delay(LCD05_I2C_INIT_DELAY);  
  set_display_type(LCD05_I2C_ADDRESS,LCD_STYLE_16X2);
  clear_screen(LCD05_I2C_ADDRESS);
  cursor_home(LCD05_I2C_ADDRESS);
  backlight_off(LCD05_I2C_ADDRESS);
  _backLight = false;

  PCMSK2 |= (1<<(PCINT22)) | (1<<(PCINT21)) | (1<<(PCINT20));
  PCICR |= (1<<(PCIE2)); //Pinchange interrupt
  PCIFR |= (1<<(PCIF2));

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  TCCR1A |= (1<<(WGM11)) | (1<<(WGM10));
  TCCR1B |= (1<<(ICNC1)) | (1<<(WGM12)) | (1 << (CS11));
  TCCR1B &= ~(1<<(ICES1)) & ~(1<<(WGM13)) & ~(1 << (CS12)) & ~(1 << (CS10));

  TIMSK1 |= (1<<(ICIE1)) | (1<<(TOIE1));
  sei();
}

void speedometer::modeSelectKP(boolean wait){
	if(wait) 
		_key = _kpd.waitForKey();
	else 
		_key = _kpd.getKey();
	if(_key == '#'){
		TIMSK1 &= ~(1<<(ICIE1)) & ~(1<<(TOIE1));
		_timeConfig = true;
		_clear_screen(LCD05_I2C_ADDRESS);
	} else if(_key == '*'){
		_printOpt = !_printOpt; //Cambiamos modos speedometer y reloj.
	}
}

void speedometer::backlight(){
	if(_backLight){
		backlight_on(LCD05_I2C_ADDRESS);
	}else{
		backlight_off(LCD05_I2C_ADDRESS);
	}
}

bool speedometer::getLCDUpdateFlag(){
	return this._lcdUpdate;
}
bool speedometer::getSleepFlag(){
	return this._sleep;
}
bool speedometer::getTimeConfigFlag(){
	return this._timeConfig;
}
uint8_t speedometer::getVelocity(){
	return this._velocity;
}
float speedometer::getDistance(){
	return this._distRec;
}

//PRIVATE FUNCTIONS


void speedometer::pc2_handler()

  if(!debounce){
    _debounceAux = _real_time;
    _debounce = true;
  } else if(((_real_time-_debounceAux)*TICK_TIME)>=BUTTON_WINDOW){
      _debounce = false;  
  }
  if(!_debounce){
    if(!(PIND&0b00010000)){
      _backLight = !_backLight;
    }
    else if(!(PIND & 0b00100000)){
      _distRec = 0;
    }
  }   
}

void speedometer::medianFilter(){
  int MAX,MIN,MED = _velocityArray[0];
  for(int i = 1;i<3;i++){
    if(_velocityArray[i] > MAX){
      MIN = MED;
      MED = MAX;
      MAX = velocityArray[i];
    }
    else if(_velocityArray[i] < MIN){
      MAX = MED;
      MED = MIN;
      MIN = _velocityArray[i];
    }
  }
  _velocity = MED*3.6;
}

// interrupt vector for timer1 overflow event
void speedometer::t1_ovf_handler()
{
  
  if(_velocityTimeout==0)_velocity=0;
  else _velocityTimeout--;
  
  
  _timer1_overflows+=TIMER1_OVERFLOW_VALUE;

  uint16_t _icr1_register=ICR1;

  if(_inputReal_timeBuffer>0){
    _inputReal_timeBuffer=0;
    _real_time = _inputReal_timeBuffer;
  } else {
    _real_time = _timer1_overflows+icr1_register;
  }

  if(
     (TIFR1 & (1<<TOV1)) && // is timer1 overflow flag set? and
     (_icr1_register<TIMER1_OVERFLOW_HALF_VALUE) // did the capture take place after the overflow?
    )
    { 
      _real_time+=TIMER1_OVERFLOW_VALUE;
    }

  if(_velocity==0 && !_wakeUp){
    if(((_real_time-_sleepTimeout)*TICK_TIME)>=(SLEEP_WINDOW)) _sleep=true;
  }
  else{
    _sleepTimeout = _real_time;
    _wakeUp=false;
  }

  if((_real_time-_showLCD)*TICK_TIME>=LCD_WINDOW){
    _printLCD=true;
    _showLCD = _real_time;
  }
  
}

void speedometer::t1_capt_handler()
{

  _velocityTimeout = MAX_VELOCITY_TIMEOUT;

  _nSectVuelt++;
  if(_nSectVuelt == 16){
    _distRec += WHEEL_LENGTH*0.001;
    //_distRec += WHEEL_LENGTH;
    _nSectVuelt = 0;
  }
  
  if(_currentTimeStamp<2){

    _timestamps[currentTimeStamp]=_real_time;
    _currentTimeStamp++;
  }else if(_currentTimeStamp==2){
    _timeBetTicks=(_timestamps[1] - _timestamps[0])*TICK_TIME;
    _W=RADIANS/_timeBetTicks; //2 sectores
    _velocityArray[pos] = _W*RADIUS;
    _pos++;
    _currentTimeStamp=0;
    if(_pos == MEDIAN_WINDOW){
      _medianFilter();
      _pos=0;
    }
  }
}

inline void speedometer::_write_command(byte command)
{ Wire.write(COMMAND_REGISTER); Wire.write(command); }

void speedometer::_set_display_type(byte address, byte type)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(SET_DISPLAY_TYPE);
  Wire.write(type);
  Wire.endTransmission();
}

void speedometer::_clear_screen(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(CLEAR_SCREEN);
  Wire.endTransmission();
}

void speedometer::_cursor_home(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(CURSOR_HOME);
  Wire.endTransmission();
}

void speedometer::_set_cursor(byte address, byte pos)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(CURSOR_HOME);
  Wire.write(pos);
  Wire.endTransmission();
}

void speedometer::_set_cursor_coords(byte address, byte line, byte column)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(SET_CURSOR_COORDS);
  Wire.write(line);
  Wire.write(column);
  Wire.endTransmission();
}

void speedometer::_backlight_on(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(BACKLIGHT_ON);
  Wire.endTransmission();
}

void speedometer::_backlight_off(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(BACKLIGHT_OFF);
  Wire.endTransmission();
}

bool speedometer::_ascii_chars(byte address, byte* bytes, int length)
{
  if(length<=0) return false;
  Wire.beginTransmission(address); // start communication with LCD 05
  Wire.write(COMMAND_REGISTER);
  for(int i=0; i<length; i++, bytes++) Wire.write(*bytes);
  Wire.endTransmission();
  return true;
}

void speedometer::_sendf(byte address,float st, int decimales)
{
  char sp[16];
  char *p = &sp[0]; // points to start of array.
  
  dtostrf( st, 8, decimales, sp );
  Wire.beginTransmission(address);
  // send *p as bytes of date
  while(*p)
    Wire.write(*(p++));
  Wire.endTransmission();
}

void speedometer::_sendu8(byte address, char* format, uint8_t st)
{
  char sp[16];
  char *p = &sp[0]; // points to start of array.
  sprintf( p, format, st);
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER);
  // send *p as bytes of date
  while(*p)
    Wire.write(*(p++));
  Wire.endTransmission();
  
} 

void speedometer::_sendc(byte address, char* format, char st)
{
  char sp[16];
  char *p = &sp[0]; // points to start of array.
  sprintf( p, format, st);
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER);
  // send *p as bytes of date
  while(*p)
    Wire.write(*(p++));
  Wire.endTransmission();
  
} 

void speedometer::_sendi(byte address, char* format, int st)
{
  char sp[16];
  char *p = &sp[0]; // points to start of array.
  sprintf( p, format, st);
  Wire.beginTransmission(address);
  // send *p as bytes of date
  while(*p)
    Wire.write(*(p++));
  Wire.endTransmission();
} 


byte speedometer::_read_fifo_length(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(FIFO_AVAILABLE_LENGTH_REGISTER);                           // Call the register for start of ranging data
  Wire.endTransmission();
  
  Wire.requestFrom(address,byte(1)); // start communication with LCD 05, request one byte
  while(!Wire.available()) {  }
  return Wire.read();
}

void speedometer::_setTime(){
  
  uint64_t aux = 0;
  
  for(int c=0;c<_newTimeStampSize;c++){
    if(!(c & 0b00000001)){ //impar
      aux += 10*(uint8_t)_newTimeStamp[c];      
    }else{ //par
      aux += (uint8_t)_newTimeStamp[c];
      _inputTimeStamp[c] = aux;
      aux = 0;
    }
  }

  int m = _inputTimeStamp[1];

  _inputReal_timeBuffer+=(_inputTimeStamp[0]-1)*secPerYear;

  for(int i=0; i<m-1;i++){
    _inputReal_timeBuffer+=_secPerMonths[i];
  }
  _inputReal_timeBuffer+=(_inputTimeStamp[2]-1)*secPerDay;
  _inputReal_timeBuffer+=(_inputTimeStamp[3]-1)*secPerHour;
  _inputReal_timeBuffer+=(_inputTimeStamp[4]-1)*secPerMin;
  _inputReal_timeBuffer+=(_inputTimeStamp[5]);

  _inputReal_timeBuffer = _inputReal_timeBuffer/TICK_TIME;
  _sleepTimeout = _inputReal_timeBuffer;
}