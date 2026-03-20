// ----------------------------------------------------------------------------
// Rotary Encoder Driver with Acceleration
// Supports Click, DoubleClick, Long Click
//
// (c) 2010 karl@pitrich.com
// (c) 2014 karl@pitrich.com
//
// Timer-based rotary encoder logic by Peter Dannegger
// http://www.mikrocontroller.net/articles/Drehgeber
// ----------------------------------------------------------------------------

#include "ClickEncoder.h"
//#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define	klDEBUG_ISR_ROUTINE		false
#define	knDEBUG_ISR_PIN_NUM		14
#define	knDEBUG_ISR_PIN_NUM2		13

// ----------------------------------------------------------------------------
// Button configuration (values for 1ms timer service calls)
//
#define ENC_BUTTONINTERVAL 10 // check button every x milliseconds, also debouce time

// ----------------------------------------------------------------------------
// Acceleration configuration (for 1000Hz calls to ::service())
//
// #define ENC_ACCEL_TOP 3072 // max. acceleration: *12 (val >> 8)
// #define ENC_ACCEL_INC 25
#define ENC_ACCEL_TOP 2000	 // max. acceleration: *12 (val >> 8)
#define ENC_ACCEL_INC 20
#define ENC_ACCEL_DEC 2

// ----------------------------------------------------------------------------

#if ENC_DECODER == ENC_FLAKY
	#if ENC_HALFSTEP
// decoding table for hardware with flaky notch (half resolution)
const int8_t ClickEncoder::table[16]
		  __attribute__((__progmem__)) = {0, 0, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 0, 0};
	#else
// decoding table for normal hardware
const int8_t ClickEncoder::table[16]
		  __attribute__((__progmem__)) = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
	#endif
#elif ENC_DECODER == ENC_ISR
	ClickEncoder* p_instance = NULL;
	static volatile int64_t liTmrClockFall=-1;
	static volatile int64_t liTmrClockRise=-1;
	static volatile int64_t liTmrDataFall=-1;
	static volatile int64_t liTmrDataRise=-1;
#endif
static volatile int16_t posCorrect = 0;







// ----------------------------------------------------------------------------

ClickEncoder::ClickEncoder(int8_t A, int8_t B, int8_t BTN, uint8_t stepsPerNotch, bool active)
		  : doubleClickEnabled(true),
			 buttonHeldEnabled(true),
			 accelerationEnabled(true),
			 delta(0),
			 last(0),
			 acceleration(0),
			 button(Open),
			 steps(stepsPerNotch),
			 pinA(A),
			 pinB(B),
			 pinBTN(BTN),
			 pinsActive(active)
#ifndef WITHOUT_BUTTON
			 ,
			 analogInput(false)
#endif
{
	pinMode_t configType = (pinsActive == LOW) ? INPUT_PULLUP : INPUT;
	if (pinA >= 0) {
		pinMode(pinA, configType);
	}
	if (pinB >= 0) {
		pinMode(pinB, configType);
	}
#ifndef WITHOUT_BUTTON
	if (pinBTN >= 0) {
		pinMode(pinBTN, configType);
	}
#endif

	if (digitalRead(pinA) == pinsActive) {
		last = 3;
	}

	if (digitalRead(pinB) == pinsActive) {
		last ^= 1;
	}

	#if ENC_DECODER == ENC_ISR
		p_instance = this;
	#endif
	#if klDEBUG_ISR_ROUTINE == true
		pinMode(knDEBUG_ISR_PIN_NUM, OUTPUT);
		pinMode(knDEBUG_ISR_PIN_NUM2, OUTPUT);
	#endif
}

#if ENC_DECODER == ENC_ISR
// aktivaci IRQ musim delat az v setup(), ve zvlast metode, protoze pri vzniku objectu jeste neni aktivni interrupt engine v Arduino
void ClickEncoder::initISR() {
	// int od kazde zmeny/hrany
	attachInterrupt(digitalPinToInterrupt(pinA), &ClickEncoder::isrPinData, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pinB), &ClickEncoder::isrPinClock, CHANGE);
	// initPCNT();
}


// otestuje casy signalu a pokud jsou validni, meni hodnotu prom. "delta" obj.
// pokud neni prubeh konzistentni, ZAHODI jej cely (nuluje vsechny 4 casy)
void IRAM_ATTR ClickEncoder::encoderDecode() {
	// pattern pro LEVY krok je:
	// clock: --\__/----
	// data : ----\__/--

	// pattern pro PRAVY krok je:
	// clock: ----\__/--
	// data : --\__/----

	// vsechny 4 casy musi byt validni
	if ((liTmrClockRise != -1)
		&&
		(liTmrClockFall != -1)
		&&
		(liTmrDataFall != -1)
		&&
		(liTmrDataRise != -1)) {

		// sekce kde menim DELTA prom. !!!
		//------------------------------
		if ((liTmrDataFall > liTmrClockFall) && (liTmrDataRise > liTmrClockRise)) {
			(p_instance->delta)--;
			
		} else if ((liTmrDataFall < liTmrClockFall) && (liTmrDataRise < liTmrClockRise)) {
			(p_instance->delta)++;
			
		} else {
			// pokud je signal nekonzistentni (data nejsou casove "prelozena" pres clock)
			// nekonzistent !
		}
		//------------------------------

	} else {
		// signalizace/DEBUG ktere casy chybi
		#if klDEBUG_ISR_ROUTINE == true
			int mul=0;
			if (liTmrClockRise == -1)
				mul+= 2;
			if (liTmrClockFall == -1)
				mul+= 4;
			if (liTmrDataFall == -1)
				mul+= 8;
			if (liTmrDataRise == -1)
				mul+= 16;

			for (int x = 0; x < mul; x++) {
				digitalWrite(knDEBUG_ISR_PIN_NUM2, !digitalRead(knDEBUG_ISR_PIN_NUM2));
			}
		#endif
	}
	
	// reset casu
	liTmrDataFall = liTmrClockFall = liTmrDataRise = liTmrClockRise = -1;

}

void IRAM_ATTR ClickEncoder::isrPinData(void) {
	static volatile int64_t tmrRise=-1, tmrFall=-1;

	volatile bool lData = digitalRead(p_instance->pinA);				// co nejdriv eulozit
	volatile bool lClock = digitalRead(p_instance->pinB);				// co nejdriv eulozit
	volatile int64_t now = esp_timer_get_time();

	if (lData) {
		#if klDEBUG_ISR_ROUTINE == true
			digitalWrite(knDEBUG_ISR_PIN_NUM2, 1);
		#endif
		tmrRise = now;
		liTmrDataRise = tmrRise;
		liTmrDataFall = tmrFall;

		if (lClock) {								// data+clock v idle
			encoderDecode();
		}
	} else {
		#if klDEBUG_ISR_ROUTINE == true
			digitalWrite(knDEBUG_ISR_PIN_NUM2, 0);
		#endif
	
		// uz nejaky byl zachycen a tento ted prichazi velmi brzy po rise ?
		if ((tmrFall!=-1) && (tmrRise!=1) && (now - tmrRise < 2000)) {
			// NEprepisuji tmrFall - necham ten starsi
		} else {
			tmrFall = now;
		}

	}
}


// volana pri rise a falling edge od HW pin change
// static ! volana jako ISR !
void IRAM_ATTR ClickEncoder::isrPinClock(void) {
	// pro filtr zakmitu
	static volatile int64_t tmrRise=-1, tmrFall=-1;

	volatile bool lData = digitalRead(p_instance->pinA);				// co nejdriv eulozit
	volatile bool lClock = digitalRead(p_instance->pinB);				// co nejdriv eulozit
	volatile int64_t now = esp_timer_get_time();
	// Čtení pinu 0-31 pomocí registru (velmi rychlé a bezpečné v ISR)
	//int stav = (GPIO.in >> pin_number) & 0x1;

	
	if (lClock) {
		#if klDEBUG_ISR_ROUTINE == true
			digitalWrite(knDEBUG_ISR_PIN_NUM, 1);
		#endif
		tmrRise = now;
		liTmrClockRise = tmrRise;
		liTmrClockFall = tmrFall;

		if (lData) {								// data+clock v idle
			// presunu casy do global. vars
			encoderDecode();
		}
				
	} else {
		#if klDEBUG_ISR_ROUTINE == true
			digitalWrite(knDEBUG_ISR_PIN_NUM, 0);
		#endif

		// uz nejaky byl zachycen a tento ted prichazi velmi brzy po rise ?
		if ((tmrFall!=-1) && (tmrRise!=1) && (now - tmrRise < 2000)) {
			// NEprepisuji tmrFall - necham ten starsi
		} else {
			tmrFall = now;
		}

	}

}
#endif

/* 
// volana pri rise a falling edge od HW pin change
// static ! volana jako ISR !
void IRAM_ATTR ClickEncoder::isrPinClock(void) {

	// pri sestupne hrane hodin (pinB encoderu) 
	// prectu pinA a podle nej je jasno
	// bud je PRED nebo ZA clock signalem, tedy bude je jeste v log.1, nebo uz v log.0
	
	volatile bool lData = digitalRead(p_instance->pinA);				// co nejdriv eulozit
	volatile bool lClock = digitalRead(p_instance->pinB);				// co nejdriv eulozit
	volatile int64_t now = esp_timer_get_time();
	// Čtení pinu 0-31 pomocí registru (velmi rychlé a bezpečné v ISR)
	//int stav = (GPIO.in >> pin_number) & 0x1;

	#if klDEBUG_ISR_ROUTINE == true
		digitalWrite(knDEBUG_ISR_PIN_NUM, !digitalRead(knDEBUG_ISR_PIN_NUM));
	#endif
	
	// potrebuji vyrusit dopady pripadneho "ruseni" = prilis kratke pulsy v CLOCK signalu ZAHODIT
	// HW ale pocita i s nimi, takze musim udelat NASLEDNOU opravu
	// vse POD cca 2ms je nesmyslne rychle toceni, nebo RUSENI / ZAKMITY -> zahazuji
	
	// sestupna hrana CLOCK, tedy cas mezi nastupnou a "novou" sestupnou je podezrele kratky
	// => resi navrat CLOCK do log.1 a hned zakmit do log.0
	if (!lClock) {									
		if (now - liTmrClockFall < 2000) {
			// vse POD cca 2ms je nesmyslne rychle toceni, nebo RUSENI / ZAKMITY -> zahazuji
			// debug
			#if klDEBUG_ISR_ROUTINE == true
			for (int x = 0; x < 20; x++) {
				digitalWrite(knDEBUG_ISR_PIN_NUM, !digitalRead(knDEBUG_ISR_PIN_NUM));
			}
			#endif
			//-------
			return;
			//-------
		}
		// Čtení pinu 0-31 pomocí registru (velmi rychlé a bezpečné v ISR)
		//int stav = (GPIO.in >> pin_number) & 0x1;
		liTmrClockFall = now;
	} else {
		liTmrClockRise = now;
	}

	if (lData) {
		(p_instance->delta)--;
	} else {
		(p_instance->delta)++;
	}
}
#endif
 */


// verze s HW PCNT modulem


// hw PCNT funguje ok, ale stejne musime delat filtry pak v sw ... nema smysl
/* void initPCNT() {
	const int pulsePin = 4;  // Vstupní signál (pulzy)
	const int ctrlPin = 21;  // Směrový signál (vlevo/vpravo, nahoru/dolů)
	pcnt_config_t pcnt_config = {};
	pcnt_config.pulse_gpio_num = pulsePin;
	pcnt_config.ctrl_gpio_num = ctrlPin;
	pcnt_config.unit = PCNT_UNIT_0;
	pcnt_config.channel = PCNT_CHANNEL_0;

	// Co dělat s pulzem na GPIO 4 (vzestupná hrana)
	pcnt_config.pos_mode = PCNT_COUNT_DIS; 		// Sestupnou hranu ignoruj
	pcnt_config.neg_mode = PCNT_COUNT_INC;			// Inkrementovat

	// Co dělat, když je na GPIO 21 (Control) určitá úroveň
	pcnt_config.lctrl_mode = PCNT_MODE_KEEP;	  // Když je LOW, drž směr (přičítej)
	pcnt_config.hctrl_mode = PCNT_MODE_REVERSE; // Když je HIGH, otoč směr (odečítej)
	
	// Limity (16-bitový registr: -32768 až 32767)
	pcnt_config.counter_h_lim = 32767;
	pcnt_config.counter_l_lim = -32768;

	pcnt_unit_config(&pcnt_config);

	// Hardwarový filtr proti zákmitům (např. pro mechanické enkodéry)
	// Hodnota 1000 při 80MHz APB clocku odfiltruje cokoli kratšího než 12.5 us
	pcnt_set_filter_value(PCNT_UNIT_0, 1000);
	pcnt_filter_enable(PCNT_UNIT_0);

	// Inicializace a start
	pcnt_counter_pause(PCNT_UNIT_0);
	pcnt_counter_clear(PCNT_UNIT_0);
	pcnt_counter_resume(PCNT_UNIT_0);
} */

/* 
// static ! volana jako ISR !
// (volana pri rise a falling edge od HW pin change)
void IRAM_ATTR ClickEncoder::isrPinB(void) {
	static int64_t lLastFall=0;

	// pri sestupne hrane hodin (pinB encoderu) 
	// prectu pinA a podle nej je jasno
	// bud je PRED nebo ZA clock signalem, tedy bude je jeste v log.1, nebo uz v log.0
	
	volatile bool lData = digitalRead(p_instance->pinA);				// co nejdriv eulozit
	volatile bool lClock = digitalRead(p_instance->pinB);				// co nejdriv eulozit
	// Čtení pinu 0-31 pomocí registru (velmi rychlé a bezpečné v ISR)
	//int stav = (GPIO.in >> pin_number) & 0x1;

	volatile int64_t now = esp_timer_get_time();
	
	#if klDEBUG_ISR_ROUTINE == true
		digitalWrite(knDEBUG_ISR_PIN_NUM, !digitalRead(knDEBUG_ISR_PIN_NUM));
	#endif
	
	// potrebuji vyrusit dopady pripadneho "ruseni" = prilis kratke pulsy v CLOCK signalu ZAHODIT
	// HW ale pocita i s nimi, takze musim udelat NASLEDNOU opravu
	// vse POD cca 2ms je nesmyslne rychle toceni, nebo RUSENI / ZAKMITY -> zahazuji
	
	// sestupna hrana CLOCK, tedy cas mezi nastupnou a "novou" sestupnou je podezrele kratky
	// => resi navrat CLOCK do log.1 a hned zakmit do log.0
	if (!lClock) {									
		lLastFall = now;
		if (now - lLastFall < 2000) {
			// stav DATA pinu v dobe sestupne CLOCK byl
			// opravu udelam dle stavu DATA
			posCorrect = (lData ? +1 : -1);

			// debug
			#if klDEBUG_ISR_ROUTINE == true
				for (int x = 0; x < 20; x++) {
					digitalWrite(knDEBUG_ISR_PIN_NUM, !digitalRead(knDEBUG_ISR_PIN_NUM));
				}
			#endif
		}
	} else {
		// nastupna hrana = konec clock pulsu
		// nekdy se stane, ze CLOCK signal zustane (i ve spravne mezipozici) v log.0
		// pri dalsim pohybu pak vygeneruje DVA (necele) pulsy, coz dekoder chape jako dva kroky !
		// mohu zmerit SIRKU v log.0 a pokud je moc dlouha - jeden puls odectu
		if (now - lLastFall > 200000) {						// delsi nez 200ms
			posCorrect = (lData ? +1 : -1);
			// debug
			#if klDEBUG_ISR_ROUTINE == true
				for (int x = 0; x < 100; x++) {
					digitalWrite(knDEBUG_ISR_PIN_NUM, !digitalRead(knDEBUG_ISR_PIN_NUM));
				}
			#endif
		}

	}


	// // vse POD cca 2ms je nesmyslne rychle toceni, nebo RUSENI / ZAKMITY -> zahazuji
	// if (now - lLastFall < 2000) {
	// 	return;
	// }
	// // Čtení pinu 0-31 pomocí registru (velmi rychlé a bezpečné v ISR)
	// //int stav = (GPIO.in >> pin_number) & 0x1;
	// lLastFall = now;

	// if (digitalRead(p_instance->pinA)) {
	// 	(p_instance->delta)--;
	// } else {
	// 	(p_instance->delta)++;
	// }
}
*/


// ----------------------------------------------------------------------------
#ifndef WITHOUT_BUTTON


// Depricated.  Use DigitalButton instead
// ClickEncoder::ClickEncoder(int8_t BTN, bool active)
// 		  : doubleClickEnabled(true),
// 			 buttonHeldEnabled(true),
// 			 accelerationEnabled(true),
// 			 delta(0),
// 			 last(0),
// 			 acceleration(0),
// 			 button(Open),
// 			 steps(1),
// 			 analogInput(false),
// 			 pinA(-1),
// 			 pinB(-1),
// 			 pinBTN(BTN),
// 			 pinsActive(active) {
// 	pinMode_t configType = (pinsActive == LOW) ? INPUT_PULLUP : INPUT;
// 	if (pinBTN >= 0) {
// 		pinMode(pinBTN, configType);
// 	}
// }

// ----------------------------------------------------------------------------
// Constructor for using digital input as a button

DigitalButton::DigitalButton(int8_t BTN, bool active)
		  : ClickEncoder(BTN, active)

{}

// ----------------------------------------------------------------------------
// Constructor for using analog input range as a button

AnalogButton::AnalogButton(int8_t BTN, int16_t rangeLow, int16_t rangeHigh)
		  : ClickEncoder(BTN, (bool)false) {
	pinMode(pinBTN, INPUT);

	anlogActiveRangeLow = rangeLow;
	anlogActiveRangeHigh = rangeHigh;
	analogInput = true;

	if (anlogActiveRangeLow > anlogActiveRangeHigh) { // swap values if provided in the wrong order
		int16_t t = anlogActiveRangeLow;
		anlogActiveRangeLow = anlogActiveRangeHigh;
		anlogActiveRangeHigh = t;
	}
}
#endif


// ----------------------------------------------------------------------------
// call this every 1 millisecond via timer ISR
//
void ClickEncoder::service(void) {
	bool moved = false;

	if (pinA >= 0 && pinB >= 0) {
		if (accelerationEnabled) { // decelerate every tick
			acceleration -= ENC_ACCEL_DEC;
			if (acceleration & 0x8000) { // handle overflow of MSB is set
				acceleration = 0;
			}
		}

#if ENC_DECODER == ENC_FLAKY
		last = (last << 2) & 0x0F;

		if (digitalRead(pinA) == pinsActive) {
			last |= 2;
		}

		if (digitalRead(pinB) == pinsActive) {
			last |= 1;
		}

		int8_t tbl = pgm_read_byte(&table[last]);
		if (tbl) {
			delta += tbl;
			moved = true;
		}
#elif ENC_DECODER == ENC_NORMAL
		int8_t curr = 0;

		if (digitalRead(pinA) == pinsActive) {
			curr = 3;
		}

		if (digitalRead(pinB) == pinsActive) {
			curr ^= 1;
		}

		int8_t diff = last - curr;

		if (diff & 1) { // bit 0 = step
			last = curr;
			delta += (diff & 2) - 1; // bit 1 = direction (+/-)
			moved = true;
		}
#elif ENC_DECODER == ENC_ISR
		if (delta) // meni isrPinB()
			moved = true;
		
		/* 	static int16_t oldCounter = 0;
		int16_t count;
		pcnt_get_counter_value(PCNT_UNIT_0, &count);
		delta = (count - oldCounter);
		delta += posCorrect;									// zapocitam pripadnou opravu
		oldCounter = count;
		posCorrect = 0; */

#else
	#error "Error: define ENC_DECODER to ENC_NORMAL or ENC_FLAKY"
#endif

		if (accelerationEnabled && moved) {
			// increment accelerator if encoder has been moved
			if (acceleration <= (ENC_ACCEL_TOP - ENC_ACCEL_INC)) {
				acceleration += ENC_ACCEL_INC;
			}
		}
	}

	// handle button
	//
#ifndef WITHOUT_BUTTON
	unsigned long currentMillis = millis();
	if (currentMillis < lastButtonCheck)
		lastButtonCheck = 0; // Handle case when millis() wraps back around to zero
	if ((pinBTN > 0 ||
		  (pinBTN == 0 && buttonOnPinZeroEnabled)) // check button only, if a pin has been provided
		 && ((currentMillis - lastButtonCheck) >=
			  ENC_BUTTONINTERVAL)) // checking button is sufficient every 10-30ms
	{
		lastButtonCheck = currentMillis;

		bool pinRead = getPinState();

		if (pinRead == pinsActive) { // key is down
			keyDownTicks++;
			if ((keyDownTicks > (buttonHoldTime / ENC_BUTTONINTERVAL)) && (buttonHeldEnabled)) {
				button = Held;
			}
		}

		if (pinRead == !pinsActive) { // key is now up
			if (keyDownTicks > 1) { // Make sure key was down through 1 complete tick to prevent random
											// transients from registering as click
				if (button == Held) {
					button = Released;
					doubleClickTicks = 0;
				} else {
	#define ENC_SINGLECLICKONLY 1
					if (doubleClickTicks > ENC_SINGLECLICKONLY) { // prevent trigger in single click mode
						if (doubleClickTicks < (buttonDoubleClickTime / ENC_BUTTONINTERVAL)) {
							button = DoubleClicked;
							doubleClickTicks = 0;
						}
					} else {
						doubleClickTicks = (doubleClickEnabled) ?
															(buttonDoubleClickTime / ENC_BUTTONINTERVAL) :
															ENC_SINGLECLICKONLY;
					}
				}
			}

			keyDownTicks = 0;
		}

		if (doubleClickTicks > 0) {
			doubleClickTicks--;
			if (doubleClickTicks == 0) {
				button = Clicked;
			}
		}
	}
#endif // WITHOUT_BUTTON
}


// ----------------------------------------------------------------------------

int16_t ClickEncoder::getValue(void) {
	int16_t val;

	noInterrupts();
	val = delta;

	if (steps == 2)
		delta = val & 1;
	else if (steps == 4)
		delta = val & 3;
	else
		delta = 0; // default to 1 step per notch

	if (steps == 4)
		val >>= 2;
	if (steps == 2)
		val >>= 1;

	int16_t r = 0;
	int16_t accel = ((accelerationEnabled) ? (acceleration >> 8) : 0);

	if (val < 0) {
		r -= 1 + accel;
	} else if (val > 0) {
		r += 1 + accel;
	}
	interrupts();

	return r;
}

// ----------------------------------------------------------------------------

#ifndef WITHOUT_BUTTON
ClickEncoder::Button ClickEncoder::getButton(void) {
	noInterrupts();
	ClickEncoder::Button ret = button;
	if (button != ClickEncoder::Held && ret != ClickEncoder::Open) {
		button = ClickEncoder::Open; // reset
	}
	interrupts();

	return ret;
}

bool ClickEncoder::getPinState() {
	bool pinState;
	if (analogInput) {
		int16_t pinValue = analogRead(pinBTN);
		pinState = ((pinValue >= anlogActiveRangeLow) && (pinValue <= anlogActiveRangeHigh)) ?
								 LOW :
								 HIGH; // set result to LOW (button pressed) if analog input is in range
	} else {
		pinState = digitalRead(pinBTN);
	}
	return pinState;
}

#endif



		// if ((tmrRise!=-1) && (now - tmrRise < 2000)) {
		// 	// zahazuji, ruseni
		// 	#if klDEBUG_ISR_ROUTINE == true
		// 	for (int x = 0; x < 50; x++) {
		// 		digitalWrite(knDEBUG_ISR_PIN_NUM2, !digitalRead(knDEBUG_ISR_PIN_NUM2));
		// 	}
		// 	#endif
		// } else {
		// }

				// idle pozice
		// filtrace moc kratkych pulsu
		// if ((tmrFall!=-1) && (now - tmrFall < 2000)) {
		// 	// zahazuji, ruseni
		// 	#if klDEBUG_ISR_ROUTINE == true
		// 		for (int x = 0; x < 50; x++) {
		// 			digitalWrite(knDEBUG_ISR_PIN_NUM2, !digitalRead(knDEBUG_ISR_PIN_NUM2));
		// 		}
		// 	#endif
		// } else {
