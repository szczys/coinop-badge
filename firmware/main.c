#define F_CPU 8000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "debounce.h"

void Delay_ms(int cnt);

#define STATE_NOSTATE       0
#define STATE_CONFIRMSLEEP  1
#define STATE_CONFIRMWAKE   2
#define STATE_ASLEEP        3
#define STATE_POST          4
#define STATE_FADE          5
#define STATE_PEW           6
#define STATE_MANUALPEW     7
#define STATE_WAIT          8
#define STATE_SPARKLE       9

#define HIBERNATE           0
#define SNOOZE              1

uint8_t state;

#define PEWLEFTIN1        (1<<PB0)
#define PEWLEFTIN2        (1<<PB1)
#define PEWLEFTIN3        (1<<PB2)
#define PEWLEFTOUT1       (1<<PB3)
#define PEWLEFTOUT2       (1<<PB4)
#define PEWLEFTOUT3       (1<<PB5)
#define PEWRIGHTIN1       (1<<PB7)
#define PEWRIGHTIN2       (1<<PB6)
#define PEWRIGHTIN3       (1<<PC5)
#define PEWRIGHTOUT1      (1<<PC4)
#define PEWRIGHTOUT2      (1<<PC3)
#define PEWRIGHTOUT3      (1<<PC2)

#define SHIELDLEFTOUT     (1<<PD6)
#define SHIELDLEFTIN      (1<<PD7)
#define SHIELDRIGHTOUT    (1<<PD2)
#define SHIELDRIGHTIN     (1<<PD5)

#define CHARLIE1          (1<<PD3)
#define CHARLIE2          (1<<PD4)
#define CHARLIE3          (1<<PD1)

#define OUT_MASK_B        (PEWLEFTIN1 | PEWLEFTIN2 | PEWLEFTIN3 | PEWLEFTOUT1 | PEWLEFTOUT2 | PEWLEFTOUT3 | PEWRIGHTIN1 | PEWRIGHTIN2)
#define OUT_MASK_C        (PEWRIGHTIN3 | PEWRIGHTOUT1 | PEWRIGHTOUT2 | PEWRIGHTOUT3)
#define OUT_MASK_D        (SHIELDLEFTOUT | SHIELDLEFTIN | SHIELDRIGHTIN | SHIELDRIGHTOUT)

//Used to multiplex RED LEDs
#define FADE_RESOLUTION   96
volatile uint8_t bmatrix[FADE_RESOLUTION];
volatile uint8_t cmatrix[FADE_RESOLUTION];
volatile uint8_t dmatrix[FADE_RESOLUTION];
const uint8_t led_order[16] = { PEWLEFTOUT1, PEWLEFTOUT2, PEWLEFTOUT3, \
                                PEWLEFTIN1, PEWLEFTIN2, PEWLEFTIN3, \
                                PEWRIGHTIN1, PEWRIGHTIN2, PEWRIGHTIN3, \
                                PEWRIGHTOUT1, PEWRIGHTOUT2, PEWRIGHTOUT3, \
                                SHIELDLEFTOUT, SHIELDLEFTIN, SHIELDRIGHTIN, SHIELDRIGHTOUT};
const uint8_t scan_order[16] = { 0,1,2,3,4,5,3,4,5,0,1,2,0,2,3,5 };
const uint8_t pwm_distribution[16] = { 0, 8, 4, 12, 1, 9, 5, 13, 2, 10, 6, 14, 3, 7, 11, 15 };
const uint8_t logscale[18] = { 1,2,3,4,5,6,7,10,14, 16, 14, 10, 7, 6, 5, 4, 3, 2 };
const uint8_t sparkle_rando[22] = {18,2,0,14,4,5,3,15,10,13,6,17,11,8,21,9,19,16,12,1,20,7};

#define POST_DELAY          100
#define PULSATE_DELAY       40
#define CHARLIE_SPIN_DELAY  120
#define PEW_BOTH            8
#define PEW_INNER           4
#define PEW_OUTER           16
#define PEW_DELAY           80
#define SPARKLE_DELAY       40

volatile uint8_t charlie_array[6] = { 0, 0, 0, 0, 0, 0 };
const uint8_t charlie_spin_idx[6] = { 1,4,3,2,5,6 };

#define CHARLIE_MASK      (CHARLIE1 | CHARLIE2 | CHARLIE3)
#define CHARLIE_DDR       DDRD
#define CHARLIE_PORT      PORTD

volatile uint32_t ticks; //Used for upcounting milliseconds
uint32_t wait_until = 0;
uint32_t wait_until2 = 0;
uint32_t charlie_timer = 0;
uint8_t counter = 0;
uint8_t counter2 = 0;
uint8_t ignore_next_key_short = 0;

/**************************** Function Prototypes *****************************/
//Hardware specific functions
void Delay_ms(int cnt);
uint32_t get_time(void);
void init_io(void);
void disable_io(void);
void init_timers(void);
void init_pcint(void);
void disable_pcint(void);
void sleep_my_pretty(uint8_t timed);
void snooze(void);
void set_led(uint8_t lednum, uint8_t onoff);
void clear_charlie_array(void);
void charlie(uint8_t led_num);
void start_fade(void);
void fade_led(uint8_t lednum, uint8_t dimness);

//LED visualization functions
uint8_t post(uint8_t * step, uint32_t * next_step_time);
uint8_t pulsate(uint8_t * step_counter, uint32_t * next_step_time);
void init_pew(uint8_t counter_value, uint8_t counter2_value);
uint8_t pew(uint8_t * left_counter, uint32_t * left_next_step_time, uint8_t * right_counter, uint32_t * right_next_step_time);
uint8_t sparkle(uint8_t * step, uint32_t * next_step_time);

//State handling functions
void clean_slate(void);
void advance_state(uint8_t newstate);
void timed_advance(void);

/**************************** Hardware specific functions *****************************/
void Delay_ms(int cnt) {
	while (cnt-->0) {
		_delay_ms(1);
	}
}

uint32_t get_time(void) {
  //Return the upcounting milliseconds timer (interrupt driven)
  return ticks;
}

void init_io(void) {
  //Set up LED pins
  DDRB |= OUT_MASK_B;
  DDRC |= OUT_MASK_C;
  DDRD |= OUT_MASK_D;

  //Set up Chalieplex pins
  CHARLIE_DDR &= ~(CHARLIE_MASK);

  //Set up button input
  KEY_DDR &= ~KEY_MASK;
}

void disable_io(void) {
  //Will be used for sleep modes

  //Set up LED pins as inputs and disable pull-ups
  DDRB &= ~(OUT_MASK_B);
  PORTB &= ~(OUT_MASK_B);
  DDRC &= ~(OUT_MASK_C);
  PORTC &= ~(OUT_MASK_C);
  DDRD &= ~(OUT_MASK_D);
  PORTD &= ~(OUT_MASK_D);

  //All charlieplexing pins input (Hi-Z mode)
  CHARLIE_DDR &= ~(CHARLIE_MASK);
  CHARLIE_PORT &= ~(CHARLIE_MASK);

  //Buttons should already be set as inputs without pull-ups
  //they connect to VCC and have external pull-downs
}

void init_timers(void) {
  TCCR0B = 1<<CS01 | 1<<CS00;	//divide by 64
  TIMSK0 = 1<<TOIE0;		//enable overflow interrupt

  TCCR1B = 1<<WGM12 | 1<<CS11 | 1<<CS10;  //CTC mode with prescaler of 64
  OCR1AL = 20; // 6.2 kHz
  //TIMSK1 |= 1<<OCIE1A;

}

void init_pcint(void) {
  PCICR |= (1<<PCIE1);
  PCMSK1 |= (1<<PCINT9);
  PCIFR |= 1<<PCIF1;    //Clear the PCINT flag (might not have been done after last wakeup)
}

void disable_pcint(void) {
  PCICR &= ~(1<<PCIE1);
}

void sleep_my_pretty(uint8_t timed) {
  cli();                //disable interrupts
  disable_io();         //make sure we're not driving pins while asleep
  init_pcint();         //enable pin-change interrupts to wake from sleep
  if (timed) {
    wdt_reset();
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE | 1<<WDE);
    WDTCSR = (1<<WDIE | 1<<WDP3);
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //Set mode for lowest power
  sleep_enable();       //Get ready for sleep
  //sleep_bod_disable();  //Disable brown-out detection for lower power (apparently not for ATmega48
  sei();                //Ensure interrupts are enabled (lest we never wake)
  sleep_cpu();          //Sleep immediately after enabling interrupts so non can fire before sleep
  if (timed) {
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE | 1<<WDE);
    WDTCSR = 0;
  }
  disable_pcint();      //Don't need pin-change interrupts when awake
  sleep_disable();      //Disable to we're in a known state next time we need to sleep
  init_io();            //Get IO pins ready for wakeful operations
  ++ignore_next_key_short;
}

void set_led(uint8_t lednum, uint8_t onoff) {
  if (lednum < 8) {
    if (onoff) PORTB |= led_order[lednum];
    else PORTB &= ~led_order[lednum];
  }
  else if (lednum < 12) {
    if (onoff) PORTC |= led_order[lednum];
    else PORTC &= ~led_order[lednum];
  }
  else if (lednum < 16) {
    if (onoff) PORTD |= led_order[lednum];
    else PORTD &= ~led_order[lednum];
  }
  else if (lednum <22) {
    if (onoff) charlie(lednum-15);
    else charlie(0);
  }
}

void clear_charlie_array(void) {
  for (uint8_t i=0; i<6; i++) charlie_array[i] = 0;
}

void charlie(uint8_t led_num) {
  //All pins input (Hi-Z mode)
  CHARLIE_DDR &= ~(CHARLIE_MASK);
  CHARLIE_PORT &= ~(CHARLIE_MASK);
  
  switch(led_num) {
    case 0:
      //Pins already Hi-Z so all off
      return;
    case 1:
      //D17
      CHARLIE_DDR |= CHARLIE1;
      CHARLIE_DDR |= CHARLIE2;
      CHARLIE_PORT |= CHARLIE1;
      break;
    case 2:
      //D18
      CHARLIE_DDR |= CHARLIE1;
      CHARLIE_DDR |= CHARLIE2;
      CHARLIE_PORT |= CHARLIE2;
      break;
    case 3:
      //D19
      CHARLIE_DDR |= CHARLIE2;
      CHARLIE_DDR |= CHARLIE3;
      CHARLIE_PORT |= CHARLIE2;
      break;
    case 4:
      //D20
      CHARLIE_DDR |= CHARLIE2;
      CHARLIE_DDR |= CHARLIE3;
      CHARLIE_PORT |= CHARLIE3;
      break;
    case 5:
      //D21
      CHARLIE_DDR |= CHARLIE1;
      CHARLIE_DDR |= CHARLIE3;
      CHARLIE_PORT |= CHARLIE1;
      break;
    case 6:
      //D22
      CHARLIE_DDR |= CHARLIE1;
      CHARLIE_DDR |= CHARLIE3;
      CHARLIE_PORT |= CHARLIE3;
      break;
  };
}

void start_fade(void) { TIMSK1 |= 1<<OCIE1A; }

void fade_led(uint8_t lednum, uint8_t dimness) {
  volatile uint8_t * dimarray;
  if (lednum < 8) dimarray = bmatrix;
  else if (lednum < 12) dimarray = cmatrix;
  else if (lednum < 16) dimarray = dmatrix;
  else return;

  uint8_t row = scan_order[lednum];
  uint8_t col = led_order[lednum];

  for (uint8_t i=0; i<16; i++) { dimarray[(i*6)+row] &= ~col; } //Clear all set values

  for (uint8_t i=0; i<dimness; i++) {
    dimarray[((pwm_distribution[i]*6)+row)] |= col;
  }
}

/**************************** LED visualization functions *****************************/
uint8_t post(uint8_t * step, uint32_t * next_step_time) {
  if (get_time() < *next_step_time) return 0;

  *next_step_time = get_time() + POST_DELAY;
  PORTB &= ~(OUT_MASK_B);
  PORTC &= ~(OUT_MASK_C);
  CHARLIE_DDR &= ~(CHARLIE_MASK);
  CHARLIE_PORT &= ~(OUT_MASK_D | CHARLIE_MASK);

  if (*step > 21) return 1;

  set_led(*step, 1);
  *step += 1;
  return 0;
}

uint8_t pulsate(uint8_t * step_counter, uint32_t * next_step_time) {
  //step_counter is which frame of the animation is next
  //next_step_time needs to be reset so main knows when to next execute this function
  if (get_time() < *next_step_time) return 0;
  else {
    if ((TIMSK1 & 1<<OCIE1A) == 0) {
      start_fade();
      *step_counter = 0;
      clear_charlie_array();
    }
    for (uint8_t i=0; i<16; i++) fade_led(i,logscale[*step_counter]);
    if (++(*step_counter) >= 18) return 1;
    *next_step_time = get_time() + PULSATE_DELAY;
  }
  if (get_time() > charlie_timer) {
    static uint8_t charlie_spin = 0;
    charlie_timer = get_time() + CHARLIE_SPIN_DELAY;
    charlie_array[0] = charlie_spin_idx[charlie_spin];
    charlie_array[4] = charlie_spin_idx[charlie_spin+3];
    if (++charlie_spin > 2) charlie_spin = 0;
  }
  return 0;
}

void init_pew(uint8_t counter_value, uint8_t counter2_value) {
  //*left_counter = PEW_MAX;
  if (counter_value > 0) {
    counter = counter_value;
    for (uint8_t i=0; i<6; i++) set_led(i,0);
  }
  if (counter2_value > 0) {
    counter2 = counter2_value;
    for (uint8_t i=6; i<12; i++) set_led(i,0);
  }
}

void laser_left(uint8_t * left_counter) {
  //Little modulo hack allows firing innner/outer lasers at separate times
  switch((*left_counter)%8) {
    case 0: set_led(2,1); break;
    case 7: set_led(2,0); set_led(1,1); break;
    case 6: set_led(1,0); set_led(0,1); break;
    case 5: set_led(0,0); break;
    case 4: set_led(5,1); break;
    case 3: set_led(5,0); set_led(4,1); break;
    case 2: set_led(4,0); set_led(3,1); break;
    case 1: set_led(3,0); break;
  };
}

void laser_right(uint8_t * right_counter) {
  switch((*right_counter)%8) {
    case 0: set_led(11,1); break;
    case 7: set_led(11,0); set_led(10,1); break;
    case 6: set_led(10,0); set_led(9,1); break;
    case 5: set_led(9,0); break;
    case 4: set_led(8,1); break;
    case 3: set_led(8,0); set_led(7,1); break;
    case 2: set_led(7,0); set_led(6,1); break;
    case 1: set_led(6,0); break;
  };
}

uint8_t pew(uint8_t * left_counter, uint32_t * left_next_step_time, uint8_t * right_counter, uint32_t * right_next_step_time) {
  if ((*left_counter == 0) & (*right_counter == 0)) return 1;
  //set up callback timers
  uint32_t next = get_time() + PEW_DELAY;
  if (*left_counter == 12) *left_counter = 0;
  if (*left_counter > 0) {
    if (get_time() > *left_next_step_time) {
      *left_next_step_time = next;
      if (*left_counter == 5) *left_next_step_time += PEW_DELAY;
      laser_left(left_counter);
      --*left_counter;
    }
  }
  if (*right_counter == 12) *right_counter = 0;
  if (*right_counter > 0) {
    if (get_time() > *right_next_step_time) {
      *right_next_step_time = next;
      if (*right_counter == 5) *right_next_step_time += PEW_DELAY;
      laser_right(right_counter);
      --*right_counter;
    }
  }
  return 0;
}

uint8_t sparkle(uint8_t * step, uint32_t * next_step_time) {
  if (get_time() < *next_step_time) return 0;
  *next_step_time = get_time() + SPARKLE_DELAY;

  if (*step > 0) set_led(sparkle_rando[*step-1], 0);
  if (*step < 22) set_led(sparkle_rando[*step], 1);
  else return 1;
  *step += 1;
  return 0;
}

/**************************** State handling functions *****************************/
void clean_slate(void) {
  //Set all IO back to initialization
  TIMSK1 &= ~(1<<OCIE1A); //Stop the LED scanning timer
  //Set all direct drive LEDs to off
  PORTB &= ~OUT_MASK_B;
  PORTC &= ~OUT_MASK_C;
  PORTD &= ~OUT_MASK_D;
  //Shut down Charlieplex
  charlie(0);
  clear_charlie_array();
  //Clear all fade values
  for (uint8_t i=0; i<FADE_RESOLUTION; i++) {
    bmatrix[i] = 0;
    cmatrix[i] = 0;
    dmatrix[i] = 0;
  }
  //Reset counters and wait times
  wait_until = 0;
  wait_until2 = 0;
  counter = 0;
  counter2 = 0;
}

void advance_state(uint8_t newstate) {
  // If newstate == 0 just increment the state
  clean_slate();
  
  if (newstate) state = newstate;
  else ++state;
  if (state > STATE_SPARKLE) state = STATE_POST;
  
  switch(state) {
    case STATE_NOSTATE:
      break;
    case STATE_CONFIRMSLEEP:
      break;
    case STATE_POST:
      break;
    case STATE_FADE:
      start_fade();
      break;
    case STATE_PEW:
      init_pew(PEW_BOTH, PEW_BOTH);      
      break;
    case STATE_MANUALPEW:
      break;
    case STATE_WAIT:
        wait_until = get_time() + 4000;
        break;
    case STATE_SPARKLE:
      break;   
  };
}

void timed_advance(void) {
  sleep_my_pretty(SNOOZE);
  advance_state(0);
}

/**************************** Main program function *****************************/
int main(void)
{
  ticks = 0;
  state = STATE_POST;
  wait_until = 0;
  wait_until2 = 0;
  charlie_timer = 0;
  counter = 0;
  counter2 = 0;
  init_io();
  uint8_t left_laser_tracker = 0;
  uint8_t right_laser_tracker = 0;
  uint8_t previous_state = STATE_FADE;
  
  advance_state(STATE_POST);

  init_timers();
  sei();

  for (uint8_t i=0; i<16; i++) fade_led(i,16);

  while(1)
  {
    switch(state) {
      case STATE_NOSTATE:
        break;
      case STATE_CONFIRMSLEEP:
        break;
      case STATE_POST:         
          if (post(&counter, &wait_until)) timed_advance();
        break;
      case STATE_FADE:
        if (pulsate(&counter, &wait_until)) {
          if (++counter2 > 5) timed_advance();
          else counter = 0;
        }
        break;
      case STATE_PEW:
        if (pew(&counter, &wait_until, &counter2, &wait_until2)) advance_state(0);
        break;
      case STATE_MANUALPEW:
        if (pew(&counter, &wait_until, &counter2, &wait_until2)) advance_state(0);
        break;
      case STATE_WAIT:
        if (get_time() > wait_until) advance_state(0);
        break;
      case STATE_SPARKLE:
        if (sparkle(&counter, &wait_until)) {
          if (++counter2 > 5) timed_advance();
          else counter = 0;
        }
        break;
    };
    
    if(get_key_short(KEY1)) {
      if (ignore_next_key_short) { ignore_next_key_short = 0; }
      else {
        advance_state(STATE_MANUALPEW);
        if (right_laser_tracker < 2) init_pew(0,PEW_INNER);
        else init_pew(0,PEW_OUTER);
        if (++right_laser_tracker > 3) right_laser_tracker = 0;
      } 
    }
    if(get_key_rpt(KEY1)) {
      advance_state(0);
      ++ignore_next_key_short;
    }

    if(get_key_short(KEY0)) {
      if (state == STATE_CONFIRMSLEEP) { advance_state(previous_state); }
      else if(ignore_next_key_short) { ignore_next_key_short = 0; }
      else {
        advance_state(STATE_MANUALPEW);
        if (left_laser_tracker < 2) init_pew(PEW_INNER,0);
        else init_pew(PEW_OUTER,0);
        if (++left_laser_tracker > 3) left_laser_tracker = 0;
      }
    }
    if(get_key_rpt(KEY0)) {
      if (state != STATE_CONFIRMSLEEP) {
        previous_state = state;
        advance_state(STATE_CONFIRMSLEEP);
        fade_led(0,16);
        fade_led(3,16);
        fade_led(6,16);
        fade_led(9,16);
        charlie_array[0] = 1;
        charlie_array[4] = 2;
        counter = 3;
        start_fade();
      }
      else {
        if (counter == 0) { 
          fade_led(counter, 0);
          while(KEY_PIN & KEY0) { ;; } //wait for key release
          sleep_my_pretty(HIBERNATE);
          advance_state(previous_state);
        }
        else {
          fade_led(counter*3,0);
          --counter;
        }
      }
      //++ignore_next_key_short;
    }
  }
}

/**************************** Interrupt servicing functions *****************************/
ISR(PCINT1_vect) {
  key_state |= KEY_MASK;  //Clear the key press (wake only, don't react as if user input)
}

ISR(WDT_vect) {
  //Don't actually need an ISR to wake from sleep
}


ISR(TIMER0_OVF_vect)           // every 1ms
{
  ++ticks;

  TCNT0 = (unsigned char)(signed short)-(((F_CPU / 64) * .001) + 0.5);   // preload for 1ms

  if (((uint8_t)ticks%10) == 0) {
    key_isr();
  }
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t dimness = 0;
  PORTB &= ~(OUT_MASK_B);
  PORTB |= bmatrix[dimness];
  PORTC &= ~(OUT_MASK_C);
  PORTC |= cmatrix[dimness];
  PORTD &= ~(OUT_MASK_D);
  PORTD |= dmatrix[dimness];

  static uint8_t c = 0;
  charlie(charlie_array[c]);
  if (++c>5) c=0;

  if (++dimness >= FADE_RESOLUTION) dimness = 0;
  
}
