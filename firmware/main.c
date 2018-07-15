#define F_CPU 8000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "debounce.h"

#define STATE_NOSTATE       0
#define STATE_CONFIRMSLEEP  1
#define STATE_CONFIRMWAKE   2
#define STATE_ASLEEP        3
#define STATE_POST          4
#define STATE_SWEEP         5
#define STATE_FADE          6
#define STATE_PEW           7
#define STATE_MANUALPEW     8
#define STATE_WAIT          9
#define STATE_SPARKLE       10

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
const uint8_t sweep0[6] = { 0,3,6,9,6,3 };
const uint8_t sweep1[6] = { 13,12,14,15,14,12 };
const uint8_t sweep2[6] = { 2,5,8,11,8,5 };
const uint8_t sweep3[6] = { 3, 1, 2, 6, 2, 1 };
uint8_t laser_l[6] = { 2,1,0,5,4,3 };
uint8_t laser_r[6] = { 11,10,9,8,7,6 };

#define POST_DELAY          100
#define PULSATE_DELAY       40
#define CHARLIE_SPIN_DELAY  120
#define PEW_BOTH            8
#define PEW_INNER           4
#define PEW_OUTER           16
#define PEW_DELAY           80
#define SPARKLE_DELAY       40
#define HARD_SLEEP_PERIOD   (uint32_t)500*60*90 //90-minutes sleep (badge snoozes rouhgly 1/2 the time so 500 == 1second)

volatile uint8_t charlie_array[6] = { 0, 0, 0, 0, 0, 0 };
const uint8_t charlie_spin_idx[6] = { 1,4,3,2,5,6 };

#define CHARLIE_MASK      (CHARLIE1 | CHARLIE2 | CHARLIE3)
#define CHARLIE_DDR       DDRD
#define CHARLIE_PORT      PORTD

volatile uint32_t ticks; //Used for upcounting milliseconds

struct TrackTime {
  uint8_t counter0, counter1;
  uint32_t wait_until0, wait_until1;
} tt;

uint8_t generic_counter;
uint32_t hard_sleep_time = HARD_SLEEP_PERIOD;
uint32_t charlie_timer = 0;
uint8_t ignore_next_key_short = 0;

/**************************** Function Prototypes *****************************/
//Hardware specific functions
uint32_t get_time(void);
void init_io(void);
void disable_io(void);
void init_timers(void);
void init_pcint(void);
void disable_pcint(void);
void disable_button_int(void);
void enable_button_int(void);
void sleep_my_pretty(uint8_t timed);
void set_led(uint8_t lednum, uint8_t onoff);
void clear_charlie_array(void);
void charlie(uint8_t led_num);
void start_fade(void);
void fade_led(uint8_t lednum, uint8_t dimness);

//LED visualization functions
uint8_t post(struct TrackTime *ptt);
uint8_t pulsate(struct TrackTime *ptt);
void init_pew(uint8_t counter0_value, uint8_t counter1_value);
void laser(uint8_t counter_value, uint8_t laser_led[]);
void laser_turret_servicer(uint8_t * counter, uint32_t * wait_until, uint32_t nexttime, uint8_t laserarray[]);
uint8_t pew(struct TrackTime *ptt);
uint8_t sparkle(struct TrackTime *ptt);
void sweep_helper(uint8_t step, const uint8_t sweep_idx[]);
uint8_t sweep(struct TrackTime *ptt);

//State handling functions
void clean_slate(void);
void advance_state(uint8_t newstate);
void timed_advance(void);
void verify_sleep(uint8_t previous_state);
void dont_wake_early(uint8_t previous_state);
void inc_hard_sleep(void);

/**************************** Hardware specific functions *****************************/

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
  PCMSK1 |= (1<<PCINT8);
  PCIFR |= 1<<PCIF1;    //Clear the PCINT flag (might not have been done after last wakeup)
}

void disable_pcint(void) {
  PCICR &= ~(1<<PCIE1);
}

void disable_button_int(void) {
  TIMSK0 &= ~(1<<TOIE0);		//enable overflow interrupt
}

void enable_button_int(void) {
  TIMSK0 |= 1<<TOIE0;		//enable overflow interrupt
}

void sleep_my_pretty(uint8_t timed) {
  cli();                //disable interrupts
  disable_io();         //make sure we're not driving pins while asleep
  init_pcint();         //enable pin-change interrupts to wake from sleep
  if (timed) {
    PCMSK1 |= 1<<PCINT9;  //Only need pcint on KEY1 when snoozing
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
    PCMSK1 &= ~(1<<PCINT9); //Only need pcint on KEY1 when snoozing
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE | 1<<WDE);
    WDTCSR = 0;
  }
  disable_pcint();      //Don't need pin-change interrupts when awake
  sleep_disable();      //Disable to we're in a known state next time we need to sleep
  init_io();            //Get IO pins ready for wakeful operations
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
uint8_t post(struct TrackTime *ptt) {
  if (get_time() < ptt->wait_until0) return 0;

  ptt->wait_until0 = get_time() + POST_DELAY;
  PORTB &= ~(OUT_MASK_B);
  PORTC &= ~(OUT_MASK_C);
  //CHARLIE_DDR &= ~(CHARLIE_MASK);
  //CHARLIE_PORT &= ~(OUT_MASK_D | CHARLIE_MASK);

  if (ptt->counter0 > 21) return 1;

  set_led(ptt->counter0, 1);
  ptt->counter0 += 1;
  return 0;
}

uint8_t pulsate(struct TrackTime *ptt) {
  //step_counter is which frame of the animation is next
  //next_step_time needs to be reset so main knows when to next execute this function
  if (get_time() < ptt->wait_until0) return 0;
  else {
    if ((TIMSK1 & 1<<OCIE1A) == 0) {
      start_fade();
      ptt->counter0 = 0;
      clear_charlie_array();
    }
    for (uint8_t i=0; i<16; i++) fade_led(i,logscale[ptt->counter0]);
    if (++(ptt->counter0) >= 18) return 1;
    ptt->wait_until0 = get_time() + PULSATE_DELAY;
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

void init_pew(uint8_t counter0_value, uint8_t counter1_value) {
  if (counter0_value > 0) {
    tt.counter0 = counter0_value;
    for (uint8_t i=0; i<6; i++) set_led(i,0);
  }
  if (counter1_value > 0) {
    tt.counter1 = counter1_value;
    for (uint8_t i=6; i<12; i++) set_led(i,0);
  }
}

void laser(uint8_t counter_value, uint8_t laser_led[]) {
  //Little modulo hack allows firing innner/outer lasers at separate times
  switch(counter_value%8) {
    case 0: set_led(laser_led[0],1); break;
    case 7: set_led(laser_led[0],0); set_led(laser_led[1],1); break;
    case 6: set_led(laser_led[1],0); set_led(laser_led[2],1); break;
    case 5: set_led(laser_led[2],0); break;
    case 4: set_led(laser_led[3],1); break;
    case 3: set_led(laser_led[3],0); set_led(laser_led[4],1); break;
    case 2: set_led(laser_led[4],0); set_led(laser_led[5],1); break;
    case 1: set_led(laser_led[5],0); break;
  };
}

void laser_turret_servicer(uint8_t * counter, uint32_t * wait_until, uint32_t nexttime, uint8_t laserarray[]) {
  if (*counter == 12) *counter = 0;
  if (*counter > 0) {
    if (get_time() > *wait_until) {
      *wait_until = nexttime;
      if (*counter == 5) *wait_until += PEW_DELAY;
      laser(*counter,laserarray);
      --(*counter);
    }
  }
}

uint8_t pew(struct TrackTime *ptt) {
  if ((ptt->counter0 == 0) & (ptt->counter1 == 0)) return 1;
  //set up callback timers
  uint32_t next = get_time() + PEW_DELAY;
  laser_turret_servicer(&(ptt->counter0),&(ptt->wait_until0), next, laser_l);
  laser_turret_servicer(&(ptt->counter1),&(ptt->wait_until1), next, laser_r);
  return 0;
}

uint8_t animate_pew(struct TrackTime *ptt) {
  if (get_time() < charlie_timer) return 0;
  switch (generic_counter) {
    case 0:
      init_pew(PEW_BOTH, PEW_BOTH);
      break;
    case 1:
      if (pew(ptt) == 0) return 0;
      break;
    case 2:
      init_pew(PEW_OUTER,0);
      break;
    case 3:
      if (pew(ptt) == 0) return 0;
      break;
    case 4:
      init_pew(PEW_OUTER,0);
      break;
    case 5:
      if (pew(ptt) == 0) return 0;
      break;
    case 6:
      init_pew(0,PEW_INNER);
      break;
    case 7:
      if (pew(ptt) == 0) return 0;
      break;
    case 8:
      init_pew(0,PEW_INNER);
      break;
    case 9:
      if (pew(ptt) == 0) return 0;
      break;
    case 10:
      init_pew(0,PEW_OUTER);
      break;
    case 11:
      if (pew(ptt) == 0) return 0;
      break;
    case 12:
      init_pew(0,PEW_OUTER);
      break;
    case 13:
      if (pew(ptt) == 0) return 0;
      break;
    case 14:
      init_pew(PEW_INNER,0);
      break;
    case 15:
      if (pew(ptt) == 0) return 0;
      break;
    case 16:
      init_pew(PEW_INNER,0);
      break;
    case 17:
      if (pew(ptt) == 0) return 0;
      break;
    case 18:
      for (uint8_t i=1; i<7; i++) charlie_array[i-1] = i;
      charlie_timer = get_time() + 2000;
      fade_led(12,8);
      fade_led(13,8);
      fade_led(14,8);
      fade_led(15,8);
      start_fade();
      break;
    default:
      return 1;
      break;
  };
  ++generic_counter;
  return 0;  
}

uint8_t sparkle(struct TrackTime *ptt) {
  if (get_time() < ptt->wait_until0) return 0;
  ptt->wait_until0 = get_time() + SPARKLE_DELAY;

  if (ptt->counter0 > 0) set_led(sparkle_rando[ptt->counter0-1], 0);
  if (ptt->counter0 < 22) set_led(sparkle_rando[ptt->counter0], 1);
  else return 1;
  ptt->counter0 += 1;
  return 0;
}


void sweep_helper(uint8_t step, const uint8_t sweep_idx[]) {
    fade_led(sweep_idx[((6+step)-1)%6],0);
    fade_led(sweep_idx[step%6],16);
}

uint8_t sweep(struct TrackTime *ptt) {
  //0,3,6,9,6,3
  if (get_time() > ptt->wait_until0) {
    ptt->wait_until0 = get_time() + 120;

    sweep_helper(ptt->counter0, sweep0);
    sweep_helper(ptt->counter0, sweep1);
    sweep_helper(ptt->counter0, sweep2);

    charlie_array[0] = sweep3[ptt->counter0%6];
    if (++(ptt->counter0) > 55) return 1;
  } 
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
  charlie_timer = 0;
  generic_counter = 0;
  //Clear all fade values
  for (uint8_t i=0; i<FADE_RESOLUTION; i++) {
    bmatrix[i] = 0;
    cmatrix[i] = 0;
    dmatrix[i] = 0;
  }
  //Reset counters and wait times
  tt.wait_until0 = 0;
  tt.wait_until1 = 0;
  tt.counter0 = 0;
  tt.counter1 = 0;
}

void advance_state(uint8_t newstate) {
  // If newstate == 0 just increment the state
  clean_slate();
  
  if (newstate) state = newstate;
  else ++state;
  if (state > STATE_SPARKLE) state = STATE_POST;
  if (get_time() > hard_sleep_time) state=STATE_ASLEEP; //Hibernate if no buttons pushed in a long time
  
  switch(state) {
    case STATE_NOSTATE:
      break;
    case STATE_CONFIRMSLEEP:
      fade_led(0,16);
      fade_led(3,16);
      fade_led(6,16);
      fade_led(9,16);
      start_fade();
      break;
    case STATE_POST:
      break;
    case STATE_SWEEP:
      start_fade();
      break;
    case STATE_FADE:
      start_fade();
      break;
    case STATE_PEW:      
      break;
    case STATE_MANUALPEW:
      break;
    case STATE_WAIT:
        tt.wait_until0 = get_time() + 4000;
        break;
    case STATE_SPARKLE:
      break;   
  };
}

void timed_advance(void) {
  sleep_my_pretty(SNOOZE);
  advance_state(0);
}

void verify_sleep(uint8_t previous_state) {
  while(1) {
    if(get_key_short(KEY0)) {
      advance_state(previous_state);
      return;
    }
    if(get_key_rpt(KEY0)) {
      if (tt.counter0 > 2) { 
        fade_led(0, 0);
        disable_button_int();
        while(KEY_PIN & KEY0) { ;; } //Wait for key release
        enable_button_int();
        advance_state(STATE_ASLEEP);
        return;
      }
      else {
        fade_led((3-tt.counter0)*3,0);
        ++(tt.counter0);
      }
    }
  }
}

void dont_wake_early(uint8_t previous_state) {
  start_fade();
  sleep_my_pretty(HIBERNATE);
  tt.counter0 = 0;
  fade_led(tt.counter0, 16);
  start_fade();
  while(1) {
    if(get_key_rpt(KEY0)) {
      if (tt.counter0 > 2) {
        fade_led(9,16);
        disable_button_int();
        while(KEY_PIN & KEY0) { ;; } //wait for key release
        enable_button_int();
        inc_hard_sleep();
        advance_state(previous_state);
        ignore_next_key_short |= KEY0;
        return;
      }
      else {
        fade_led(tt.counter0*3,16);
        ++(tt.counter0);
      }
    }
    if(get_key_short(KEY0)) {
      advance_state(STATE_ASLEEP);
      return;
    }
  }
}

void inc_hard_sleep(void) { hard_sleep_time = get_time() + HARD_SLEEP_PERIOD; }

/**************************** Main program function *****************************/
int main(void)
{
  ticks = 0;
  state = STATE_POST;

  init_io();
  uint8_t left_laser_tracker = 0;
  uint8_t right_laser_tracker = 0;
  uint8_t previous_state = STATE_FADE;
  
  advance_state(STATE_POST);  //This will initialize the global variables too

  init_timers();
  sei();

  for (uint8_t i=0; i<16; i++) fade_led(i,16);

  while(1)
  {
    switch(state) {
      case STATE_NOSTATE:
        break;
      case STATE_CONFIRMSLEEP:
        verify_sleep(previous_state);
        break;
      case STATE_ASLEEP:
        dont_wake_early(previous_state);
        break;
      case STATE_POST:         
          if (post(&tt)) timed_advance();
        break;
      case STATE_SWEEP:
        if (sweep(&tt)) timed_advance();
        break;
      case STATE_FADE:
        if (pulsate(&tt)) {
          if (++tt.counter1 > 5) timed_advance();
          else tt.counter0 = 0;
        }
        break;
      case STATE_PEW:
        if (animate_pew(&tt)) advance_state(0);
        break;
      case STATE_MANUALPEW:
        if (pew(&tt)) advance_state(0);
        break;
      case STATE_WAIT:
        if (get_time() > tt.wait_until0) advance_state(0);
        break;
      case STATE_SPARKLE:
        if (sparkle(&tt)) {
          if (++(tt.counter1) > 5) timed_advance();
          else tt.counter0 = 0;
        }
        break;
    };
    
    if(get_key_short(KEY1)) {
      if (ignore_next_key_short & KEY1) { ignore_next_key_short &= ~KEY1; }
      else {
        advance_state(STATE_MANUALPEW);
        if (right_laser_tracker < 2) init_pew(0,PEW_INNER);
        else init_pew(0,PEW_OUTER);
        if (++right_laser_tracker > 3) right_laser_tracker = 0;
        inc_hard_sleep();
      } 
    }
    if(get_key_rpt(KEY1)) {
      advance_state(0);
      ignore_next_key_short |= KEY1;
      inc_hard_sleep();
    }

    if(get_key_short(KEY0)) {
      if(ignore_next_key_short & KEY0) { ignore_next_key_short &= ~KEY0; }
      else {
        advance_state(STATE_MANUALPEW);
        if (left_laser_tracker < 2) init_pew(PEW_INNER,0);
        else init_pew(PEW_OUTER,0);
        if (++left_laser_tracker > 3) left_laser_tracker = 0;
        inc_hard_sleep();
      }
    }
    if(get_key_rpt(KEY0)) {
      if ((state != STATE_CONFIRMSLEEP) && (state != STATE_ASLEEP)) previous_state = state;
      advance_state(STATE_CONFIRMSLEEP);
      inc_hard_sleep();
    }
  }
}

/**************************** Interrupt servicing functions *****************************/
ISR(PCINT1_vect) {
  //Don't actually need an ISR to wake from sleep
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
