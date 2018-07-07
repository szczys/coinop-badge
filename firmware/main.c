#define F_CPU 8000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

void Delay_ms(int cnt);

#define STATE_NOSTATE     0
#define STATE_POST        1
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
volatile uint8_t bmatrix[6];
volatile uint8_t cmatrix[6];
volatile uint8_t dmatrix[6];
const uint8_t led_order[16] = { PEWLEFTOUT1, PEWLEFTOUT2, PEWLEFTOUT3, \
                                PEWLEFTIN1, PEWLEFTIN2, PEWLEFTIN3, \
                                PEWRIGHTIN1, PEWRIGHTIN2, PEWRIGHTIN3, \
                                PEWRIGHTOUT1, PEWRIGHTOUT2, PEWRIGHTOUT3, \
                                SHIELDLEFTOUT, SHIELDLEFTIN, SHIELDRIGHTIN, SHIELDRIGHTOUT};
const uint8_t scan_order[16] = { 0,1,2,3,4,5,3,4,5,0,1,2,0,2,3,5 };

#define CHARLIE_MASK      (CHARLIE1 | CHARLIE2 | CHARLIE3)
#define CHARLIE_DDR       DDRD
#define CHARLIE_PORT      PORTD

#define KEY0              (1<<PC0)
#define KEY1              (1<<PC1)
#define KEY_DDR           DDRC
#define KEY_PORT          PORTC
#define KEY_PIN           PINC
#define KEY_MASK          (KEY0 | KEY1)

/*********************** Debounce ****************************
* Info on debounce code:
* https://github.com/szczys/Button-Debounce/blob/master/debounce-test.c 
*/
//Debounce
#define REPEAT_MASK   (KEY0 | KEY1)   // repeat: key1, key2 
#define REPEAT_START   50      // after 500ms 
#define REPEAT_NEXT   20      // every 200ms
volatile unsigned char key_press;
volatile unsigned char key_state;
volatile unsigned char key_rpt;
/************************************************************/

//Used for upcounting milliseconds
volatile uint32_t ticks;

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
  OCR1AL = 125; // 1 kHz
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

void post(void) {
  static uint8_t post_tracker = 0;
  PORTB &= ~(OUT_MASK_B);
  PORTC &= ~(OUT_MASK_C);
  CHARLIE_DDR &= ~(CHARLIE_MASK);
  CHARLIE_PORT &= ~(OUT_MASK_D | CHARLIE_MASK);

  if (post_tracker > 22) {
    post_tracker = 0;
    state = STATE_NOSTATE;
    return;
  }

  switch(post_tracker++) {
    case 0: PORTB |= PEWLEFTOUT1; break;
    case 1: PORTB |= PEWLEFTOUT2; break;
    case 2: PORTB |= PEWLEFTOUT3; break;
    case 3: PORTB |= PEWLEFTIN1; break;
    case 4: PORTB |= PEWLEFTIN2; break;
    case 5: PORTB |= PEWLEFTIN3; break;
    case 6: PORTB |= PEWRIGHTIN1; break;
    case 7: PORTB |= PEWRIGHTIN2; break;
    case 8: PORTC |= PEWRIGHTIN3; break;
    case 9: PORTC |= PEWRIGHTOUT1; break;
    case 10: PORTC |= PEWRIGHTOUT2; break;
    case 11: PORTC |= PEWRIGHTOUT3; break;
    case 12: PORTD |= SHIELDLEFTOUT; break;
    case 13: PORTD |= SHIELDLEFTIN; break;
    case 14: PORTD |= SHIELDRIGHTIN; break;
    case 15: PORTD |= SHIELDRIGHTOUT; break;
    //Hack to run charlie([1:6]);
    default: charlie(post_tracker-16); break; // (16 because we already incremented)
    };
}

/*--------------------------------------------------------------------------
  FUNC: 8/1/11 - Used to read debounced button presses
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a button press
--------------------------------------------------------------------------*/
unsigned char get_key_press( unsigned char key_mask )
{
  cli();			// read and clear atomic !
  key_mask &= key_press;	// read key(s)
  key_press ^= key_mask;	// clear key(s)
  sei();
  return key_mask;
}

/*--------------------------------------------------------------------------
  FUNC: 8/1/11 - Used to check for debounced buttons that are held down
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits is a button held long enough for
		its input to be repeated
--------------------------------------------------------------------------*/
unsigned char get_key_rpt( unsigned char key_mask ) 
{ 
  cli();               // read and clear atomic ! 
  key_mask &= key_rpt;                           // read key(s) 
  key_rpt ^= key_mask;                           // clear key(s) 
  sei(); 
  return key_mask; 
} 

/*--------------------------------------------------------------------------
  FUNC: 8/1/11 - Used to read debounced button released after a short press
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a quick press and release
--------------------------------------------------------------------------*/
unsigned char get_key_short( unsigned char key_mask ) 
{ 
  cli();         // read key state and key press atomic ! 
  return get_key_press( ~key_state & key_mask ); 
} 

/*--------------------------------------------------------------------------
  FUNC: 8/1/11 - Used to read debounced button held for REPEAT_START amount
	of time.
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a long button press
--------------------------------------------------------------------------*/
unsigned char get_key_long( unsigned char key_mask ) 
{ 
  return get_key_press( get_key_rpt( key_mask )); 
} 

void sleep_my_pretty(void) {
  cli();                //disable interrupts
  disable_io();         //make sure we're not driving pins while asleep
  init_pcint();         //enable pin-change interrupts to wake from sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //Set mode for lowest power
  sleep_enable();       //Get ready for sleep
  //sleep_bod_disable();  //Disable brown-out detection for lower power (apparently not for ATmega48
  sei();                //Ensure interrupts are enabled (lest we never wake)
  sleep_cpu();          //Sleep immediately after enabling interrupts so non can fire before sleep
  disable_pcint();      //Don't need pin-change interrupts when awake
  sleep_disable();      //Disable to we're in a known state next time we need to sleep
  init_io();            //Get IO pins ready for wakeful operations
}

void set_led(uint8_t lednum, uint8_t onoff) {
  if (lednum < 8) {
    if (onoff) bmatrix[scan_order[lednum]] |= led_order[lednum];
    else bmatrix[scan_order[lednum]] &= ~led_order[lednum];
  }
  else if (lednum < 12) {
    if (onoff) cmatrix[scan_order[lednum]] |= led_order[lednum];
    else cmatrix[scan_order[lednum]] &= ~led_order[lednum];
  }
  else if (lednum < 16) {
    if (onoff) dmatrix[scan_order[lednum]] |= led_order[lednum];
    else dmatrix[scan_order[lednum]] &= ~led_order[lednum];
  }
}

int main(void)
{
  ticks = 0;
  state = STATE_POST;
  static uint32_t wait_until = 0;
  init_io();
  
  
  
  for (uint8_t i=0; i<6; i++) {
    bmatrix[i] = 0;
    cmatrix[i] = 0;
    dmatrix[i] = 0;
  }

  init_timers();
  sei();

  for (uint8_t i=0; i<16; i++) set_led(i,1);


  while(1)
  {
    switch(state) {
      case STATE_NOSTATE:
        TIMSK1 |= 1<<OCIE1A;
        while(1) {;;}
        break;
      case STATE_POST:
        if (get_time() > wait_until) {
          wait_until = get_time() + 100;
          post();
        }
        break;
    };
    
    if(get_key_press(KEY1)) {
      //PORTB |= PEWLEFTIN2;    
      PINB |= PEWLEFTIN2;
    }
    if(get_key_press(KEY0)) sleep_my_pretty();
  }
}


ISR(PCINT1_vect) {
  //Don't actually need an ISR to wake from sleep
}


ISR(TIMER0_OVF_vect)           // every 1ms
{
  static unsigned char ct0, ct1, rpt;
  unsigned char i;

  ++ticks;

  TCNT0 = (unsigned char)(signed short)-(((F_CPU / 64) * .001) + 0.5);   // preload for 1ms

  if (((uint8_t)ticks%10) == 0) {
    i = key_state ^ KEY_PIN;    // key changed ? (natural state is high so no need for ~KEY_PIN
    ct0 = ~( ct0 & i );          // reset or count ct0
    ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
    i &= ct0 & ct1;              // count until roll over ?
    key_state ^= i;              // then toggle debounced state
    key_press |= key_state & i;  // 0->1: key press detect

    if( (key_state & REPEAT_MASK) == 0 )   // check repeat function 
       rpt = REPEAT_START;      // start delay 
    if( --rpt == 0 ){ 
      rpt = REPEAT_NEXT;         // repeat delay 
      key_rpt |= key_state & REPEAT_MASK; 
    } 
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

  static uint8_t c = 1;
  charlie(c++);
  if (c>6) c=1;

  if (++dimness > 5) dimness = 0;
  
}
