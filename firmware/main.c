#define F_CPU 8000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

void Delay_ms(int cnt);

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
#define OUT_MASK_D        (SHIELDLEFTOUT | SHIELDLEFTIN | SHIELDRIGHTOUT | SHIELDRIGHTOUT)

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

void Delay_ms(int cnt) {
	while (cnt-->0) {
		_delay_ms(1);
	}
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
  TCCR0B = 1<<CS02 | 1<<CS00;	//divide by 1024
  TIMSK0 = 1<<TOIE0;		//enable overflow interrupt
}

void disable_timers(void) {
  TIMSK0 &= 1<<TOIE0;
}

void init_pcint(void) {
  PCICR |= (1<<PCIE1);
  PCMSK1 |= (1<<PCINT9);
}

void disable_pcint(void) {
  PCICR &= ~(1<<PCIE1);
}

void toggle_pin(char * pinreg, uint8_t pin) {
  *pinreg |= pin;
  Delay_ms(500);
  *pinreg |= pin;
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

  toggle_pin(&PINB, PEWLEFTOUT1);
  toggle_pin(&PINB, PEWLEFTOUT2);
  toggle_pin(&PINB, PEWLEFTOUT3);
  toggle_pin(&PINB, PEWLEFTIN1);
  toggle_pin(&PINB, PEWLEFTIN2);
  toggle_pin(&PINB, PEWLEFTIN3);
  toggle_pin(&PINB, PEWRIGHTIN1);
  toggle_pin(&PINB, PEWRIGHTIN2);
  toggle_pin(&PINC, PEWRIGHTIN3);
  toggle_pin(&PINC, PEWRIGHTOUT1);
  toggle_pin(&PINC, PEWRIGHTOUT2);
  toggle_pin(&PINC, PEWRIGHTOUT3);

  toggle_pin(&PIND, SHIELDLEFTOUT);
  toggle_pin(&PIND, SHIELDLEFTIN);
  toggle_pin(&PIND, SHIELDRIGHTIN);
  toggle_pin(&PIND, SHIELDRIGHTOUT);


  for (uint8_t i=1; i<7; i++) {
    charlie(i);
    Delay_ms(500);
  }
  //Shut of Charlieplexed LEDS
  charlie(0);
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
  //disable_timers();
  init_pcint();         //enable pin-change interrupts to wake from sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //Set mode for lowest power
  sleep_enable();       //Get ready for sleep
  sleep_bod_disable();  //Disable brown-out detection for lower power
  sei();                //Ensure interrupts are enabled (lest we never wake)
  sleep_cpu();          //Sleep immediately after enabling interrupts so non can fire before sleep
  cli();
  sleep_disable();      //Disable to we're in a known state next time we need to sleep
  disable_pcint();      //Don't need pin-change interrupts when awake
  init_io();            //Get IO pins ready for wakeful operations
  //init_timers();
  sei();
}

int main(void)
{
  init_io();
  init_timers();
  sei();
  
  while(1)
  {
    //TODO: make post() non-blocking
    //post();
    if(get_key_press(KEY1)) PINB |= PEWLEFTIN2;
    //if(get_key_press(KEY0)) sleep_my_pretty();
    if(PINC & KEY0) sleep_my_pretty();
  }
}

/*
ISR(PCINT1_vect) {
  //Don't actually need an ISR to wake from sleep
}
*/

ISR(TIMER0_OVF_vect)           // every 10ms
{
  static unsigned char ct0, ct1, rpt;
  unsigned char i;

  TCNT0 = (unsigned char)(signed short)-(((F_CPU / 1024) * .01) + 0.5);   // preload for 10ms

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

