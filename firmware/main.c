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

#define BUT1              (1<<PC0)
#define BUT2              (1<<PC1)

#define OUT_MASK_B        (PEWLEFTIN1 | PEWLEFTIN2 | PEWLEFTIN3 | PEWLEFTOUT1 | PEWLEFTOUT2 | PEWLEFTOUT3 | PEWRIGHTIN1 | PEWRIGHTIN2)
#define OUT_MASK_C        (PEWRIGHTIN3 | PEWRIGHTOUT1 | PEWRIGHTOUT2 | PEWRIGHTOUT3)
#define OUT_MASK_D        (SHIELDLEFTOUT | SHIELDLEFTIN | SHIELDRIGHTOUT | SHIELDRIGHTOUT)

#define CHARLIE_MASK      (CHARLIE1 | CHARLIE2 | CHARLIE3)
#define CHARLIE_DDR       DDRD
#define CHARLIE_PORT      PORTD

#define IN_MASK_C         (BUT1 | BUT2)

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
  DDRC &= ~(IN_MASK_C);
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

void sleep_my_pretty(void) {
  cli();            //disable interrupts
  disable_io();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();   //Get ready for sleep
  sleep_bod_disable();
  sei();            //Ensure interrupts are enabled (lest we never wake)
  sleep_cpu();      //Sleep immediately after enabling interrupts so non can fire before sleep
  sleep_disable();  //Disable to we're in a known state next time we need to sleep
  init_io();        //Get IO pins ready for wakeful operations
}

int main(void)
{
  init_io();

  PCICR |= (1<<PCIE1);
  PCMSK1 |= (1<<PCINT9);
  sei();
  
  while(1)
  {
    //TODO: Implement button presses to enter sleep mode
    //post();
    //if (PINC & BUT2) PORTB |= PEWLEFTIN1;
    if (PINC & BUT1) {
      sleep_my_pretty();
    }
  }
}

ISR(PCINT1_vect) {
  PORTB |= PEWLEFTIN1;
}
