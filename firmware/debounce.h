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

unsigned char get_key_press( unsigned char key_mask );
unsigned char get_key_rpt( unsigned char key_mask );
unsigned char get_key_short( unsigned char key_mask ); 
unsigned char get_key_long( unsigned char key_mask );

