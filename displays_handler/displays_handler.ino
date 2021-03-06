 /*file
 *      DISPLAY-HANDLER
 * \author
 *      Cristian Alderete <cristian.alderete.92@gmail.com>
 */
 
 // #include <EEPROM.h>

//#include "TimerOne.h"
#include <LiquidCrystal_I2C.h>     // DiSPLAY
#include "Adafruit_LEDBackpack.h" // LED BAR
#include <Servo.h>                // ENGINE

//resource general information.
#define MAX_RESOURCE_FORMAT_SIZE 15
#define ERROR_CODE 1
#define NUMBER_OF_RESOURCES 37

//DISPLAY BUTTON
#define BUTTON_PIN 2

//MENU SELECTION  - POSSIBLE MENUS
#define MAIN_MENU 1
#define MENU_CELLS_1_8 2
#define MENU_CELLS_9_16 3
#define MENU_CELLS_17_24 4
#define MENU_PWM_CURRENT 5
#define MENU_BATTERY_CHARGING 6
#define AMOUNT_OF_MENUS 6

//TO HANDLE MENUS AND LINES (distribution in display )
#define CHARS_PER_LINE 20
#define NUMBER_OF_LINES 4
#define KMS_VALUE_CURSOR 9
#define TMP_FB_VALUE_CURSOR 4
#define TMP_RB_VALUE_CURSOR 14
#define TMP_MB_VALUE_CURSOR 4
#define TMP_BMS_VALUE_CURSOR 15

#define CELL_1D_1_VALUE_CURSOR 3
#define CELL_1D_2_VALUE_CURSOR 13

#define CELL_2D_1_VALUE_CURSOR 4
#define CELL_2D_2_VALUE_CURSOR 15

#define PWM_VALUE_CURSOR 15
#define CURRENT_VALUE_CURSOR 9

#define Temp_Start_2 11
#define Cell_start2 11

//DISPLAY DISPLAY FORMAT
#define D_KILOMETRAGE "Km = "
#define D_TFB "Tfb="
#define D_TRB "Trb="
#define D_TMB "Tmb="
#define D_TBMS "Tbms="
#define D_TBMS "Tbms="
#define D_PWM_EC "Pwm to EC = "
#define D_PWM_BC "Pwm to BC = "
#define D_CURRENT "I = "


//KEY VALUE TO VERIFY RESOURCE (no corrupted data):
#define CAR_SPEED_KEY "km/h"
#define CAR_SoC_KEY "%bat"
#define CAR_KILOMETRAGE_KEY "kms"
#define PWM_EC_KEY "pec"   //PWM Engine Controler
#define PWM_BC_KEY "pbc"  //PWM Battery Charger
#define TEMP_FB_KEY "ta"
#define TEMP_RB_KEY "tb"
#define TEMP_MB_KEY "tc"
#define TEMP_BMS_KEY "td"
#define CELL_VOLT_KEY "cs" // a "." separates the cell value and number---> ej:  cs=18.649    (cell value is not devided by 200 yet (done in server) TODO : maybe do it here..

#define DEGREES_SYMBOL (char)223

//DATA DETAILS
//cells
#define NUM_CELLS 24
//temps
#define NUM_TEMPERATURES 4
//bar led
#define AMOUNT_OF_LEDS 24

//RESEOURCE ID
#define FIRST_CELL_VOLTAGE_ID 1
#define LAST_CELL_VOLTAGE_ID 24
#define TEMP_FB_ID 25
#define TEMP_RB_ID 26
#define TEMP_MB_ID 27
#define TEMP_BMS_ID 28
#define PWM_EC_ID 29
#define BATTERY_SOC_ID 30
#define PWM_BATTERY_CHARGER_ID 31
#define CAR_SPEED_ID 32
#define CAR_KILOMETRAGE_ID 33
#define BMS_CURRENT_ID 34

//#define EEPROM_KILOMETRAGE_ADDRESS 0
//INCLUDED OBJECTS
Servo myservo;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_24bargraph bar = Adafruit_24bargraph();


//To set things first time
static bool first_turn_on = true;

//cuRRENT MENU OF THE 

volatile static short int current_menu = MAIN_MENU;

//auxiliar flags
volatile static bool battery_charging = false;
volatile static bool toggle_menu_flag = false;
volatile static bool flag_timer = false;

//JUST THE VALUES AS CHARS.  ( TO USE ATOI if necessary )
static char resources_values[NUMBER_OF_RESOURCES][MAX_RESOURCE_FORMAT_SIZE];

//AUXILIAR FOR LED BAR
static short int number_leds_on;
static short int last_led_on = 0;
const int interval_effect=150; //MIlisecs
const int read_wait_max=100;  //milisecs


//AUXILIAR FOR TIME COMPARISON
static unsigned long contador_millis = 0;
static unsigned long previous_time=0;
static unsigned long initial_time=0;
unsigned long current_time=0;


//AVOID DEBOUNCE IN BUTTON
static unsigned long last_button_press=0;
static unsigned long current_button_press;

static unsigned long prev_time_speed=0;
static unsigned long curr_time_speed;
static int aux_speed=60;

static bool handling_lcd=false;
static bool led_bar=false;

static int reading_counter=0;



static long andando=0;
//EEPROM
//static int address_eeprom = EEPROM_KILOMETRAGE_ADDRESS ;


//TO TEST WITHOUT READING
//static int line_sent_counter = 0;
//static int battery_counter = 0;
//static int speed_counter = 0;
//static int kilometrage_counter = 0;
//static bool up_down=true;


//FUNCTIONS TO KEEP DATA IN MEMORY ( even if supply is turned off )#include "TimerTwo.h"
//void write_int_to_eeprom( int address, int value) {
//
//  byte two = (value & 0xFF);
//  byte one = ( (value >> 8) & 0xFF );
//
//  EEPROM.update(address,two);
//  EEPROM.update(address+1, one);
//
//}
//int read_int_from_eeprom( int address) {
//
//  byte two = EEPROM.read(address);
//  byte one = EEPROM.read(address+1);
//
//
//  return ( (two << 0) & 0xFFFFFF ) + ( (one << 8 ) & 0xFFFFFF );
//}

//
void button_interrupt() {
  //delay(50); // to avoid debounce when releasing the button, etc ( falling edge interrupt ) . check
  toggle_menu_flag = true;
}

void battery_charging_effects() {

  if (last_led_on == AMOUNT_OF_LEDS   || last_led_on <= 0) {
    last_led_on = number_leds_on ;

    for ( int i = number_leds_on; i < AMOUNT_OF_LEDS ; i++) {
      bar.setBar(i, LED_OFF);
    }
    bar.setBar(last_led_on, LED_GREEN);
    
     last_led_on++;
     if(last_led_on>AMOUNT_OF_LEDS){
      last_led_on=AMOUNT_OF_LEDS;
     }
//    if (last_led_on == 0) {
//      last_led_on++;
//    }
  } else {
    bar.setBar(last_led_on-1, LED_OFF);
    last_led_on++;
     if(last_led_on>AMOUNT_OF_LEDS){
       last_led_on=AMOUNT_OF_LEDS;
     }
    bar.setBar(last_led_on-1, LED_GREEN);

  }
  bar.writeDisplay();

}


void change_particular_value(int resource_id, int menu_code) { //TO UPDATE A PARTICULAR VALUE AND NOT BLINK ALL THE DISPLAY so often ( also optimize the time used to set the display) - called by handle_lcd_printing
  if (menu_code == MAIN_MENU || menu_code == MENU_BATTERY_CHARGING) {

    if (resource_id == BATTERY_SOC_ID && menu_code == MENU_BATTERY_CHARGING) {
      lcd.setCursor(14, 0);
      lcd.print( "%");
      lcd.print(resources_values[resource_id]);
      lcd.print("  ");
    



    } else if (resource_id == CAR_KILOMETRAGE_ID) {

      lcd.setCursor( KMS_VALUE_CURSOR , 1); //after '='
      lcd.print(resources_values[resource_id]);
      for (int i = 0; i < CHARS_PER_LINE - KMS_VALUE_CURSOR - strlen(resources_values[resource_id]) ; i++) {
        lcd.print(" ");
      }


    } else if (resource_id == TEMP_FB_ID) {

      lcd.setCursor( TMP_FB_VALUE_CURSOR , 2);
      lcd.print(resources_values[resource_id]);
      lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
      lcd.print("C");

      for (int i = TMP_FB_VALUE_CURSOR + strlen(resources_values[resource_id]) + 2; i < Temp_Start_2 - 1 ; i++) {

        lcd.print(" ");
      }


    } else if (resource_id == TEMP_RB_ID) {
      lcd.setCursor( TMP_RB_VALUE_CURSOR , 2);
      lcd.print(resources_values[resource_id]);
      lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
      lcd.print("C");
      for (int i = TMP_RB_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE - 1 ; i++) {
        lcd.print(" ");
      }
    } else if (resource_id == TEMP_MB_ID) {
      lcd.setCursor( TMP_MB_VALUE_CURSOR , 3);
      lcd.print(resources_values[resource_id]);
      lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
      lcd.print("C");

      for (int i = TMP_MB_VALUE_CURSOR + strlen(resources_values[resource_id]) + 2; i < Temp_Start_2 - 1; i++) {
        lcd.print(" ");
      }
    } else if (resource_id == TEMP_BMS_ID) {
      lcd.setCursor( TMP_BMS_VALUE_CURSOR , 3);
      lcd.print(resources_values[resource_id]);
      lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
      lcd.print("C");
      for (int i = TMP_BMS_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE - 1; i++) {
        lcd.print(" ");
      }



    }

  } else if (menu_code == MENU_CELLS_1_8) {

    switch (resource_id) {
      case 1:
      case 3:
      case 5:
      case 7:
        lcd.setCursor( CELL_1D_1_VALUE_CURSOR , (resource_id - 1) / 2); //  (resource_id-1)/2  trick to set the line.
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_1D_1_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < Cell_start2 - 1; i++) {
          lcd.print(" ");
        }
        break;
      case 2:
      case 4:
      case 6:
      case 8:
        lcd.setCursor( CELL_1D_2_VALUE_CURSOR , (resource_id - 1) / 2);
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_1D_2_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE - 1; i++) {
          lcd.print(" ");
        }
        break;
      default:
        ;

    }

  } else if (menu_code == MENU_CELLS_9_16) {
    switch (resource_id) {
      case 9:
        lcd.setCursor( CELL_1D_1_VALUE_CURSOR , (resource_id - 9) / 2); //  (resource_id-9)/2  trick to set the line.
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_1D_1_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < Cell_start2 - 1; i++) {
          lcd.print(" ");
        }
        break;

      case 11:
      case 13:
      case 15:
        lcd.setCursor( CELL_2D_1_VALUE_CURSOR , (resource_id - 9) / 2); //  (resource_id-9)/2  trick to set the line.
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_2D_1_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < Cell_start2 - 1; i++) {
          lcd.print(" ");

        }
        break;
      case 10:
      case 12:
      case 14:
      case 16:
        lcd.setCursor( CELL_2D_2_VALUE_CURSOR , (resource_id - 9) / 2);
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_2D_2_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE - 1; i++) {
          lcd.print(" ");
        }
        break;
      default:
        ;

    }

  } else if (menu_code == MENU_CELLS_17_24) {

    switch (resource_id) {
      case 17:
      case 19:
      case 21:
      case 23:
        lcd.setCursor( CELL_2D_1_VALUE_CURSOR , (resource_id - 17) / 2); //  (resource_id-1)/2  trick to set the line.
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_2D_1_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < Cell_start2 - 1; i++) {
          lcd.print(" ");
        }
        break;
      case 18:
      case 20:
      case 22:
      case 24:
        lcd.setCursor( CELL_2D_2_VALUE_CURSOR , (resource_id - 17) / 2);
        lcd.print(resources_values[resource_id]);
        lcd.print("V");
        for (int i = CELL_2D_2_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE - 1; i++) {
          lcd.print(" ");
        }
        break;
      default:
        ;

    }


  } else if (menu_code == MENU_PWM_CURRENT) {
    if (resource_id == PWM_EC_ID) {
      lcd.setCursor( 3, 0);
      lcd.print(D_PWM_EC);
      lcd.print(resources_values[resource_id]);
      lcd.print("%");
      for (int i = PWM_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE; i++) {
        lcd.print(" ");
      }
    } else if (resource_id == PWM_BATTERY_CHARGER_ID) {
      lcd.setCursor( 3, 1);
      lcd.print(D_PWM_BC);
      lcd.print(resources_values[resource_id]);
      lcd.print("%");
      for (int i = PWM_VALUE_CURSOR + strlen(resources_values[resource_id]) + 1; i < CHARS_PER_LINE; i++) {
        lcd.print(" ");
      }
    } else if (resource_id == BMS_CURRENT_ID) {
      lcd.setCursor( 5, 2);
      lcd.print(D_CURRENT);
      lcd.print(resources_values[resource_id]);
      lcd.print(" A");

      for (int i = CURRENT_VALUE_CURSOR + strlen(resources_values[resource_id]) + 2; i < CHARS_PER_LINE; i++) {
        lcd.print(" ");
      }
    }

  }
}
//// PRINTS THE MENU. IF THE MENU TO BE PRINTED IS THE SAME BEING DISPLAYED, IT JUST CHANGES THE CORRESPONDING RESOURCE VALUE.
int handle_lcd_printing( int menu_code, int resource_id) { // Prints a whole menu ( if resource_id == 0 ) or changes a particular resource (resource_id) in the current menu.
   handling_lcd=true;
  if (menu_code == current_menu && resource_id!=0) { //just change the corresponding value.
    change_particular_value(resource_id, menu_code);

    // otherwise, change to the requested menu_code ( probably called from toggle menu )
  } else if(resource_id==0){
      if (menu_code == MAIN_MENU ) {
          lcd.clear();
          lcd.print("OPEN SOURCE VEHICLE");
          lcd.setCursor( 4, 1);
          lcd.print(D_KILOMETRAGE);
          lcd.print(resources_values[CAR_KILOMETRAGE_ID]);
          lcd.setCursor( 0, 2);
          lcd.print(D_TFB);
          lcd.print(resources_values[TEMP_FB_ID]);
          lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
          lcd.print("C");
          lcd.setCursor( 10, 2);
          lcd.print(D_TRB);
          lcd.print(resources_values[TEMP_RB_ID]);
          lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
          lcd.print("C");
          lcd.setCursor( 0, 3);
          lcd.print(D_TMB);
          lcd.print(resources_values[TEMP_MB_ID]);
          lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
          lcd.print("C");
          lcd.setCursor( 10, 3);
          lcd.print(D_TBMS);
          lcd.print(resources_values[TEMP_BMS_ID]);
          lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
          lcd.print("C");
          current_menu = MAIN_MENU;
         // return 0;

        } else if (menu_code == MENU_CELLS_1_8) {
        char aux_str[10];
        char aux_num[3];
        int counter = 1;
        lcd.clear();
        for (int i = 0 ; i < 4 ; i++) {
          for (int j = 0 ; j < 13 ; j = j + 10) {
          lcd.setCursor( j, i);
          strcpy(aux_str, "C");
          sprintf(aux_num, "%d", counter);
          strcat(aux_str, aux_num);
          strcat(aux_str, "=");
          lcd.print(aux_str);
          lcd.print(resources_values[counter]);
          lcd.print("V");
          counter++;
          }
        }
        current_menu = MENU_CELLS_1_8;

        } else if (menu_code == MENU_CELLS_9_16) {
        char aux_str[10];
        char aux_num[3];
        int counter = 9;

        lcd.clear();
        for (int i = 0 ; i < 4 ; i++) {
          for (int j = 0 ; j < 13 ; j = j + 11) {

          lcd.setCursor( j, i);
          strcpy(aux_str, "C");
          sprintf(aux_num, "%d", counter);
          strcat(aux_str, aux_num);
          strcat(aux_str, "=");
          lcd.print(aux_str);
          lcd.print(resources_values[counter]);
          lcd.print("V");
          counter++;
          }
        }
        current_menu = MENU_CELLS_9_16;

        } else if (menu_code == MENU_CELLS_17_24) {
        char aux_str[10];
        char aux_num[3];
        int counter = 17;
        lcd.clear();
        for (int i = 0 ; i < 4 ; i++) {
          for (int j = 0 ; j < 13 ; j = j + 11) {
          lcd.setCursor( j, i);
          strcpy(aux_str, "C");
          sprintf(aux_num, "%d", counter);
          strcat(aux_str, aux_num);
          strcat(aux_str, "=");
          lcd.print(aux_str);
          lcd.print(resources_values[counter]);
          lcd.print("V");
          counter++;
          }
        }
        current_menu = MENU_CELLS_17_24;

        } else if (menu_code == MENU_PWM_CURRENT) {
        lcd.clear();
        lcd.setCursor( 3, 0);
        lcd.print(D_PWM_EC);
        lcd.print(resources_values[PWM_EC_ID]);
        lcd.print("%");
        lcd.setCursor( 3, 1);
        lcd.print(D_PWM_BC);
        lcd.print(resources_values[PWM_BATTERY_CHARGER_ID]);
        lcd.print("%");
        lcd.setCursor( 5, 2);
        lcd.print(D_CURRENT);
        lcd.print(resources_values[BMS_CURRENT_ID]);
        lcd.print(" A");
        lcd.setCursor(0, 3);
        lcd.print("   TO MAIN MENU   ");
        current_menu = MENU_PWM_CURRENT;

        } else if (menu_code == MENU_BATTERY_CHARGING && battery_charging) {

        lcd.clear();
        lcd.setCursor( 2, 0);
        lcd.print("Charging : ");
        lcd.setCursor( 14, 0);
        lcd.print(" %");
        lcd.print(resources_values[BATTERY_SOC_ID]);


        lcd.setCursor( 4, 1);
        lcd.print(D_KILOMETRAGE);
        lcd.print(resources_values[CAR_KILOMETRAGE_ID]);
        lcd.setCursor( 0, 2);
        lcd.print(D_TFB);
        lcd.print(resources_values[TEMP_FB_ID]);
        lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
        lcd.print("C");
        lcd.setCursor( 10, 2);
        lcd.print(D_TRB);
        lcd.print(resources_values[TEMP_RB_ID]);
        lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
        lcd.print("C");
        lcd.setCursor( 0, 3);
        lcd.print(D_TMB);
        lcd.print(resources_values[TEMP_MB_ID]);
        lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
        lcd.print("C");
        lcd.setCursor( 10, 3);
        lcd.print(D_TBMS);
        lcd.print(resources_values[TEMP_BMS_ID]);
        lcd.print(DEGREES_SYMBOL);  //PRINTS '°'
        lcd.print("C");
        //current_menu = MAIN_MENU;
          current_menu=MENU_BATTERY_CHARGING;

        } else {
       // return ERROR_CODE;
      }
}
handling_lcd=false;
  return 0;

}

void menu_toggle() { // changes the display to the next menu.
  if (battery_charging) {
    if (current_menu == MENU_BATTERY_CHARGING) {
      current_menu = MENU_CELLS_1_8;   //T AVOID MAIN MENU if BATTERY CHARGING
    }else{
      current_menu++;
    }
  } else {
    if (current_menu >= AMOUNT_OF_MENUS-1 ) {
      current_menu = MAIN_MENU;
    } else {
      current_menu++;
    }
  }
    handle_lcd_printing(current_menu, 0);
}

void update_ledbar(char *new_soc_value) { // led bar has 2 led colors, but mixing creates the yellow.
  int colour = LED_OFF;
  number_leds_on = (atoi(new_soc_value) * AMOUNT_OF_LEDS / 100 );

  if (!battery_charging) {           //check if necessary
    for (int i = number_leds_on; i < AMOUNT_OF_LEDS; i++) {
      bar.setBar(i, LED_OFF);
    }
  }


  // SETS THE COLOUR ACORDING TO THE PERCENTAGE
  if (atoi(new_soc_value) <= 15) { //15 %   red
    colour = LED_RED;


  } else if ( atoi(new_soc_value) > 15  && atoi(new_soc_value) <= 60 ) { //orange
    colour = LED_YELLOW; // turn it orange after

  } else {
    colour = LED_GREEN;
  }

  for (int i = 0; i < number_leds_on; i++) { //the ones to turn on. 24 (AMOUNT_OF_LEDS) on --> 100%     ,     12 ON -->50%  ... etc

      bar.setBar(i, colour);
    
    
  }
  bar.writeDisplay();   //update led_bar.
}

void update_speed_display(char *new_speed_value) {

  double val = map(atoi(new_speed_value), 0, 200, 0, 180);
  if (val > 170)
    val = 170;
     if (val <= 0)
    val = 0;
  myservo.write(val);

}

int update_incoming_resource(char *data) {
 
 detachInterrupt(2);

 
  char *pch;
  char *aux_pointer;
  char data_cpy[10];
  
  andando++;  //FOR SIMULATION of speed 
  
  aux_pointer=data_cpy;
  strcpy(aux_pointer,data);


  pch = strtok (data, "=.\n");

  if (pch == NULL) {
    
attachInterrupt(digitalPinToInterrupt(2), button_interrupt, FALLING);
    return 0; //ERROR
  }

  if (!strcmp(CELL_VOLT_KEY, pch)) {
    int aux_atoi;
    char auxy[10];
    char auxy2[10];

    pch = strtok (NULL, "=.\n");
    if (pch == NULL) {
      
attachInterrupt(digitalPinToInterrupt(2), button_interrupt, FALLING);
      return 0; //ERROR
    }
    aux_atoi = atoi(pch);
    pch = strtok (NULL, "=.\n");
      Serial.println(pch);
    sprintf(auxy, "%d", atoi(pch) / 100 );
    strcat(auxy, ".");
    sprintf(auxy2, "%d", atoi(pch) - ((atoi(pch) / 100) * 100) );
    strcat(auxy, auxy2 ); //PRINTING Floats in arduino is annoying, I am doing this.
    Serial.println(auxy2);   //ME : check what I wrote down in paper
    Serial.println(auxy);
    strcpy(resources_values[aux_atoi], auxy);

    if(aux_atoi>=1 && aux_atoi<=8){
       handle_lcd_printing(MENU_CELLS_1_8 , aux_atoi );
    }else if(aux_atoi>=9 && aux_atoi<=16){
       handle_lcd_printing(MENU_CELLS_9_16 , aux_atoi );
    }else if(aux_atoi>=17 && aux_atoi<=24){
      handle_lcd_printing(MENU_CELLS_17_24 , aux_atoi );
      
    }
   

  
  } else if (!strcmp(TEMP_FB_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    strcpy(resources_values[TEMP_FB_ID], pch);
      handle_lcd_printing(MAIN_MENU , TEMP_FB_ID );
          if(battery_charging){
      handle_lcd_printing(MENU_BATTERY_CHARGING , TEMP_FB_ID );
    }else{
       handle_lcd_printing(MAIN_MENU , TEMP_FB_ID );
    }
  } else if (!strcmp(TEMP_RB_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
   
    strcpy(resources_values[TEMP_RB_ID], pch);
        if(battery_charging){
      handle_lcd_printing(MENU_BATTERY_CHARGING , TEMP_RB_ID );
    }else{
       handle_lcd_printing(MAIN_MENU , TEMP_RB_ID );
    }
  } else if (!strcmp(TEMP_MB_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    strcpy(resources_values[TEMP_MB_ID], pch);
    if(battery_charging){
      handle_lcd_printing(MENU_BATTERY_CHARGING , TEMP_MB_ID );
    }else{
       handle_lcd_printing(MAIN_MENU , TEMP_MB_ID );
    }
  } else if (!strcmp(TEMP_BMS_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    strcpy(resources_values[TEMP_BMS_ID], pch);
         if(battery_charging){
      handle_lcd_printing(MENU_BATTERY_CHARGING , TEMP_BMS_ID );
    }else{
       handle_lcd_printing(MAIN_MENU , TEMP_BMS_ID );
    }
  } else if (!strcmp(PWM_BC_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    if (atoi(pch) == 95 && atoi(resources_values[PWM_BATTERY_CHARGER_ID]) != 95  ) {     //CHECK IF THIS IS THE ONLY CONDITION TO BE " CHARGING "
      battery_charging = true;

      // TODO CHECK WITH TIMERS
      handle_lcd_printing(6, 0);  // SET CHARGING MAIN MENU.

    } else if ( atoi(resources_values[PWM_BATTERY_CHARGER_ID]) == 95 && atoi(pch) != 95 ) {
      battery_charging = false;
      handle_lcd_printing(1, 0);  // SET REGULAR MAIN MENU
      delay(10);
      update_ledbar(resources_values[BATTERY_SOC_ID]);
    } else if( atoi(resources_values[PWM_BATTERY_CHARGER_ID]) == 95 && !battery_charging ){   //for when  display connects when car is already charging
      battery_charging=true;
    }
    strcpy(resources_values[PWM_BATTERY_CHARGER_ID], pch);
    
  } else if (!strcmp(PWM_EC_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    strcpy(resources_values[PWM_EC_ID], pch);
  } else if (!strcmp(CAR_KILOMETRAGE_KEY, pch)) {
    pch = strtok (NULL, "=.\n");


    strcpy(resources_values[CAR_KILOMETRAGE_ID], pch);
    //write_int_to_eeprom(EEPROM_KILOMETRAGE_ADDRESS, atoi(pch) );      //SAVES KILOMETRAGE ON ADDRESS OF EEPROM
    if(battery_charging){
         handle_lcd_printing(MENU_BATTERY_CHARGING , CAR_KILOMETRAGE_ID );
    }else{
           handle_lcd_printing(MAIN_MENU , CAR_KILOMETRAGE_ID );
    }
   

  } else if (!strcmp(CAR_SoC_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    strcpy(resources_values[BATTERY_SOC_ID], pch);
    update_ledbar(resources_values[BATTERY_SOC_ID]);
    if(battery_charging){
      handle_lcd_printing(MENU_BATTERY_CHARGING , BATTERY_SOC_ID );
    }
    
  } else if (!strcmp(CAR_SPEED_KEY, pch)) {
    pch = strtok (NULL, "=.\n");
    if(strcmp(pch,resources_values[CAR_SPEED_ID]) ){   //if not 0 like before
        strcpy(resources_values[CAR_SPEED_ID], pch);
        update_speed_display(resources_values[CAR_SPEED_ID]);
    }
    
    // ENGINE

  } else {
   
  }

attachInterrupt(digitalPinToInterrupt(2), button_interrupt, FALLING);
  return 0;
}



void timer_interrupt() {
  flag_timer = true;
}

void setup() {

  Serial.begin(9600);

  //LCD
  lcd.init();     // initialize the lcd
  lcd.backlight();
  //LED BAR
  bar.begin(0x70);  // pass in the address

  

 pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), button_interrupt, FALLING);

 //PWM
  myservo.attach(5);
   myservo.write(0);
    //FOR BUTTON -- Falling edge interrupt.
 

}

void loop() {

  char buffer_read[64];
  int s = 0;



  //FIRST TIME AFTER TURN ON
  if (first_turn_on) {                   // TODO : put it in set Up.  (is useful here in case something detaches, etc.. maybe )
          for (int i = 0; i < NUMBER_OF_RESOURCES; i++) {
            strcpy(resources_values[i], "___");
          }
          handle_lcd_printing(MAIN_MENU, 0);
          delay(2000);
          first_turn_on = false;

         for (int i = 0; i < AMOUNT_OF_LEDS; i++) { // the rest, turned off.
          bar.setBar(i, LED_OFF);
          bar.writeDisplay();    //
        }
  }


 //LISTEN FROM OPENMOTE
  if (Serial.available()) {
 //while(Serial.available()){
    initial_time = millis();
    do {
      if ( ( buffer_read[s] = Serial.read() ) != -1) {
        s++;
      }
    } while ( millis() < (initial_time + read_wait_max) && buffer_read[s - 1] != '\n');   //try to read one data line before 100 milis, otherwise, drop it.
 // }
  if (buffer_read[s - 1] == '\n') {
    buffer_read[s] = '\0';
    // Serial.print(buffer_read);
    update_incoming_resource(buffer_read);
  }else{
   //Serial.println(" garbage read " );
  }
 }
//Battery charging effects  ... Done with millis because Timer1 is used by class Servo.
if(battery_charging){
  current_time=millis();
  if(current_time>=previous_time + interval_effect){
        battery_charging_effects();
      //previous_time=current_time;
      previous_time=millis();
  }
}

  //BUTTON PRESSED
  if (toggle_menu_flag) {
      //  current_button_press=millis();
       // if(last_button_press +  300 <= current_button_press){
            menu_toggle();
         //   last_button_press=current_button_press;
       // }else{}//FALSE TRIGGER
        toggle_menu_flag = false;
  }



}

 
