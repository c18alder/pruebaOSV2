/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Erbium (Er) CoAP Engine example.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "coap-engine.h"
#include "net/routing/routing.h" //INCLUDE TO SET UP A FLOATING DODAG
#include "net/mac/tsch/tsch.h"   //INCLUDE TO TURN ON MAC LAYER



#if PLATFORM_SUPPORTS_BUTTON_HAL
#include "dev/button-hal.h"
#else
#include "dev/button-sensor.h"
#endif


// FOR UART WORKING AND DEBUGING

#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/serial-line.h"
#include <stdint.h>

//KEY VALUE FOR THE RESOURCE TO BE SENT :
#define CAR_SPEED_KEY "km/h"
#define CAR_SoC_KEY "%bat"
#define CAR_KILOMETRAGE_KEY "kms"
#define PWM_EC_KEY "pec"   //PWM Engine Controler
#define PWM_BC_KEY "pbc"  //PWM Battery Charger
#define TEMP_1_KEY "ta"
#define TEMP_2_KEY "tb"
#define TEMP_3_KEY "tc"
#define TEMP_4_KEY "td"
#define CELL_VOLT_KEY "cs" // a "." separates the cell value and number---> ej:  cs=18.649    (cell value is not devided by 200 yet (done in server) TODO : maybe do it here..
////////////////////////////////


#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


//max size for the home_made format: km/h=120}  size 8 EXCEPT CELLS Voltage



//TODO: ACK ARDUINO
//#define ARDUINO_ACK "ACKNOWLEDGED"
//#define ARDUINO_ERROR "WRONG_PACKET"  //GOT A UART PACKET BUT NOT THE CORRECT FORMAT (i.e not  kms=10)
//#define ARDUINO_ACK_SIZE ((sizeof(ARDUINO_ACK))-1)

// HERE IS GOING TO BE STORED THE LAST VALUE OF EACH RESOURCE (TODO: probably create a struct
//or vector of pointers, etc to store more values)

extern char battery_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char battery_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char speed_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char speed_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char kilometrage_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char kilometrage_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern int cells_voltage_from_arduino[NUMBER_OF_CELLS];
int cells_voltage_from_arduino[NUMBER_OF_CELLS];
extern char temp_one_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char temp_one_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char temp_two_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char temp_two_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char temp_three_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char temp_three_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char temp_four_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char temp_four_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char ec_pwm_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char ec_pwm_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
extern char bc_pwm_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];
char bc_pwm_value_from_arduino[MAX_RESOURCE_FORMAT_SIZE];




int auxy=0;

/*
 * Resources to be activated need to be imported through the extern keyword.
 * The build system automatically compiles the resources in the corresponding sub-directory.
 */
extern coap_resource_t
  res_car_battery,
  res_car_kilometrage,
  res_car_speed,
  res_hello,
  res_car_cells,
  res_car_temp_one,
  res_car_temp_two,
  res_car_temp_three,
  res_car_temp_four,
  res_car_ec_pwm,
  res_car_bc_pwm;

/////CLEAR TERMINAL
//void clrscr()
//{
//	system("clear");

//}
///////////////////////
//UART SENT FUNCTION


unsigned int
uart1_send_bytes( char *s, unsigned int len) // TODO: Send ack to each Arduino.
{
  unsigned int i = 0;

  while(s && *s != 0) {
    if(i >= len) {
      break;
    }
    uart_write_byte(1, *s++);
    i++;
  }
  return i;
}

//CHECK WHICH RESOURCE IS (depending of the KEY) AND SAVES IT ACCORDINGLY ( FULL FORMAT )
//carefull! strtok modifies rxdata
int handle_incoming_resource(char *in_data){
	// A comparison is made between new incoming value and the one currently hold as a resource before updating it
	//mostly done to avoid a full notification to the client whenever a full update from arduino comes.

  	        	   char *pch;
  	        	   //char *aux2;
  	        	   char aux[MAX_RESOURCE_FORMAT_SIZE];
  	        	   int aux_atoi;

  	        	   strcpy(aux,in_data);
  	        	   pch= strtok (aux, "=.");   // "." is for cells
  	        	   strcat(in_data,"\n");

  	        	   	   if( !(strcmp(pch,CAR_SPEED_KEY))){
  	        	   		   if( (strcmp(speed_value_from_arduino,in_data) )  ){    //if new value is different from the resource, update it.
  	        	   		   strcat(in_data,"\n");
  	        	   		   strcpy(speed_value_from_arduino,in_data);
  	        	   		   res_car_speed.trigger();  //CALL THE RESOURCE BEING OBSERVED (TODO: put a condition if "no observated resources", dont do this)
  	        	   		   }
  	        	   		   return 0;
  	        	   	   }else if(!(strcmp(pch,CAR_SoC_KEY))){
  	        	   		   if( (strcmp(battery_value_from_arduino,in_data) )  ){
  	        	   			   strcpy(battery_value_from_arduino,in_data);
  	        	   			   res_car_battery.trigger();
  	        	   		   }
  	        	   	   }else if(!(strcmp(pch,CAR_KILOMETRAGE_KEY))){
  	        	   		   if( (strcmp(kilometrage_value_from_arduino,in_data) )  ){
  	        	   				strcpy(kilometrage_value_from_arduino,in_data);
  	        	   				res_car_kilometrage.trigger();
  	        	   		   }
  	        	   		 return 0;
  	        	   	   }else if(!(strcmp(pch,TEMP_1_KEY))){
  	        	   		   if( (strcmp(temp_one_from_arduino,in_data) )  ){
  	        	   			   strcpy(temp_one_from_arduino,in_data);
  	        	   			   res_car_temp_one.trigger();
  	        	   		   }
  	          	   		   return 0;
  	        	   	   }else if(!(strcmp(pch,TEMP_2_KEY))){
  	        	   		   if( (strcmp(temp_two_from_arduino,in_data) )  ){
  	        	   		   strcpy(temp_two_from_arduino,in_data);
  	        	   		   res_car_temp_two.trigger();
  	        	   		   }
  	        	   		   return 0;
  	        	   	   }else if(!(strcmp(pch,TEMP_3_KEY))){
  	        	   		   if( (strcmp(temp_three_from_arduino,in_data) )  ){
  	        	   			 strcpy(temp_three_from_arduino,in_data);
  	        	   			 res_car_temp_three.trigger();
  	        	   		   }
  	        	   		   return 0;
  	    	       	   }else if(!(strcmp(pch,TEMP_4_KEY))){
  	    	       		   if( (strcmp(temp_four_from_arduino,in_data) )  ){
  	    	       			   strcpy(temp_four_from_arduino,in_data);
  	    	       			   res_car_temp_four.trigger();
  	    	       		   }
  	        	   		   return 0;
  	    	       	   }else if(!(strcmp(pch,PWM_EC_KEY))){
  	    	       		   if( (strcmp(ec_pwm_value_from_arduino,in_data) )  ){
  	    	       			   strcpy(ec_pwm_value_from_arduino,in_data);
  	    	       			   res_car_ec_pwm.trigger();
  	    	       		   }
  	        	   		   return 0;
  	    	       	   }else if(!(strcmp(pch,PWM_BC_KEY))){
  	    	       		   if( (strcmp(bc_pwm_value_from_arduino,in_data) )  ){
  	        	   		   strcpy(bc_pwm_value_from_arduino,in_data);
  	        	   		   res_car_bc_pwm.trigger();
  	    	       		   }
  	        	   		   return 0;
  	    	       	   }else if(!(strcmp(pch,CELL_VOLT_KEY))){

  	    	       		pch = strtok (NULL, "=.\n");
  	    	       		aux_atoi= atoi(pch);
  	    	       		pch = strtok (NULL, "=.\n");
  	    	       		auxy=atoi(pch);
  	    	       		if( auxy != cells_voltage_from_arduino[aux_atoi-1]){
  	    	       				cells_voltage_from_arduino[aux_atoi-1]=atoi(pch);
  	    	       		}
  	    	       		   return 0;
  	    	       	   }else{

  	    	       		   	   	   	   	   //IMPLEMENT ERROR CONTROl OR SIMPLY DO NOTHING ( Discard this data )
  	    	       	   }

  	        	  return 0;
  	           }


PROCESS(er_example_server, "Erbium Example Server");
AUTOSTART_PROCESSES(&er_example_server);

PROCESS_THREAD(er_example_server, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

//UART RECEIVE
 char *rxdata;

/////////////
//DEBUG AT RESTART
  PRINTF("Starting Erbium Example Server\n");

#ifdef RF_CHANNEL
  PRINTF("RF channel: %u\n", RF_CHANNEL);
#endif
#ifdef IEEE802154_PANID
  PRINTF("PAN ID: 0x%04X\n", IEEE802154_PANID);
#endif

  PRINTF("uIP buffer: %u\n", UIP_BUFSIZE);
  PRINTF("LL header: %u\n", UIP_LLH_LEN);
  PRINTF("IP+UDP header: %u\n", UIP_IPUDPH_LEN);
  PRINTF("CoAP max chunk: %u\n", COAP_MAX_CHUNK_SIZE);


  /* Initialize the REST engine. */
  coap_engine_init();
  /*
   * Bind the resources to their Uri-Path.
   * WARNING: Activating twice only means alternate path, not two instances!
   * All static variables are the same for each URI path.
   */
  coap_activate_resource(&res_car_speed, "Car_Speed");
  coap_activate_resource(&res_car_kilometrage, "Car_Kilometrage");
  coap_activate_resource(&res_car_battery, "Car_Battery");
  coap_activate_resource(&res_car_cells, "Car_Cells");
  coap_activate_resource(&res_car_temp_one, "Car_Temp_One");
  coap_activate_resource(&res_car_temp_two, "Car_Temp_Two");
  coap_activate_resource(&res_car_temp_three, "Car_Temp_Three");
  coap_activate_resource(&res_car_temp_four, "Car_Temp_Four");
  coap_activate_resource(&res_car_ec_pwm, "Car_Ec_Pwm");
  coap_activate_resource(&res_car_bc_pwm, "Car_Bc_Pwm");

  //coap_activate_resource(&res_hello, "test/hello");


  // CREATes A FLOTING DODAG NET , WITH THIS MOTE AS A ROOT (Following  6TiSCH/simple-node/node.c example )
  NETSTACK_ROUTING.root_start();
  NETSTACK_MAC.on();

  /* Define application-specific events here. */
  while(1) {

	 PROCESS_WAIT_EVENT();
//INCOMING UART DATA
    if(ev == serial_line_event_message) {


    	   //lights toggle if some data is read at UART
    		leds_toggle(LEDS_RED);
    		rxdata = data;

			//clrscr();
    		printf("Resourse received over UART %s\n\n", rxdata);

    		handle_incoming_resource(rxdata);

    		//printf("\n");
    		PRINTF("Last Speed Value %s\n" , speed_value_from_arduino);
    		//printf("\n");
    		PRINTF("Last Battery Value %s\n" , battery_value_from_arduino);
    		//printf("\n");
    		PRINTF("Last Kilometrage Value %s\n\n" , kilometrage_value_from_arduino);
    		//printf("\n");
    		PRINTF("Last Temps:  %s  %s  %s %s \n\n" , temp_one_from_arduino,temp_two_from_arduino,temp_three_from_arduino,temp_four_from_arduino);
    		//printf("\n");
    		PRINTF("Last PWM to EC %s\n\n" , ec_pwm_value_from_arduino);
    		//printf("\n");
    		PRINTF("Last PWM to BC %s\n\n" , bc_pwm_value_from_arduino);
    		//printf("\n");
    		for (int i=0 ; i<24; i++){
    				PRINTF(" %d , ", cells_voltage_from_arduino[i]);
    	    }

    } // END INCOMING UART DATA

  }                             /* while (1) */

  PROCESS_END();
}
