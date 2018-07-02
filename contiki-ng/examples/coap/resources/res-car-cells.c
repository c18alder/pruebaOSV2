#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "coap-engine.h"

extern int cells_voltage_from_arduino[24];   //TODO:  CODE IT SOMEHOW TO SEND THE RESOURCE NOT OVEREXTENDING THE CHUNCKSIZE.

static void res_get_handler(coap_message_t *request, coap_message_t *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);



RESOURCE(res_car_cells,
         "title=\"Cell status\";rt=\"allcells\"",
         res_get_handler,
         NULL,
         NULL,
         NULL);

static void
res_get_handler(coap_message_t *request, coap_message_t *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
				{
		short int auxiliar[24];      //CELL VALUES ARE SENT IN A SHORT INT (-32000 to 32000 ) TO SAVE SPACE:   format :    abcde   ab = cell number    cde= cell value
		for (int i=0; i < 24; i++){
			auxiliar[i]=  ((i+1)*1000) + cells_voltage_from_arduino[i];
		}

        unsigned int accept=-1;

        //IF MORE CELLS OR USE FLOAT OR MORE DIGITS NEEDED, PROBABLY NEED TO USE BLOCKWISE ( check MAX_CHUNK_SIZE )
        memcpy(buffer, auxiliar, 48);
        buffer[48] = '\0';
        // TODO see why it is not working if I don't hardcode it. MAYBE BECAUSE IT IS PIGGYBAGGED
        //coap_set_header_uri_path(response, "Car_Cells");

        if(accept == -1 || accept == TEXT_PLAIN) {
            coap_set_header_content_format(response, TEXT_PLAIN);


            coap_set_payload(response, (uint8_t *)buffer,(NUMBER_OF_CELLS*2) );


        }       else {
                coap_set_status_code(response, NOT_ACCEPTABLE_4_06);
                const char *msg = "Supporting content-types text/plain";
                coap_set_payload(response, msg, strlen(msg));
              }




}

