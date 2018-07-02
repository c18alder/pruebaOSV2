#include <string.h>
#include <stdio.h>
#include "coap-engine.h"

extern char cell_one_from_arduino[10];

static void res_get_handler(coap_message_t *request, coap_message_t *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

/* A simple getter example. Returns the reading from light sensor with a simple etag */
RESOURCE(res_car_cell_one,
         "title=\"Cell status\";rt=\"cell2\"",
         res_get_handler,
         NULL,
         NULL,
         NULL);

static void
res_get_handler(coap_message_t *request, coap_message_t *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

        unsigned int accept=-1;

        if(accept == -1 || accept == TEXT_PLAIN) {
            coap_set_header_content_format(response, TEXT_PLAIN);
            snprintf((char *)buffer, COAP_MAX_CHUNK_SIZE, "%s", cell_one_from_arduino);

            coap_set_payload(response, (uint8_t *)buffer, strlen((char *)buffer));
        }       else {
                coap_set_status_code(response, NOT_ACCEPTABLE_4_06);
                const char *msg = "Supporting content-types text/plain";
                coap_set_payload(response, msg, strlen(msg));
              }




}

