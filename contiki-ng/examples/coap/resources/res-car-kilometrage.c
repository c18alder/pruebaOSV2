#include <stdlib.h>
#include <string.h>
#include "coap-engine.h"

extern char kilometrage_value_from_arduino[10];

static void res_get_handler(coap_message_t *request, coap_message_t *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_event_handler(void);

/*
 * A handler function named [resource name]_handler must be implemented for each RESOURCE.
 * A buffer for the response payload is provided through the buffer pointer. Simple resources can ignore
 * preferred_size and offset, but must respect the REST_MAX_CHUNK_SIZE limit for the buffer.
 * If a smaller block size is requested for CoAP, the REST framework automatically splits the data.
 */
EVENT_RESOURCE(res_car_kilometrage,
         "title=\"Speed: ?len=0..\";rt=\"Variable\"",
         res_get_handler,
         NULL,
         NULL,
         NULL,
		 res_event_handler);

static void
res_get_handler(coap_message_t *request, coap_message_t *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

        unsigned int accept=-1;

        if(accept == -1 || accept == TEXT_PLAIN) {
            coap_set_header_content_format(response, TEXT_PLAIN);
            snprintf((char *)buffer, COAP_MAX_CHUNK_SIZE, "%s", kilometrage_value_from_arduino);

            coap_set_payload(response, (uint8_t *)buffer, strlen((char *)buffer));
        }       else {
                coap_set_status_code(response, NOT_ACCEPTABLE_4_06);
                const char *msg = "Supporting content-types text/plain";
                coap_set_payload(response, msg, strlen(msg));
              }




}
static void res_event_handler(void)
{
  /* Make a condition to notify clients . */

    /* Notify the registered observers which will trigger the res_get_handler to create the response. */

    coap_notify_observers(&res_car_kilometrage);

}
