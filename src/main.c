#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "adc.h"

extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

int main(void)
{
	//gecko_bgapi_classes_init_server_friend();
  // Initialize stack
  gecko_main_init();

  logInit();

  setupSensor();

  while (1) {
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
		handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
	}
  };
}
