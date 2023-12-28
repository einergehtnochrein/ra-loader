
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "config.h"

extern uint32_t __CONFIG_ADDRESS__;

/* Pointer to default configuration set at link time */
const Config_t *config_g = (Config_t *)&__CONFIG_ADDRESS__;


