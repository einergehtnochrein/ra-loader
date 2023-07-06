
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"

#include "app.h"
#include "info.h"

__SECTION(".info")
const struct _Info_t _loaderInfo = {
    .version = LOADER_VERSION,
    .version_inv = LOADER_VERSION ^ 0xFFFF,
};
