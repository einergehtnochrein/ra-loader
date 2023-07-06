
#ifndef __INFO_H
#define __INFO_H

#include "lpclib.h"

__PACKED(struct _Info_t {
    uint16_t version;
    uint16_t version_inv;

    uint8_t __reserved02__[252];
});

typedef struct _Info_t Info_t;

extern const Info_t _loaderInfo;

#endif
