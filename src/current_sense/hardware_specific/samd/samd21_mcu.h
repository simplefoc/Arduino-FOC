#include "Arduino.h"

#if defined(_SAMD21_)

#ifndef CURRENT_SENSE_SAMD21_H
#define CURRENT_SENSE_SAMD21_H

#define SIMPLEFOC_SAMD_DEBUG
#if !defined(SIMPLEFOC_SAMD_DEBUG_SERIAL)
#define SIMPLEFOC_SAMD_DEBUG_SERIAL Serial
#endif

#include "../../hardware_api.h" 

#endif



#endif
