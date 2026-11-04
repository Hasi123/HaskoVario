/* Real settings, with the rounds per step overridable from the test build */
#ifndef VARIO_SETTINGS_STUB_H
#define VARIO_SETTINGS_STUB_H

#include "../../../libraries/VarioSettings/VarioSettings.h"

#ifdef TEST_ROUNDS_PER_STEP
#undef IGC_SHA256_ROUNDS_PER_STEP
#define IGC_SHA256_ROUNDS_PER_STEP TEST_ROUNDS_PER_STEP
#endif

#endif
