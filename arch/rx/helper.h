#include "def-helper.h"

DEF_HELPER_2(rx_exec, void, env, tl)
DEF_HELPER_1(rx_debug, void, env)
DEF_HELPER_1(rx_itof, tl, tl)
DEF_HELPER_1(rx_utof, tl, tl)
DEF_HELPER_4(rx_dsp, tl, env, tl, tl, tl)
