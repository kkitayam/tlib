#include "def-helper.h"

DEF_HELPER_2(rx_exec, void, env, tl)
DEF_HELPER_1(rx_debug, void, env)
DEF_HELPER_1(rx_itof, tl, tl)
DEF_HELPER_1(rx_utof, tl, tl)
DEF_HELPER_6(rx_dsp_mul, void, env, tl, tl, tl, tl, tl)
DEF_HELPER_4(rx_dsp_mvtac, void, env, tl, tl, tl)
DEF_HELPER_5(rx_dsp_rac, void, env, tl, tl, tl, tl)
DEF_HELPER_4(rx_dsp_mvfac, tl, env, tl, tl, tl)
