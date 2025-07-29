#ifndef AL_H_
#define AL_H_
#include "main.h"
#include "string.h"

bool handle_event(AppContext *ctx, KeyboardButton key, CallbackWithParam startPwmCallback, CallbackFunction stopPwmCallback);
void InitializeAppContext(AppContext* ctx);
uint16_t GetPwmForVoltage(AppContext* ctx);

#endif
