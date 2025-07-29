#ifndef D_H_
#define D_H_
#include "main.h"
#include "string.h"

void InitializeLcd(void);
void UartRenderState(AppContext *ctx);
void UartClearScreen();
void DisplayRenderState(AppContext *ctx);

#endif
