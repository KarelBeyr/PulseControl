#include "appLogic.h"

uint32_t lastCursorTime = 0;

void clearInput(AppContext *ctx) {
  ctx->inputValue = 0;
}

void revertCalibration(AppContext *ctx) {
  ctx->inputValue = ctx->calibrationPoints[ctx->calibrationIndex];
}

void clearVoltage(AppContext *ctx) {
  ctx->voltage = 0;
  ctx->isVoltageEntered = false;
}

void backspace(AppContext *ctx) {
  ctx->inputValue = ctx->inputValue / 10;
}

void stopPWM(AppContext *ctx, CallbackFunction stopPwmCallback) {
  ctx->isPwmRunning = false;
  stopPwmCallback();
}

uint16_t LinearInterpolate(uint16_t x, uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1) {
    if (x1 == x0) return y0; // Prevent division by zero
    return y0 + ((uint32_t)(x - x0) * (y1 - y0)) / (x1 - x0);
}

uint16_t GetPwmForVoltage(AppContext* ctx) {
    if (ctx->voltage <= 80) {
        return ctx->calibrationPoints[0];
    } else if (ctx->voltage <= 200) {
        return LinearInterpolate(ctx->voltage, 80, 200, ctx->calibrationPoints[0], ctx->calibrationPoints[1]);
    } else if (ctx->voltage <= 400) {
        return LinearInterpolate(ctx->voltage, 200, 400, ctx->calibrationPoints[1], ctx->calibrationPoints[2]);
    } else {
        return ctx->calibrationPoints[2];
    }
}

void startPWM(AppContext *ctx, CallbackWithParam startPwmCallback) {
  ctx->isPwmRunning = true;
  uint16_t percent = GetPwmForVoltage(ctx);
  startPwmCallback(percent);
}

void setSTATE_F3(AppContext *ctx) {
  clearInput(ctx);
  ctx->currentState = STATE_F3;
}

void setSTATE_F2(AppContext *ctx) {
  ctx->currentState = STATE_F2;
  ctx->calibrationIndex = 0;
  ctx->inputValue = ctx->calibrationPoints[0];
}

void setSTATE_F1(AppContext *ctx) {
  clearInput(ctx);
  ctx->currentState = STATE_F1;
}

void validateAndSetVoltage(AppContext *ctx) {
  if (ctx->inputValue < 80 || ctx->inputValue > 400)
  {
    strcpy(ctx->message, "Not in range 80 - 400!");
    clearInput(ctx);
    return;
  }
  ctx->isVoltageEntered = true;
  ctx->voltage = ctx->inputValue;
  ctx->inputValue = 0;
}

void validateAndSetCalibration(AppContext *ctx) {
  if (ctx->inputValue < 0 || ctx->inputValue > 100)
  {
    strcpy(ctx->message, "Not in range 0 - 100!");
    clearInput(ctx);
    return;
  }
  ctx->calibrationPoints[ctx->calibrationIndex] = ctx->inputValue;
  ctx->calibrationIndex = (ctx->calibrationIndex + 1) % 3;
  ctx->inputValue = ctx->calibrationPoints[ctx->calibrationIndex];
}

void updateInput(AppContext *ctx, KeyboardButton key, uint8_t maxValue) // maxValue is actually divided by 10
{
  uint8_t digit = key - '0';
  if (ctx->inputValue > maxValue) {
    strcpy(ctx->message, "Input too high!");
    return;
  }
  ctx->inputValue = ctx->inputValue * 10 + digit;
}

bool handle_event(AppContext *ctx, KeyboardButton key, CallbackWithParam startPwmCallback, CallbackFunction stopPwmCallback)
{
  if (key == KEY_NULL)
  {
	  // cursor logic
	  uint32_t now = HAL_GetTick();
	  if (now - lastCursorTime > 500)
	  {
		  ctx->displayCursor = !ctx->displayCursor;
		  lastCursorTime = now;
		  return true;
	  }
	  return false;
  }
  strcpy(ctx->message, "");

  if (ctx->currentState == STATE_F1) {
	if (ctx->isPwmRunning == true)
	{
	  if (key == KEY_Stop) stopPWM(ctx, stopPwmCallback);
	  return false; // when PWM is running, we can only press the "STOP" button
	}

	if (ctx->isVoltageEntered == true) // valid voltage has been entered
	{
	  if (key == KEY_Start) {
		  startPWM(ctx, startPwmCallback);
	  }
      if (key == KEY_Clear) clearVoltage(ctx);
	} else
	{
	  if (key >= KEY_0 && key <= KEY_9) updateInput(ctx, key, 40);
	  if (key == KEY_Enter) validateAndSetVoltage(ctx);
	  if (key == KEY_BkSp) backspace(ctx);
	  if (key == KEY_ESC) clearInput(ctx);
	}

    if (key == KEY_F2) setSTATE_F2(ctx);
    if (key == KEY_F3) setSTATE_F3(ctx);
  }

  if (ctx->currentState == STATE_F2) {
	if (key >= KEY_0 && key <= KEY_9) updateInput(ctx, key, 10);
	if (key == KEY_Enter) validateAndSetCalibration(ctx);
	if (key == KEY_BkSp) backspace(ctx);
	if (key == KEY_ESC) revertCalibration(ctx);
	if (key == KEY_Clear) clearInput(ctx);
	if (key == KEY_F1) setSTATE_F1(ctx);
	if (key == KEY_F3) setSTATE_F3(ctx);
  }

  if (ctx->currentState == STATE_F3) {
	if (key == KEY_F1) setSTATE_F1(ctx);
	if (key == KEY_F2) setSTATE_F2(ctx);
  }
return true;
}

void InitializeAppContext(AppContext* ctx)
{
	ctx->isVoltageEntered = false;
	ctx->isPwmRunning = false;
	ctx->voltage = 0;
	ctx->inputValue = 0;
	strcpy(ctx->message, " ");

	ctx->calibrationPoints[0] = 20; // TODO - persist those values in flash
	ctx->calibrationPoints[1] = 50;
	ctx->calibrationPoints[2] = 100;

	setSTATE_F2(ctx);
}
