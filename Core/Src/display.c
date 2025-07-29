#include "display.h"
#include "appLogic.h"
#include "stm32_lcd.h"
#include <stdlib.h>

uint32_t x_size, y_size;
#define LCD_LINE_WIDTH 24

#define LCD_WIDTH  480
#define LCD_HEIGHT 272
#define BUFFER_SIZE (LCD_WIDTH * LCD_HEIGHT * 4) // ARGB8888 format = 4 bytes per pixel

#define FRAMEBUFFER1_ADDR ((uint32_t *)SDRAM_DEVICE_ADDR)
#define FRAMEBUFFER2_ADDR ((uint32_t *)(SDRAM_DEVICE_ADDR + BUFFER_SIZE))

uint32_t *frontBuffer = FRAMEBUFFER1_ADDR;
uint32_t *backBuffer  = FRAMEBUFFER2_ADDR;

//extern LTDC_HandleTypeDef hlcd_ltdc;

uint32_t foregroundColor = UTIL_LCD_COLOR_GREEN; // Default: white
uint32_t backgroundColor = UTIL_LCD_COLOR_BLACK; // Default: black
sFONT *font = &Font32;

void ClearCache()
{
	// otherwise I get LCD artefacts
	//SCB_CleanDCache_by_Addr((void *)SDRAM_DEVICE_ADDR, 480*272*4*2);

	//SCB_CleanDCache_by_Addr((uint32_t *)backBuffer, BUFFER_SIZE);
}

void InitializeLcd(void)
{
  /* Initialize the LCD */
  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);

  /* Set Foreground Layer */
  UTIL_LCD_SetLayer(0);

  /* Clear the LCD Background layer */
  UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);

  BSP_LCD_SetLayerAddress(0, 0, (uint32_t)backBuffer);
  HAL_LTDC_Reload(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING);
  UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);

  BSP_LCD_GetXSize(0, &x_size);
  BSP_LCD_GetYSize(0, &y_size);

  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_GREEN);
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLACK);
  UTIL_LCD_SetFont(&Font32);
}

void uartSetCursorPosition(int row, int col) {
  printf("\033[%d;%dH", row, col);
}

void UartClearScreen() {
  printf("\033[2J");
  printf("\033[H"); // Move cursor to top-left corner
}

void UartRenderState(AppContext *ctx) {
  uartSetCursorPosition(1, 1);
  if (ctx->currentState == STATE_F1) {
    printf("F1: Voltage control    ");
    uartSetCursorPosition(2, 1);
    printf("Current input: %d     ", ctx->inputValue);
    uartSetCursorPosition(3, 1);
    if (ctx->voltage > 0) {
      printf("Voltage: %dV     ", ctx->voltage);
    } else {
      printf("Voltage: N/A     ");
    }
    uartSetCursorPosition(4, 1);
    if (ctx->isPwmRunning == true) {
      printf("PWM is running at %dV   ", ctx->voltage);
    } else {
      printf("PWM is OFF              ");
    }
    uartSetCursorPosition(5, 1);
    printf("%s                                                               \r\n", ctx->message);
  }
}

void DrawPixel(uint16_t x, uint16_t y, uint32_t color)
{
    uint32_t index = y * LCD_WIDTH + x;
    backBuffer[index] = color;
}


void DrawLine(int x0, int y0, int x1, int y1, uint32_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;  // error term

    while (1)
    {
        DrawPixel(x0, y0, color);

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}


void DrawChar(uint16_t x, uint16_t y, char c)
{
    uint16_t i, j;
    uint16_t width = font->Width;
    uint16_t height = font->Height;
    uint8_t bytesPerRow = (width + 7) / 8;

    const uint8_t *char_table = font->table + (c - 32) * height * bytesPerRow;

    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            // Find the byte and bit for this pixel
            uint8_t byte = char_table[i * bytesPerRow + (j / 8)];
            uint8_t bit = 1 << (7 - (j % 8));

            if (byte & bit)
            	DrawPixel(x + j, y + i, foregroundColor);
            else
            	DrawPixel(x + j, y + i, backgroundColor);
        }
    }
}

void DrawText(uint16_t x, uint16_t y, uint8_t *text)
{
    while (*text)
    {
        DrawChar(x, y, *text);
        x += font->Width;
        text++;
    }
}

void displayPaddedLine(uint16_t y, const char *text)
{
	// the sole purpose of this helper method is to overwrite the whole row so that we do not need to rely on clear screen
//
//    char padded[LCD_LINE_WIDTH];
//    memset(padded, ' ', LCD_LINE_WIDTH);
//
//    size_t len = strlen(text);
//    if (len > LCD_LINE_WIDTH) len = LCD_LINE_WIDTH;
//    memcpy(padded, text, len);



    DrawText(0, y*(font->Height), (uint8_t *)text);
}

char getCursor(AppContext *ctx)
{
	return ctx->displayCursor ? '_' : ' ';
}

const char* getValueWithCursor(AppContext *ctx) {
    static char result[7];  // adjust size as needed
    if (ctx->inputValue != 0)
        sprintf(result, "%d%c", ctx->inputValue, getCursor(ctx));
    else
        sprintf(result, "%c", getCursor(ctx));
    return result;
}

void DrawCalibrationLine(AppContext *ctx, uint16_t voltage, uint8_t idx)
{
	char buffer[LCD_LINE_WIDTH+5];
    if (ctx->calibrationIndex == idx)
    	sprintf(buffer, "PWM pct for %dV: %s", voltage, getValueWithCursor(ctx));
    else
    	sprintf(buffer, "PWM pct for %dV: %d", voltage, ctx->calibrationPoints[idx]);
    displayPaddedLine(idx+1, buffer);
}

void RenderNewFrame() // do the double buffering magic
{
  ClearCache();

  BSP_LCD_SetLayerAddress(0, 0, (uint32_t)backBuffer);
  HAL_LTDC_Reload(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING);

  // Swap pointers
  uint32_t *temp = frontBuffer;
  frontBuffer = backBuffer;
  backBuffer = temp;
}

void DisplayRenderState(AppContext *ctx)
{
  char buffer[LCD_LINE_WIDTH+1];
  for (int x = 0; x < 480; x++) // basically clear screen. TODO: Make it much faster
  {
	  for (int y = 0; y <272; y++)
	  {
		  DrawPixel(x, y, UTIL_LCD_COLOR_BLACK);
	  }
  }
  font = &Font32;
  foregroundColor = UTIL_LCD_COLOR_GREEN;

  if (ctx->currentState == STATE_F1) {
	  //font = &Font40;
	displayPaddedLine(0, "F1: Voltage control");
    if (ctx->isPwmRunning == true) {
        sprintf(buffer, "PWM is running at %dV", ctx->voltage);
        displayPaddedLine(1, buffer);

        sprintf(buffer, "PWM duty %d%%", GetPwmForVoltage(ctx));
        displayPaddedLine(2, buffer);

      displayPaddedLine(3, "Press STOP");
    }
    else if (ctx->isVoltageEntered)
    {
	  sprintf(buffer, "Voltage: %dV", ctx->voltage);
      displayPaddedLine(1, buffer);
      displayPaddedLine(2, "Press START or Clear");
      displayPaddedLine(3, "");
    }
    else
    {
	  sprintf(buffer, "Enter voltage: %s", getValueWithCursor(ctx));
      displayPaddedLine(1, buffer);
      displayPaddedLine(2, "Press Enter");
      displayPaddedLine(3, "");
    }
  }
  else if (ctx->currentState == STATE_F2)
  {
	font = &Font24;
    DrawText(0, 0, (uint8_t *)"F2: Calibration");
    DrawCalibrationLine(ctx, 80, 0);
    DrawCalibrationLine(ctx, 200, 1);
    DrawCalibrationLine(ctx, 400, 2);

    uint16_t cx = 50;
    uint16_t cy = 250;
    uint16_t dx = 400;
    uint16_t dy = 120;

    DrawLine(cx, cy, cx+dx, cy, UTIL_LCD_COLOR_WHITE);
    DrawLine(cx, cy, cx, cy-dy, UTIL_LCD_COLOR_WHITE);

    DrawLine(cx+(dx / 5), cy, cx+(dx / 5), cy-5, UTIL_LCD_COLOR_WHITE);
    DrawLine(cx+(dx / 2), cy, cx+(dx / 2), cy-5, UTIL_LCD_COLOR_WHITE);
    DrawLine(cx+dx, cy, cx+dx, cy-5, UTIL_LCD_COLOR_WHITE);

    uint16_t y0 = cy - (ctx->calibrationPoints[0] * dy / 100);
    uint16_t y1 = cy - (ctx->calibrationPoints[1] * dy / 100);
    uint16_t y2 = cy - (ctx->calibrationPoints[2] * dy / 100);

    DrawLine(cx, y0, cx+5, y0, UTIL_LCD_COLOR_WHITE);
    DrawLine(cx, y1, cx+5, y1, UTIL_LCD_COLOR_WHITE);
    DrawLine(cx, y2, cx+5, y2, UTIL_LCD_COLOR_WHITE);

    font = &Font16;
    foregroundColor = UTIL_LCD_COLOR_WHITE;
    DrawText(cx+(dx/5)-15, cy+8, (uint8_t *)"80V");
    DrawText(cx+(dx/2)-20, cy+8, (uint8_t *)"200V");
    DrawText(cx+dx-25, cy+8, (uint8_t *)"400V");

    sprintf(buffer, "%d%%", ctx->calibrationPoints[0]);
    DrawText(cx-35, y0-6, (uint8_t *)buffer);
    sprintf(buffer, "%d%%", ctx->calibrationPoints[1]);
    DrawText(cx-35, y1-6, (uint8_t *)buffer);
    sprintf(buffer, "%d%%", ctx->calibrationPoints[2]);
    uint16_t shift = 0;
    if (ctx->calibrationPoints[2] == 100) shift = -10;
    DrawText(cx-35+shift, y2-6, (uint8_t *)buffer);

    DrawLine(cx+(dx / 5), y0, cx+(dx / 2), y1, UTIL_LCD_COLOR_ST_GRAY_LIGHT);
    DrawLine(cx+(dx / 5), y0+1, cx+(dx / 2), y1+1, UTIL_LCD_COLOR_ST_GRAY_LIGHT);

    DrawLine(cx+(dx / 2), y1, cx+dx, y2, UTIL_LCD_COLOR_ST_GRAY_LIGHT);
    DrawLine(cx+(dx / 2), y1+1, cx+dx, y2+1, UTIL_LCD_COLOR_ST_GRAY_LIGHT);
    font = &Font24; // for potential error message
  }
  else if (ctx->currentState == STATE_F3)
  {
	displayPaddedLine(0, "F3: Voltage and current");
    displayPaddedLine(1, "");
    displayPaddedLine(2, "");
    displayPaddedLine(3, "");
  }

  foregroundColor = UTIL_LCD_COLOR_RED;
  displayPaddedLine(4, ctx->message);

  RenderNewFrame();
}
