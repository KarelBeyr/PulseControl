#include "FlexiKeyboard.h"

#define NUM_ROWS 5
#define NUM_COLS 5

GPIOPin rowPins[NUM_ROWS] = {
    { GPIOG, GPIO_PIN_3 }, //D2
    { GPIOA, GPIO_PIN_6 }, //D3
    { GPIOK, GPIO_PIN_1 }, //D4  // careful, this pin is also used in stm32h750b_discovery_lcd.c for LTDC_G6 (green) pin. We overwrite it and hope for the best.
    { GPIOA, GPIO_PIN_8 }, //D5
    { GPIOE, GPIO_PIN_6 }  //D6
};

GPIOPin colPins[NUM_COLS] = {
    { GPIOI, GPIO_PIN_8 },  //D7
    { GPIOE, GPIO_PIN_3 },  //D8
    { GPIOH, GPIO_PIN_15 }, //D9
    { GPIOB, GPIO_PIN_4 },  //D10
	{ GPIOB, GPIO_PIN_15 }  //D11
};

const KeyboardButton keymap[5][5] = {
    {KEY_ESC,   KEY_6,    KEY_8,     KEY_Lock, KEY_F1},
    {KEY_3,     KEY_5,    KEY_7,     KEY_F5,   KEY_OFF},
    {KEY_2,     KEY_4,    KEY_Enter, KEY_F4,   KEY_ON},
    {KEY_1,     KEY_Clear,KEY_Dot,   KEY_F3,   KEY_Stop},
    {KEY_BkSp,  KEY_9,    KEY_0,     KEY_F2,   KEY_Start}
};

volatile int lastRow = -1;
volatile int lastCol = -1;
volatile uint32_t lastTriggerTime = 0;

void setAllRowsInactive(void)
{
    for (int i = 0; i < NUM_ROWS; i++) {
        HAL_GPIO_WritePin(rowPins[i].port, rowPins[i].pin, GPIO_PIN_RESET);
    }
}

void setRowActive(int row)
{
    if (row < 0 || row >= NUM_ROWS)
        return;

    setAllRowsInactive();
    HAL_GPIO_WritePin(rowPins[row].port, rowPins[row].pin, GPIO_PIN_SET);
}

KeyboardButton ReadFlexiKeyboard(void)
{
    for (int row = 0; row < NUM_ROWS; row++)
    {
        setRowActive(row);    // Set current row LOW, others HIGH
        HAL_Delay(1);         // Small delay for settling

        for (int col = 0; col < NUM_COLS; col++)
        {
            if (HAL_GPIO_ReadPin(colPins[col].port, colPins[col].pin) == GPIO_PIN_SET)
            {
                uint32_t now = HAL_GetTick();

                if (lastRow == row && lastCol == col && (now - lastTriggerTime < 300)) { // Debounce/repeat suppression
                    return KEY_NULL;
                }

                lastRow = row;
                lastCol = col;
                lastTriggerTime = now;

                KeyboardButton receivedChar = keymap[row][col];
                //printf("Pressed row %d and col %d hopefully it is %c\r\n", row, col, receivedChar);
                return receivedChar;
            }
        }
    }
    return KEY_NULL;
}

void InitFlexiKeyboard(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//we have to enable this CLK so that we can use output pins
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();


	// --- Configure row pins as OUTPUT ---
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	for (int i = 0; i < NUM_ROWS; i++) {
		GPIO_InitStruct.Pin = rowPins[i].pin;
		HAL_GPIO_Init(rowPins[i].port, &GPIO_InitStruct);
	}

	// --- Configure column pins as INPUT with PULL-DOWN ---
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	for (int i = 0; i < NUM_COLS; i++) {
		GPIO_InitStruct.Pin = colPins[i].pin;
		HAL_GPIO_Init(colPins[i].port, &GPIO_InitStruct);
	}
}
