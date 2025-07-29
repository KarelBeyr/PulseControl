#ifndef FLEXIKEYBOARD_FLEXIKEYBOARD_H_
#define FLEXIKEYBOARD_FLEXIKEYBOARD_H_

#include "main.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIOPin;

KeyboardButton ReadFlexiKeyboard(void);
void InitFlexiKeyboard(void);


#endif /* FLEXIKEYBOARD_FLEXIKEYBOARD_H_ */
