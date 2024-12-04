//
// Created by gl on 09.10.24.
//

#ifndef OPENCANBOX_FILTERS_H
#define OPENCANBOX_FILTERS_H
#include "main.h"

void fixACCbuttons(canMsg *msg);
void eqPresets(canMsg *msg);
void fixTSR(uint32_t delay, uint8_t rep);

#endif //OPENCANBOX_FILTERS_H
