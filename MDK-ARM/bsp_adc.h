#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

extern void init_vrefint_reciprocal(void);
extern fp32 get_temperature(void);
extern fp32 get_battery_voltage(void);
#endif
