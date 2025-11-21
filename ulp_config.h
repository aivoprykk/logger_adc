#ifndef E7737681_C7A7_48A5_B2CA_9C5F2A177F60
#define E7737681_C7A7_48A5_B2CA_9C5F2A177F60

#include "ulp_adc_config.h"

#ifdef __ASSEMBLER__
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
/* Assembly code - use our ULP definitions directly */
#define _ADC_UNIT_0     ULP_ADC_UNIT
#define _ADC_ATTEN      ULP_ADC_ATTEN
#else
/* C code - map to ESP-IDF enum values */
#if ULP_ADC_UNIT == 0
#define _ADC_UNIT_0     ADC_UNIT_1
#elif ULP_ADC_UNIT == 1
#define _ADC_UNIT_0     ADC_UNIT_2
#else
#define _ADC_UNIT_0     ADC_UNIT_1
#endif

#if ULP_ADC_ATTEN == 0
#define _ADC_ATTEN      ADC_ATTEN_DB_0
#elif ULP_ADC_ATTEN == 1
#define _ADC_ATTEN      ADC_ATTEN_DB_2_5
#elif ULP_ADC_ATTEN == 2
#define _ADC_ATTEN      ADC_ATTEN_DB_6
#elif ULP_ADC_ATTEN == 3
#define _ADC_ATTEN      ADC_ATTEN_DB_12  /* Use DB_12 instead of deprecated DB_11 */
#else
#define _ADC_ATTEN      ADC_ATTEN_DB_0
#endif

/* Map channel from config to ESP-IDF enum */
#if ULP_ADC_CHANNEL == 0
#define _ADC_CHANNEL_0  ADC_CHANNEL_0
#elif ULP_ADC_CHANNEL == 1
#define _ADC_CHANNEL_0  ADC_CHANNEL_1
#elif ULP_ADC_CHANNEL == 2
#define _ADC_CHANNEL_0  ADC_CHANNEL_2
#elif ULP_ADC_CHANNEL == 3
#define _ADC_CHANNEL_0  ADC_CHANNEL_3
#elif ULP_ADC_CHANNEL == 4
#define _ADC_CHANNEL_0  ADC_CHANNEL_4
#elif ULP_ADC_CHANNEL == 5
#define _ADC_CHANNEL_0  ADC_CHANNEL_5
#elif ULP_ADC_CHANNEL == 6
#define _ADC_CHANNEL_0  ADC_CHANNEL_6
#elif ULP_ADC_CHANNEL == 7
#define _ADC_CHANNEL_0  ADC_CHANNEL_7
#elif ULP_ADC_CHANNEL == 8
#define _ADC_CHANNEL_0  ADC_CHANNEL_8
#elif ULP_ADC_CHANNEL == 9
#define _ADC_CHANNEL_0  ADC_CHANNEL_9
#else
#define _ADC_CHANNEL_0  ADC_CHANNEL_0
#endif

/* Map bitwidth to ESP-IDF enum */
#if ULP_ADC_BITWIDTH == 9
#define _ADC_BITWIDTH   ADC_BITWIDTH_9
#elif ULP_ADC_BITWIDTH == 10
#define _ADC_BITWIDTH   ADC_BITWIDTH_10
#elif ULP_ADC_BITWIDTH == 11
#define _ADC_BITWIDTH   ADC_BITWIDTH_11
#elif ULP_ADC_BITWIDTH == 12
#define _ADC_BITWIDTH   ADC_BITWIDTH_12
#else
#define _ADC_BITWIDTH   ADC_BITWIDTH_DEFAULT
#endif
#endif /* __ASSEMBLER__ */

#endif /* E7737681_C7A7_48A5_B2CA_9C5F2A177F60 */
