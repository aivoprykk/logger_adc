
/* Ints are used here to be able to include the file in assembly as well */

#ifndef A1D8D856_FFE1_4FF9_B520_2CE501EE95E1
#define A1D8D856_FFE1_4FF9_B520_2CE501EE95E1

#define EXAMPLE_ADC_CHANNEL     7 // ADC_CHANNEL_7, GPIO35 on ESP32, GPIO8 on ESP32-S3
#define EXAMPLE_ADC_UNIT        0 // ADC_UNIT_1
#define EXAMPLE_ADC_ATTEN       3 // ADC_ATTEN_DB_12
#define EXAMPLE_ADC_WIDTH       0 // ADC_BITWIDTH_DEFAULT

/* Set low and high thresholds, approx. 1.35V - 1.75V*/
#define EXAMPLE_ADC_LOW_TRESHOLD    1500
#define EXAMPLE_ADC_HIGH_TRESHOLD   2000


#endif /* A1D8D856_FFE1_4FF9_B520_2CE501EE95E1 */
