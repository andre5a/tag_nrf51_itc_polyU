#ifndef ADC_H__
#define ADC_H__

#define NR_BATTERY_READINGS_FOR_MEAN 6

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     300//1200                                              /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      3                                                 /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS    270                                               /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define ADC_BUF_SIZE 50                           /**< ADC buffer size. */



/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)


uint16_t adc_read_fifo(void);
void adc_write_fifo(uint16_t val);
uint16_t adc_used_fifo(void);


uint8_t battery_level(void);
uint16_t battery_level_mean(const uint16_t mvolts);
void battery_reading_start(void);
void adc_configure(void);
void battery_start_read(void);
void battery_adv_update(void);
void battery_perc_update(unsigned char val);

#endif


