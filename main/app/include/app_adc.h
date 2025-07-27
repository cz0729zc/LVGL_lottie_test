#ifndef _ADC_APP_H_
#define _ADC_APP_H_

void adc_app_init(void);
void adc_app_task(void *param);
void adc_app_task_start_read_task(uint32_t priority, uint32_t stack_size);

#endif