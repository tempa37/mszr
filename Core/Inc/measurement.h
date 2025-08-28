#ifndef __MEASUREMENT_H__
#define __MEASUREMENT_H__

#define ADC_BUF 500
#define ADC_HALF_BUF 250

#define SQ3         1.732050807f    // sqrt(3)
#define COS30       0.866025403f    // cos(30°)
#define OMEGA       314.1592653f    // 2 * M_PI * 50 ≈ 6.283185307 * 50
#define MULT_UP     43824.0f        // 14608 * 3
#define MULT_DOWN   2.057065f       // 11.365 * 0.181

void HighPriorityTask(void *argument);

#endif /* __MEASUREMENT_H__ */