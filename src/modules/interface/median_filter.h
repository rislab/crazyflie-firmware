#ifndef MEDIAN_FILTER_H_
#define MEDIAN_FILTER_H_

void MedianFilterInit(void);
void MedianFilterUpdate(float gx, float gy, float gz);
void MedianFilterQuery(float* gx, float* gy, float* gz);

#endif /* MEDIAN_FILTER_H_ */
