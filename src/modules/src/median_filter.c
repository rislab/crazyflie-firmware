#include "median_filter.h"
#include "log.h"
#include "param.h"
#include "math.h"

#define N_ELEM 11

static float gyro_x[N_ELEM];
static uint8_t gyro_x_ind = 0;
static float gyro_y[N_ELEM];
static uint8_t gyro_y_ind = 0;
static float gyro_z[N_ELEM];
static uint8_t gyro_z_ind = 0;

void MedianFilterInit(void)
{
  for(int i=0;i<N_ELEM;i++){
    gyro_x[i] = 0.0f;
    gyro_y[i] = 0.0f;
    gyro_z[i] = 0.0f;
  }
  gyro_x_ind = 0;
  gyro_y_ind = 0;
  gyro_z_ind = 0;
}

void MedianFilterUpdate(float gx, float gy, float gz)
{
  gyro_x[gyro_x_ind] = gx;
  gyro_x_ind++;
  if(gyro_x_ind >= N_ELEM){
    gyro_x_ind = 0;
  }

  gyro_y[gyro_y_ind] = gy;
  gyro_y_ind++;
  if(gyro_y_ind >= N_ELEM){
    gyro_y_ind = 0;
  }

  gyro_z[gyro_z_ind] = gz;
  gyro_z_ind++;
  if(gyro_z_ind >= N_ELEM){
    gyro_z_ind = 0;
  }
}

double getMedian(const float* array, int size)
{
  float temp;
  float sorted[size];

  for(int i=0;i<size;i++){
    sorted[i] = array[i];
  }

  for(int i=size-1;i > 0; i--){
    for(int j=0;j < i; j++){
      temp = sorted[j];
      sorted[j] = sorted[j+1];
      sorted[j+1] = temp;
    }
  }
  if((size % 2) == 0){
    return (sorted[size/2] + sorted[(size/2) - 1])*0.5f;
  }else{
    return sorted[size/2];
  }
}

void MedianFilterQuery(float* gx, float* gy, float* gz)
{
  *gx = getMedian(gyro_x, N_ELEM);
  *gy = getMedian(gyro_y, N_ELEM);
  *gz = getMedian(gyro_z, N_ELEM);
}
