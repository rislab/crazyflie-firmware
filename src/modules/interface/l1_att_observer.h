#ifndef L1_ATT_OBSERVER_H_
#define L1_ATT_OBSERVER_H_

#include "stabilizer_types.h"

void L1AttObserverSetParameters(float Ix, float Iy, float Iz, float Mscale_xy, float Mscale_z, float massThrust);
void L1AttObserverInit(control_t *control, sensorData_t *sensors);
void L1AttObserverUpdate(control_t *control, sensorData_t *sensors, float dt);
void L1AttObserverApply(float *moment);

#endif /* L1_ATT_OBSERVER_H_ */
