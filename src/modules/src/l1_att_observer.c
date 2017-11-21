#include "l1_att_observer.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math.h"

// Global variables
static bool l1_initialized = false;
static bool enable_l1 = true;
static bool in_nominal_flight = false;

static float mass = 0.04f;
static float gravity = 9.81f;
static float l1_engage_level = 0.1f;
static float cmd_ttl_us = 0.3f;
//static float observer_gain[3] = {0.0f, 0.0f, 0.0f};
static float observer_gain[3] = {400.0f, 400.0f, 400.0f};
//static float adaptation_gain[3] = {0.0f, 0.0f, 0.0f};
static float adaptation_gain[3] = {5.0f, 5.0f, 10.0f};
static float motor_constant = 3.6f;
static float bandwidth[3] = {5.0f, 5.0f, 5.0f};

static float rpmhat[4];
static float avlhat[3];
static float dsthat[3];
static float lpd[3];

static float Ixx = 0.0;
static float Iyy = 0.0;
static float Izz = 0.0;
static float mass_thrust = 0.0;
static float mscale_xy = 0.0;
static float mscale_z = 0.0;

#define M_PI_F ((float) M_PI)
#define limitThrust(VAL) limitUint16(VAL)

void L1AttObserverSetParameters(float Ix, float Iy, float Iz, float Mscale_xy, float Mscale_z, float massThrust)
{
  Ixx = Ix;
  Iyy = Iy;
  Izz = Iz;
  mscale_xy = Mscale_xy;
  mscale_z = Mscale_z;
  mass_thrust = massThrust;
}

void L1AttObserverInit(const fm_t *fm, sensorData_t *sensors)
{
  in_nominal_flight = false;

  dsthat[0] = 0.0f;
  dsthat[1] = 0.0f;
  dsthat[2] = 0.0f;

  lpd[0] = dsthat[0];
  lpd[1] = dsthat[1];
  lpd[2] = dsthat[2];

  avlhat[0] = sensors->gyro.x/180.0f*M_PI_F;
  avlhat[1] = sensors->gyro.y/180.0f*M_PI_F;
  avlhat[2] = sensors->gyro.z/180.0f*M_PI_F;

  float m1 = 0.25f*fm->thrust - 7.6859f*fm->moment_x + 7.6859f*fm->moment_y + 147.0588f*fm->moment_z;
  float m2 = 0.25f*fm->thrust - 7.6859f*fm->moment_x - 7.6859f*fm->moment_y - 147.0588f*fm->moment_z;
  float m3 = 0.25f*fm->thrust + 7.6859f*fm->moment_x - 7.6859f*fm->moment_y + 147.0588f*fm->moment_z;
  float m4 = 0.25f*fm->thrust + 7.6859f*fm->moment_x + 7.6859f*fm->moment_y - 147.0588f*fm->moment_z;

  rpmhat[0] = limitThrust(3.991e6f*m1/9.81f - 1260.0f);
  rpmhat[1] = limitThrust(3.991e6f*m2/9.81f - 1260.0f);
  rpmhat[2] = limitThrust(3.991e6f*m3/9.81f - 1260.0f);
  rpmhat[3] = limitThrust(3.991e6f*m4/9.81f - 1260.0f);

  l1_initialized = true;
}

void L1AttObserverUpdate(const fm_t *fm, sensorData_t *sensors, float dt)
{
  if(enable_l1)
  {
    float rates[3];
    rates[0] = sensors->gyro.x/180.0f*M_PI_F;
    rates[1] = sensors->gyro.y/180.0f*M_PI_F;
    rates[2] = sensors->gyro.z/180.0f*M_PI_F;

    if (!l1_initialized || (dt > cmd_ttl_us))
      L1AttObserverInit(fm, sensors);

    if(fm->thrust < l1_engage_level*mass*gravity)
    {
      l1_initialized = false;
      L1AttObserverInit(fm, sensors);
      in_nominal_flight = false;
    }else{
      in_nominal_flight = true;
    }

    if(in_nominal_flight){
      // From power_distribution_stock.c
      float rpm_cmd[4];
      {
        float m1 = 0.25f*fm->thrust - 7.6859f*fm->moment_x + 7.6859f*fm->moment_y + 147.0588f*fm->moment_z;
        float m2 = 0.25f*fm->thrust - 7.6859f*fm->moment_x - 7.6859f*fm->moment_y - 147.0588f*fm->moment_z;
        float m3 = 0.25f*fm->thrust + 7.6859f*fm->moment_x - 7.6859f*fm->moment_y + 147.0588f*fm->moment_z;
        float m4 = 0.25f*fm->thrust + 7.6859f*fm->moment_x + 7.6859f*fm->moment_y - 147.0588f*fm->moment_z;

        rpm_cmd[0] = limitThrust(3.991e6f*m1/9.81f - 1260.0f);
        rpm_cmd[1] = limitThrust(3.991e6f*m2/9.81f - 1260.0f);
        rpm_cmd[2] = limitThrust(3.991e6f*m3/9.81f - 1260.0f);
        rpm_cmd[3] = limitThrust(3.991e6f*m4/9.81f - 1260.0f);
      }

      float werr[3];
      werr[0] = avlhat[0] - rates[0];
      werr[1] = avlhat[1] - rates[1];
      werr[2] = avlhat[2] - rates[2];

      // Mixer stuff
      float tauhat[3];
      float motor_force[4];
      motor_force[0] = 2.483e-7f*rpmhat[0] + 0.0003859f;
      motor_force[1] = 2.483e-7f*rpmhat[1] + 0.0003859f;
      motor_force[2] = 2.483e-7f*rpmhat[2] + 0.0003859f;
      motor_force[3] = 2.483e-7f*rpmhat[3] + 0.0003859f;
      tauhat[0] = (-0.0325f*motor_force[0] -0.0325f*motor_force[1] + 0.0325f*motor_force[2] + 0.0325f*motor_force[3]);
      tauhat[1] = (0.0325f*motor_force[0] -0.0325f*motor_force[1] - 0.0325f*motor_force[2] + 0.0325f*motor_force[3]);
      tauhat[2] = (0.0017f*motor_force[0] -0.0017f*motor_force[1] + 0.0017f*motor_force[2] - 0.0017f*motor_force[3]);

      // avlhatdot = I_inv*(tauhat + dsthat - rates x (I * rates)) - observer_gain .* werr
      float avlhatdot[3];
      avlhatdot[0] = (tauhat[0] + dsthat[0] - rates[1] * Izz * rates[2] - rates[2] * Iyy * rates[1]) / Ixx - observer_gain[0] * werr[0];
      avlhatdot[1] = (tauhat[1] + dsthat[1] - rates[2] * Ixx * rates[0] - rates[0] * Izz * rates[2]) / Iyy - observer_gain[1] * werr[1];
      avlhatdot[2] = (tauhat[2] + dsthat[2] - rates[0] * Iyy * rates[1] - rates[1] * Ixx * rates[0]) / Izz - observer_gain[2] * werr[2];

      float rpmhatdot[4];
      rpmhatdot[0] = (rpm_cmd[0] - rpmhat[0])*motor_constant;
      rpmhatdot[1] = (rpm_cmd[1] - rpmhat[1])*motor_constant;
      rpmhatdot[2] = (rpm_cmd[2] - rpmhat[2])*motor_constant;
      rpmhatdot[3] = (rpm_cmd[3] - rpmhat[3])*motor_constant;

      float dsthatdot[3];
      dsthatdot[0] = -Ixx*adaptation_gain[0]*werr[0];
      dsthatdot[1] = -Iyy*adaptation_gain[1]*werr[1];
      dsthatdot[2] = -Izz*adaptation_gain[2]*werr[2];

      rpmhat[0] += rpmhatdot[0]*dt;
      rpmhat[1] += rpmhatdot[1]*dt;
      rpmhat[2] += rpmhatdot[2]*dt;
      rpmhat[3] += rpmhatdot[3]*dt;

      avlhat[0] += avlhatdot[0]*dt;
      avlhat[1] += avlhatdot[1]*dt;
      avlhat[2] += avlhatdot[2]*dt;

      dsthat[0] += dsthatdot[0]*dt;
      dsthat[1] += dsthatdot[1]*dt;
      dsthat[2] += dsthatdot[2]*dt;

      lpd[0] += bandwidth[0]*(dsthat[0] - lpd[0])*dt;
      lpd[1] += bandwidth[1]*(dsthat[1] - lpd[1])*dt;
      lpd[2] += bandwidth[2]*(dsthat[2] - lpd[2])*dt;
    }
  }
}

void L1AttObserverApply(float *moment)
{
  if(enable_l1 && in_nominal_flight)
  {
    moment[0] += lpd[0];
    moment[1] += lpd[1];
    moment[2] += lpd[2];
  }
}

//PARAM_GROUP_START(l1_observer_params)
//PARAM_ADD(PARAM_FLOAT, mass, &mass)
//PARAM_ADD(PARAM_FLOAT, gravity, &gravity)
//PARAM_ADD(PARAM_FLOAT, l1_engage_level, &l1_engage_level)
//PARAM_ADD(PARAM_FLOAT, cmd_ttl_us, &cmd_ttl_us)
//PARAM_GROUP_STOP(l1_observer_params)

LOG_GROUP_START(l1_att_observer)
LOG_ADD(LOG_FLOAT, lpd_x, &lpd[0])
LOG_ADD(LOG_FLOAT, lpd_y, &lpd[1])
LOG_ADD(LOG_FLOAT, lpd_z, &lpd[2])
LOG_GROUP_STOP(l1_att_observer)
