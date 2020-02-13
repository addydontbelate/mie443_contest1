#ifndef _INCLUDE_PID_H_
#define _INCLUDE_PID_H_

class PID
{
 private:
   float dt;
   float max;
   float min;
   float Kp;
   float Kd;
   float Ki;
   float prev_error;
   float integral;

 public:
   // Kp -  proportional gain
   // Ki -  Integral gain
   // Kd -  derivative gain
   // dt -  loop interval time
   // max - maximum value of manipulated variable
   // min - minimum value of manipulated variable
   PID(float i_dt, float i_max, float i_min, float i_Kp, float i_Kd, float i_Ki);
   PID();

   // returns the manipulated variable given a setpoint and current process value
   float calculate(float setpoint, float pv);
   ~PID() {};
};

#endif // INCLUDE_PID_H