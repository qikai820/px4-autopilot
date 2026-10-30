#include "VoliroTAllocation.hpp"
#include <cstdio>
#include <cstdlib>
using namespace matrix;
int main()
{
 const voliro_t::Geometry g{.03f,.45f,0.f,.75f,39.5f,10.f,.02f};
 // At zero tilt, each motor's yaw torque identifies its reaction sign.
 const float expected_yaw_sign[4]{1.f,-1.f,-1.f,1.f};
 for (int i=0;i<4;++i) {
  voliro_t::Result single;
  single.thrust[i]=1.f;
  voliro_t::forward(g,single);
  if (fabsf(single.torque(2)-expected_yaw_sign[i]*g.km)>1e-6f) return 6;
 }
 // A small negative pitch correction during takeoff must not send the front
 // rotors towards 180 degrees while upward thrust is requested.
 auto takeoff=voliro_t::allocate(g,Vector3f(.005f,0.f,-.168f),Vector3f(0.f,-.18f,0.f),
                                0.f,math::radians(120.f),math::radians(2.f));
 if (fabsf(takeoff.tilt)>1e-5f || takeoff.force(2)>0.f) return 5;
 for(int tail_sign=-1;tail_sign<=1;tail_sign+=2) {
 for(int degree=-100;degree<=100;degree+=5) {
  voliro_t::Result source; source.tilt=math::radians(static_cast<float>(degree));
  source.thrust[0]=8.2f;source.thrust[1]=7.8f;source.thrust[2]=7.9f;source.thrust[3]=8.1f;source.thrust[4]=2.f*static_cast<float>(tail_sign);
  voliro_t::forward(g,source);
  auto r=voliro_t::allocate(g,source.force,source.torque,source.tilt,math::radians(120.f),.1f);
  if ((source.force-r.force).norm()>1e-4f || (source.torque-r.torque).norm()>1e-4f) {puts("FAIL feasible wrench");return 1;}
 }
 }
 auto limited=voliro_t::allocate(g,Vector3f(80.f,3.f,-300.f),Vector3f(10.f,15.f,20.f),0.f,1.f,.01f);
 if(fabsf(limited.tilt)>.01001f || fabsf(limited.force(1))>1e-5f) return 2;
 for(int i=0;i<4;++i) if(limited.thrust[i]<0.f || limited.thrust[i]>g.front_max) return 3;
 if(fabsf(limited.thrust[4])>g.tail_max) return 4;
 puts("PASS: feasible common-tilt wrench -100..100 degrees; side force residual; actuator/angle/rate limits");
}
