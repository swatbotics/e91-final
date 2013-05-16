
/*
 *   Waving.cpp
 *
 *   Author: Jordan Cheney
 *
 */
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <algorithm>
#include "Vector.h"
#include "Matrix.h"
#include "MX28.h"
#include "MotionStatus.h"
#include "Kinematics.h"
#include "waving.h"
#include "LinuxCM730.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "Body.h"

using namespace Robot;

int motion_initialization(){
  //////////////////// Framework Initialize ////////////////////////////
  LinuxCM730* linux_cm730 = new LinuxCM730("/dev/ttyUSB0");
  CM730* cm730 = new CM730(linux_cm730);
  if(MotionManager::GetInstance()->Initialize(cm730) == false)
  {
    std::cout<<"Fail to initialize Motion Manager!\n";
    return -1;
  }
  MotionManager::GetInstance()->AddModule((MotionModule*)Body::GetInstance());	
  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
  motion_timer->Start();

  std::cout << "before\n";

  sleep(1);

  std::cout << "after\n";

  MotionManager::GetInstance()->SetEnable(true);
  for (int i=1; i<=20; i++){
    std::cout << "Joint " << i << " at " << Body::GetInstance()->m_Joint.GetAngle(i) << "\n";
    Body::GetInstance()->m_Joint.SetAngle(i, Body::GetInstance()->m_Joint.GetAngle(i));
    Body::GetInstance()->m_Joint.SetEnable(i,true,true);
  }

  return 0;
  
}

double lerp(int a, int b, double t){
  return a*(1-t)+b*t;
}

double getTimeAsDouble() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return tp.tv_sec + 1e-6*tp.tv_usec;
}

int main(){
  if (motion_initialization()!=0){
		std::cout<<"Shit went down"<<std::endl;
		return 0;
	}

  double start = getTimeAsDouble(); 
  double init_angle[21];
  for (int i=1;i<21;i++){
    init_angle[i] = Body::GetInstance()->m_Joint.GetAngle(i);
  }
  while (1) {
    double t = getTimeAsDouble()-start;
    double sin_val = 20*sin(2*(t));
    for (int i=1; i<20; ++i) {
      //      std::cout << "Joint " << i << " at " << Body::GetInstance()->m_Joint.GetAngle(i) << " at time " << (getTimeAsDouble()-start) << "\n";
    }
    double amp = std::min(.01*t, 1.0);
    double a = lerp(0, 1, amp);
    Body::GetInstance()->m_Joint.SetAngle(4,init_angle[4]+a*(-45-init_angle[4]));
    Body::GetInstance()->m_Joint.SetAngle(2,init_angle[2]+a*(-90-init_angle[2]));
    Body::GetInstance()->m_Joint.SetAngle(12,init_angle[12]+a*(90-init_angle[12]));
    Body::GetInstance()->m_Joint.SetAngle(7,.5*sin_val);
    Body::GetInstance()->m_Joint.SetAngle(15, .5*sin_val);

    Body::GetInstance()->m_Joint.SetAngle(6,amp*sin_val);
  }

  return 0;
}
