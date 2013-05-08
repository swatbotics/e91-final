
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

  MotionManager::GetInstance()->SetEnable(true);
  for (int i=1; i<=20; i++){
    Body::GetInstance()->m_Joint.SetEnable(i,true,true);
    Body::GetInstance()->m_Joint.SetAngle(i,0);
  }

  return 0;
  
}



double getTimeAsDouble() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return tp.tv_sec + 1e-6*tp.tv_usec;
}

int main(){
  /*
 LinuxCM730 linux_cm730("/dev/ttyUSB0");
  CM730 cm730(&linux_cm730);
  if(MotionManager::GetInstance()->Initialize(&cm730) == false)
  {
    std::cout<<"Fail to initialize Motion Manager!\n";
    return -1;
  }
  MotionManager::GetInstance()->AddModule((MotionModule*)Body::GetInstance());	
  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());

  std::cout << "Made timer\n";
  motion_timer->Start();
  std::cout << "Did timer\n";

  MotionManager::GetInstance()->SetEnable(true);
  for (int i=1; i<=20; i++){
    Body::GetInstance()->m_Joint.SetEnable(i,true,true);
    Body::GetInstance()->m_Joint.SetAngle(i,0);
  }

  std::cout << "About to loop\n";
  */
  if (motion_initialization()!=0){
		std::cout<<"Shit went down"<<std::endl;
		return 0;
	}

  double start = getTimeAsDouble();

  while (1) {
    double t = getTimeAsDouble()-start;
    double sin_val = 20*sin(2*(t));
    for (int i=1; i<20; ++i) {
      std::cout << "Joint " << i << " at " << Body::GetInstance()->m_Joint.GetAngle(i) << " at time " << (getTimeAsDouble()-start) << "\n";
    }
    double amp = std::min(4*t, 1.0);

    Body::GetInstance()->m_Joint.SetAngle(4,-45);
    Body::GetInstance()->m_Joint.SetAngle(2,-90);
    Body::GetInstance()->m_Joint.SetAngle(12,90);
    Body::GetInstance()->m_Joint.SetAngle(7,.5*sin_val);
    Body::GetInstance()->m_Joint.SetAngle(15, .5*sin_val);

    Body::GetInstance()->m_Joint.SetAngle(6,amp*sin_val);

  };

  return 0;
}
