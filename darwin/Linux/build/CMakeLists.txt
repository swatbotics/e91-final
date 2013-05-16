# swatbotics/darwin/Linux/build/CMakeLists.txt



set(src_files 
  ../../Framework/src/CM730.cpp
  ../../Framework/src/MX28.cpp
  ../../Framework/src/math/Matrix.cpp   
  ../../Framework/src/math/Plane.cpp    
  ../../Framework/src/math/Point.cpp    
  ../../Framework/src/math/Vector.cpp   
  ../../Framework/src/motion/JointData.cpp  	
  ../../Framework/src/motion/Kinematics.cpp 	
  ../../Framework/src/motion/MotionManager.cpp  
  ../../Framework/src/motion/MotionStatus.cpp   
  ../../Framework/src/motion/modules/Action.cpp 
  ../../Framework/src/motion/modules/Head.cpp   
  ../../Framework/src/motion/modules/Walking.cpp
  ../../Framework/src/motion/modules/Body.cpp
  ../../Framework/src/vision/BallFollower.cpp   
  ../../Framework/src/vision/BallTracker.cpp    
  ../../Framework/src/vision/Camera.cpp
  ../../Framework/src/vision/ColorFinder.cpp    
  ../../Framework/src/vision/Image.cpp  		
  ../../Framework/src/vision/ImgProcess.cpp 	
  ../../Framework/src/minIni/minIni.c	
  streamer/httpd.cpp           
  streamer/jpeg_utils.cpp      
  streamer/mjpg_streamer.cpp   
  LinuxActionScript.cpp   
  LinuxCamera.cpp   
  LinuxCM730.cpp    
  LinuxMotionTimer.cpp    
  LinuxNetwork.cpp)

add_library(darwin ${src_files})


