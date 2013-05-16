#ifndef _BODY_H_
#define _BODY_H_

#include <string.h>

#include "MotionModule.h"

namespace Robot {
  class Body : public MotionModule {
  private:
    static Body* m_UniqueInstance;
    Body();
  public:
    static Body *GetInstance() {return m_UniqueInstance;}
    ~Body();
    void Initialize();
    void Process();
  };
}

#endif  
