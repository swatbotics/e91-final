/* Hello_World.h for testing darwin functionality
* Author: Jordan Cheney
*/

#include <string.h>

#include "minIni.h"
#include "MotionModule.h"

namespace Robot
{
	class Waving : public MotionModule
	{
	private:
		static Waving* m_UniqueInstance;

		double m_min_angle;
		double m_max_angle;

		void wave();
	};
}
