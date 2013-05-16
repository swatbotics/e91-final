#include <math.h>
#include <vector>
#include <iostream>
#include "footprint.h"

int main() {
    double radius = 1;
    double distance = 2;
    double width = .2;
    double max_length = .5;
    double max_angle = M_PI / 6;

    Footprint* foot_l = new Footprint(0, width, 0, true);
    Footprint* foot_r = new Footprint(0, -width, 0, false);
    bool stance_is_left = false;        // start on right foot
    std::vector<Footprint> footprints;

    footprints = walkCircle(radius,
                            distance,
                            width,
                            max_length,
                            max_angle,
                            foot_l,
                            foot_r,
                            stance_is_left);

    for(std::vector<Footprint>::iterator it = footprints.begin(); it < footprints.end(); it++) {
        // std::cout << "[" << it->x << ", " << it->y << " @ " << it->theta << "]" << std::endl;
        std::cout
            << it->x() << ", "
            << it->y() << ", "
            << .3 * cos(it->theta()) << ", "
            << .3 * sin(it->theta())
            << std::endl;
    }
	return 0;
}

