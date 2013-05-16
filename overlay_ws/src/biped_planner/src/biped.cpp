#include "biped.h"
#include <float.h>
#include <cstddef>

#define PI 3.141592654

biped::biped() {
	x = 0;
	y = 0;
	theta = 0;
	ft = LEFT;
	pred = NULL;
	depth = 0;
	costToCome = 0;
	costToGo = FLT_MAX;
}

biped::biped(float xin, float yin, float thetain, foot footin) {
	x = xin;
	y = yin;
	theta = thetain;
	ft = footin;
	pred = NULL;
	depth = 0;
	costToCome = 0;
	costToGo = FLT_MAX;
}

float biped::totalCost() {
	return (costToCome+costToGo);
}

void biped::draw(FILE* svg, float scale, float width, float height){
	int color = 127*256*256;
	if (ft==RIGHT) {
		color = 127;
	}
	fprintf(svg, "    <rect x=\"%f\" y=\"%f\" width=\"%f\" height=\"%f\" fill=\"%06x\" transform=\"rotate(%f %f %f)\"/>\n", scale*(x-width/2), scale*(y-height/2), scale*width, scale*height, color, theta*180/PI, scale*x, scale*y);

}
