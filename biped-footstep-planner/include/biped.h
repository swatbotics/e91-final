#ifndef _BIPED_H_
#define _BIPED_H_

#include <cstdio>

enum foot {LEFT, RIGHT};

class biped {
public:
	float x;
	float y;
	float theta;
	foot ft;
	
	biped* pred;
	unsigned int depth;
	
	float costToCome;
	float costToGo;
	
	biped();
	biped(float xin, float yin, float thetain, foot footin);
	float totalCost();
	void draw(FILE* svg, float scale, float width, float height);

};

#endif
