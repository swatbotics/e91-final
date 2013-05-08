#ifndef _RUBIN_H_
#define _RUBIN_H_


#include <vector>


// This is a class to hold the node for a state of the dubin's car

class Dubin {
public:

    float x;
    float y;
    float theta;
    
    float costToCome;
    float costToGo;

    Dubin* predecessor;
	float archRadius;
    unsigned int depth;
    
    Dubin(); // Default constructor
    Dubin(float x, float y, float angle); // predecessor is NULL

    void setPos(float x, float y);

    float getTotalCost();
	
	void setPred(Dubin* pred, float r);

    void print();

};


#endif
    
