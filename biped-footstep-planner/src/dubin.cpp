#include "dubin.h"
#include <float.h>
#include <iostream>

using namespace std;

Dubin::Dubin(){
    x = 0;
    y = 0;
    theta=0;
    depth = 0;
    predecessor = NULL;
	archRadius = 0;
    costToCome = FLT_MAX;
    costToGo = FLT_MAX;
}

Dubin::Dubin(float inx, float iny, float angle){
    x = inx;
    y = iny;
    depth=0;
    theta = angle;
    predecessor = NULL;
	archRadius = 0;
    costToCome = FLT_MAX;
    costToGo = FLT_MAX;
}

    void Dubin::setPos(float newx, float newy){x = newx; y = newy;}

    float Dubin::getTotalCost(){return costToGo + costToCome;}

    void Dubin::setPred(Dubin* pred, float r){
        predecessor = pred;
        depth = pred? pred->depth + 1:0;
		archRadius = r;
	}

void Dubin::print(){
    cout<<"x="<<x<<"  y="<<y<<"  theta="<<theta<<"  total cost="<<getTotalCost()<<endl;
}

