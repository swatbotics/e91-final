/*PathPlanner implementation by Jackie Kay*/
#include "pathplanner.h"
#include <iostream>
#include <sstream>

using namespace std;

//PathPlanner implementation

string pointToString(Point2d p1){
    stringstream stream;
    stream << "Point: ("<<p1[0]<< ", " <<p1[1] << ")" << endl;
    return stream.str();
}

Line::Line(Point2d q1, Point2d q2){
    p1 = q1;
    p2 = q2;
}

Line::Line(float x1, float y1, float x2, float y2){
    p1 = Point2d(x1, y1);
    p2 = Point2d(x2, y2);
}

string Line::toString() const{
    stringstream stream;
    stream << "Line: (" << p1[0] << ", " << p1[1] << "), (" << p2[0] << ", " <<
        p2[1] << ")";
    return stream.str();
}

RobotSegment::RobotSegment(float x, float y, float t, Line s){
    RobotSegment( Point2d(x, y), t, s);
}

RobotSegment::RobotSegment(Point2d p, float t, Line s){
    robot_pos = p;
    theta = t;
    stroke = s;
}

string RobotSegment::toString() const {
    stringstream stream;
    stream << "Robot pos: " << pointToString(robot_pos) << ", Theta: "<< theta << ", Stroke: " << stroke.toString();
    return stream.str();
}

string PathState::toString() const{
    stringstream stream;
    stream << "State: " << state << ", " << pointToString(pen_pos);
    return stream.str();
}

KinSpace::KinSpace(){
    center = Point2d(centerx, centery);
    radius = default_radius;
    circle = true;
}

KinSpace::KinSpace(Point2d c, float r){
    center = c;
    radius = r;
    circle = true;
}

//helper function
float euclideanDistance(Point2d p1, Point2d p2){
    Point2d diff = p1-p2;
    return sqrt(diff.dot(diff));
}


//is theta_new within the semicircle which is at theta degrees
/*bool isAngleWithinSemicircle(float theta, float theta_new){
    //cout << "checking if " << theta_new << " is within semicircle with angle " << theta << endl;
    //return ((theta_new > theta - PI - epsilon) && (theta_new < theta + epsilon));
    return true;
}*/

bool KinSpace::isPointReachable(Point2d goal, Point2d position, float theta) const{
    Point2d goal_t = goal - position; //goal translated so position is origin
    if(!circle){
        //rectangle check...
        if(goal_t[0] < boundaries[2][0] && goal_t[0] > boundaries[0][0]
           && goal_t[1] < boundaries[1][1] && goal_t[1] > boundaries[3][1] ){
            return true;
        }
    }

    //semicircle check
    /*Need to check if point is in the circle and on the correct side: angle
    assume that theta of the robot works for theta of the workspace*/
    Point2d origin = position;//-center; //had center in case there is an offset from
                                         //the robot position and the end effector
    //cout << "Distance from goal to origin: " << euclideanDistance(goal_t, origin) <<endl;
    return euclideanDistance(goal_t, origin) < radius;
    /*if(euclideanDistance(goal_t, origin) < radius){
        //TODO: add left/right-handedness check
        //find angle of the vector made by goal in world
        float theta_new = atan2(goal_t[0], goal_t[1]);
        return isAngleWithinSemicircle(theta, theta_new);
    }
    return false;*/
}

//Constructors
PathPlanner::PathPlanner(){
    //FIXME: read file to get center of kinspace and radius dumb
    //workspace = KinSpace(Point2d(centerx, centery), default_radius);
    //state_traj = new vector<PathState, stateAllocator>();
    //line_segments = new vector<Line, lineAllocator>();
}

PathPlanner::PathPlanner(const vector<Line, lineAllocator> segments){
    PathPlanner();
    line_segments = segments;
}

//Public methods
void PathPlanner::setInputSegments(const vector<Line, lineAllocator> segments){
    line_segments = segments;
}

vector<RobotSegment, stateAllocator>* PathPlanner::getPathTrajectory(){
    return &state_traj;
}

//TODO: need to get structural data to implement this
Point2d PathPlanner::getCOMfromPenPos(Point2d pen_pos) const{
    return pen_pos + safety;
}

Point2d PathPlanner::getPenPosfromCOM(Point2d com) const{
    return com - safety;
}

//helper function to give angle of line
float lineAngle(Line line){
    Point2d difference = line.p2 - line.p1;
    return atan2(difference[1], difference[0]);
}

Point2d getLineMidpoint(Line segment){
    return Point2d((segment.p1+segment.p2)/2);
}

bool PathPlanner::isLineWithinWorkspace(Line segment) const{
    return euclideanDistance(segment.p1, segment.p2) < workspace.radius - epsilon;
}

Line PathPlanner::cutSegment(const Line cur_seg){
    if ( isLineWithinWorkspace(cur_seg) ){
        return cur_seg;
    }
    Point2d p1 = cur_seg.p1;
    Point2d p2 = cur_seg.p2;
    //always assume p1 first
    Point2d p2_new = findFarthestReachablePoint(p1, cur_seg);
    Line output = Line(p1, p2_new);
    line_segments.push_back(Line(p2_new, p2));
    //cout << "pushing back line: " << line_segments.back().toString() <<endl;
    return output;
}

//Private methods
void PathPlanner::populateTrajectory(){
    //place robot near first line segment.
    //p1 of the first line segment determines end effector position, which
    //determines position of robot
    cur_state = RobotSegment();
    cur_state.robot_pos = getCOMfromPenPos(line_segments.back().p1);
    cur_state.theta = lineAngle(line_segments.back());
    
    while (line_segments.size() != 0){
        Line cur_seg = line_segments.back();
        line_segments.pop_back();
        cout << "Current segment: " << cur_seg.toString() << endl;
        //We want to cut a segment that is reachable from the current robot
        //position
        Line cut = cutSegment(cur_seg);
        cout << "got stroke: " << cut.toString() <<endl;
        float theta = lineAngle(cut);
        //robot should stand in the middle of the cut
        pushCurrentState(getCOMfromPenPos(getLineMidpoint(cut)), theta, cut);
        
    }
}

void PathPlanner::pushCurrentState(Point2d robot_pos, float theta, Line stroke){
    state_traj.push_back(RobotSegment(robot_pos, theta, stroke));
    cur_state = state_traj.back();
}

void PathPlanner::pushWalkState(const Point2d robot_pos, const float theta){
    Line stroke = Line(INT_MIN, INT_MIN, INT_MIN, INT_MIN); //THIS IS SUCH A HACK I DON'T EVEN
    state_traj.push_back(RobotSegment(robot_pos, theta, stroke));
    cur_state = state_traj.back();
}

//TODO: implement if needed
void PathPlanner::orderSegments(){

}

class LineUnreachableException: public exception {
    const char* message() const throw(){
        return "Invoked findFarthestReachablePoint with incorrect params";
    }
} myLineUnreachableException;

bool isPointWithinSegment(Point2d point, Line cur_seg){
    if (point == cur_seg.p1 || point == cur_seg.p2){
        return true;
    }
    Point2d p1 = cur_seg.p1;
    Point2d p2 = cur_seg.p2;
    bool dx = cur_seg.p2[0] > cur_seg.p1[0];
    bool dy = cur_seg.p2[1] > cur_seg.p1[1];
    if (dx){
        if (dy){
            return ( point[0] >= p1[0] && point[0] <= p2[0] && point[1] >= p1[1] && point[1] <= p2[1]);
        }
        return ( point[0] >= p1[0] && point[0] <= p2[0] && point[1] <= p1[1] && point[1] >= p2[1]);
    }
    if(dy){
        return (point[0] <= p1[0] && point[0] >= p2[0] && point[1] >= p1[1] && point[1] <= p2[1]);
    }
    return point[0] <= p1[0] && point[0] >= p2[0] && point[1] <= p1[1] && point[1] >= p2[1];
}

//find farthest reachable point on cur_seg from origin 
//don't need curpos (position of robot) because we have a function for that?
Point2d PathPlanner::findFarthestReachablePoint(Point2d origin,
                                                Line cur_seg) const{
    //vomiting algebra and the quadratic formula all over here, sorry
    float x1 = cur_seg.p1[0];
    float y1 = cur_seg.p1[1];
    float x2 = cur_seg.p2[0];
    float y2 = cur_seg.p2[1];
    float p  = cur_state.robot_pos[0]; //+ workspace.center[0];
    float q  = cur_state.robot_pos[1]; //+ workspace.center[1];
    float r  = workspace.radius - epsilon; //epsilon is safety factor
    if(x1==x2){
        Point2d point;
        if (y2-y1 >= 0){
            point = Point2d(x1, y1 + r);
        } else {
            point = Point2d(x1, y1 - r);
        }
        if (isPointWithinSegment(point, cur_seg) ){
            return point;
        }
    }

    float m = (y2-y1)/(x2-x1);
    float c = (y1-m*x1);
    float A = m*m+1;
    float B = 2*(m*c-m*q-p);
    float C = q*q-r*r+p*p-2*c*q+c*c;
    float eval = B*B-4*A*C;

    if(eval < 0){
        //line not reachable
        cout << "quadratic equation failed" <<endl;
        cout << "Current robot state: " << cur_state.toString() << endl;
        throw myLineUnreachableException;
    }
    if(eval == 0){
        Point2d point = Point2d( -B, m*-B+c);
        if (isPointWithinSegment(point, cur_seg) ){
            //cout << "returning farthest reachable point :" << pointToString(point)<<endl;
            return point;
        }
    }
    //two solutions, need to determine which one is farther.
    x1 = (-B+sqrt(eval))/(2*A);
    x2 = (-B-sqrt(eval))/(2*A);
    Point2d point1 = Point2d(x1, m*x1+c);
    Point2d point2 = Point2d(x2, m*x2+c);
    //float theta_2 = atan2(point2[0], point2[1]);
    //cout << "got point1: " << pointToString(point1) <<endl;
    //cout << "got point2: " << pointToString(point2) <<endl;
    if (! isnan(x1) || ! isnan(x2) ){
        if(isPointWithinSegment(point1, cur_seg) ){
        //TODO: is this the correct quadrant of the circle anyway?
            //cout << "returning farthest reachable point :" << pointToString(point1)<<endl;
            return point1;
        }
        if (isPointWithinSegment(point2, cur_seg)){
            //cout << "returning farthest reachable point :" << pointToString(point2)<<endl;
            return point2;
        }
    }

    //TODO: we should rotate a bit here?
    cout << "ya herped when ya should'a derped" << endl;
    throw myLineUnreachableException;
}

//cutSegment: Slice off a piece of line_segments based on size of workspace
//curpos: current position of the robot
//Modifies line_segments
/*bool PathPlanner::cutSegmentOld(const Line cur_seg, Line& output,
                             const Point2d curpos){
    Point2d p1 = cur_seg.p1;
    Point2d p2 = cur_seg.p2;
    float theta = cur_state.theta;
    bool p1_reachable = workspace.isPointReachable(p1, curpos, theta);
    //cout << "P1: Is " << pointToString(p1) << " reachable from " << pointToString(curpos) << "? " <<p1_reachable<<endl;

    bool p2_reachable = workspace.isPointReachable(p2, curpos, theta);
    //cout << "P2: Is " << pointToString(p2) << " reachable from " << pointToString(curpos) << "? " <<p2_reachable<<endl;
    if( p1_reachable && p2_reachable){
        output = cur_seg;
        return true;
    }
    if( !p1_reachable ){ //just switch p1 and p2 here
        Point2d temp = p1;
        p1 = p2;
        p2 = temp;
    }
    if( p1_reachable || p2_reachable){
        //cut a slice
        Point2d p2_new = findFarthestReachablePoint(p1, cur_seg);
        output = Line(p1, p2_new);
        line_segments.push_back(Line(p2_new, p2));
        //cout << "pushing back line: " << line_segments.back().toString() <<endl;
        return true;
    }
    //if neither reachable, we need to walk more
    return false;
}*/
