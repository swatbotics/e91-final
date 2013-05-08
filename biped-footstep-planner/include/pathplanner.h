/*PathPlanner class by Jackie Kay
Takes a collection of line segments and outputs a trajectory of robot states,
including arm configurations and positions
list of "walking goals" and "drawing goals"

Things to think about:
TIMING. how to time a robot state. is this my problem?
*/
#ifndef _PATHPLANNER_H_
#define _PATHPLANNER_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <string>
#include <eigen3/Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
#define PI 3.14159265

//typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Vector2d Point2d;
typedef Eigen::aligned_allocator<Eigen::Vector4d> lineAllocator;
//typedef Eigen::aligned_allocator<Eigen::Vector4d> stateAllocator;
typedef Eigen::aligned_allocator<Vector6d> stateAllocator;

const Point2d safety = Point2d(-0.02, -0.02);
const float epsilon = 0.05;
const float default_radius = 30;
const float delta = default_radius*0.8;
const float centerx = 0;
const float centery = 0;

/****************Definitions****************/
// State enumerations
enum {
    STATE_WALKING  = 0, /*ZMP walking. (x, y) of our COM is moving, hopefully
                            our z is constant, and our arms are stationary*/
    STATE_PUT_PEN  = 1, /*Legs are stationary, (x, y) is mostly constant.
                        We are placing the pen on the paper*/
    STATE_DRAWING  = 2, /*We are doing some crazy stuff to draw the line.
                         (x,y,z) might move to maximize kinematic workspace?
                         legs stationary.*/
    STATE_LIFT_PEN = 3 /*We are lifting the pen from the paper. like put_pen
                        but in reverse*/
};

/* PathState: stores kinematic and state information about the robot 
I might be storing too much or too little stuff here but it's good to keep
around for now; still questioning the timestep variable. At the very least
I need the state variable (to keep track of what the other parts of the program
need to be doing). robot_pos also seems pretty important as it is an input to
ZMP walking
*/
struct PathState {
    int t; //current timestep
    int state; //will store enums
    //Vector6d arm_state; //arm joint values
    //Vector3d robot_pos_3d; //(x, y, z) of robot
    Point2d robot_pos;
    float theta;
    Point2d pen_pos;   //(x, y, z) of pen end effector
    std::string toString() const;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PathState(){ }
};

/*Line segment: a pair of points... or an uncountably infinite number of points?!*/
struct Line {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Line(Point2d q1, Point2d q2);
    Line(float x1, float y1, float x2, float y2);
    Line(){ }
    std::string toString() const;
    Point2d p1;
    Point2d p2;
};

struct RobotSegment {
    //robot x, y, theta in the world frame
    //float x;
    //float y;
    Point2d robot_pos;
    float theta;
    Line stroke; //the line drawn in the current segment. if NAN, we are not drawing

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotSegment() { }
    RobotSegment(float x, float y, float t, Line s);
    RobotSegment(Point2d p, float t, Line s);
    std::string toString() const;
};

typedef std::vector<Line, lineAllocator> SegmentList;
typedef std::vector<RobotSegment, stateAllocator> StateTrajectory;

class KinSpace {

public:
    KinSpace();
    KinSpace(Point2d c, float r);
    
    /*Boundaries is used if our robot has a rectangular workspace.
    The origin in the workspace is the current position of the robot.*/
    Point2d boundaries[4];
    bool circle;
    Point2d center; /*Center of the semicircle; actually the translation
                    transformation from the robot's center to the workspace
                    center?*/
    float radius; //Radius of the semicircle
    bool righthanded; //so we can change robot handedness-untested
    
    //Is the goal reachable from the robot's current position in the plane?
    bool isPointReachable(Point2d goal, Point2d position, float theta) const;

};

class PathPlanner {
public:


    /****************Class members****************/
    KinSpace workspace;
    //std::vector<PathState, stateAllocator> state_traj; //trajectory of PathState objects
    StateTrajectory state_traj;

    std::vector<Line, lineAllocator> line_segments; //initial input

    /****************Functions****************/
    PathPlanner();
    PathPlanner(std::vector<Line, lineAllocator> segments);
    void setInputSegments(std::vector<Line, lineAllocator> segments);
    //std::vector<PathState, stateAllocator>* getPathTrajectory();
    StateTrajectory* getPathTrajectory();
    void populateTrajectory(); //fill in the PathState objects

private:
    /****************Class members****************/
    RobotSegment cur_state;

    /****************Functions****************/
    void orderSegments(); /*if initially out of order. For now I will ignore
                            this and assume we are getting them in order*/
    Point2d getCOMfromPenPos(Point2d pen_pos) const;
    Point2d getPenPosfromCOM(Point2d com) const;
    void pushCurrentState(Point2d robot_pos, float theta, Line stroke);
    void pushWalkState(Point2d robot_pos, float theta);
    void pushWalkStates(const Point2d origin, const Point2d goal);
    void pushDrawStates(const Line draw_path);
    Line cutSegment(const Line segment);
    bool isLineWithinWorkspace(Line segment) const;
    bool cutSegmentOld(const Line cur_seg, Line& output, const Point2d curpos);
                        /*get slice of input based on size of workspace*/
    Point2d findFarthestReachablePoint(Point2d origin, Line cur_seg) const;
    
};
#endif
