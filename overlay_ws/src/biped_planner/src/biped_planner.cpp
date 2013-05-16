#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bipedSearch.h"
#include "pathplanner.h"
#include <biped_planner/RobotSegment.h>
#include <biped_planner/Footstep.h>
#include <biped_planner/Line.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

double planTime = 0;
bool auto_plan = true;
bool show_help = false;
int goalr = 3;
float inflate_h = 1.25;
float inflate_z = 1;
OccupancyGrid<unsigned char> grid;

int maxDepth = 1000;
int viewDepth = 30;

using namespace cv;
using namespace std;

ros::NodeHandle n;
//wrapper that combines PathPlanner and bipedSearch
class BipedPlanner{
    PathPlanner planner;
    StateTrajectory* stateTraj;
    vector<biped*> bipedTrajectory;
    SegmentList segments;

    BipedChecker* checker;
    bipedSearch helper;

    biped* searchResult;

    biped_planner::RobotSegment curseg;
    //GLUquadric* quadric;
    
    public:


    BipedPlanner(){
        //quadric = 0;
        searchResult=0;
        checker=0;
        //TODO: throw in real topic name
    }


    void initializeSegments(char* filename){
        Mat image;
        image = imread(filename, CV_LOAD_IMAGE_COLOR);
        if(!image.data){
            cout << "couldn't open file";
        }
        namedWindow("Display", CV_WINDOW_AUTOSIZE);
        imshow("Display", image);
        waitKey(0);
        Mat binaryImage;
        //adaptiveThreshold(image, binaryImage, 50);
        //vector<Point2d> contours;
        //findContours(binaryImage, contours);
    }

    void initializeDefaultSegments(){
        segments.push_back(Line(20, 55, 60, 55)); 
        segments.push_back(Line(20, 5, 20, 55)); //TODO: real input
        segments.push_back(Line(60, 5, 20, 5)); 
        segments.push_back(Line(60, 55, 60, 5));
    }

    void initializePathPlanner(){
        //initializeSegments(filename);
        planner = PathPlanner(segments);
        planner.populateTrajectory();
        StateTrajectory* traj = planner.getPathTrajectory();
        for (int i = 0; i < traj->size(); i++){
            cout << traj->at(i).toString() << endl;
        }
    }

    biped_planner::RobotSegment getCurrentSegment(){
        return curseg;
    }

    biped* getCurrentBiped(){
        return bipedTrajectory.back();
    }

    biped* popBiped(){
        biped* out = getCurrentBiped();
        bipedTrajectory.pop_back();
        return out;
    }

    biped* searchTrajectory(){
        stateTraj = planner.getPathTrajectory();
        RobotSegment curstate = stateTraj->back();
        for(int i = stateTraj->size() - 2; i >= 0; i --){
            RobotSegment goalstate = stateTraj->at(i);
            checker = new BipedChecker(&grid, goalstate.robot_pos[0],
                                      goalstate.robot_pos[1], inflate_h, inflate_z);
            //hardcoded r? TODO: fix
            biped* output=helper.search(curstate.robot_pos[0], curstate.robot_pos[1],
                                   curstate.theta, goalstate.robot_pos[0],
                                   goalstate.robot_pos[1], 3, goalstate.theta,
                                   checker, maxDepth, viewDepth); 
            bipedTrajectory.push_back(output);
            curstate = goalstate;
            //stateTraj->pop_back();
        }

    }

};

    
//TODO: fill RobotSegment object
void updateSegment(const std_msgs::String::ConstPtr& msg){
    biped_planner::RobotSegment out;
    biped_planner::Footstep fstep;
    //fstep.

    //curseg.stroke = ;
    //curseg.footsteps = ;
    //curseg = out;
}

int main(int argc, char ** argv){
    BipedPlanner bPlanner;
    if(argc > 1){
        bPlanner.initializeSegments(argv[1]);
    } else {
        bPlanner.initializeDefaultSegments();
    }
    bPlanner.initializePathPlanner();
    bPlanner.searchTrajectory();

    ros::init(argc, argv, "biped_planner");

    ros::Subscriber sub = n.subscribe("herpaderp", 1000, updateSegment);
    ros::Publisher pub = n.advertise<biped_planner::RobotSegment>(
                                                "current_segment", 1000);

    //bPlanner.setSubscriber(sub);
    ros::Rate loop_rate(10);
    while(ros::ok()){

        pub.publish(bPlanner.getCurrentSegment());
        ros::spinOnce();
    }
}
