#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include "zmp/footprint.h"
#include <math.h>

using namespace std;

#define EXPECT_FEET_EQ(foot, _x, _y, _theta, _is_left)  \
  EXPECT_NEAR((foot).x(), _x, 1e-12);                   \
  EXPECT_NEAR((foot).y(), _y, 1e-12);                   \
  EXPECT_DOUBLE_EQ(_theta, (foot).theta());             \
  EXPECT_EQ(_is_left,(foot).is_left);                   \

#define EXPECT_FEET_EQ2(foot1, foot2)                                   \
  EXPECT_FEET_EQ(foot1, (foot2).x(), (foot2).y(), (foot2).theta(), (foot2).is_left);

const bool LEFT = true;
const bool RIGHT = false;
const double pi = atan(1.0)*4;

struct TestHelper {
  double radius;
  double distance;
  double width;
  double max_step_length;
  double max_step_angle;
  Footprint* init_left;
  Footprint* init_right;
  bool left_is_stance_foot;
  vector<Footprint> res;

  /* TestHelper () {} */
  TestHelper (
    double radius,
    double distance,
    double width,
    double max_step_length,
    double max_step_angle,
    Footprint* init_left,
    Footprint* init_right,
    bool left_is_stance_foot
    ) : radius(radius), distance(distance), width(width),
        max_step_length(max_step_length), max_step_angle(max_step_angle),
        init_left(init_left), init_right(init_right),
        left_is_stance_foot(left_is_stance_foot) {
    calcAndSetRes();
  }

  void setRandom() {

  }

  void calcAndSetRes() {
    res = walkCircle(
      radius,
      distance,
      width,
      max_step_length,
      max_step_angle,
      init_left,
      init_right,
      left_is_stance_foot
      );
  }

  void checkAllProperties(){
    propertyFirstFootIsCorrect();
    propertyStepLengthAndAngle();
  }

  void propertyFirstFootIsCorrect() {
    EXPECT_EQ(left_is_stance_foot, res[0].is_left);
    Footprint *foot = left_is_stance_foot ? init_left : init_right;
    EXPECT_FEET_EQ2(*foot, res[0]);
  }

  void propertyStepLengthAndAngle() {
    vector<Footprint> lefts, rights;
    for(unsigned int i = 0; i < res.size(); i++) {
      (res[i].is_left ? lefts : rights).push_back(res[i]);
    }
    propertyOnesStepLength(lefts);
    propertyOnesStepLength(rights);
    propertyOnesAngle(lefts);
    propertyOnesAngle(rights);
  }

  void propertyOnesStepLength(vector<Footprint> fps) {
    // Here, we check that the footsteps for the SAME foot are correct.
    for(unsigned int i = 0; i < fps.size() - 1; i++) {
      // TODO Much pretteier when using Eigen later
      double dx = res[i+1].x() - res[i].x();
      double dy = res[i+1].y() - res[i].y();
      EXPECT_LT(dx*dx + dy*dy, max_step_length*max_step_length*2*2);
    }
  }

  void propertyOnesAngle(vector<Footprint> fps) {
    for(unsigned int i = 0; i < fps.size() - 1; i++) {
      double normalized_change_in_angle = res[i+1].theta() - res[i].theta();
      while(normalized_change_in_angle > M_PI) normalized_change_in_angle -= 2 * M_PI;
      while(normalized_change_in_angle < -M_PI) normalized_change_in_angle += 2 * M_PI;
      EXPECT_LT(abs(normalized_change_in_angle), max_step_angle*2);
    }
  }
};


TEST(Footprint, lineWalking)
{
  vector<Footprint> res = walkLine(10,
                                   1,
                                   3,
                                   new Footprint(0.0, 1.0, 0.0, true),
                                   new Footprint(0.0, -1.0, 0.0, false),
                                   true);

  // FIXME: line should not start with two side-by-side steps

  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);
  int N = res.size();
  EXPECT_DOUBLE_EQ(0, res[0].x());
  EXPECT_DOUBLE_EQ(0, res[1].x());
  EXPECT_DOUBLE_EQ(2.5, res[2].x());
  EXPECT_DOUBLE_EQ(5, res[3].x());
  EXPECT_DOUBLE_EQ(7.5, res[4].x());
  EXPECT_DOUBLE_EQ(10, res[N-2].x());
  EXPECT_DOUBLE_EQ(10, res[N-1].x());
  EXPECT_LT(res[N-3].x(), 9);

  EXPECT_TRUE(res[0].is_left);
  EXPECT_FALSE(res[1].is_left);

  EXPECT_FALSE(res[N-2].is_left);
  EXPECT_TRUE(res[N-1].is_left);


  for ( int i = 0; i < N; i++ ) {
    Footprint &fp = res[i];
    EXPECT_DOUBLE_EQ((fp.is_left-0.5)*2, fp.y());
  }
}

TEST(Footprint, lineWalking2)
{
  double theta = M_PI / 4;
  vector<Footprint> res = walkLine(sqrt(2) * 10,
                                   sqrt(2),
                                   sqrt(2),
                                   new Footprint(-1, 1, theta, true),
                                   new Footprint(1, -1, theta, false),
                                   true);
  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);

  EXPECT_FEET_EQ(res[0], -1, 1, theta, LEFT);
  EXPECT_FEET_EQ(res[1], 1, -1, theta, RIGHT);
  EXPECT_FEET_EQ(res[2], 0, 2, theta, LEFT);
  EXPECT_FEET_EQ(res[3], 3, 1, theta, RIGHT);
  EXPECT_FEET_EQ(res[4], 2, 4, theta, LEFT);
  EXPECT_FEET_EQ(res[5], 5, 3, theta, RIGHT);
  EXPECT_FEET_EQ(res[6], 4, 6, theta, LEFT);

  int N = res.size();
  EXPECT_TRUE(res[0].is_left);
  EXPECT_FALSE(res[1].is_left);

  EXPECT_FALSE(res[N-2].is_left);
  EXPECT_TRUE(res[N-1].is_left);
}

TEST(Footprint, lineWalking3)
{
  double theta = -M_PI / 4;
  vector<Footprint> res = walkLine(sqrt(2) * 10,
                                   sqrt(2),
                                   sqrt(2),
                                   new Footprint(1, 1, theta, true),
                                   new Footprint(-1, -1, theta, false),
                                   false);
  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);

  EXPECT_FEET_EQ(res[0], -1, -1, theta, RIGHT);
  EXPECT_FEET_EQ(res[1], 1, 1, theta, LEFT);
  EXPECT_FEET_EQ(res[2], 0, -2, theta, RIGHT);
  EXPECT_FEET_EQ(res[3], 3, -1, theta, LEFT);
  EXPECT_FEET_EQ(res[4], 2, -4, theta, RIGHT);
  EXPECT_FEET_EQ(res[5], 5, -3, theta, LEFT);
  EXPECT_FEET_EQ(res[6], 4, -6, theta, RIGHT);

  EXPECT_FALSE(res[0].is_left);
  EXPECT_TRUE(res[1].is_left);

  int N = res.size();
  EXPECT_TRUE(res[N-2].is_left);
  EXPECT_FALSE(res[N-1].is_left);
}

TEST(Footprint, circleWalking)
{
  double radius = 10;
  double distance = 10*pi;
  double width = 0.1;
  double max_step_length = ((radius+width+0.0)*pi/10.0);
  double max_step_angle = pi;
  Footprint* init_left = new Footprint(0, 0.1, 0, true);
  Footprint* init_right = new Footprint(0, -0.1, 0, false);
  bool left_is_stance_foot = false;


  vector<Footprint> res = walkCircle(
    radius,
    distance, // Walk half the circle
    width,
    max_step_length,
    max_step_angle, // We don't care about it for now
    init_left,
    init_right,
    left_is_stance_foot // We start moving the left foot
    );

  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);
  int N = res.size();

  EXPECT_EQ(N, 12);
  ASSERT_GE(N, 12);
  EXPECT_FEET_EQ(res[0]  , 0        , -width   , 0    , RIGHT);
  EXPECT_FEET_EQ(res[5]  , 10-width , 10       , pi/2 , LEFT);
  EXPECT_FEET_EQ(res[10] , 0        , 20+width , pi   , RIGHT);
  EXPECT_FEET_EQ(res[11] , 0        , 20-width , pi   , LEFT);


}



TEST(Footprint, circleWalking2)
{
  double width;
  TestHelper th(-10, //radius
                10*pi, //distance
                width = 0.1, //width
                10000, // max_step_length
                pi/10+1e-14, //max_step_angle
                new Footprint(0, 0.1, 0, true), // init_left
                NULL, // init_right
                true // left_is_stance_foot
    );
  th.checkAllProperties();
  vector<Footprint> res(th.res);

  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);
  int N = res.size();

  EXPECT_EQ(N, 12);
  ASSERT_GE(N, 12);
  EXPECT_FEET_EQ(res[0]  , 0        , width     , 0     , LEFT);
  EXPECT_FEET_EQ(res[5]  , 10-width , -10       , -pi/2 , RIGHT);
  EXPECT_FEET_EQ(res[10] , 0        , -20-width , -pi   , LEFT);
  EXPECT_FEET_EQ(res[11] , 0        , -20+width , -pi   , RIGHT);

}

TEST(Footprint, circleWalking3)
{
  TestHelper th(-10, //radius
                10*pi, //distance
                0.1, //width
                10000, // max_step_length
                pi/10+1e-14, //max_step_angle
                new Footprint(0, -10000.1, 0, true), // init_left
                NULL, // init_right
                true // left_is_stance_foot
    );
  th.checkAllProperties();
  /* vector<Footprint> res(th.res); */
}

TEST(Footprint, circleWalking4)
{
  TestHelper th(10, //radius
                10*pi/2, //distance
                0.3, //width
                0.5, // max_step_length
                pi/4, //max_step_angle
                new Footprint(-10, 10-0.3, pi, true), // init_left
                NULL, // init_right
                true // left_is_stance_foot
    );
  th.checkAllProperties();
  vector<Footprint> res(th.res);
  int N = res.size();

  bool last_is_left = N&1; // if N is odd
  if(last_is_left) {
    EXPECT_FEET_EQ(res[N-1] , -20+0.3 , 0 , -pi/2  , LEFT);
    EXPECT_FEET_EQ(res[N-2] , -20-0.3 , 0 , -pi/2  , RIGHT);
  }
  else {
    EXPECT_FEET_EQ(res[N-1] , -20-0.3 , 0 , -pi/2   , RIGHT);
    EXPECT_FEET_EQ(res[N-2] , -20+0.3 , 0 , -pi/2   , LEFT);
  }

}

// Local Variables:
// c-basic-offset: 2
// End:
