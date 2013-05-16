#include "gait-timer.h"

using namespace std;
using namespace Eigen;


///////////////////////////////////////////////////////////////
// some prototypes for private helper functions

// generates a stepping pattern
std::vector<step_t> steps(int, bool);

// compute step times from step pattern
step_traj_t step_times(std::vector<step_t>, GaitTimer timer);



int main() {

	//////////////////////////////////////////////////////////////////////
	// test gait timer
	GaitTimer timer;
	timer.startup_time = 1.0; // hold ZMP at center for 1s at start
	timer.shutdown_time = 1.0; // hold ZMP at center for 1s at end
	timer.single_support_time = 0.70; // .70s of single support each step
	timer.double_support_time = 0.05; // .05s of double support each step

	//timer.set_duty_single(0.55, 0.70);

	std::cerr << timer << std::endl;

	int n_steps = 8;

	step_traj_t step_traj = step_times(steps(n_steps, true), timer);

	//////////////////////////////////////////////////////////////////////
	// print out, pipe to matlab/timing.txt and run matlab/zmp_plot.m
	size_t eric_ticks = step_traj.stance.size();
	EIGEN_V_VEC2D eric_zmpref = step_traj.zmpref;
	std::vector<stance_t> eric_stance = step_traj.stance;

	std::cout << "i x y stance" << std::endl;
	for(size_t i=0; i < eric_ticks; i++) {
		using namespace std;
		using namespace Eigen;

		Vector2d zmp = eric_zmpref[i];
		stance_t stance = eric_stance[i];

		cout << i << " "
				<< zmp(0) << " "
				<< zmp(1) << " "
				<< stance << endl;
	}
}



///////////////////////////////////////////////////////////////
// implementations of private helper functions


const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};


vector<step_t> steps(int num_steps, bool firstleft) {
	vector<step_t> step_seq;
	step_seq.reserve(num_steps + 2);
	step_t step;

	double sway = 0.085;
	double forward = 0.15;

	int side = 1;
	stance_t stance = DOUBLE_LEFT; // Always start with left foot; w/e

	// Start step in double stance
	// pos will determine zmp in this step
	step.pos << 0,0;  // x, y: forward, side
	step.support = stance;
	step_seq.push_back(step);
	for(int i=0; i < num_steps; i++) {
		stance = next_stance_table[stance]; // SINGLE type stance
		if(i == 0) {
			// First step is a half step
			step.pos(0) += forward / 2;
			step.pos(1) = side * sway;
		} else if(i < num_steps-1) {
			// Middle steps are full steps
			step.pos(0) += forward;
			step.pos(1) = side * sway;
		} else {
			// Last step is also a half step
			step.pos(0) += forward / 2;
			step.pos(1) = side * sway;
		}
		step.support = stance;
		step_seq.push_back(step);
		// update vars to next step
		side *= -1;
		stance = next_stance_table[stance]; // DOUBLE type stance
	}
	// End step in double stance
	step.pos(1) = 0; // set zmp to center
	step.support = stance; // Should be DOUBLE type
	step_seq.push_back(step);

	return step_seq;
}

step_traj_t step_times(vector<step_t> step_seq, GaitTimer timer) {
	EIGEN_V_VEC2D zmpref; // b/c cannot dynamically resize matrixxd
	vector<stance_t> stance;

	int n_steps = step_seq.size(); // number of real foot steps

	assert(n_steps >= 0); // first and last step are stationary steps

	size_t curr_ticks = 0;
	size_t step_ticks;
	size_t double_support_ticks = 0;
	size_t single_support_ticks = 0;
	stance_t s = DOUBLE_LEFT;
	Vector2d zmp;
	for(int i=0; i < n_steps; i++) {
		s = step_seq[i].support;
		// Always start step in double support phase
		//
		if(s == SINGLE_LEFT) {
			s = DOUBLE_LEFT;
		}
		if(s == SINGLE_RIGHT) {
			s = DOUBLE_RIGHT;
		}
		zmp = step_seq[i].pos;

		if(i == 0) {
			// hold starting position
			double_support_ticks = timer.compute_startup();
			single_support_ticks = 0;
		} else if(i < n_steps - 1) {
			// real foot steps
			double_support_ticks = timer.compute_double(0);
			single_support_ticks = timer.compute_single(0,0,0);
		} else {
			// hold ending position
			double_support_ticks = timer.compute_shutdown();
			single_support_ticks = 0;
		}
		step_ticks = double_support_ticks + single_support_ticks;

		// reserve space in arrays
		zmpref.reserve(curr_ticks + step_ticks);
		stance.reserve(curr_ticks + step_ticks);

		for(size_t j=0; j < double_support_ticks; j++) {
			zmpref.push_back(zmp);
			stance.push_back(s);
		}
		s = next_stance_table[s];
		curr_ticks += double_support_ticks;
		for(size_t j=0; j < single_support_ticks; j++) {
			zmpref.push_back(zmp);
			stance.push_back(s);
		}
		s = next_stance_table[s];
		curr_ticks += single_support_ticks;
	}

	step_traj_t traj;
	traj.zmpref = zmpref;
	traj.stance = stance;

	return traj;
}


