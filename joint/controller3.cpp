// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

// added force sensor
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda2.urdf";

// State Machine Definition
#define INITIAL_POS				1 // Starting position
#define MOVING 						2	// Moving with nozzle up (joint space)
#define POSITION_HOLD			3	// Pause for 100 controller loop cycles
#define NOZZLE_DOWN				4	// Move nozzle down until force felt exceeds threshold (task space)
#define POURING_RIGHT			5	// Track the groove and move right w/nozzle down until force felt changes (task space?) -- or just move to next spot
#define NOZZLE_UP					6	// Move nozzle up back to initial position

int state = INITIAL_POS;


// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::robot::mmp_panda::actuators::fgc";

	// added force sensor
	const std::string EE_FORCE_KEY = "cs225a::sensor::force";
	const std::string EE_MOMENT_KEY = "cs225a::sensor::moment";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// load force sensor
	Eigen::Vector3d sensed_force;
	Eigen::Vector3d sensed_moment;
	sensed_force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
	sensed_moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Set up positions for each task
	MatrixXd positions(14, 4);
	positions << 1.8, 2.42, 0.0, 0.0, // pos1a - down
							 -0.6, 2.42, 0.0, 0.0, // pos1b - up
							 -0.6, 3.7, 0.0, 0.0, // pos2a - down
							 -0.6, 2.42, 0.0, 0.0, // pos2b - up
							 -5.5, 2.42, 0.0, 0.0, // pos3a - down
							 -0.6, 2.42, 0.0, 0.0, // pos3b - up
							 -0.6, 2.42, 0.0, 0.0, // pos4a - down
							 -0.6, -2.6, 0.0, 0.0, // pos4b - up
							 1.8, -2.6, 0.0, 0.0, // pos5a
							 -0.6,-2.6, 0.0, 0.0, // pos5b
							 -0.6, -3.7, 0.0, 0.0, // pos6a
							 -0.6, -2.6, 0.0, 0.0, // pos6b
							 -5.5, -2.6, 0.0, 0.0, // pos7a
							 -0.6, -2.6, 0.0, 0.0; // pos7b


	cout << "positions = " << positions << "\n";
	cout << "positions.row(0) = " << positions.row(0) << "\n";
	// VectorXd pos1a = VectorXd::Zero(dof);
	// VectorXd pos1b = VectorXd::Zero(dof);
	// pos1a << 1.5, 1.5, 0.0, 0.0;
	// pos1b << 1.5, 1.0, 0.0, 0.0;

	/*** SET UP POSORI TASK***/
	const string control_link = "linkTool";
	const Vector3d control_point = Vector3d(0,0.104,0.203);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	#ifdef USING_OTG
		posori_task->_use_interpolation_flag = true; //maybe on false
	#else
		posori_task->_use_velocity_saturation_flag = true;
	#endif

	posori_task->_linear_saturation_velocity = 0.05; // set new slower velocity (to help visualize)

	// controller gains
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0; // 200.0
	posori_task->_kv_pos = 20.0; // 20.0
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// controller desired positions
	double tolerance = 0.001;
	Vector3d x_des = Vector3d::Zero(3);
	MatrixXd ori_des = Matrix3d::Zero();

	/*** SET UP JOINT TASK ***/
	auto joint_task = new Sai2Primitives::JointTask(robot);

	#ifdef USING_OTG
		joint_task->_use_interpolation_flag = true;
	#else
		joint_task->_use_velocity_saturation_flag = true;
	#endif
    joint_task->_saturation_velocity << M_PI/12, M_PI/12, M_PI/12; //M_PI/6, M_PI/6, M_PI/6; // set new slower velocity (to help visualize)


	// controller gains
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	// controller desired angles
	VectorXd q_des = VectorXd::Zero(dof);

	/*** BEGIN LOOP ***/
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	double lastStopped = controller_counter; // Update this when you stop
	double pauseCounter = 0;
	double pc = 0;
	int nozzle_pos = 0; // 0 is up, 1 is down

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		// Every 100 cycles, print the angles and current state
		if(controller_counter % 1000 == 0) // %1000
		{
			cout << "current state: " << state << "\n";
			cout << "current position:" << posori_task->_current_position(0) << " " << posori_task->_current_position(1) << " " << posori_task->_current_position(2) << endl;
			cout << "base joint angles:" << robot->_q(0) << " " << robot->_q(1) << " " << robot->_q(2) << " " << robot->_q(3) << endl;
			cout << "arm joint angles:" << robot->_q(4) << " " << endl;
			cout << "current speed:" << posori_task->_current_velocity(0) << " " << posori_task->_current_velocity(1) << " " << posori_task->_current_velocity(2) << endl;
			cout << endl;
			cout << "counter: " << controller_counter << "\n";
		}

		// Use sensed force to switch controllers
		sensed_force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
		// cout << abs(sensed_force(2)) << "\n";
		if (abs(sensed_force(0)) > 0.0 || abs(sensed_force(1)) > 0.0 || abs(sensed_force(2)) > 0.0) {
			cout << "sensed force = " << sensed_force << "\n";
		}

		if (state == INITIAL_POS) {
			joint_task->reInitializeTask();
			// q_des << pos1a;
			VectorXd newPos = positions.row(pc);
			q_des << newPos;
			cout << "qdes = " << q_des << "\n";
			// cout << "sensed force = " << sensed_force << "\n";
			state = MOVING;
		}

		else if (state == MOVING) {
			// cout << "Made it to moving state\n";
			if((robot->_q - q_des).norm() < tolerance){ // check if goal position reached
				// cout << "Made it to pos1a\n";
				joint_task->reInitializeTask();

				// After reaching goal, switch nozzle position
				if (nozzle_pos == 1) {
					// Set to nozzle up
					q_des(3) = 0.0;
					state = NOZZLE_UP;
					nozzle_pos = 0;
				} else {
					// set to nozzle down
					q_des(3) = -0.02;
					state = NOZZLE_DOWN;
					nozzle_pos = 1;
				}
			}
		}

		else if (state == NOZZLE_DOWN) {
			// When the nozzle is all the way down, go to POSITION_HOLD
			if((robot->_q - q_des).norm() < tolerance && nozzle_pos == 1){
				joint_task->reInitializeTask();
				pc++;
				state = POSITION_HOLD;
			}
		}

		else if (state == NOZZLE_UP) {
			// When the nozzle is all the way up, go to POSITION_HOLD
			if((robot->_q - q_des).norm() < tolerance && nozzle_pos == 0){
				joint_task->reInitializeTask();
				pc++;
				state = POSITION_HOLD;
			}
		}

		else if (state == POSITION_HOLD) {
			// cout << "Made it to position hold\n";
			if (pc < 14) {
				joint_task->reInitializeTask();
				VectorXd newPos = positions.row(pc);
				q_des << newPos;
				// Maintain nozzle position when moving again
				if (nozzle_pos == 1) {
					q_des(3) = -0.02;
				} else {
					q_des(3) = 0.0;
				}
				cout << "qdes = " << q_des << "\n";
				state = MOVING;
			}
		}

		if (state == MOVING || state == NOZZLE_DOWN || state == NOZZLE_UP) {
			/* Primary Joint Task Control */
			/* Invoked when we want the base to move and the nozzle to maintain position. */
			/* RUTA: consider making POURING task posori instead of joint for better accuracy. */
			joint_task->_desired_position = q_des;

			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;
		} else if (state == POSITION_HOLD) {
			// Maintain all joint angles at the current position
			joint_task->_desired_position = robot->_q;

			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;
		}
		// else if (state == NOZZLE_DOWN || state == NOZZLE_UP) {
		// 	/* Primary POSORI Control */
		// 	cout << "posori control\n";
		// }

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
