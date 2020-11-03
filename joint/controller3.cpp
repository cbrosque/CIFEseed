// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda2.urdf";

#define A_SIDE_BASE_NAV       1
#define A_SIDE_BASE_NAV2      2
#define A_SIDE_BASE_NAV3      3
#define A_SIDE_BASE_NAV4      4



int state = A_SIDE_BASE_NAV;
//int elev_counter = 0; // Counter to check whether arm is ascending or descending to point parallel to bottom of beam
//int pull_counter = 0; // counter to check if drill is going into our out of hole
//int drop_counter = 0; // Counter to check if arm is dropping after A-side, or after B-side


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

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	/*** SET UP POSORI TASK***/
	//const string control_link = "linkTool";
	//const Vector3d control_point = Vector3d(0,0.104,0.203);
	//auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	//#ifdef USING_OTG
		//posori_task->_use_interpolation_flag = true; //maybe on false
	//#else
		//posori_task->_use_velocity_saturation_flag = true;
	//#endif

	//posori_task->_linear_saturation_velocity = 0.05; // set new slower velocity (to help visualize)

	// controller gains
	//VectorXd posori_task_torques = VectorXd::Zero(dof);
	//posori_task->_kp_pos = 200.0; // 200.0
	//posori_task->_kv_pos = 20.0; // 20.0
	//posori_task->_kp_ori = 200.0;
	//posori_task->_kv_ori = 20.0;

	// controller desired positions
	double tolerance = 0.001;
	//Vector3d x_des = Vector3d::Zero(3);
	//MatrixXd ori_des = Matrix3d::Zero();

	/*** SET UP JOINT TASK ***/
	auto joint_task = new Sai2Primitives::JointTask(robot);

	#ifdef USING_OTG
		joint_task->_use_interpolation_flag = true;
	#else
		joint_task->_use_velocity_saturation_flag = true;
	#endif
    joint_task->_saturation_velocity << M_PI/6, M_PI/6, M_PI/6; // set new slower velocity (to help visualize)


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

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		if(controller_counter % 100 == 0) // %1000
		{
			cout << "current state: " << state << "\n";
			//cout << "current position:" << posori_task->_current_position(0) << " " << posori_task->_current_position(1) << " " << posori_task->_current_position(2) << endl;
			cout << "base joint angles:" << robot->_q(0) << " " << robot->_q(1) << " " << robot->_q(2) << " " << robot->_q(3) << endl;
			cout << "arm joint angles:" << robot->_q(4) << " " << endl;
			//cout << "current speed:" << posori_task->_current_velocity(0) << " " << posori_task->_current_velocity(1) << " " << posori_task->_current_velocity(2) << endl;
			cout << endl; 
			// cout << "counter: " << controller_counter << "\n";
		}

		// state switching
		if(controller_counter % 1000 == 0)
		{	
			state = A_SIDE_BASE_NAV; 
			// Set desired task position
			q_des << initial_q;
			q_des(0) = 0;
			q_des(1) = 0;
			q_des(2) = 0;
			q_des(3) = 0;
			q_des(4) = 0;

			// Set desired orientation
			//ori_des.setIdentity();

			if ((robot->_q - q_des).norm() < tolerance){ // check if goal position reached
			joint_task->reInitializeTask();
				//posori_task->reInitializeTask();
			q_des << robot->_q; // set desired joint angles

			state = A_SIDE_BASE_NAV2; // advance to next state
			}
		}

		if(controller_counter % 3000 == 0){
			state = A_SIDE_BASE_NAV2; //
			// Set new position for opposite side of hole (i.e. add wall thickness)
			//q_des << initial_q;
			q_des(0) = 1;
			q_des(1) = 0;
			q_des(2) = 0;
			q_des(3) = 0;
			q_des(4) = 0;

		}
		
		if(controller_counter % 5000 == 0){
			state = A_SIDE_BASE_NAV3; //
			// Set new position for opposite side of hole (i.e. add wall thickness)
			//q_des << initial_q;
			q_des(0) = 1;
			q_des(1) = -2.5;
			q_des(2) = 0;
			q_des(3) = 0;
			q_des(4) = 0;

		}

		if(controller_counter % 7000 == 0){

			state = A_SIDE_BASE_NAV4; //
			// Set new position for opposite side of hole (i.e. add wall thickness)
			//q_des << initial_q;
			q_des(0) = -1;
			q_des(1) = 0;
			q_des(2) = 0;
			q_des(3) = 0;
			q_des(4) = -0.1;

		}

		if(state == A_SIDE_BASE_NAV || state == A_SIDE_BASE_NAV2 || state == A_SIDE_BASE_NAV3 || state == A_SIDE_BASE_NAV4){
			/*** PRIMARY JOINT CONTROL***/

			joint_task->_desired_position = q_des;

			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

		}
		//else{
			/*** PRIMARY POSORI CONTROL W/ JOINT CONTROL IN NULLSPACE***/
			// update controlller posiitons
			//posori_task->_desired_position = x_des;
			//posori_task->_desired_orientation = ori_des;
			//joint_task->_desired_position = q_des;

			// update task model and set hierarchy
			//N_prec.setIdentity();
			//posori_task->updateTaskModel(N_prec);
			//N_prec = posori_task->_N;
			//joint_task->updateTaskModel(N_prec);

			// compute torques
			//posori_task->computeTorques(posori_task_torques);
			//joint_task->computeTorques(joint_task_torques);

			//command_torques = posori_task_torques + joint_task_torques;
			//command_torques(0) = joint_task_torques(0);
			//command_torques(1) = joint_task_torques(1);
			//command_torques(2) = joint_task_torques(2);

		//}




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

