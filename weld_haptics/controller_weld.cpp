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

const string robot_file = "./resources/mmp_panda_weld.urdf";

//int elev_counter = 0; // Counter to check whether arm is ascending or descending to point parallel to bottom of beam
//int pull_counter = 0; // counter to check if drill is going into or out of hole
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

const std::string HAPTIC_POS_KEY = "Haptic_POS";
const std::string HAPTIC_ORIENTATION_KEY = "Haptic_ORIENTATION";
const std::string HAPTIC_VELOCITY_KEY = "Haptic_VELOCITY";
const std::string HAPTIC_FORCE_KEY = "Haptic_FORCE";
const std::string HAPTIC_MOMENT_KEY = "Haptic_MOMENT";
const std::string HAPTIC_SWITCH_KEY = "Haptic_SWITCH";
const std::string HAPTIC_INFO_KEY = "Haptic_INFO";
const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
const std::string EE_FORCE_MAG_FORCE_KEY = "sai2::optoforceSensor::3Dsensor::force";
const std::string ACTIVE_STATE_KEY = "active_state";
const std::string BASE_POSE_KEY = "base_pose";

static const double fadeInTimeMS = 20;

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
	const string control_link = "linkTool";
	const Vector3d control_point = Vector3d(0,0,0.104);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	#ifdef USING_OTG
		posori_task->_use_interpolation_flag = true; //maybe on false
	#else
		posori_task->_use_velocity_saturation_flag = true;
	#endif

	posori_task->_linear_saturation_velocity = 0.05; // set new slower velocity (to help visualize)

	// controller gains
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 50.0; // 200.0
	posori_task->_kv_pos = 50.0; // 20.0
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_ori = 50.0;

	// controller desired positions
	double tolerance = 0.001;
	Vector3d x_des = Vector3d::Zero(3);
	Vector3d x_des_temp = Vector3d::Zero(3);
	//MatrixXd ori_des = Matrix3d::Zero();
	Eigen::Matrix3d ori_des;
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
	joint_task->_kp = 50.0;
	joint_task->_kv = 20.0;

	// controller desired angles
	VectorXd q_des = VectorXd::Zero(dof);
	VectorXd q_hold = VectorXd::Zero(dof);
	Vector3d P_hold = VectorXd::Zero(3);
	Vector3d P_current = VectorXd::Zero(3);

	/*** BEGIN LOOP ***/
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	Eigen::Vector3d hapticposition;
	Eigen::Vector3d x_master;
	Eigen::Vector3d x_slave;
	Eigen::Matrix3d hapticorientation;
	Eigen::Vector3d hapticvelocity;
	Eigen::Vector3d hapticforce;
	Eigen::Vector3d hapticmoment;
	Eigen::Vector3d base_pose_vec;
	string button;

	Eigen::VectorXd ee_sensed_force = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_mag_force = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_moment = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_force_and_moment = Eigen::VectorXd::Zero(6); //robot
	int hapticFadeInCounter = fadeInTimeMS;

	enum states
	{
		AUTO_1,
		AUTO_2,
		B_SIDE_CORNER2,
		C_SIDE_CORNER3,
		D_SIDE_CORNER4,
		HOLD_CALIBRATION,
		HAPTICS
	};

	bool auto_state;
	states state;
	redis_client.set(ACTIVE_STATE_KEY, "AUTO_1");

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, ee_sensed_force_and_moment); // fixed simulated force sensor to also return moments
		ee_sensed_force = ee_sensed_force_and_moment.head<3>();
		ee_sensed_moment = ee_sensed_force_and_moment.tail<3>();
		

		if(controller_counter % 200 == 0) // %1000
		{
			cout << "current state: " << state << "\n";
			cout << "current position:" << posori_task->_current_position(0) << " " << posori_task->_current_position(1) << " " << posori_task->_current_position(2) << endl;
			cout << "base joint angles:" << robot->_q(0) << " " << robot->_q(1) << " " << robot->_q(2) << endl;
			cout << "arm joint angles:" << robot->_q(3) << " " << robot->_q(4) << " " << robot->_q(5) << " " <<
										robot->_q(6) << " " << robot->_q(7) << " " << robot->_q(8) << " " << robot->_q(9) << endl;
			cout << "current speed:" << posori_task->_current_velocity(0) << " " << posori_task->_current_velocity(1) << " " << posori_task->_current_velocity(2) << endl;
			cout << "haptic position:" << hapticposition(0) << " " << hapticposition(1) << " " << hapticposition(2) << endl;
			cout << "hold position:" << P_hold(0) << " " << P_hold(1) << " " << P_hold(2) << endl;
			cout << "xdes position:" << x_des(0) << " " << x_des(1) << " " << x_des(2) << endl;
			cout << "joint torque" << command_torques(0) << " " << command_torques(1) << " " << command_torques(2) << " " << command_torques(3) << " " << command_torques(4) << " " << command_torques(5) << " " << command_torques(6) << " " << command_torques(7) << " " << command_torques(8) << " " << command_torques(9) << endl;
			cout << "force" << ee_sensed_force(0) << "," << ee_sensed_force(1) << "," << ee_sensed_force(2) << endl;
			cout << "moment" << ee_sensed_moment(0) << "," << ee_sensed_moment(1) << "," << ee_sensed_moment(2) << endl;
			cout << "base hold joint angles:" << q_hold(0) << " " << q_hold(1) << " " << q_hold(2) << endl;
			cout << "arm hold joint angles:" << q_hold(3) << " " << q_hold(4) << " " << q_hold(5) << " " <<
										q_hold(6) << " " << q_hold(7) << " " << q_hold(8) << " " << q_hold(9) << endl;
			cout << endl; 
			cout << "q_des base:" << q_des(0) << " " << q_des(1) << " " << q_des(2) << endl;
			cout << "q_des joint:" << q_des(3) << " " << q_des(4) << " " << q_des(5) << " " <<
										q_des(6) << " " << q_des(7) << " " << q_des(8) << " " << q_des(9) << endl;
			cout << endl;
			// cout << "counter: " << controller_counter << "\n";
		}
		base_pose_vec = Vector3d (robot->_q(0)-initial_q(0), robot->_q(1)-initial_q(1), robot->_q(2)-initial_q(2));
		redis_client.setEigenMatrixJSON(BASE_POSE_KEY, base_pose_vec);

		string activeStateString = redis_client.get(ACTIVE_STATE_KEY);
		//cout << "Current state: " << activeState << "\r \n";

		//band-aid fix 	TODO find out how to use enum as redis key //update: not a thing
		if (activeStateString == "AUTO_1")
			state = AUTO_1;
		if (activeStateString == "HOLD_CALIBRATION")
			state = HOLD_CALIBRATION;
		if (activeStateString == "AUTO_2")
			state = AUTO_2;
		if (activeStateString == "HAPTICS")
			state = HAPTICS;

		// state switching
		switch(state)
		{
		case AUTO_1:
		{	
			auto_state = 1;
			q_des = initial_q;
			q_des[0] = 0.21;
			q_des[1] = 0.1;
			ori_des.setIdentity();
			q_hold = q_des;
			robot->rotation(ori_des, control_link);
			robot->position(P_hold, control_link, control_point);
			x_des = P_hold;
			ee_mag_force << 0,0,0;
			if ((robot->_q - q_des).norm() <= 0.001)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				state=HOLD_CALIBRATION;
				redis_client.set(ACTIVE_STATE_KEY, "HOLD_CALIBRATION");
			}
			break;

		}

		case AUTO_2:
		{	
			auto_state = 2;
			q_des = initial_q;
			q_des[0] = 2.82;
			q_des[1] = 0.1;
			ori_des.setIdentity();
			q_hold = q_des;
			robot->rotation(ori_des, control_link);
			robot->position(P_hold, control_link, control_point);
			x_des = P_hold;
			ee_mag_force << 0,0,0;
			if ((robot->_q - q_des).norm() <= 0.001)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				state=HOLD_CALIBRATION;
				redis_client.set(ACTIVE_STATE_KEY, "HOLD_CALIBRATION");
			}
			break;

		}

		case HAPTICS:
		{
			redis_client.getEigenMatrixDerived(HAPTIC_POS_KEY, hapticposition);
			redis_client.getCommandIs(HAPTIC_SWITCH_KEY, button);
			q_des = q_hold;
			x_des_temp = P_hold + Vector3d(-7 * hapticposition(0), -7 * hapticposition(1), 7 * hapticposition(2));
			x_des = x_des_temp;
			if (ee_sensed_force.norm() > 2)
			{
				ee_sensed_force = 2 * ee_sensed_force / ee_sensed_force.norm();
			}
			robot->position(P_current, control_link, control_point);
			if ((abs(P_current[0]+2.325)<=0.01 || abs(P_current[0]+1.925)<=0.01 || abs(P_current[0]-0.225)<=0.01 || abs(P_current[0]-0.625)) <= 0.01 || abs(P_current[1]+0.3)<=0.01 || abs(P_current[1]-0.1)<=0.01 || abs(P_current[2]-0.02) <= 0.01)
			{
				break;

			}
			else
			{
				ee_mag_force(0) = 0.05*(1/(P_current[0]+2.325) + 1/(P_current[0]+1.925) + 1/(P_current[0]-0.225) + 1/(P_current[0]-0.625));
				ee_mag_force(1) = 0.1*(1/(P_current[1]+0.3) + 1/(P_current[1]-0.1));
				ee_mag_force(2) = -1/(P_current[2]-0.02);
			}
			if (ee_mag_force.norm() > 2)
			{
				ee_mag_force(0) = 2 * ee_mag_force(0) / ee_mag_force.norm();
				ee_mag_force(1) = 2 * ee_mag_force(1) / ee_mag_force.norm();
				ee_mag_force(2) = 8 * ee_mag_force(2) / ee_mag_force.norm();
			}
			
			
			//redis_client.setEigenMatrixDerived(HAPTIC_FORCE_KEY, Vector3d(1 * ee_sensed_force(0), 1 * ee_sensed_force(0), 1 * ee_sensed_force(0)));
			/*if(auto_state == 1)
			{
					if(x_des_temp(2) <= 0.2 && ((x_des_temp(0) >= -2.21 && x_des_temp(0) <= -2.05 && x_des_temp(1) >= -0.29 && x_des_temp(1) <= -0.14) || x_des_temp(0) <= -2.24 || x_des_temp(0) >= -2.01 || x_des_temp(1) <= -0.29 || x_des_temp(1) >= -0.10))
					{
						//(x_des_temp(0) >= -2.23 && x_des_temp(0) <= -2.03 && x_des_temp(1) >= -0.31 && x_des_temp(1) <= -0.12) ||
						robot->position(P_current, control_link, control_point);
						x_des = P_current;
					}
					else
					{
						x_des = x_des_temp;
					}
			}
			if(auto_state == 2)
			{
					if(x_des_temp(2) <= 0.2 && ((x_des_temp(0) >= 0.31 && x_des_temp(0) <= 0.47 && x_des_temp(1) >= -0.29 && x_des_temp(1) <= -0.14) || x_des_temp(0) <= 0.28 || x_des_temp(0) >= 0.51 || x_des_temp(1) <= -0.29 || x_des_temp(1) >= -0.10))
					{
						//(x_des_temp(0) >= -2.23 && x_des_temp(0) <= -2.03 && x_des_temp(1) >= -0.31 && x_des_temp(1) <= -0.12) ||
						robot->position(P_current, control_link, control_point);
						x_des = P_current;
					}
					else
					{
						x_des = x_des_temp;
					}
			}*/
			if(hapticposition(2) >= 0.05)
			{
				if(auto_state == 1)
				{
					x_des = x_des_temp;
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					state=AUTO_2;
					redis_client.set(ACTIVE_STATE_KEY, "AUTO_2");
				}
			}
			//x_des << -2.325,-0.3,0.02;
			//x_des << -2.325,0.1,0.02;
			//x_des << -1.925,0.1,0.02;	
			//x_des << 0.225,0.1,0.02;
			//x_des << 0.625,0.1,0.02;	
			break;
		}

		case HOLD_CALIBRATION:
		{
			robot->position(P_hold, control_link, control_point);
			q_hold = robot->_q;
			ee_mag_force << 0,0,0;
			break;
		}

		}
	
		redis_client.setEigenMatrixJSON(HAPTIC_FORCE_KEY, ee_mag_force);
		redis_client.setEigenMatrixJSON(EE_FORCE_MAG_FORCE_KEY, ee_mag_force+ee_sensed_force);
		if(state == AUTO_1 || state == AUTO_2){
			/*** PRIMARY JOINT CONTROL***/

			joint_task->_desired_position = q_des;

			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

		}

		else{
			/*** PRIMARY POSORI CONTROL W/ JOINT CONTROL IN NULLSPACE***/
			// update controlller posiitons
			// q_des[2] = 0;
			// q_des[3] = -1.22173;
			// q_des[4] = -0.523599;
			// q_des[5] = -0;
			// q_des[6] = -2.96706;
			// q_des[7] = -0.523599;
			// q_des[8] = 1.5708;
			// q_des[9] = 0;

			posori_task->_desired_position = x_des;
			posori_task->_desired_orientation = ori_des;
			joint_task->_desired_position = q_des;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
			//command_torques(0) = joint_task_torques(0);
			//command_torques(1) = joint_task_torques(1);
			//command_torques(2) = joint_task_torques(2);

		}




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
