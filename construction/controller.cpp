/*======================================================================================
 * controller.cpp
 *
 * Two Panda robots do a bolting task.
 * One drills while the other one acts as a fixture using a wrench.
 *
 * Sept 2020
 *======================================================================================*/

/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

#include <iostream>
#include <string>

#include <signal.h>

/* --------------------------------------------------------------------------------------
	Initialize 
-------------------------------------------------------------------------------------*/
bool runloop = false;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const vector<string> robot_files = {
	"./resources/panda_arm_wrench.urdf",
	"./resources/panda_arm_drill.urdf",
};
const vector<string> robot_names = {
	"PANDA_DRILL",
	"PANDA_WRENCH",
};

const int n_robots = robot_names.size();

/* --------- redis keys ---------*/
// - read:
const string TIMESTAMP_KEY = "sai2::ConstructionBolting::timestamp";

const vector<string> JOINT_ANGLES_KEYS  = {
	"sai2::ConstructionBolting::pandaDrill::sensors::q",
	"sai2::ConstructionBolting::pandaWrench::sensors::q",
};
const vector<string> JOINT_VELOCITIES_KEYS = {
	"sai2::ConstructionBolting::pandaDrill::sensors::dq",
	"sai2::ConstructionBolting::pandaWrench::sensors::dq",
};
const vector<string> FORCE_SENSED_KEYS = {
	"sai2::ConstructionBolting::pandaDrill::sensors::f_op",
	"sai2::ConstructionBolting::pandaWrench::sensors::f_op",
};

// - write
const vector<string> TORQUES_COMMANDED_KEYS = {
	"sai2::ConstructionBolting::pandaDrill::actuators::fgc",
	"sai2::ConstructionBolting::pandaWrench::actuators::fgc",
}; 

// function to update model at a slower rate
void updateModelThread(vector<shared_ptr<Sai2Model::Sai2Model>> robots, 
		vector<shared_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks);

#define APPROACH               0
// #define ALIGN_DRILL
// #define WIGGLE_WRENCH
// #define SPIN_SCREW
#define LIFT_TRAY               1

int translation_counter = 0;

int state = APPROACH;
unsigned long long controller_counter = 0;

RedisClient redis_client;

/* =======================================================================================
   MAIN LOOP
========================================================================================== */

int main() {

	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();

	pose.translation() = Vector3d(0, -0.5, 0.0);
	pose.linear() = AngleAxisd(0.3010693, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	pose.translation() = Vector3d(-0.06, 0.57, 0.0);
	pose.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	// redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	vector<shared_ptr<Sai2Model::Sai2Model>> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(make_shared<Sai2Model::Sai2Model>(robot_files[i], false));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	// prepare task controllers
	vector<int> dof;
	vector<VectorXd> command_torques;
	vector<VectorXd> coriolis;
	vector<MatrixXd> N_prec;

	vector<shared_ptr<Sai2Primitives::JointTask>> joint_tasks;
	vector<VectorXd> joint_task_torques;
	vector<shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks;
	vector<VectorXd> posori_task_torques;

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		command_torques.push_back(VectorXd::Zero(dof[i]));
		coriolis.push_back(VectorXd::Zero(dof[i]));
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));

		// joint tasks
		joint_tasks.push_back(make_shared<Sai2Primitives::JointTask>(robots[i].get()));
		joint_task_torques.push_back(VectorXd::Zero(dof[i]));

		joint_tasks[i]->_kp = 50.0;
		joint_tasks[i]->_kv = 14.0;

		// end effector tasks
		string link_name = "link7";
		Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.2);
		posori_tasks.push_back(make_shared<Sai2Primitives::PosOriTask>(robots[i].get(), link_name, pos_in_link));
		posori_task_torques.push_back(VectorXd::Zero(dof[i]));

		posori_tasks[i]->_kp_pos = 200.0;
		posori_tasks[i]->_kv_pos = 25.0;
		posori_tasks[i]->_kp_ori = 400.0;
		posori_tasks[i]->_kv_ori = 40.0;		

		posori_tasks[i]->_use_velocity_saturation_flag = true;
		posori_tasks[i]->_linear_saturation_velocity = 0.1;
		posori_tasks[i]->_angular_saturation_velocity = 30.0/180.0;

	}

	// start update_model thread
	thread model_update_thread(updateModelThread, robots, joint_tasks, posori_tasks);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	runloop = true;

	/* --------------------------------------------------------------------------------------
		Control Loop - State Machine
	-------------------------------------------------------------------------------------*/
	while (runloop) {
		
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		// read robot state from redis and update robot model
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
			robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);

			// robots[i]->updateModel();
			robots[i]->coriolisForce(coriolis[i]);
		}

		if(state == APPROACH)
		{
			// set goal positions
			Vector3d robotDrill_desired_position_in_world = Vector3d(0.2, -0.3, 0.2);
			Matrix3d robotDrill_desired_orientation_in_world = AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
			Vector3d robotWrench_desired_position_in_world = Vector3d(0.2,  0.3, 0.2);
			Matrix3d robotWrench_desired_orientation_in_world = AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();

			posori_tasks[0]->_desired_position = robot_pose_in_world[0].linear().transpose()*(robotDrill_desired_position_in_world - robot_pose_in_world[0].translation());
			posori_tasks[0]->_desired_orientation = robot_pose_in_world[0].linear().transpose()*robotDrill_desired_orientation_in_world;
			posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(robotWrench_desired_position_in_world - robot_pose_in_world[1].translation());
			posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*robotWrench_desired_orientation_in_world;

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}

			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() 
				+ (posori_tasks[1]->_desired_position - posori_tasks[1]->_current_position).norm() < 1e-3)
			{
				state = LIFT_TRAY;
			}
		}

		else if(state == LIFT_TRAY)
		{
			Vector3d robotDrill_desired_position_in_world = Vector3d(0.2, -0.15, 0.2);
			Vector3d robotWrench_desired_position_in_world = Vector3d(0.2, 0.15, 0.2);
			posori_tasks[0]->_desired_position = robot_pose_in_world[0].linear().transpose()*(robotDrill_desired_position_in_world - robot_pose_in_world[0].translation());
			posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(robotWrench_desired_position_in_world - robot_pose_in_world[1].translation());
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}

		}

		else
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				command_torques[i].setZero(dof[i]);
			}
		}

		// send to redis
		for(int i=0 ; i<n_robots ; i++)
		{
			redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		}

		prev_time = current_time;
		controller_counter++;
	}

	for(int i=0 ; i<n_robots ; i++)
	{
		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

	model_update_thread.join();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

void updateModelThread(vector<shared_ptr<Sai2Model::Sai2Model>> robots, 
		vector<shared_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks)
{

	// prepare task controllers
	vector<int> dof;
	vector<MatrixXd> N_prec;

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));
	}

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(runloop)
	{
		timer.waitForNextLoop();
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->updateModel();
		}

		if(state == APPROACH)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}
		}

		else if(state == LIFT_TRAY)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}
		}
	}
}
