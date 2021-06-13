#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "uiforce/UIForceWidget.h"
#include "force_sensor/ForceSensorSim.h" // references src folder in sai2-common directory
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_weld.urdf";
const string robot_file = "./resources/mmp_panda_weld.urdf";
const string obj_file = "mmp_panda_tool_no_collision.urdf";
const string robot_name = "mmp_panda";
const string camera_name = "camera_fixed";
const string control_link = "linkTool";
// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::dq";
// - read
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::robot::mmp_panda::actuators::fgc";

const std::string HAPTIC_POS_KEY = "Haptic_POS";
const std::string HAPTIC_ORIENTATION_KEY = "Haptic_ORIENTATION";
const std::string HAPTIC_VELOCITY_KEY = "hAPTIC_VELOCITY";
const std::string HAPTIC_FORCE_KEY = "Haptic_FORCE";
const std::string HAPTIC_MOMENT_KEY = "Haptic_MOMENT";
const std::string HAPTIC_SWITCH_KEY = "Haptic_SWITCH";
const std::string HAPTIC_INFO_KEY = "Haptic_INFO";
const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
const std::string ACTIVE_STATE_KEY = "active_state";
const std::string BASE_POSE_KEY = "base_pose";

// force sensor
ForceSensorSim *force_sensor;
// display widget for forces at end effector
ForceSensorDisplay *force_display;

RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool CameraZoom1 = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

chai3d::cShapeSphere* sphere0;

chai3d::cGenericHapticDevicePtr hapticDevice;
chai3d::cToolGripper *tool;
chai3d::cHapticDeviceHandler *handler;
chai3d::cVector3d hapticPos;
chai3d::cMatrix3d hapticOri;
chai3d::cVector3d hapticVel;
chai3d::cVector3d force_field;
chai3d::cVector3d moment_field;

Eigen::Vector3d hapticposition;
Eigen::Matrix3d hapticorientation;
Eigen::Vector3d hapticvelocity;
Eigen::Vector3d hapticforce;
Eigen::Vector3d hapticmoment;
Eigen::Vector3d base_pose_vec;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();
	cout << "hiiiiiiii"  << endl;
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);
	cout << "hiiiiiiii"  << endl;
	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	//robot->_q(0) = -0.8;
	robot->updateModel();
	

	//Sphere
	sphere0 = new chai3d::cShapeSphere(0.05);
    graphics->_world->addChild(sphere0);

    // set position
    sphere0->setLocalPos(-3.4,2.3, 0.2);
    
    // set material color
    sphere0->m_material->setBlueCyan();

    // create haptic effect and set properties
    sphere0->createEffectSurface();
    
	//Haptic Devices
	handler = new chai3d::cHapticDeviceHandler();
	handler->getDevice(hapticDevice, 0);
	cout << "hapticdevice" << hapticDevice << endl;
	hapticDevice->open();
	hapticDevice->calibrate();


	tool = new chai3d::cToolGripper(graphics->_world);
	graphics->_world->addChild(tool);
	tool->setHapticDevice(hapticDevice);
	tool->setWorkspaceRadius(1.0);
	double toolRadius = 0.05;
	tool->setRadius(toolRadius);
	tool->setShowContactPoints(false, false);
	tool->enableDynamicObjects(true);
	tool->setWaitForSmallForce(true);
	tool->start();
	tool->setShowFrame(false);
	tool->setFrameSize(0.1);
	
	// hapticDevice->enableForces(true);
	chai3d::cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
	double maxLinearStiffness = hapticDeviceInfo.m_maxLinearStiffness;
	double maxLinearDamping = hapticDeviceInfo.m_maxLinearDamping;
	Eigen::Vector2d stiffness_AND_damping;
	stiffness_AND_damping(0) = maxLinearStiffness;
	stiffness_AND_damping(1) = maxLinearDamping;
	redis_client.setEigenMatrixDerived(HAPTIC_INFO_KEY, stiffness_AND_damping);
	hapticDevice->setEnableGripperUserSwitch(true);

	force_sensor = new ForceSensorSim(robot_name, control_link, Eigen::Affine3d::Identity(), robot);
	force_display = new ForceSensorDisplay(force_sensor, graphics);
	force_sensor->removeSpike(3);
	force_sensor->enableFilter(0.1);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0.0);
	sim->setCoeffFrictionStatic(0.8);
	sim->setCoeffFrictionDynamic(0.8);

	// read joint positions, velocities, update model
	sim->getJointPositions("mmp_panda", robot->_q);
	sim->getJointVelocities("mmp_panda", robot->_dq);
	robot->updateKinematics();

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// init click force widget
	auto ui_force_widget = new UIForceWidget("mmp_panda", robot, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// initialize glew
	glewInitialize();

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, ui_force_widget);

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics("mmp_panda", robot);
		graphics->render(camera_name, width, height);
		force_display->update();
		
		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		//cout << "camera_pos" <<  camera_pos(0) << "," << camera_pos(1)  << "," << camera_pos(2)  << "\n"; 
		//cout << "camera_lookat" << camera_lookat(0) << "," << camera_lookat(1) << "," << camera_lookat(2) << "\n"; 
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		if(CameraZoom1)
		{
			camera_pos = Vector3d(1.40738,0.857643,0.546852);
			camera_lookat = Vector3d(0.525109,1.32772,0.521878);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	hapticforce = VectorXd::Zero(3);
	hapticmoment = VectorXd::Zero(3);
	
	redis_client.setEigenMatrixDerived(HAPTIC_FORCE_KEY, hapticforce);
	redis_client.setEigenMatrixDerived(HAPTIC_MOMENT_KEY, hapticmoment);
	redis_client.setEigenMatrixDerived(BASE_POSE_KEY, Vector3d(0,0,0));

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double time_slowdown_factor = 2.0;
	//double last_time = timer.elapsedTime(); //secs
	double start_time = timer.elapsedTime() / time_slowdown_factor;
	bool fTimerDidSleep = true;
	double last_time = start_time;

	// init variables
	VectorXd g(dof);

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	// sensed forces and moments from sensor
	Eigen::Vector3d sensed_force;
	Eigen::Vector3d sensed_moment;
	Eigen::Vector3d forceInEEFrame = Eigen::Vector3d::Zero(3);
	Eigen::Vector3d momentInEEFrame = Eigen::Vector3d::Zero(3);

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// ********************* HAPTIC DEVICE ***********************
		hapticDevice->getPosition(hapticPos);
		hapticposition = hapticPos.eigen(); //save master and slave origin as present positions respectively
		redis_client.setEigenMatrixDerived(HAPTIC_POS_KEY, hapticposition);

		hapticDevice->getRotation(hapticOri);
		hapticorientation = hapticOri.eigen();
		redis_client.setEigenMatrixDerived(HAPTIC_ORIENTATION_KEY, hapticorientation);

		hapticDevice->getLinearVelocity(hapticVel);
		hapticvelocity = hapticVel.eigen();
		redis_client.setEigenMatrixDerived(HAPTIC_VELOCITY_KEY, hapticvelocity);

		redis_client.getEigenMatrixDerived(HAPTIC_FORCE_KEY, hapticforce);
		force_field = chai3d::cVector3d(hapticforce);
		redis_client.getEigenMatrixDerived(HAPTIC_MOMENT_KEY, hapticmoment);
		moment_field = chai3d::cVector3d(hapticmoment);

		redis_client.getEigenMatrixDerived(BASE_POSE_KEY, base_pose_vec);
		//cout << "pose" << -3.4+base_pose_vec(0) << "," << 2.3+base_pose_vec(1) << "," << 0.2+base_pose_vec(2) << endl;
		sphere0->setLocalPos(-3.4+base_pose_vec(0),2.3+base_pose_vec(1), 0.2+base_pose_vec(2));
		//sphere0->setLocalPos(-2.75, 2.402, 0.02);
		string activeStateString = redis_client.get(ACTIVE_STATE_KEY);
		if (activeStateString == "HAPTICS")
		{
			sphere0->m_material->setGreenLawn();
		}
		else
		{
			sphere0->m_material->setBlueCyan();
		}

		bool button = tool->getUserSwitch(0);
		//cout << "button" << button << endl;
		if (button)
		{
			redis_client.setCommandIs(HAPTIC_SWITCH_KEY, "true");
			tool->updateFromDevice();
		}

		hapticDevice->setForceAndTorqueAndGripperForce(force_field, moment_field, 0);

		// get gravity torques
		robot->gravityVector(g);

		// read arm torques from redis and apply to simulated robot
		command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);

		ui_force_widget->getUIForce(ui_force);
		ui_force_widget->getUIJointTorques(ui_force_command_torques);

		if (fRobotLinkSelect)
			sim->setJointTorques("mmp_panda", command_torques + ui_force_command_torques + g);
		else
			sim->setJointTorques("mmp_panda", command_torques + g);

		// integrate forward
		double curr_time = timer.elapsedTime() / time_slowdown_factor;
		double loop_dt = curr_time - last_time;
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions("mmp_panda", robot->_q);
		sim->getJointVelocities("mmp_panda", robot->_dq);
		robot->updateModel();
		
		// update force sensor readings
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force); // refer to ForceSensorSim.h in sai2-common/src/force_sensor (can also get wrt global frame)
		force_sensor->getMomentLocalFrame(sensed_moment);
		
		Eigen::VectorXd sensed_force_and_moment_EE_Frame(6);
		sensed_force_and_moment_EE_Frame << sensed_force, sensed_moment;
		//cout << "force" << sensed_force(0) << "," << sensed_force(1) << "," << sensed_force(2) << endl;
		//cout << "moment" << sensed_moment(0) << "," << sensed_moment(1) << "," << sensed_moment(2) << endl;

		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, sensed_force_and_moment_EE_Frame);
		
		// write new robot state to redis

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);

		//update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime() / time_slowdown_factor;
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		case GLFW_KEY_M:
			CameraZoom1 = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
