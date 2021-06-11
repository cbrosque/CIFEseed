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

// redis keys:
// - read:

const std::string HAPTIC_POS_KEY = "Haptic_POS";
const std::string HAPTIC_ORIENTATION_KEY = "Haptic_ORIENTATION";
const std::string HAPTIC_VELOCITY_KEY = "hAPTIC_VELOCITY";
const std::string HAPTIC_FORCE_KEY = "Haptic_FORCE";
const std::string HAPTIC_MOMENT_KEY = "Haptic_MOMENT";
const std::string HAPTIC_SWITCH_KEY = "Haptic_SWITCH";
const std::string HAPTIC_INFO_KEY = "Haptic_INFO";
const std::string EARTHQUAKE_KEY = "EARTHQUAKE";


static const double fadeInTimeMS = 20;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	// start redis client

	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	Eigen::Vector3d hapticposition;
	Eigen::Vector3d ee_mag_force;
	
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	string level;
	int level_int = 0;
	float r1;
	float r2;
	float r3;
	while (runloop) {
		// wait for next scheduled loop
		level = redis_client.get(EARTHQUAKE_KEY);
		level_int = std::stoi( level );
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		if (level_int==0){
			r1 = 0;
			r2 = 0;
			r3 = 0;
		}
		else{
			r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/level_int));
			r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/level_int));
			r3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/level_int));
		}
		

		ee_mag_force << r1,r2,r3;
		
		redis_client.getEigenMatrixDerived(HAPTIC_POS_KEY, hapticposition);
		redis_client.setEigenMatrixJSON(HAPTIC_FORCE_KEY, ee_mag_force);
		cout << level_int << endl;
		controller_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
