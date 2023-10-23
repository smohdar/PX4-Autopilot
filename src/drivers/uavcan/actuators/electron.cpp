#include "electron.hpp"
#include <systemlib/err.h>

UavcanElectron::UavcanElectron(uavcan::INode &node):
	_node(node),
	_uavcan_pub_electron_send(node),
	_timer(node)
{

}

UavcanElectron::~UavcanElectron() {

}

int UavcanElectron::init() {

	/*
	 * Setup timer and call back function for periodic updates
	 */
	_timer.setCallback(TimerCbBinder(this, &UavcanElectron::periodic_update));
	return 0;
}

void UavcanElectron::sendTelemetry(int value) {

	if (!_timer.isRunning()) {
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(value));
		memset(&battStatus, 0, sizeof(battStatus));
		memset(&actuatorOutputs, 0, sizeof(actuatorOutputs));
		memset(&vehicleAttitude, 0, sizeof(vehicleAttitude));
		memset(&distanceSensor, 0, sizeof(distanceSensor));
	}
}

void UavcanElectron::periodic_update(const uavcan::TimerEvent &) {

	static int started = 0;

	if(started == 0) {
		batt_sub = orb_subscribe(ORB_ID(battery_status));
		// vehPos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
		// adcRep_sub = orb_subscribe(ORB_ID(adc_report));
		actOut_sub = orb_subscribe(ORB_ID(actuator_outputs));
		vehAtt_sub = orb_subscribe(ORB_ID(vehicle_attitude));
		disSen_sub = orb_subscribe(ORB_ID(distance_sensor));

		started++;
	}

	//check for any new update
	bool actOutputsUpdated;
	orb_check(actOut_sub, &actOutputsUpdated);

	//copy a local copy, can also check for any change with above boolean
	if(actOutputsUpdated) {
		orb_copy(ORB_ID(actuator_outputs), actOut_sub, &actuatorOutputs);
	}

	//Remap motor & servo to 0-255
	int LM = (((int)actuatorOutputs.output[0] == 1900) ? 255 : (((int)actuatorOutputs.output[0] - 1100) / 25) * 8);
	int RM = (((int)actuatorOutputs.output[1] == 1900) ? 255 : (((int)actuatorOutputs.output[1] - 1100) / 25) * 8);
	int FLS = (((int)actuatorOutputs.output[4] == 2000) ? 255 : (((int)actuatorOutputs.output[4] - 1000) / 125) * 32);
	int FRS = (((int)actuatorOutputs.output[5] == 2000) ? 255 : (((int)actuatorOutputs.output[5] - 1000) / 125) * 32);
	int RLS = (((int)actuatorOutputs.output[6] == 2000) ? 255 : (((int)actuatorOutputs.output[6] - 1000) / 125) * 32);
	int RRS = (((int)actuatorOutputs.output[7] == 2000) ? 255 : (((int)actuatorOutputs.output[7] - 1000) / 125) * 32);

	uavcan::equipment::Electron msg;

	// msg.buffer = 1;
	msg.leftMotor = LM;
	msg.rightMotor = RM;

	// msg.buffer2 = 2;
	msg.FLServo = FLS;
	msg.FRServo = FRS;

	// msg.buffer3 = 3;
	msg.RLServo = RLS;
	msg.RRServo = RRS;

	// Broadcast command at MAX_RATE_HZ
	(void)_uavcan_pub_electron_send.broadcast(msg);
}
