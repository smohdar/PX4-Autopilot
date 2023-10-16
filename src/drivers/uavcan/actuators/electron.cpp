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

	bool battStatusUpdated;
	orb_check(batt_sub, &battStatusUpdated);

	// bool vehPosUpdated;
	// orb_check(vehPos_sub, &vehPosUpdated);

	// bool adcReportUpdated;
	// orb_check(adcRep_sub, &adcReportUpdated);

	bool actOutputsUpdated;
	orb_check(actOut_sub, &actOutputsUpdated);

	bool vehAttUpdated;
	orb_check(vehAtt_sub, &vehAttUpdated);

	bool disSenUpdated;
	orb_check(disSen_sub, &disSenUpdated);

	//copy a local copy, can also check for any change with above boolean

	if(battStatusUpdated) {
		orb_copy(ORB_ID(battery_status), batt_sub, &battStatus);
	}
	// if(vehPosUpdated) {
 	// 	orb_copy(ORB_ID(vehicle_gps_position), vehPos_sub, &vehPos);
	// }

	// if(adcReportUpdated) {
 	// 	orb_copy(ORB_ID(adc_report), adcRep_sub, &adcReport);
	// }

	if(actOutputsUpdated) {
		orb_copy(ORB_ID(actuator_outputs), actOut_sub, &actuatorOutputs);
	}

	if(vehAttUpdated) {
		orb_copy(ORB_ID(vehicle_attitude), vehAtt_sub, &vehicleAttitude);
	}

	if(disSenUpdated) {
		orb_copy(ORB_ID(distance_sensor), disSen_sub, &distanceSensor);
	}

 	//orb_copy(ORB_ID(system_power), sysPow_sub, &sysPower);

	uavcan::equipment::Electron msg;

	//First Packet
	msg.buffer = 1;
	//in decavolts
	msg.voltage = (int)((double)battStatus.voltage_filtered_v*100.0);
	//in m/s
	// msg.speed =  (int)((double)vehPos.vel_m_s*100.0);


	//Second Packet
	// msg.buffer2 = 2;
	//scale factor for current 20.48
	// msg.currentLeft = (int)(adcReport.raw_data[1]/20.48);
	// msg.currentRight = (int)(adcReport.raw_data[7]/20.48);


	//Third Packet
	msg.buffer2 = 2;
	msg.quatW = (long)((double)vehicleAttitude.q[0]*1000000000.0);

	msg.buffer3 = 3;
	msg.quatX = (long)((double)vehicleAttitude.q[1]*1000000000.0);

	msg.buffer4 = 4;
	msg.quatY = (long)((double)vehicleAttitude.q[2]*1000000000.0);

	msg.buffer5 = 5;
	msg.quatZ = (long)((double)vehicleAttitude.q[3]*1000000000.0);


	//Fifth Packet, ntoe need to check the resolution/scale of this....
	msg.buffer6 = 6;
	msg.servo1 = (int)actuatorOutputs.output[0];
	msg.servo2 = (int)actuatorOutputs.output[1];
	msg.servo3 = (int)actuatorOutputs.output[2];


	//Fifth Packet, ntoe need to check the resolution/scale of this....
	msg.buffer7 = 7;
	msg.servo4 = (int)actuatorOutputs.output[3];
	msg.servo5 = (int)actuatorOutputs.output[4];
	msg.servo6 = (int)actuatorOutputs.output[5];

	//Sixth Packet
	// msg.buffer9 = 8;
	// msg.distanceSensor = (long)((double)distanceSensor.current_distance*1000.0);

	// Broadcast command at MAX_RATE_HZ
	(void)_uavcan_pub_electron_send.broadcast(msg);
}
