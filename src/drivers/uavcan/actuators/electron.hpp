#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/Electron.hpp>
#include <uORB/topics/battery_status.h>
// #include <uORB/topics/vehicle_gps_position.h>
// #include <uORB/topics/adc_report.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/distance_sensor.h>

#include <perf/perf_counter.h>

class UavcanElectron
{
	public:
		UavcanElectron(uavcan::INode &node);
		~UavcanElectron();

		int init();

		void sendTelemetry(int output);

	private:

		struct battery_status_s battStatus;
		// struct vehicle_gps_position_s vehPos;
		// struct adc_report_s adcReport;
		struct actuator_outputs_s actuatorOutputs;
		struct vehicle_attitude_s vehicleAttitude;
		struct distance_sensor_s distanceSensor;

		int batt_sub;
		// int vehPos_sub;
		// int adcRep_sub;
		int actOut_sub;
		int vehAtt_sub;
		int disSen_sub;

		void periodic_update(const uavcan::TimerEvent &);
		typedef uavcan::MethodBinder
		<UavcanElectron *, void (UavcanElectron::*)(const uavcan::TimerEvent &)> TimerCbBinder;

		uavcan::INode 		&_node;
		uavcan::Publisher<uavcan::equipment::Electron> _uavcan_pub_electron_send;
		uavcan::TimerEventForwarder<TimerCbBinder> _timer;
};
