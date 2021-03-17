#include "config.hpp"

#include "src/portable/include.hpp"
using namespace portable;


#define BLOCK_FOR(time_usec, to_usec) while ((Usec() - time_usec) < to_usec); time_usec = Usec()


void setup() {
	clk::Init();
	pwr::Init();

	PRINT_INIT(config::uart_baud_rate);
	PRINT_INFO(F("\nRESET -----\n\n--- Flight controller ---\n"));
	PRINT_VERBOSE(F("SysClk: ")); PRINT_VERBOSE(F_CPU); PRINT_VERBOSE(F("\n"));

	gpio::Init();

	rf::Init();
	// while (rf::Init() != true) {
	//   gpio::Toggle(&gpio::signal);
	//   delay(50);
	// }
	// gpio::Low(&gpio::signal);

	tmr::Init();

	adc::Init();
	adc::SelectChannel(static_cast<adc::channel_t>(config::analog::first_channel));

	PRINT_VERBOSE(F("setup done\n"));
}


void loop() {
	static uint32_t usec_cycle = Usec();
	usec_cycle = Usec();

	static uint32_t cycle = 0;


	#if DEBUG_CYCLE_PERIOD == true

	#endif

	/* TODO:
	 * Maybe keep different blocks in different files.
	 * Integration test with serial output.
	 * Accel median filter.
	 */

	/* Remote control
	 * --------------
	 * input: -
	 * output: RC state
	 * cycles: * * _
	 * timing: loose
	 * priority: exclusive
	 * flight modes: 'Acro', 'Angle'
	 */
	if (!cycle % 1) { // todo
		static uint32_t remote_control_time_usec = usec_cycle;
		BLOCK_FOR(remote_control_time_usec, config::cycle_period_usec);

		TIME_REMOTE_CONTROL_BEGIN();
		if (rc::IsAvailable()) {
			uint8_t message_raw[message_size];
			rc::Read(&message_raw, message_size);

			switch (message_raw[0]) {
				case MessageType::Control:
				break;
				case MessageType::Config:
				break;
				default:
				break;
			}

		} // if (rc::IsAvailable())
		TIME_REMOTE_CONTROL_END();

		#if PRINT_REMOTE_CONTROL == true
		#endif
	}


	/* Remote control transformation
	 * -----------------------------
	 * input: RC state
	 * output: control
	 * cycles: * * _
	 * timing: loose
	 * priority: N/A
	 * flight modes: 'Acro', 'Angle'
	 */
	if (!cycle % 1) {

		#if PRINT_RC_TRANSFORMATION == true
		#endif
	}


	/* Get position
	 * ------------
	 * input: -
	 * output: position input
	 * cycles:
	 * timing:
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (!cycle % 1) {

		#if PRINT_POSITION == true
		#endif
	}


	/* Calculate position
	 * ---
	 * input: position
	 * output: roll and pitch alpha, distance to target
	 * cycles:
	 * timing:
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (!cycle % 1) {
		if (RC.x == 0 && RC.y == 0 && RC.yaw == 0) {
			target.latitude -= self.latitude;
			target.longitude -= self.longitude;
			
			target.distance = sqrt(sqr(target.latitude) + sqr(target.longitude));
			target.heading = atan2(target.longitude, target.latitude) - self.heading;

			// pitch_coefficient = target.longitude / mod(target.latitude);
			// roll_coefficient = target.latitude / mod(target.longitude);
			// pitch_alpha = pitch_coefficient / (pitch_coefficient + roll_coefficient);
			// roll_alpha = roll_coefficient / (pitch_coefficient + roll_coefficient);

		}

		#if PRINT_CALCULATE_POSITION == true
		#endif
	}


	/* Position controller
	 * ---
	 * input: RC state, GPS coordinates
	 * output: attitude setpoint
	 * cycles:
	 * timing:
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (!cycle % 1) {

		#if PRINT_POSITION_CONTROLLER == true
		#endif
	}


	/* IMU
	 * ---
	 * input: -
	 * output: acceleration, angular velocity
	 * cycles: * * *
	 * timing: exact
	 * priority: N/A
	 * flight modes: 'Acro', 'Angle'
	 */
	if (!cycle % 1) {
		static uint32_t imu_time_usec = usec_cycle;
		BLOCK_FOR(imu_time_usec, config::cycle_period_usec);

		imu::ReadAccelGyro(
			&accel_x, &accel_y, &accel_z,
			&gyro_x, &gyro_y, &gyro_z
		);

		#if PRINT_IMU == true
		#endif
	}


	/* Sensor fusion
	 * -------------
	 * input: acceleration, angular velocity
	 * output: attitude
	 * cycles: * * *
	 * timing: exact
	 * priority: N/A
	 * flight modes: 'Acro', 'Angle'
	 */
	if (!cycle % 1) {
		static uint32_t sensor_fusion_time_usec = usec_cycle;
		BLOCK_FOR(sensor_fusion_time_usec, config::cycle_period_usec);

		fusion::fuse(
			accel_x, accel_y, accel_z,
			gyro_x, gyro_y, gyro_z
		);
		attitude.pitch = fusion::GetPitch();
		attitude.roll = fusion::GetRoll();

		#if PRINT_SENSOR_FUSION == true
		#endif
	}


	/* Attitude controllers
	 * -----------------
	 * input: RC state, attitude
	 * output: angular velocity setpoint
	 * cycles: _ _ *
	 * timing: exact
	 * priority: exclusive
	 * flight modes: 'Angle'
	 *
	 * These get skipped in acro mode.
	 */
	if (!cycle % 3) {
	// enable next task (process new config / read battery)

		static uint32_t controller_attitude_time_usec = usec_cycle;
		BLOCK_FOR(controller_attitude_time_usec, config::cycle_period_usec);

		controller::outer::pitch::Compute();
		controller::outer::roll::Compute();

		#if PRINT_CONTROLLER_OUTER == true
		#endif
	}


	/* Angular velocity controllers
	 * -----------------
	 * input: angular velocity setpoint, angular velocity input
	 * output: motor ratio
	 * cycles: * * *
	 * timing: exact
	 * priority: N/A
	 * flight modes: 'Acro', 'Angle'
	 */
	if (!cycle % 1) {
		static uint32_t controller_angular_velocity_time_usec = usec_cycle;
		BLOCK_FOR(controller_angular_velocity_time_usec, config::cycle_period_usec);

		controller::inner::yaw::Compute();
		controller::inner::pitch::Compute();
		controller::inner::row::Compute();

		#if PRINT_CONTROLLER_INNER == true
		#endif
	}


	/* Vertical position controller
	 * -----------------
	 * input: vertical position setpoint, vertical position input
	 * output: vertical speed setpoint
	 * cycles: 
	 * timing: exact
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (altitude_control == true) {
		if (!cycle % 1) {
			static uint32_t controller_vertical_position_time_usec = usec_cycle;
			BLOCK_FOR(controller_vertical_position_time_usec, config::cycle_period_usec);

			controller::vert_pos::Compute();

			#if PRINT_CONTROLLER_VERTICAL_POSITION == true
			#endif
		}
	}


	/* Vertical velocity controller
	 * -----------------
	 * input: vertical velocity setpoint, vertical velocity input
	 * output: vertical acceleration setpoint
	 * cycles: 
	 * timing: exact
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (altitude_control == true) {
		if (!cycle % 1) {
			static uint32_t controller_vertical_velocity_time_usec = usec_cycle;
			BLOCK_FOR(controller_vertical_velocity_time_usec, config::cycle_period_usec);

			controller::vert_velocity::Compute();

			#if PRINT_CONTROLLER_VERTICAL_VELOCITY == true
			#endif
		}
	}


	/* Vertical acceleration controller
	 * -----------------
	 * input: vertical acceleration setpoint, vertical acceleration input
	 * output: throttle
	 * cycles: 
	 * timing: exact
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (altitude_control == true) {
		if (!cycle % 1) {
			static uint32_t controller_vertical_acceleration_time_usec = usec_cycle;
			BLOCK_FOR(controller_vertical_acceleration_time_usec, config::cycle_period_usec);

			controller::vert_acceleration::Compute();

			#if PRINT_CONTROLLER_VERTICAL_ACCELERATION == true
			#endif
		}
	}


	/* Throttle compensation
	 * ---------------------
	 * input: motor ratio
	 * output: throttle
	 * cycles: * * *
	 * timing: loose
	 * priority: N/A
	 * flight modes: 'Angle'
	 */
	if (!cycle % 1) {
		// high roll and pitch increase the throttle

		#if PRINT_THROTTLE_COMPENSATION == true
		#endif
	}


	/* Motor mix
	 * ---------
	 * input: motor ratio
	 * output: motor control
	 * cycles: * * *
	 * timing: loose
	 * priority: N/A
	 * flight modes: 'Acro', 'Angle'
	 */
	if (!cycle % 1) {
		// static uint32_t motor_mix_time_usec = usec_cycle;
		// BLOCK_FOR(motor_mix_time_usec, config::cycle_period_usec);

		MotorSignal_t top_left = 0;
		MotorSignal_t top_lright = 0;
		MotorSignal_t bottom_left = 0;
		MotorSignal_t bottom_right = 0;

		// shift motor speed when clipping
		// this should probably be portable to allow different multirotors


		// 'EscCalib' and 'Direct' modes
		// 'EscCalib'

		// 'Direct'
		// rc.throttle: 0..1023
		// rc.roll: -511..512
		// rc.pitch: -511..512
		// rc.yaw: -511..512
		top_left = rc.throttle
			+ rc.pitch
			+ rc.roll
			- rc.yaw;

		top_right = rc.throttle
			+ rc.pitch
			- rc.roll
			+ rc.yaw;

		bottom_left = rc.throttle
			- rc.pitch
			+ rc.roll
			+ rc.yaw;

		bottom_right = rc.throttle
			- rc.pitch
			- rc.roll
			- rc.yaw;


		// 'Acro', 'Horizon' and 'Angle' modes
		top_left = throttle
		  + controller::inner::pitch::Output()
		  + controller::inner::roll::Output()
		  - controller::inner::yaw::Output();

		top_right = throttle
		  + controller::inner::pitch::Output()
		  - controller::inner::roll::Output()
		  + controller::inner::yaw::Output();

		bottom_left = throttle
		  - controller::inner::pitch::Output()
		  + controller::inner::roll::Output()
		  + controller::inner::yaw::Output();

		bottom_right = throttle
		  - controller::inner::pitch::Output()
		  - controller::inner::roll::Output()
		  - controller::inner::yaw::Output();

		motor::top_left::Set(top_left);
		motor::top_right::Set(top_right);
		motor::bottom_left::Set(bottom_left);
		motor::bottom_right::Set(bottom_right);


		#if PRINT_MOTOR_MIX == true
		#endif
	}


	/* Black box
	 * ---------
	 * input: state
	 * output: -
	 * cycles: * * _
	 * timing: loose
	 * priority: low
	 * flight modes: 'Acro', 'Angle'
	 */
	// Use a RAM buffer to allow this to run as low priority

	#if PRINT_BLACK_BOX == true
	#endif


	/* Process config
	 * --------------
	 * input: RC state
	 * output: -
	 * cycles: * * _
	 * timing: loose
	 * priority: low
	 * flight modes: 'Acro', 'Angle'
	 */

	#if PRINT_PROCESS_CONFIG == true
	#endif


	/* Update indication
	 * -----------------
	 * input: state
	 * output: indication state
	 * cycles: * * _
	 * timing: loose
	 * priority: low
	 * flight modes: 'Acro', 'Angle'
	 */

	#if PRINT_UPDATE_INDICATION == true
	#endif


	/* Read battery
	 * ------------
	 * input: -
	 * output: battery level
	 * cycles: * * _
	 * timing: loose
	 * priority: low
	 * flight modes: 'Acro', 'Angle'
	 */

	#if PRINT_READ_BATTERY == true
	#endif


}
