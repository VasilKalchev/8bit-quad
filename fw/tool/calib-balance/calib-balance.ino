/* Calibrate & balance
 * Calibrate IMU, ESCs range and balance motors.
 * 
 * Set mode with RC switches.
 *  - calibrate accelerometer
 *  - calibrate gyroscope
 *  - run a motor and measure RMS x, y acceleration
 *  - output acceleration
 *  - output angular velocity
 *  - set ESC range
 */

#include "config.hpp"
#include "config_rx.hpp"

#include "src/board/board.hpp"
#include <MPU6050.h>
#include "src/rx/rx.hpp"

#include <math.h>


namespace imu {

MPU6050 mpu(0x68);
} // namespace imu

enum class k_Mode : uint8_t {
	Off,
	CalibrateAccel, CalibrateGyro,
	RmsAccel,
	Accel, Gyro,
	SetEscs,
};


int16_t rc_raw[(uint8_t)cfg::rx::k_Ch::Count] = { 0 };


enum class k_Motor : uint8_t {
	None,
	TL, TR, BL, BR,
};


void setup() {
	board::Init();

	board::SetTL(127);
	board::SetTR(127);
	board::SetBL(127);
	board::SetBR(127);

	Serial.begin(cfg::baud_rate);
	Serial.print(F("---\nRESET---\ncalibrate_n_balance v1.0.0\n\n"));


	Wire.begin();
	Wire.setClock(cfg::imu::clock_speed);


	imu::mpu.initialize();
	imu::mpu.setInterruptMode(1); // active low
	imu::mpu.setInterruptDrive(1); // open-drain

	imu::mpu.setRate(cfg::imu::sample_rate_divider-1); // divide gyro sample rate by 1

	imu::mpu.setDLPFMode(cfg::imu::dlpf);

	imu::mpu.setFullScaleAccelRange((uint8_t)cfg::imu::accel_range); // 2g
	imu::mpu.setFullScaleGyroRange((uint8_t)cfg::imu::gyro_range); // 250 deg/sec

    imu::mpu.setXAccelOffset(cfg::imu::offset::accel::x);
    imu::mpu.setYAccelOffset(cfg::imu::offset::accel::y);
    imu::mpu.setZAccelOffset(cfg::imu::offset::accel::z);
    imu::mpu.setXGyroOffset(cfg::imu::offset::gyro::x);
    imu::mpu.setYGyroOffset(cfg::imu::offset::gyro::y);
    imu::mpu.setZGyroOffset(cfg::imu::offset::gyro::z);

    Serial.println("Available modes:");
    Serial.print("Off: "); Serial.println(cfg::rx::switch_state::mode::off, BIN);
    Serial.print("Calib. accel: "); Serial.println(cfg::rx::switch_state::mode::calib_accel, BIN);
    Serial.print("Calib. gyro: "); Serial.println(cfg::rx::switch_state::mode::calib_gyro, BIN);
    Serial.print("RMS accel: "); Serial.println(cfg::rx::switch_state::mode::rms_accel, BIN);
    Serial.print("Accel: "); Serial.println(cfg::rx::switch_state::mode::accel, BIN);
    Serial.print("Gyro: "); Serial.println(cfg::rx::switch_state::mode::gyro, BIN);
}

void loop() {
	rx::ReadRx(2, rc_raw, 8);
	static uint16_t switches = 0;
	static int16_t switch_arr[4] = { 0 };
	if (rc_raw[0] != 0) {
		rx::SplitChannel(rc_raw[(uint8_t)cfg::rx::k_Ch::SwAB],
			switch_arr[0], 2, switch_arr[1], 2);
		rx::SplitChannel(rc_raw[(uint8_t)cfg::rx::k_Ch::SwCD],
			switch_arr[2], 3, switch_arr[3], 2);
	    switches = rx::CombineChannels(switch_arr, 4);
	}

    #define PRINT_RC_RAW false
    #if PRINT_RC_RAW
    static uint8_t print_rc_raw_div = 0;
    if (++print_rc_raw_div > 1) {
      print_rc_raw_div = 0;

      Serial.print(F("t ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::Throttle]);
      Serial.print(F("\tp ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch]);
      Serial.print(F("\tr ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::Roll]);
      Serial.print(F("\ty ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::Yaw]);
      Serial.print(F("\tab ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::SwAB]);
      Serial.print(F("\tcd ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::SwCD]);
      Serial.print(F("\tpA ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::VrA]);
      Serial.print(F("\tpB ")); Serial.print(rc_raw[(uint8_t)cfg::rx::k_Ch::VrB]);
      Serial.print(F("\n"));
    }
    #endif

    static k_Motor motor = k_Motor::None;
    if (rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch] < 1300
    	&& rc_raw[(uint8_t)cfg::rx::k_Ch::Roll] < 1300) {
    	if (motor != k_Motor::TL) {
	    	motor = k_Motor::TL;
			Serial.println("TL");
    	}
    } else if (rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch] < 1300
    	&& rc_raw[(uint8_t)cfg::rx::k_Ch::Roll] > 1700) {
    	if (motor != k_Motor::TR) {
	    	motor = k_Motor::TR;
			Serial.println("TR");
		}
    } else if (rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch] > 1700
    	&& rc_raw[(uint8_t)cfg::rx::k_Ch::Roll] < 1300) {
    	if (motor != k_Motor::BL) {
	    	motor = k_Motor::BL;
			Serial.println("BL");
		}
    } else if (rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch] > 1700
    	&& rc_raw[(uint8_t)cfg::rx::k_Ch::Roll] > 1700) {
    	if (motor != k_Motor::BR) {
	    	motor = k_Motor::BR;
			Serial.println("BR");
		}
    }


    static uint16_t sw_last = 0;
    if (switches != sw_last) {
    	sw_last = switches;

	    // Serial.print("switches: "); Serial.println(switches, BIN);
	    // Serial.print("A "); Serial.print(switch_arr[0]);
	    // Serial.print("\tB "); Serial.print(switch_arr[1]);
	    // Serial.print("\tC "); Serial.print(switch_arr[2]);
	    // Serial.print("\tD "); Serial.println(switch_arr[3]);

	    // uint16_t m_off = switches & cfg::rx::switch_state::mode::off;
	    // uint16_t m_calib_accel = switches & cfg::rx::switch_state::mode::calib_accel;
	    // uint16_t m_calib_gyro = switches & cfg::rx::switch_state::mode::calib_gyro;
	    // uint16_t m_rms_accel = switches & cfg::rx::switch_state::mode::rms_accel;
	    // uint16_t m_accel = switches & cfg::rx::switch_state::mode::accel;
	    // uint16_t m_gyro = switches & cfg::rx::switch_state::mode::gyro;
	    // Serial.print("m_off "); Serial.println(m_off, BIN);
	    // Serial.print("m_calib_accel "); Serial.println(m_calib_accel, BIN);
	    // Serial.print("m_calib_gyro "); Serial.println(m_calib_gyro, BIN);
	    // Serial.print("m_rms_accel "); Serial.println(m_rms_accel, BIN);
	    // Serial.print("m_accel "); Serial.println(m_accel, BIN);
	    // Serial.print("m_gyro "); Serial.println(m_gyro, BIN);
	    // Serial.println();
    }

	static k_Mode mode = k_Mode::Off;
	static k_Mode mode_last = k_Mode::Off;
    if ((switches & cfg::rx::switch_state::mode::off) == cfg::rx::switch_state::mode::off) {
    	if (mode != k_Mode::Off) {
    		mode = k_Mode::Off;
    		Serial.print(F("mode: off\n"));
    		Serial.print("sw&mode "); Serial.println(switches & cfg::rx::switch_state::mode::off, BIN);
    	}
    }
    if ((switches & cfg::rx::switch_state::mode::calib_accel) == cfg::rx::switch_state::mode::calib_accel) {
    	if (mode != k_Mode::CalibrateAccel) {
    		mode = k_Mode::CalibrateAccel;
    		Serial.print(F("mode: calib accel\n"));
    		Serial.print("sw&mode "); Serial.println(switches & cfg::rx::switch_state::mode::calib_accel, BIN);
    	}
    }
    if ((switches & cfg::rx::switch_state::mode::calib_gyro) == cfg::rx::switch_state::mode::calib_gyro) {
    	if (mode != k_Mode::CalibrateGyro) {
    		mode = k_Mode::CalibrateGyro;
    		Serial.print(F("mode: calib gyro\n"));
    		Serial.print("sw&mode "); Serial.println(switches & cfg::rx::switch_state::mode::calib_gyro, BIN);
    	}
    }
    if ((switches & cfg::rx::switch_state::mode::rms_accel) == cfg::rx::switch_state::mode::rms_accel) {
    	if (mode != k_Mode::RmsAccel) {
    		mode = k_Mode::RmsAccel;
    		Serial.print(F("mode: RMS accel\n"));
    		Serial.print("sw&mode "); Serial.println(switches & cfg::rx::switch_state::mode::rms_accel, BIN);
    	}
    }
    if ((switches & cfg::rx::switch_state::mode::accel) == cfg::rx::switch_state::mode::accel) {
    	if (mode != k_Mode::Accel) {
    		mode = k_Mode::Accel;
    		Serial.print(F("mode: accel\n"));
    		Serial.print("sw&mode "); Serial.println(switches & cfg::rx::switch_state::mode::accel, BIN);
    	}
    }
    if ((switches & cfg::rx::switch_state::mode::gyro) == cfg::rx::switch_state::mode::gyro) {
    	if (mode != k_Mode::Gyro) {
    		mode = k_Mode::Gyro;
    		Serial.print(F("mode: gyro\n"));
    		Serial.print("sw&mode "); Serial.println(switches & cfg::rx::switch_state::mode::gyro, BIN);
    	}
    }


	switch (mode) {
		case k_Mode::Off: {
		} break;

		case k_Mode::CalibrateAccel: {
			static const uint8_t readings_count = 9;
			static const float tolerance = 0.1;

			int16_t x_raw = 0;
			int16_t y_raw = 0;
			int16_t z_raw = 0;

			imu::mpu.getAcceleration(&x_raw, &y_raw, &z_raw);

			static float x = 0;
			static float y = 0;
			static float z = 0;
			static uint8_t ndx = 0;

			x += x_raw;
			y += y_raw;
			z += z_raw;

			if (++ndx >= readings_count) {
				ndx = 0;

				x /= readings_count;
				y /= readings_count;
				z /= readings_count;

				if (x < -tolerance) {
					cfg::imu::offset::accel::x += 1;
					imu::mpu.setXAccelOffset(cfg::imu::offset::accel::x);
				} else if (x > tolerance) {
					cfg::imu::offset::accel::x -= 1;
					imu::mpu.setXAccelOffset(cfg::imu::offset::accel::x);
				}
				if (y < -tolerance) {
					cfg::imu::offset::accel::y += 1;
					imu::mpu.setYAccelOffset(cfg::imu::offset::accel::y);
				} else if (y > tolerance) {
					cfg::imu::offset::accel::y -= 1;
					imu::mpu.setYAccelOffset(cfg::imu::offset::accel::y);
				}

				Serial.print(F("x: ")); Serial.print(x);
				Serial.print(F(" ")); Serial.print(cfg::imu::offset::accel::x);
				Serial.print(F("\ty: ")); Serial.print(y);
				Serial.print(F(" ")); Serial.print(cfg::imu::offset::accel::y);
				Serial.println();
			}

		} break;
		case k_Mode::CalibrateGyro:

		break;
		case k_Mode::SetEscs:

		break;
		case k_Mode::RmsAccel: {
			if (mode_last != k_Mode::RmsAccel) {
				Serial.print("Test ");

				uint8_t motor_speed =
					(rc_raw[(uint8_t)cfg::rx::k_Ch::VrA] - 1000) / 8;
				if (motor == k_Motor::TL) {
					board::SetTL(127 + motor_speed);
					Serial.print("TL");
				} else if (motor == k_Motor::TR) {
					board::SetTR(127 + motor_speed);
					Serial.print("TR");
				} else if (motor == k_Motor::BL) {
					board::SetBL(127 + motor_speed);
					Serial.print("BL");
				} else if (motor == k_Motor::BR) {
					board::SetBR(127 + motor_speed);
					Serial.print("BR");
				}
				Serial.print(" at ");
				Serial.println(motor_speed);

				delay(1000); // allow motor to spin-up

				float rms = 0;
				for (uint32_t i = 0; i < 10000; ++i) {
					int16_t x = 0;
					int16_t y = 0;
					int16_t z = 0;
					imu::mpu.getAcceleration(&x, &y, &z);
					rms += sqrt( (float)pow(x, 2) + (float)pow(y, 2) );
				}

				board::SetTL(127); board::SetTR(127);
				board::SetBL(127); board::SetBR(127);
				Serial.print("RMSxy: "); Serial.println(rms / 10000);
			}
		} break;
		case k_Mode::Accel: {
			int16_t x_raw = 0;
			int16_t y_raw = 0;
			int16_t z_raw = 0;
			
			board::SetTL(127 + 60);

			imu::mpu.getAcceleration(&x_raw, &y_raw, &z_raw);
			Serial.print(x_raw); Serial.print(",");
			Serial.print(y_raw); Serial.print(",");
			Serial.println(z_raw);
		} break;
		case k_Mode::Gyro:

		break;
		default:

		break;
	}

	mode_last = mode;
}
