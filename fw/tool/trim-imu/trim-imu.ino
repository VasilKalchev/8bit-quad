// #define IMU_MPU6050
#define IMU_MPU925X

#include "I2Cdev.h"

#if defined(IMU_MPU6050)
	#include <MPU6050.h>
#elif defined(IMU_MPU925X)
	#include <MPU925x_I2C.hpp>
#endif

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

#if defined(IMU_MPU6050)
	MPU6050 imu;
#elif defined(IMU_MPU925X)
	MPU925x_I2C imu;
#endif


int16_t const target_output = 0;
int16_t const target_output_az = 16384;

// int16_t offsets[6] = {-5333, -6027, 1446, 51, -62, 10};
int16_t offsets[6] = {-2606, -2476, 6251, 25, -3, 64};

// [2606,2607] --> [-3,5] [-2480,-2479] --> [-4,3]  [6251,6252] --> [16384,16392]
// [25,26] --> [0,2] [-3,-2] --> [0,1] [64,65] --> [0,2]
// Temp: 17.60, 64400

enum AxisNdx : uint8_t {
	NDX_AX = 0, NDX_AY = 1, NDX_AZ = 2,
	NDX_GX = 3, NDX_GY = 4, NDX_GZ = 5,
};

int16_t const avg_count = 300;


void ReadImuAvg(int16_t * const ax, int16_t * const ay, int16_t * const az, 
				int16_t * const gx, int16_t * const gy, int16_t * const gz) {
	int16_t imu_raw[6] = {0,0,0,0,0,0};
	int32_t imu_sum[6] = {0,0,0,0,0,0};

	for (int16_t i = 0; i < avg_count; ++i) {
		#if defined(IMU_MPU6050)
			imu.getMotion6(
				&imu_raw[NDX_AX], &imu_raw[NDX_AY], &imu_raw[NDX_AZ],
				&imu_raw[NDX_GX], &imu_raw[NDX_GY], &imu_raw[NDX_GZ]);
		#elif defined(IMU_MPU925X)
			imu.getAccelAndGyroRaw(
				&imu_raw[NDX_AX], &imu_raw[NDX_AY], &imu_raw[NDX_AZ],
				&imu_raw[NDX_GX], &imu_raw[NDX_GY], &imu_raw[NDX_GZ]);
		#endif
		delayMicroseconds(3500);

		for (int16_t j = NDX_AX; j <= NDX_GZ; ++j) {
			imu_sum[j] = imu_sum[j] + imu_raw[j];
		}
	}

	*ax = (imu_sum[NDX_AX] + (avg_count/2)) / avg_count;
	*ay = (imu_sum[NDX_AY] + (avg_count/2)) / avg_count;
	*az = (imu_sum[NDX_AZ] + (avg_count/2)) / avg_count;
	*gx = (imu_sum[NDX_GX] + (avg_count/2)) / avg_count;
	*gy = (imu_sum[NDX_GY] + (avg_count/2)) / avg_count;
	*gz = (imu_sum[NDX_GZ] + (avg_count/2)) / avg_count;
}

void SetOffsets(int16_t offsets[6]) {
	imu.setXAccelOffset(offsets[NDX_AX]);
	imu.setYAccelOffset(offsets[NDX_AY]);
	imu.setZAccelOffset(offsets[NDX_AZ]);
	imu.setXGyroOffset(offsets[NDX_GX]);
	imu.setYGyroOffset(offsets[NDX_GY]);
	imu.setZGyroOffset(offsets[NDX_GZ]);
}


void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	Serial.begin(2000000);

	// initialize device
	// Serial.println("Initializing I2C devices...");
	imu.initialize();

	// verify connection
	// Serial.println("Testing device connections...");
	// Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	SetOffsets(offsets);

	Serial.print("ax, ay, az, gx, gy, gz\n");
}

void loop() {
	static int16_t ax, ay, az, gx, gy, gz;

	ReadImuAvg(&ax, &ay, &az, &gx, &gy, &gz);
	// delay(3);

	Serial.print(ax);
	Serial.print(", ");
	Serial.print(ay);
	Serial.print(", ");
	Serial.print(az - target_output_az);
	Serial.print(",\t");
	Serial.print(gx);
	Serial.print(", ");
	Serial.print(gy);
	Serial.print(", ");
	Serial.print(gz);
	Serial.print("\n");


	while (Serial.available() > 0) {
		int8_t ndx = Serial.parseInt();
		char direction_char = Serial.read();

		if (ndx >= 0 && ndx <=5) {
			if (direction_char == '+') {
				offsets[ndx]++;
			} else if (direction_char == '-') {
				offsets[ndx]--;
			}
		}
		SetOffsets(offsets);


		#if defined(IMU_MPU6050)
			int16_t tempRaw = imu.getTemperature();
			float temp = (float)tempRaw / 340.0 + 36.53;
		#elif defined(IMU_MPU925X)
			uint16_t tempRaw = 0;
			imu.getTemperatureRaw(&tempRaw);
			float temp = 0.0f;
			imu.getTemperature(&temp);
		#endif
		// Serial.print("t: "); Serial.print(tempRaw);
		// Serial.print(", "); Serial.print(temp);
		// Serial.print(", ");

		// Serial.print("axo: "); Serial.print(offsets[NDX_AX]);
		// Serial.print(", ");
		// Serial.print("ayo: "); Serial.print(offsets[NDX_AY]);
		// Serial.print(", ");
		// Serial.print("azo: "); Serial.print(offsets[NDX_AZ]);
		// Serial.print(", ");
		// Serial.print("gxo: "); Serial.print(offsets[NDX_GX]);
		// Serial.print(", ");
		// Serial.print("gyo: "); Serial.print(offsets[NDX_GY]);
		// Serial.print(", ");
		// Serial.print("gzo: "); Serial.print(offsets[NDX_GZ]);
		// Serial.print("\n");

		Serial.print(tempRaw);
		Serial.print("\t"); Serial.print(temp);
		Serial.print("\t");

		Serial.print(offsets[NDX_AX]);
		Serial.print("\t");
		Serial.print(offsets[NDX_AY]);
		Serial.print("\t");
		Serial.print(offsets[NDX_AZ]);
		Serial.print("\t");
		Serial.print(offsets[NDX_GX]);
		Serial.print("\t");
		Serial.print(offsets[NDX_GY]);
		Serial.print("\t");
		Serial.print(offsets[NDX_GZ]);
		Serial.print("\n");
	}

}
