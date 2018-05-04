#include <Wire.h>
#include <MPU925x_I2C.hpp>

static MPU925x_I2C imu(MPU925x_ADDRESS_AD0_LOW);


void setup() {
	Serial.begin(115200);
	Serial.print("RESET -----\n\n--- MPU925x_I2C basic example ---\n\n");

	imu.initialize();
	Serial.print("init\n");

	imu.setAccelDLPF(2);
	imu.setAccelFullScaleRange(1);
	imu.setGyroDLPF(2);
	imu.setGyroFullScaleRange(1);
	Serial.print("filters and ranges\n");

	imu.setXAccelOffset(2594);
	imu.setYAccelOffset(-2479);
	imu.setZAccelOffset(5227);
	imu.setXGyroOffset (6);
	imu.setYGyroOffset (-3);
	imu.setZGyroOffset (8);
	Serial.print("offsets\n");

	Serial.print("Test conn: "); Serial.print(imu.testConnection());
	Serial.print("\n");
}

void loop() {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	imu.getAccelAndGyroRaw(&ax, &ay, &az, &gx, &gy, &gz);

	Serial.print(ax); Serial.print("\t");
	Serial.print(ay); Serial.print("\t");
	Serial.print(az); Serial.print("\t");
	Serial.print("\n");
}
