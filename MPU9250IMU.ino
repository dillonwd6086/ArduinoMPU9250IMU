
#include <Wire.h>

#define MPU9250ADDRESS 	0x68
#define MAGADDRESS		0x0C
#define GYRO250DPS 		0x00
#define ACCSCALE2G 		0x00
#define GYROCONFIGREG 	0x1B
#define GYROPOWERREG 	0x6B
#define ACCELCONFIGREG	0x1C

float Roll = 0.0f;
float Pitch = 0.0f;
float Yaw = 0.0f;
unsigned long time = millis();

void setup(){
	
	Serial.begin(9600);

	Serial.println("Starting MPU9250 Program");

	initMPU9250();

}
//Main loop
void loop(){

	//define the variables that will store the raw values
	int16_t rollAcl, pitchAcl, yawAcl;
	int16_t xAcl, yAcl, zAcl;
	int16_t temp;

	getSensorData(rollAcl, pitchAcl, yawAcl, xAcl, yAcl, zAcl, temp);

	//call the method to grab the newest data form the sensor
	//Scale the data from the gyroscope
	rollAcl /= 161.0;
	pitchAcl /= 161.0;
	yawAcl /= 161.0;

	//Apply offset
	rollAcl -= 0.5;
	pitchAcl -= 1.2;

	//Find out how long ago before the last update
	double elapsedTime = ((double) millis() - time) / 1000.0;

	Roll -= (rollAcl * elapsedTime);
	//Integrate roll and pitch
	Pitch += (pitchAcl * elapsedTime);

	//update the time
	time = millis();

	//Time to find Roll and Pitch based on the Accelerometer
	double AclRoll = (atan2(-xAcl, zAcl) * 180.0) / PI;
	double AclPitch = ((double) atan2(xAcl, sqrt(((float) yAcl * (float) yAcl) + ((float) zAcl * (float) zAcl))) * 180.0) / PI;

	//Filter out garbage from accelerometer
	long forceApx = abs(yAcl) + abs(xAcl) + abs(zAcl);

	if(forceApx > 8192 && forceApx < 32768){

		double constant = 0.998;

		//Apply the complimentary filter
		Roll = Roll * constant + ((1.0f - constant) * AclRoll);
		Pitch = Pitch * constant + ((1.0f - constant) * AclPitch);

	}

	int16_t garbage, mx;
	
	getMagnetometerData(mx, garbage, garbage);

	Serial.print("Roll: ");
	Serial.print(Roll);
	Serial.print(" Pitch: ");
	Serial.print(Pitch);
	Serial.print(" Yaw: ");
	Serial.println(Yaw);
	delay(100);

}
void I2CwriteByte(uint8_t address, uint8_t reg, uint8_t data){
	
	//Start the transmission use the Wire lib
	Wire.beginTransmission(address);
	
	//Specifiy the register
	Wire.write(reg);

	//Speecify the data
	Wire.write(data);

	//Send the data
	Wire.endTransmission();
}

//This method writes the data to the register at the specified 
void I2CwriteBit(uint8_t address, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data){
	
	uint8_t bit;

	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);

	data <<= (bitStart - length + 1);

	bit &= mask;

	bit &= ~(mask);

	bit |= data;

	//Send the modified byte to be sent
	I2CwriteByte(address, reg, bit);

}

void initMPU9250(){
	
	//Tell the sensor to wake up
	I2CwriteByte(MPU9250ADDRESS, GYROPOWERREG, 0x00);

	//Configure the gyrscope range to 250DPS
	I2CwriteByte(MPU9250ADDRESS, GYROPOWERREG, GYRO250DPS);

	//Configure the accelerometer range
	I2CwriteByte(MPU9250ADDRESS, ACCELCONFIGREG,ACCSCALE2G);

	//Config the Sensors lowpass filter
	I2CwriteBit(MPU9250ADDRESS, GYROCONFIGREG, 2, 3, 3);

}
void initMPUMagnetometer(){

	//Set the control register to Continous Mesaurment mode 1
	//"0000": Power-down mode
	//"0001": Single measurement mode
	//"0010": Coninous measurement mode 1
	//"0110": Continous measurement mode 2
	//"0100": External trigger measurement mode
	//"1000": Self-test mode
	//"1111": Fuse  ROM access mode
	I2CwriteByte(MAGADDRESS, 0x0A, 2);

}
void getMagnetometerData(int16_t& mx, int16_t& my, int16_t& mz){

	//Start talking to the magnetometer
	Wire.beginTransmission(MAGADDRESS);

	//Specify the starting register
	Wire.write(0x03);

	//Let the wire library know more stuff is to come
	Wire.endTransmission(false);

	//Specify how many regisetrs you are are going to read, three
	//16 bit values split into two 8bit registers a piece so 6 registers
	Wire.requestFrom(MAGADDRESS, 6, true);

	//0x3H (HXL) & 0x4H (HXH)
	mx << Wire.read() <<8| Wire.read();

	//0x5H (HYL) & 0x6H (HYH)
	my << Wire.read() <<8| Wire.read();

	//0x7H (HZL) & 0x8H (HZH)
	mz << Wire.read() <<8| Wire.read();
}
void getSensorData(int16_t& gx, int16_t& gy, int16_t& gz, int16_t& ax, int16_t& ay, int16_t& az, int16_t& temp){
	
	//Start talking to the sensor
	Wire.beginTransmission(MPU9250ADDRESS);

	//Specify the starting register that you are going to read from
	Wire.write(0x3B);

	//et the wire library know that there is more stuff to come
	Wire.endTransmission(false);

	//Specify how many registers you are about to read from
	//Since the values are 16bit they are split between two
	//8 Bit registers that you will need to read from and 
	//concatinate the values
	Wire.requestFrom(MPU9250ADDRESS, 14, true);

	//0x3B (ACCEL_XOUT_H Register) & 0x3C (ACCEL_XOUT_L)
	ax = Wire.read() << 8|  Wire.read();

	//0x3D (ACCEL_YOUT_H)  & 0x3E (ACCELL_YOUT_L)
	ay = Wire.read() <<8| Wire.read();

	//0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	az = Wire.read() <<8| Wire.read();

	//0x41 (TEMP_OUT_H) & 0X42 (TEMP_OUT_L)
	temp = Wire.read() <<8| Wire.read();

	//0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gx = Wire.read() <<8| Wire.read();

	//0x45 (GYRO_YOUT_L) & 0x46 (GYRO_YOUT_L)
	gy = Wire.read() <<8| Wire.read();

	//0x47 (GYRO_ZOUT_L) & 0x47 (GYRO_ZOUT_L)
	gz = Wire.read() <<8| Wire.read();
}