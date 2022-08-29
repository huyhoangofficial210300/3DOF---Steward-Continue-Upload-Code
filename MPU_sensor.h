#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>

class MPU_sensor{
private:
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t tempRaw;

    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

    Kalman kalmanX; // Create the Kalman instances for X axis
    Kalman kalmanY; // Create the Kalman instances for Y axis

    double roll, pitch;

    uint32_t timer;
public:
    MPU_sensor(){
        this->roll = 0;
        this->pitch = 0;
    }
    void read_mpu_6050_data(){                                  //Subroutine for reading the raw gyro and accelerometer data
        Wire.beginTransmission(0x68);                           //Start communicating with the MPU-6050
        Wire.write(0x3B);                                       //Send the requested starting register
        Wire.endTransmission();                                 //End the transmission
        Wire.requestFrom(0x68,14);                              //Request 14 bytes from the MPU-6050
        while(Wire.available() < 14);                           //Wait until all the bytes are received
        this->accX = Wire.read()<<8|Wire.read();                //Add the low and high byte to the acc_x variable
        this->accY = Wire.read()<<8|Wire.read();                //Add the low and high byte to the acc_y variable
        this->accZ = Wire.read()<<8|Wire.read();                //Add the low and high byte to the acc_z variable
        this->tempRaw = Wire.read()<<8|Wire.read();             //Add the low and high byte to the temperature variable
        this->gyroX = Wire.read()<<8|Wire.read();               //Add the low and high byte to the gyro_x variable
        this->gyroY = Wire.read()<<8|Wire.read();               //Add the low and high byte to the gyro_y variable
        this->gyroZ = Wire.read()<<8|Wire.read();               //Add the low and high byte to the gyro_z variable
    }
    void init(){
        Wire.setClock(400000);
        Wire.begin();
        //Activate the MPU-6050
        Wire.beginTransmission(0x68);                                        
        Wire.write(0x6B);                                                    
        Wire.write(0x00);                                                    
        Wire.endTransmission();                                              
        //Configure the accelerometer (+/-2g)
        Wire.beginTransmission(0x68);                                        
        Wire.write(0x1C);                                                    
        Wire.write(0x00);                                                    
        Wire.endTransmission();                                              
        //Configure the gyro (250dps full scale)
        Wire.beginTransmission(0x68);                                        
        Wire.write(0x1B);                                                    
        Wire.write(0x00);                                                    
        Wire.endTransmission();        

        Serial.println("MPU sensor setup done !");
        this->read_mpu_6050_data();      
        
        double roll  = atan2(accY, accZ) * RAD_TO_DEG;
        double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;  
        if(isnan(roll)) {
            roll = 0;
        } 
        if(isnan(pitch)) {
            pitch = 0;
        } 
        this->kalmanX.setAngle(roll); // Set starting angle
        this->kalmanY.setAngle(pitch);
        this->gyroXangle = roll;
        this->gyroYangle = pitch;
        this->compAngleX = roll;
        this->compAngleY = pitch;
        this->timer = micros();                           
    }
    void update(){
        this->read_mpu_6050_data();
        double dt = (double)(micros() - this->timer) / 1000000; // Calculate delta time
        this->timer = micros();
        this->roll  = atan2(accY, accZ) * RAD_TO_DEG;
        this->pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        if(isnan(this->roll)) {
            this->roll = 0;
        } 
        if(isnan(this->pitch)) {
            this->pitch = 0;
        } 
        double gyroXrate = this->gyroX / 131.0; // Convert to deg/s
        double gyroYrate = this->gyroY / 131.0; // Convert to deg/s
        if ((this->roll < -90 && this->kalAngleX > 90) || (this->roll > 90 && this->kalAngleX < -90)) {
            this->kalmanX.setAngle(roll);
            this->compAngleX = this->roll;
            this->kalAngleX =  this->roll;
            this->gyroXangle = this->roll;
        } else
            this->kalAngleX = this->kalmanX.getAngle(this->roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        if (abs(this->kalAngleX) > 90)
            gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
        this->kalAngleY = this->kalmanY.getAngle(this->pitch, gyroYrate, dt);
    }
    double get_roll_angle(){
        return this->kalAngleX;
    }
    double get_pitch_angle(){
        return this->kalAngleY;
    }
};