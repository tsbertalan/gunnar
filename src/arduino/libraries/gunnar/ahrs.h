#ifndef AHRS_H
#define AHRS_H
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include "utils.h"

/* Assign a unique ID to the sensors */
// If it turns out the argumens are unnecessary, make these private members of AHRS.
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(303023);

class AHRS {
public:

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_event_t gyro_event;
    sensors_vec_t   orientation;

    void init() {
        if(!accel.begin()) {
            /* There was a problem detecting the LSM303 ... check your connections */
            Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
            while(1);
        }
        if(!mag.begin()) {
            /* There was a problem detecting the LSM303 ... check your connections */
            Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
            while(1);
        }
    }

    boolean update() {
        accel.getEvent(&accel_event);
        mag.getEvent(&mag_event);
        gyro.getEvent(&gyro_event);

        /* Use the new fusionGetOrientation function to merge accel/mag data. */
        boolean output = dof.fusionGetOrientation(&accel_event, &mag_event, &orientation);
        if(!output) {
            Serial.println("Error while performing sensor fusion.");
        }
        rotateArray(headingHistories, NUMHEADINGHISTS);
        headingHistories[NUMHEADINGHISTS-1] = orientation.heading;
        return output;
    }

    double getHeading() {
        return average(headingHistories, NUMHEADINGHISTS);
    }


private:
    double headingHistories[NUMHEADINGHISTS];
};

#endif
