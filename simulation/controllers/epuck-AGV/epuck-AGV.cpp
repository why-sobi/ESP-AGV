#include <iostream>
#include <cstdio>
#include "SerialPort.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    Motor *leftMotor = static_cast<Motor *>(robot->getDevice("left wheel motor"));
    Motor *rightMotor = static_cast<Motor *>(robot->getDevice("right wheel motor"));
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    DistanceSensor *gsLeft = static_cast<DistanceSensor *>(robot->getDevice("gs0"));
    DistanceSensor *gsCenter = static_cast<DistanceSensor *>(robot->getDevice("gs1"));
    DistanceSensor *gsRight = static_cast<DistanceSensor *>(robot->getDevice("gs2"));
    
    gsLeft->enable(timeStep);
    gsCenter->enable(timeStep);
    gsRight->enable(timeStep);

    SerialPort serial(Port::COM3);

    while (robot->step(timeStep) != -1) {
        double vLeft = gsLeft->getValue();
        double vCenter = gsCenter->getValue();
        double vRight = gsRight->getValue();

        // Send packet to ESP32
        char buffer[128];
        int len = std::snprintf(buffer, sizeof(buffer), "%.1f,%.1f,%.1f\n", vLeft, vCenter, vRight);

        if (serial.isConnected()) {
            serial.writeData(std::string(buffer, len));
            std::cout << "Sent: " << buffer;

            // Read back response from ESP32
            std::string response = serial.readData();
            if (!response.empty()) {
                float cmdLeft = 0.0, cmdRight = 0.0;
                if (std::sscanf(response.c_str(), "%f,%f", &cmdLeft, &cmdRight) == 2) {
                    leftMotor->setVelocity(cmdLeft);
                    rightMotor->setVelocity(cmdRight);
                    continue;
                }
            }
        }

        // Fallback local logic
        double maxSpeed = 3.5;
        if (vCenter < 400) {
            leftMotor->setVelocity(maxSpeed);
            rightMotor->setVelocity(maxSpeed);
        } else if (vLeft < 400) {
            leftMotor->setVelocity(maxSpeed * 0.1);
            rightMotor->setVelocity(maxSpeed);
        } else if (vRight < 400) {
            leftMotor->setVelocity(maxSpeed);
            rightMotor->setVelocity(maxSpeed * 0.1);
        } else {
            leftMotor->setVelocity(maxSpeed * 0.5);
            rightMotor->setVelocity(maxSpeed * 0.5);
        }
    }

    delete robot;
    return 0;
}