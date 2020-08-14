/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };

void multiplyMatrices(int firstMatrix[][4], int secondMatrix[][4], int mult[][4])
{
  int i, j, k;
  // Initializing elements of matrix mult to 0.
  for(i = 0; i < 2; ++i)
  {
    for(j = 0; j < 2; ++j)
    {
      mult[i][j] = 0;
    }
  }

  // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
  for(i = 0; i < 2; ++i)
  {
    for(j = 0; j < 2; ++j)
    {
      for(k=0; k < 2; ++k)
      {
        mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
      }
    }
  }
}

double toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

void buildMatrix(double matrix[][4], double theta, double alfa, double r, double d) {
  double H[4][4] = {
    { cos(theta), -sin(theta)*cos(alfa),  sin(theta)*sin(alfa), r*cos(theta) },
    { sin(theta),  cos(theta)*cos(alfa), -cos(theta)*sin(alfa), r*sin(theta) },
    {          0,             sin(alfa),             cos(alfa),            d },
    {          0,                     0,                     0,            1 }
  };
  
  for (int i = 0; i < 4; i++) {
    memcpy(matrix[i], H[i], sizeof(matrix[i]));
  }
}

double* denavitHartenberg(double* currentPosition, ) {
  
}

int main(int argc, char **argv) {
  wb_robot_init();
  int i = 0;
 
  const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};
  double speed = 1.0;
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
   
 // double result[4][4]; 
 // buildMatrix(result, toRadians(90), toRadians(90), 0, 0.15185);  
  
  // for (int i = 0; i < 4; i++) {
    // for (int j = 0; j < 4; j++) {
      // printf("%.5f ", result[i][j]);
    // }
    // printf("\n");
  // }
  
  for (i = 0; i < 4; ++i)
    wb_motor_set_position(ur_motors[i], target_positions[i]);

  wb_robot_cleanup();
  return 0;
}
