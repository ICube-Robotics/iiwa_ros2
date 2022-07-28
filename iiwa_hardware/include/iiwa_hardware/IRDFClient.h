// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IRDFCLIENT_H
#define IRDFCLIENT_H

#include "friLBRState.h"
#include "friLBRCommand.h"
#include "friUdpConnection.h"
#include "friClientData.h"

#include "low_pass_filter.h"

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <vector>
#include <fstream>
#include <chrono>



#define WORLD_DOF 6
#define NUMBER_OF_JOINTS 7
#define VEL_FILTER_CUTOFF 40
#define TRQ_FILTER_CUTOFF 40

#define VectorWDOFd Eigen::Matrix<double,WORLD_DOF,1>
#define VectorNJd Eigen::Matrix<double,NUMBER_OF_JOINTS,1>


#define DEFAULT_PORTID 30200

namespace KUKA::FRI {

/**
  * @brief structure of current iiwa status
  */
typedef struct{
    double measuredExtTorque[NUMBER_OF_JOINTS];
    double measuredTorque[NUMBER_OF_JOINTS];
    double commandedTorque[NUMBER_OF_JOINTS];
    double measuredJointPosition[NUMBER_OF_JOINTS];
    double commandedJointPosition[NUMBER_OF_JOINTS];
    double targetJointPosition[NUMBER_OF_JOINTS];
    double targetJointTorque[NUMBER_OF_JOINTS];
    double estimatedJointVelocity[NUMBER_OF_JOINTS];
    double filteredTorque[NUMBER_OF_JOINTS];
    double filteredExtTorque[NUMBER_OF_JOINTS];
    KUKA::FRI::EClientCommandMode commandMode;
    unsigned int timeStampNS;
    unsigned int timeStampS;
    double sampleTime;
    unsigned long long syncTime;
}iiwa_status;



/**
 * @brief The IRDFClient class is the main iiwaFRI driver. No control callback is implemented but logging of rorobt states is performed.
 */
class IRDFClient
{
public:
    /**
     * @brief IRDFClient constructor
     */
    IRDFClient();

    ~IRDFClient();

    /**
       * \brief Connect the FRI client application with a KUKA Sunrise controller.
       *
       * @param port The port ID
       * @param remoteHost The address of the remote host
       * @return True if connection was established
       */
    bool connect(int port, const char *remoteHost = NULL);

    /**
     * \brief Disconnect the FRI client application from a KUKA Sunrise controller.
     */
    void disconnect();

    void setVelocityFilterCutOffFreq(double vff);
    void setTorqueFilterCutOffFreq(double tff);

    bool updateFromRobot();
    bool updateToRobot();

    iiwa_status getRobotStatus();
    /**
     * @brief set robot status with target joint position data
     */
    void setTargetJointPosition(std::vector<double> target_joint_position);
    /**
     * @brief set robot status with target joint torque data
     */
    void setTargetJointTorque(std::vector<double> target_joint_torque);

    void getRobotJointPosition(std::vector<double>& jp);
    void getRobotJointVelocity(std::vector<double>& jv);
    void getRobotJointTorque(std::vector<double>& jt);
    void getRobotJointExternalTorque(std::vector<double>& jet);

    ESessionState getCurrentControllerState(){return _currentState;}

protected:
    /**
     * \brief Callback that is called whenever the FRI session state changes.
     *
     * @param oldState previous FRI session state
     * @param newState current FRI session state
     */
    virtual void onStateChange(ESessionState oldState, ESessionState newState);

    /**
     * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
     */
    virtual void monitor();
    /**
     * \brief Callback for the FRI session state 'Commanding Wait'.
     */
    virtual void waitForCommand();

    /**
     * \brief Callback for the FRI session state 'Commanding'.
     */
    virtual void command();

    /**
     * @brief update robot status from iiwa client
     */
    void updateRobotStatus();




    // TODO : allow effort control
    // /**
    //  * @brief set robot status with target joint effort data
    //  */
    // void updateTargetJointTorque(std::vector<double> target_joint_torque);

private:
    /**
     * @brief array of command wrenche for FRI
     */
    double _wrench[WORLD_DOF];
    VectorNJd _prev_q;
    DiscreteTimeLowPassFilter<VectorNJd> _qVel_filter, _trq_filter,_extTrq_filter;
    bool _isFirstUpdate;
    double _vel_filter_cutoff, _trq_filter_cutoff;
     /**
     * @brief status structure for information exchange
     */
    iiwa_status _iiwaStatus;
    int _size;
    UdpConnection _connection;
    LBRState _robotState;
    LBRCommand _robotCommand;
    ClientData* _data;
    ESessionState _currentState;

    bool _isTargetUpdated;



};

}  // namespace KUKA::FRI

#endif  // IRDFCLIENT_H
