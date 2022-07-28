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

#include "iiwa_hardware/IRDFClient.h"

using namespace KUKA::FRI;
using namespace std;

IRDFClient::IRDFClient():
      _isFirstUpdate(true),
      _vel_filter_cutoff(VEL_FILTER_CUTOFF),
      _trq_filter_cutoff(TRQ_FILTER_CUTOFF),
      _isTargetUpdated(false)
{
    _prev_q = VectorNJd::Zero();
    for(int i=0;i<NUMBER_OF_JOINTS;i++){
        _iiwaStatus.targetJointTorque[i] = 0;
    }
    for(int i=0;i<WORLD_DOF;i++){
        _wrench[i] = 0;
    }

    _data = new ClientData(NUMBER_OF_JOINTS);

   // link monitoring and command message to wrappers
   _robotState._message = &_data->monitoringMsg;
   _robotCommand._cmdMessage = &_data->commandMsg;
   _robotCommand._monMessage = &_data->monitoringMsg;

   // set specific message IDs
   _data->expectedMonitorMsgID = _robotState.LBRMONITORMESSAGEID;
   _data->commandMsg.header.messageIdentifier = _robotCommand.LBRCOMMANDMESSAGEID;


}
//*****************************************************************************
IRDFClient::~IRDFClient(){
    delete _data;
}
//******************************************************************************
void IRDFClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   printf("LBRiiwaClient state changed from %d to %d\n", oldState, newState);
}
//******************************************************************************
void IRDFClient::setVelocityFilterCutOffFreq(double vff){
   _vel_filter_cutoff = vff;
}
//******************************************************************************
void IRDFClient::setTorqueFilterCutOffFreq(double tff){
   _trq_filter_cutoff = tff;
}
//******************************************************************************
void IRDFClient::monitor(){
    updateRobotStatus();
    _robotCommand.setJointPosition(_robotState.getCommandedJointPosition());
}
//******************************************************************************
void IRDFClient::waitForCommand()
{
   //  updateRobotStatus();
   // In waitForCommand(), the joint values have to be mirrored. Which _iiwaStatus done,
   // by calling the base method.
   _robotCommand.setJointPosition(_robotState.getIpoJointPosition());
   if (_robotState.getClientCommandMode() == TORQUE)
      {
         _robotCommand.setTorque(_iiwaStatus.targetJointTorque);
      }
   if (_robotState.getClientCommandMode() == WRENCH)
      {
         _robotCommand.setWrench(_wrench);
      }
}

//******************************************************************************
void IRDFClient::command()
{
    _robotCommand.setJointPosition(_robotState.getIpoJointPosition());
    double zero[NUMBER_OF_JOINTS] = {0};
    _robotCommand.setTorque(zero);

   // In command(), the joint angle values have to be set.
   if(_isTargetUpdated){
      if (_robotState.getClientCommandMode() == TORQUE)
         _robotCommand.setTorque(_iiwaStatus.targetJointTorque);
      _robotCommand.setJointPosition(_iiwaStatus.targetJointPosition );
      _isTargetUpdated = false;
   }
}

//******************************************************************************
void IRDFClient::updateRobotStatus(){

    const double sampleTime = _robotState.getSampleTime();

    memcpy(&_iiwaStatus.measuredExtTorque, _robotState.getExternalTorque(),NUMBER_OF_JOINTS*sizeof(double));
    memcpy(&_iiwaStatus.measuredTorque, _robotState.getMeasuredTorque(),NUMBER_OF_JOINTS*sizeof(double));
    memcpy(&_iiwaStatus.commandedTorque, _robotState.getCommandedTorque(),NUMBER_OF_JOINTS*sizeof(double));
    memcpy(&_iiwaStatus.measuredJointPosition, _robotState.getMeasuredJointPosition(),NUMBER_OF_JOINTS*sizeof(double));
    memcpy(&_iiwaStatus.commandedJointPosition, _robotState.getCommandedJointPosition(),NUMBER_OF_JOINTS*sizeof(double));
    memcpy(&_iiwaStatus.sampleTime,&sampleTime,sizeof(double));

    _iiwaStatus.commandMode = _robotState.getClientCommandMode();

    Eigen::Map<VectorNJd> measured_q(_iiwaStatus.measuredJointPosition);

    if(_isFirstUpdate){
        _prev_q = measured_q;
        _qVel_filter = DiscreteTimeLowPassFilter<VectorNJd>(_vel_filter_cutoff,_robotState.getSampleTime());
        _qVel_filter.set_initial(VectorNJd::Zero());

        _trq_filter = DiscreteTimeLowPassFilter<VectorNJd>(_trq_filter_cutoff,_robotState.getSampleTime());
        _trq_filter.set_initial(Eigen::Map<VectorNJd>(_iiwaStatus.measuredTorque));

        _extTrq_filter = DiscreteTimeLowPassFilter<VectorNJd>(_trq_filter_cutoff,_robotState.getSampleTime());
        _extTrq_filter.set_initial(Eigen::Map<VectorNJd>(_iiwaStatus.measuredExtTorque));
        _isFirstUpdate = false;
    }

    VectorNJd q_est = _qVel_filter.filter((measured_q-_prev_q)/_robotState.getSampleTime());
//    if(_iiwaStatus.timeStampS != _robotState.getTimestampSec() || _iiwaStatus.timeStampNS != _robotState.getTimestampNanoSec())
        _prev_q = measured_q;

    _iiwaStatus.timeStampNS = _robotState.getTimestampNanoSec();
    _iiwaStatus.timeStampS = _robotState.getTimestampSec();

    VectorNJd trq_est = _trq_filter.filter(Eigen::Map<VectorNJd>(_iiwaStatus.measuredTorque));
    VectorNJd extTrq_est = _extTrq_filter.filter(Eigen::Map<VectorNJd>(_iiwaStatus.measuredExtTorque));

    for(int i=0;i<NUMBER_OF_JOINTS;i++){
        _iiwaStatus.estimatedJointVelocity[i] = q_est[i];
        _iiwaStatus.filteredTorque[i] = trq_est[i];
        _iiwaStatus.filteredExtTorque[i] = extTrq_est[i];
    }
}
//******************************************************************************
void IRDFClient::setTargetJointPosition(std::vector<double> target_joint_position){
    if(target_joint_position.size()==NUMBER_OF_JOINTS){
        for(int i=0;i<NUMBER_OF_JOINTS;i++){
            _iiwaStatus.targetJointPosition[i] = target_joint_position[i];
        }
        _isTargetUpdated = true;
    }
    else{
       _isTargetUpdated = false;
    }
}
//******************************************************************************
void IRDFClient::setTargetJointTorque(std::vector<double> target_joint_torque){
    if(target_joint_torque.size()==NUMBER_OF_JOINTS){
        for(int i=0;i<NUMBER_OF_JOINTS;i++){
            _iiwaStatus.targetJointTorque[i] = target_joint_torque[i];
        }
    }
}
//******************************************************************************
iiwa_status IRDFClient::getRobotStatus(){
   return _iiwaStatus;
}

//******************************************************************************
void IRDFClient::getRobotJointPosition(std::vector<double>& jp){
   for(auto i=0ul;i<jp.size();i++)
      jp[i] = _iiwaStatus.measuredJointPosition[i];
}
//******************************************************************************
void IRDFClient::getRobotJointVelocity(std::vector<double>& jv){
   for(auto i=0ul;i<jv.size();i++)
      jv[i] = _iiwaStatus.estimatedJointVelocity[i];
}
//******************************************************************************
void IRDFClient::getRobotJointTorque(std::vector<double>& jf){
   for(auto i=0ul;i<jf.size();i++)
      jf[i] = _iiwaStatus.filteredTorque[i];
}
//******************************************************************************
void IRDFClient::getRobotJointExternalTorque(std::vector<double>& jet){
   for(auto i=0ul;i<jet.size();i++)
      jet[i] = _iiwaStatus.measuredExtTorque[i];
}
//******************************************************************************
bool IRDFClient::connect(int port, const char *remoteHost){
    if (_connection.isOpen())
    {
        printf("Warning: client application already connected!\n");
        return true;
    }

    return _connection.open(port, remoteHost);
}

//******************************************************************************
void IRDFClient::disconnect(){
    if (_connection.isOpen()) _connection.close();
}
//******************************************************************************
bool IRDFClient::updateFromRobot(){
    if (!_connection.isOpen())
   {
      printf("Error: client application is not connected!\n");
      return false;
   }
   // **************************************************************************
   // Receive and decode new monitoring message
   // **************************************************************************
   _size = _connection.receive(_data->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);
   if (_size <= 0)
   {  // TODO: _size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
      printf("Error: failed while trying to receive monitoring message!\n");
      return false;
   }

   if (!_data->decoder.decode(_data->receiveBuffer, _size))
   {
      return false;
   }

   // check message type (so that our wrappers match)
   if (_data->expectedMonitorMsgID != _data->monitoringMsg.header.messageIdentifier)
   {
      printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            (int)_data->monitoringMsg.header.messageIdentifier,
            (int)_data->expectedMonitorMsgID);
      return false;
   }

   // **************************************************************************
   // callbacks
   // **************************************************************************
   // reset command message before callbacks
   _data->resetCommandMessage();

   // callbacks for robot client
   _currentState = (ESessionState)_data->monitoringMsg.connectionInfo.sessionState;

   // std::cout << currentState << std::endl;

   if (_data->lastState != _currentState)
   {
      onStateChange(_data->lastState, _currentState);
      _data->lastState = _currentState;
   }
   if(_currentState!=MONITORING_WAIT)
      monitor();

   if (_currentState == IDLE) return false;
    return true;
}
//******************************************************************************
bool IRDFClient::updateToRobot(){
    switch (_currentState)
   {
      case MONITORING_WAIT:
      case MONITORING_READY:
         break;
      case COMMANDING_WAIT:
         waitForCommand();
         break;
      case COMMANDING_ACTIVE:
         command();
         break;
      case IDLE:
        return false;
      default:
        return true; // nothing to send back
   }

   // **************************************************************************
   // Encode and send command message
   // **************************************************************************

   _data->lastSendCounter++;
   // check if its time to send an answer
   if (_data->lastSendCounter >= _data->monitoringMsg.connectionInfo.receiveMultiplier)
   {
      _data->lastSendCounter = 0;

      // set sequence counters
      _data->commandMsg.header.sequenceCounter = _data->sequenceCounter++;
      _data->commandMsg.header.reflectedSequenceCounter =
            _data->monitoringMsg.header.sequenceCounter;

      if (!_data->encoder.encode(_data->sendBuffer, _size))
      {
         return false;
      }

      if (!_connection.send(_data->sendBuffer, _size))
      {
         printf("Error: failed while trying to send command message!\n");
         return false;
      }
   }

   return true;
}
