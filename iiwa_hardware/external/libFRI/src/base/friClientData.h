/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.Connectivity FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.Connectivity FastRobotInterface” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2017
KUKA Roboter GmbH
Augsburg, Germany

LICENSE

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only.
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.13}
*/
#ifndef _KUKA_FRI_CLIENT_DATA_H
#define _KUKA_FRI_CLIENT_DATA_H

#include <vector>

#include "FRIMessages.pb.h"
#include "friMonitoringMessageDecoder.h"
#include "friCommandMessageEncoder.h"
#include "friClientIf.h"
#include "friException.h"

namespace KUKA
{
namespace FRI
{

   struct ClientData
   {
      char receiveBuffer[FRI_MONITOR_MSG_MAX_SIZE];//!< monitoring message receive buffer
      char sendBuffer[FRI_COMMAND_MSG_MAX_SIZE];   //!< command message send buffer

      FRIMonitoringMessage monitoringMsg;          //!< monitoring message struct
      FRICommandMessage commandMsg;                //!< command message struct

      MonitoringMessageDecoder decoder;            //!< monitoring message decoder
      CommandMessageEncoder encoder;               //!< command message encoder

      ESessionState lastState;                     //!< last FRI state
      uint32_t sequenceCounter;                    //!< sequence counter for command messages
      uint32_t lastSendCounter;                    //!< steps since last send command
      uint32_t expectedMonitorMsgID;               //!< expected ID for received monitoring messages

      const size_t MAX_REQUESTED_TRANSFORMATIONS;  //!< maximum count of requested transformations
      const size_t MAX_SIZE_TRANSFORMATION_ID;     //!< maximum size in bytes of a transformation ID
      std::vector<const char*> requestedTrafoIDs;  //!< list of requested transformation ids
//      std::vector<const char*> requestedIO_IDs;    //!< list of requested IO IDs

      ClientData(int numDofs)
      : decoder(&monitoringMsg, numDofs),
        encoder(&commandMsg, numDofs),
        lastState(IDLE),
        sequenceCounter(0),
        lastSendCounter(0),
        expectedMonitorMsgID(0),
        MAX_REQUESTED_TRANSFORMATIONS(sizeof(monitoringMsg.requestedTransformations) /
                                      sizeof(monitoringMsg.requestedTransformations[0])),
        MAX_SIZE_TRANSFORMATION_ID(sizeof(monitoringMsg.requestedTransformations[0].name))
      {
         requestedTrafoIDs.reserve(MAX_REQUESTED_TRANSFORMATIONS);
         monitoringMsg.monitorData.readIORequest_count = 0;
         commandMsg.commandData.writeIORequest_count = 0;
      }

      void resetCommandMessage()
      {
         commandMsg.commandData.has_jointPosition = false;
         commandMsg.commandData.has_cartesianWrenchFeedForward = false;
         commandMsg.commandData.has_jointTorque = false;
         commandMsg.commandData.commandedTransformations_count = 0;
         commandMsg.has_commandData = false;
         commandMsg.commandData.writeIORequest_count = 0;
      }

      //******************************************************************************
      static const FriIOValue& getBooleanIOValue(FRIMonitoringMessage* message, const char* name)
      {
         if(message != NULL && message->has_monitorData == true)
         {
            const MessageMonitorData& monData = message->monitorData;
            for(size_t i = 0; i < monData.readIORequest_count; i++)
            {
               const FriIOValue& ioValue = monData.readIORequest[i];
               if(strcmp(name, ioValue.name) == 0)
               {
                  if(ioValue.has_digitalValue == true && ioValue.type == FriIOType_BOOLEAN)
                  {
                     return ioValue;
                  }
                  throw FRIException("IO %s is not of type boolean.", name);
               }
            }
         }

         throw FRIException("Could not locate IO %s in monitor message.", name);
      }

      //******************************************************************************
      static const FriIOValue& getDigitalIOValue(FRIMonitoringMessage* message, const char* name)
      {
         if (message != NULL && message->has_monitorData == true)
         {
            const MessageMonitorData& monData = message->monitorData;
            for(size_t i = 0; i < monData.readIORequest_count; i++)
            {
               const FriIOValue& ioValue = monData.readIORequest[i];
               if(strcmp(name, ioValue.name) == 0)
               {
                  if(ioValue.has_digitalValue == true && ioValue.type == FriIOType_DIGITAL)
                  {
                     return ioValue;
                  }
                  throw FRIException("IO %s is not of type digital value.", name);
               }
            }
         }

         throw FRIException("Could not locate IO %s in monitor message.", name);
      }

      //******************************************************************************
      static const FriIOValue& getAnalogIOValue(FRIMonitoringMessage* message, const char* name)
      {
         if (message != NULL && message->has_monitorData == true)
         {
            const MessageMonitorData& monData = message->monitorData;
            for(size_t i = 0; i < monData.readIORequest_count; i++)
            {
               const FriIOValue& ioValue = monData.readIORequest[i];
               if(strcmp(name, ioValue.name) == 0)
               {
                  if(ioValue.has_analogValue == true && ioValue.type == FriIOType_ANALOG)
                  {
                     return ioValue;
                  }
                  throw FRIException("IO %s is not of type analog value.", name);
               }
            }
         }

         throw FRIException("Could not locate IO %s in monitor message.", name);
      }

      //******************************************************************************
      static void setBooleanIOValue(FRICommandMessage* message, const char* name, const bool value, FRIMonitoringMessage* monMessage)
      {
         MessageCommandData& cmdData = message->commandData;
         const size_t maxIOs = sizeof(cmdData.writeIORequest) / sizeof(cmdData.writeIORequest[0]);
         if(cmdData.writeIORequest_count < maxIOs)
         {
            // call getter which will raise an exception if the output doesn't exist
            // or is of wrong type.
            if(getBooleanIOValue(monMessage, name).direction != FriIODirection_OUTPUT)
            {
               throw FRIException("IO %s is not an output value.", name);
            }

            // add IO value to command message
            FriIOValue& ioValue = cmdData.writeIORequest[cmdData.writeIORequest_count];

            strncpy(ioValue.name, name, sizeof(ioValue.name) - 1);
            ioValue.name[sizeof(ioValue.name) - 1] = 0; // ensure termination
            ioValue.type = FriIOType_BOOLEAN;
            ioValue.has_digitalValue = true;
            ioValue.digitalValue = value;
            ioValue.direction = FriIODirection_OUTPUT;
            ioValue.has_analogValue = false;

            cmdData.writeIORequest_count ++;
            message->has_commandData = true;
         }
         else
         {
            throw FRIException("Exceeded maximum number of IOs that can be set.");
         }
      }

      //******************************************************************************
      static void setDigitalIOValue(FRICommandMessage* message, const char* name, const unsigned long long value, FRIMonitoringMessage* monMessage)
      {
         MessageCommandData& cmdData = message->commandData;
         const size_t maxIOs = sizeof(cmdData.writeIORequest) / sizeof(cmdData.writeIORequest[0]);
         if(cmdData.writeIORequest_count < maxIOs)
         {
            // call getter which will raise an exception if the output doesn't exist
            // or is of wrong type.
            if(getDigitalIOValue(monMessage, name).direction != FriIODirection_OUTPUT)
            {
               throw FRIException("IO %s is not an output value.", name);
            }

            // add IO value to command message
            FriIOValue& ioValue = cmdData.writeIORequest[cmdData.writeIORequest_count];

            strncpy(ioValue.name, name, sizeof(ioValue.name) - 1);
            ioValue.name[sizeof(ioValue.name) - 1] = 0; // ensure termination
            ioValue.type = FriIOType_DIGITAL;
            ioValue.has_digitalValue = true;
            ioValue.digitalValue = value;
            ioValue.has_analogValue = false;
            ioValue.direction = FriIODirection_OUTPUT;

            cmdData.writeIORequest_count ++;
            message->has_commandData = true;
         }
         else
         {
            throw FRIException("Exceeded maximum number of IOs that can be set.");
         }
      }

      //******************************************************************************
      static void setAnalogIOValue(FRICommandMessage* message, const char* name, const double value, FRIMonitoringMessage* monMessage)
      {
         MessageCommandData& cmdData = message->commandData;
         const size_t maxIOs = sizeof(cmdData.writeIORequest) / sizeof(cmdData.writeIORequest[0]);
         if(cmdData.writeIORequest_count < maxIOs)
         {
            // call getter which will raise an exception if the output doesn't exist
            // or is of wrong type.
            if(getAnalogIOValue(monMessage, name).direction != FriIODirection_OUTPUT)
            {
               throw FRIException("IO %s is not an output value.", name);
            }

            // add IO value to command message
            FriIOValue& ioValue = cmdData.writeIORequest[cmdData.writeIORequest_count];

            strncpy(ioValue.name, name, sizeof(ioValue.name) - 1);
            ioValue.name[sizeof(ioValue.name) - 1] = 0; // ensure termination
            ioValue.type = FriIOType_ANALOG;
            ioValue.has_digitalValue = false;
            ioValue.has_analogValue = true;
            ioValue.analogValue = value;
            ioValue.direction = FriIODirection_OUTPUT;

            cmdData.writeIORequest_count ++;
            message->has_commandData = true;
         }
         else
         {
            throw FRIException("Exceeded maximum number of IOs that can be set.");
         }
      }

      //******************************************************************************
/*      const std::vector<const char*>& getRequestedIO_IDs()
      {
         if(monitoringMsg.has_monitorData == false)
         {
            requestedIO_IDs.clear();
            return requestedIO_IDs;
         }

         const MessageMonitorData& monData = monitoringMsg.monitorData;
         requestedIO_IDs.resize(monData.readIORequest_count);
         for(size_t i = 0; i < monData.readIORequest_count; i++)
         {
            const FriIOValue& ioValue = monData.readIORequest[i];
            requestedIO_IDs[i] = ioValue.name;
         }

         return requestedIO_IDs;
      }
*/
   };

}
}


#endif // _KUKA_FRI_CLIENT_DATA_H
