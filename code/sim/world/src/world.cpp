/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "world.hpp"

namespace opendlv {
namespace sim {

World::World(int32_t const &a_argc, char **a_argv)
: TimeTriggeredConferenceClientModule(a_argc, a_argv, "sim-world"),
  m_clientRootFrames(),
  m_clientKinematicStates(),
  m_kinematicsMutex()
{
}

World::~World()
{
}

void World::nextContainer(odcore::data::Container &a_container)
{
  uint32_t const senderStamp = a_container.getSenderStamp();
  if (a_container.getDataType() == opendlv::sim::KinematicState::ID() &&
      m_clientRootFrames.count(senderStamp)) {

    odcore::base::Lock l(m_kinematicsMutex);
    auto kinematicState = a_container.getData<opendlv::sim::KinematicState>();
    m_clientKinematicStates[senderStamp] = kinematicState;
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode World::body()
{
  double dt = 0.01; //TODO!!
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    std::map<uint32_t, opendlv::sim::KinematicState> 
      clientKinematicStatesCopy;
    {
      odcore::base::Lock l(m_kinematicsMutex);
      clientKinematicStatesCopy.insert(m_clientKinematicStates.begin(), 
          m_clientKinematicStates.end());
    }

    for (auto clientKinematicState : m_clientKinematicStates) {
      uint32_t clientId = clientKinematicState.first; 
      auto kinematicState = clientKinematicState.second;

      double vx = kinematicState.getVx();
      double vy = kinematicState.getVy();
      double vz = kinematicState.getVz();

      double x = m_clientRootFrames[clientId].getX();
      double y = m_clientRootFrames[clientId].getY();
      double z = m_clientRootFrames[clientId].getZ();

      double newX = x + vx * dt;
      double newY = y + vy * dt;
      double newZ = z + vz * dt;



      double rollRate = kinematicState.getRollRate();
      double pitchRate = kinematicState.getPitchRate();
      double yawRate = kinematicState.getYawRate();

      double deltaRoll = rollRate * dt;
      double deltaPitch = pitchRate * dt;
      double deltaYaw = yawRate * dt;

      Eigen::AngleAxisd deltaRollAngle(deltaRoll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd deltaPitchAngle(deltaPitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd deltaYawAngle(deltaYaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaternion<double> deltaQ = deltaRollAngle * deltaPitchAngle 
        * deltaYawAngle;

      double roll = m_clientRootFrames[clientId].getRoll();
      double pitch = m_clientRootFrames[clientId].getPitch();
      double yaw = m_clientRootFrames[clientId].getYaw();

      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

      Eigen::Quaternion<double> newQ = deltaQ * q;

      Eigen::Vector3d newEuler = newQ.toRotationMatrix().eulerAngles(0, 1, 2);

      double newRoll = newEuler[0];
      double newPitch = newEuler[1];
      double newYaw = newEuler[2];



      m_clientRootFrames[clientId] = opendlv::sim::Frame(newX, newY, newZ,
          newRoll, newPitch, newYaw);
    }

  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void World::setUp()
{
  std::string const clientRootFrameIdsString = 
    getKeyValueConfiguration().getValue<std::string>(
        "sim-world.client_root_frame_ids");

  auto const clientRootFrameIdsStringVec = 
    odcore::strings::StringToolbox::split(clientRootFrameIdsString, ',');
  for (size_t i = 0; i < clientRootFrameIdsStringVec.size(); i++) {

    uint32_t const clientRootFrameId = 
      std::stoul(clientRootFrameIdsStringVec[i]);

    std::string const clientStartPoseString = 
      getKeyValueConfiguration().getValue<std::string>(
          "sim-world.client_start_pose_" + std::to_string(i));
  
    auto const clientStartPoseStringVec = 
      odcore::strings::StringToolbox::split(clientStartPoseString, ',');
    double const startX = std::stod(clientStartPoseStringVec[0]);
    double const startY = std::stod(clientStartPoseStringVec[1]);
    double const startZ = std::stod(clientStartPoseStringVec[2]);
    double const startRoll = std::stod(clientStartPoseStringVec[3]);
    double const startPitch = std::stod(clientStartPoseStringVec[4]);
    double const startYaw = std::stod(clientStartPoseStringVec[5]);

    opendlv::sim::Frame frame(startX, startY, startZ, startRoll, startPitch,
        startYaw);
    m_clientRootFrames[clientRootFrameId] = frame;

    if (isVerbose()) {
      std::cout << "Client with root frame id of " << clientRootFrameId
        << " starts from position [x=" << startX << ", y=" << startY << ", z="
        << startZ << "] with the rotation [roll=" << startRoll << ", pitch="
        << startPitch << ", yaw=" << startYaw << "]." << std::endl;
    }
  }
}

void World::tearDown()
{
}

}
}
