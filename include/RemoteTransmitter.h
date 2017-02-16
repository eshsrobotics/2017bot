#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"
#include "Message.h"

#include <thread>
#include <string>
#include <deque>

namespace robot {

/// This object connects to the robot and driver station addresses in the
/// config file and then transmits whatever messages are given to it.
///
/// It is the sole link between the camera vision code running on the NVidia
/// Jetson TK1 and the robot driver code running on the RoboRIO (see
/// ServerRunnable.java for that.)
class RemoteTransmitter {
    public:
        /// What should the RemoteTransmitter constructor do if it can't
        /// connect to the robot?
        enum TransmissionMode {
            THROW_EXCEPTION_WHEN_ROBOT_CONNECTION_FAILS, // Fail.
            IGNORE_ROBOT_CONNECTION_FAILURE              // Continue.
        };

    public:

        // A remote transmitter needs to know where to transmit to.
        //
        // This function also starts our internal transmission thread.
        RemoteTransmitter(const Config& config, TransmissionMode transmissionMode = THROW_EXCEPTION_WHEN_ROBOT_CONNECTION_FAILS);

        // Local stderr messages:

        // Shuts down the transmission queue.
        ~RemoteTransmitter();

        // Adds a CameraMessage to the transmission queue for the robot.
        // Those are the only types of XML messages that the robot
        // understands.
        //
        // This call does not block.
        void enqueueRobotMessage(const CameraMessage& cameraMessage) const;

        // Adds a message to the transmission queue for the driver station.
        //
        // This call does not block.
        void enqueueDriverStationMessage(const Message& message) const;

    public:
        // The types that can be passed into RemoteTransmitter::logMessage().
        enum LogType {
            camera,
            sentToRobot,
            sentToDriverStation,
            cantSendToRobot,
            cantSendToDriverStation,
            debug
        };

        // Logs status messages to stderr.
        static void logMessage(LogType logType, const std::string& messageString);

    private:
        Config config_;
        bool ignoreRobotConnectionFailure;
        std::thread transmissionThread;

        // The function executed by the transmission thread.
        static void threadFunction(const Config& config, bool ignoreRobotConnectionFailure);

        static std::deque<std::string> robotTransmissionBuffer;
        static std::deque<std::string> driverStationTransmissionBuffer;
        static bool shutdown;  // If set to true, the thread will (eventually) end.
};


} // end (namespace robot)

#endif // (#ifndef REMOTE_TRANSMITTER_H__)
