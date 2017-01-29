#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"
#include "Message.h"

#include <thread>
#include <string>
#include <deque>

namespace robot {

class RemoteTransmitter {
    public:

        // A remote transmitter needs to know where to transmit to.
        //
        // This function also starts our internal transmission thread.
        RemoteTransmitter(const Config& config);

        // Local stderr messages:

        // Shuts down the transmission queue.
        ~RemoteTransmitter();

        // Adds a message to the transmission queue for the robot.
        // TODO: Shouldn't this function only take arguments of type RobotMessage?
        //
        // This call does not block.
        void enqueueRobotMessage(const Message& message) const;

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
        static void logMessage(LogType logType, const std::string& message);

    private:
        Config config_;
        std::thread transmissionThread;

        // The function executed by the transmission thread.
        static void threadFunction(const Config& config);

        static std::deque<std::string> robotTransmissionBuffer;
        static std::deque<std::string> driverStationTransmissionBuffer;
        static bool shutdown;  // If set to true, the thread will (eventually) end.
};


} // end (namespace robot)

#endif // (#ifndef REMOTE_TRANSMITTER_H__)
