#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"
#include "Message.h"

#include <thread>
#include <string>
#include <deque>

namespace robot {


/// A simple, copyable object that holds messages for later transmission.
///
/// This was created out of the need to log messages independently of the
/// thread-owning (and therefore uncopyable) RemoteTransmitter class --
/// passing around the MoveConstructible and MoveAssignable RemoteTransmitter
/// references _sort of_ works up until the point where you actually need to
/// pass one as an argument to a thread function (or anything else that uses
/// bind().  Then things quickly get way too tricky to be worth it.
///
/// The only reason those references were needed was for logging _anyway_, so
/// I created a logging object to take care of that need.

class TransmissionBuffer {
    public:
        // Disables echoing to the terminal.
        TransmissionBuffer();

        // Returns the queue of robot messages.
        std::deque<std::string>& robotQueue();

        // Returns the queue of driver station messages.
        std::deque<std::string>& driverStationQueue();

    public:
        // The types that can be passed into logMessage().
        enum LogType {
            camera,
            sentToRobot,
            sentToDriverStation,
            cantSendToRobot,
            cantSendToDriverStation,
            debug
        };

        // Records a debug message.
        //
        // Ordinarily, these will be enqueued and transmitted to the driver
        // station whenever it is available.  If the echoToTerminal() setting
        // is true, the messages will also be duplicated on standard output.
        void logMessage(LogType logType, const std::string& messageString);

        // Should logMessage() strings be printed to standard output?
        //
        // The setting is false by default; only the interactive menu is
        // really interested in setting this to true.
        void echoToTerminal(bool enable);

        // Are logMessage() strings being echoed to standard output?
        bool echoToTerminal() const;

    private:
        bool echoToTerminal_;
        std::deque<std::string> robotTransmissionBuffer;
        std::deque<std::string> driverStationTransmissionBuffer;
};


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

        // Move other's members into *this.
        RemoteTransmitter(RemoteTransmitter&& other);
        RemoteTransmitter& operator=(RemoteTransmitter&& other);

        // Shuts down the transmission queue.
        ~RemoteTransmitter();

        // If we're connected to the RoboRIO, returns hostname + ":" +
        // port number.
        //
        // If we're not connected, returns an empty string.
        std::string robotAddressAndPort() const;

        // If we're connected to the driver station, returns hostname + ":" +
        // port number.
        //
        // If we're not connected, returns an empty string.
        std::string driverStationAddressAndPort() const;

        // Adds a CameraMessage to the transmission queue for the robot.
        // Those are the only types of XML messages that the robot
        // understands.
        //
        // This call does not block.
        void enqueueRobotMessage(const CameraMessage& cameraMessage);

        // Adds a message to the transmission queue for the driver station.
        //
        // This call does not block.
        void enqueueDriverStationMessage(const Message& message);

        // Get the object we use to store log messages.
        TransmissionBuffer& buffer();

    private:
        Config config_;
        bool ignoreRobotConnectionFailure;
        TransmissionBuffer buffer_;

        // The function executed by the transmission thread.
        static void threadFunction(const Config& config,
                                   bool ignoreRobotConnectionFailure,
                                   TransmissionBuffer& buffer,
                                   std::string& robotAddressAndPort,
                                   std::string& driverStationAddressAndPort,
                                   const bool& shutdown);

        std::string robotAddressAndPort_;
        std::string driverStationAddressAndPort_;
        bool shutdown;  // If set to true, the thread will (eventually) end.

        std::thread transmissionThread;
};

} // end (namespace robot)

#endif // (#ifndef REMOTE_TRANSMITTER_H__)
