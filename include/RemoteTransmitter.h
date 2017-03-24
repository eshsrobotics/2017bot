#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"
#include "Message.h"
#include "TransmissionBuffer.h"
#include "Connection.h"

#include <thread>


namespace robot {


/// This object connects to the robot and driver station addresses in the
/// config file and then transmits whatever messages are given to it.
///
/// It is the sole link between the camera vision code running on the NVidia
/// Jetson TK1 and the robot driver code running on the RoboRIO (see
/// ServerRunnable.java for that.)
class RemoteTransmitter {
    public:
        /// If we can't connect to a remote host (i.e., the RoboRIO or the
        /// driver station), what should we do?
        enum ConnectionPolicy {
            /// Quietly cycle through the hosts we tried connecting to in the
            /// past, hoping one of them responds.
            AUTO_RECONNECT_ON_FAILURE,

            /// Just stop trying.
            ///
            /// Not very useful in production, but for the interactive mode,
            /// it helps with debugging.
            STOP_CONNECTING_ON_FAILURE
        };

    public:

        // A remote transmitter needs to know where to transmit to.
        //
        // This function also starts our internal transmission thread.
        RemoteTransmitter(const Config& config, ConnectionPolicy connectionPolicy=AUTO_RECONNECT_ON_FAILURE);

        // Move other's members into *this.
        RemoteTransmitter(RemoteTransmitter&& other);
        RemoteTransmitter& operator=(RemoteTransmitter&& other);

        // Shuts down the transmission queue.
        ~RemoteTransmitter();

        /////////////////////////
        // Connection methods. //
        /////////////////////////

        /// Gets our current connection policy.
        ConnectionPolicy connectionPolicy() const;

        /// Changes our current connection policy.
        void connectionPolicy(ConnectionPolicy connectionPolicy);

        /// Returns the object that holds the state of our connection to the
        /// RoboRIO.
        Connection& robotConnection();

        /// Returns the object that holds the state of our connection to the
        /// RoboRIO.
        const Connection& robotConnection() const;

        /// Returns the object that holds the state of our connection to the
        /// driver station.
        Connection& driverStationConnection();

        /// Returns the object that holds the state of our connection to the
        /// driver station.
        const Connection& driverStationConnection() const;

        ////////////////////////
        // Messaging methods. //
        ////////////////////////

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

        /// Get the object we use to store log messages.
        ///
        /// We don't have a const version of this because a const
        /// TransmissionBuffer is effectively useless.
        TransmissionBuffer& buffer();

    private:
        Config config_;
        ConnectionPolicy connectionPolicy_;
        TransmissionBuffer buffer_;

        // The function executed by the transmission thread.
        static void threadFunction(const Config& config,
                                   const ConnectionPolicy& connectionPolicy,
                                   TransmissionBuffer& buffer,
                                   Connection& robotConnection,
                                   Connection& driverStationConnection,
                                   const bool& shutdown);

        Connection robotConnection_;
        Connection driverStationConnection_;

        bool shutdown;  // If set to true, the thread will (eventually) end.

        std::thread transmissionThread;
};

} // end (namespace robot)

#endif // (#ifndef REMOTE_TRANSMITTER_H__)
