#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"
#include "Message.h"

#include <thread>
#include <string>
#include <deque>
#include <chrono>

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

/// A high-level wrapper around the underlying system's concept of a
/// (client-side) network connection (to a server and a port.)
///
/// Every operating performed by this class is asynchronous.
class Connection {
    public:
        /// Creates a Connection that is not connected to anything.
        ///
        /// @param buffer The TransmissionBuffer to use for logging debug
        ///               messages about the state of the connection.
        Connection(TransmissionBuffer& buffer);

        /// Creates a Connection by transferring ownership of an existing Connection.
        ///
        /// @param other An existing Connection that may or may not be
        ///              connected().
        /// @post other.connected() == false and other.descriptor() == -1.
        Connection(Connection&& other);

        /// Assigns ownership of an existing Connection to this object.
        ///
        /// @param other An existing Connection that may or may not be
        ///              connected().
        /// @post other.connected() == false and other.descriptor() == -1.
        /// @return *this.
        Connection& operator= (Connection&& other);

        /// Forcibly disconnects if connected().  This allows connections to
        /// be gracefully closed even in the face of exceptions.
        ~Connection();

        /// Given a list of addresses and a port number to try, this function
        /// opens multiple simultaneous connections to each of those
        /// addresses.  The first one to connect "wins" and becomes out
        /// descriptor().
        ///
        /// @param addressesToTry   A list of hostname or IPv4 address
        ///                         strings.
        ///
        /// @param port A port number to connect to.  It will be tried for
        ///             each of the addresses.
        ///
        /// @param timeoutInMilliseconds The number of milliseconds to wait
        ///                              for the connecting threads before
        ///                              giving up.
        ///
        /// @param logTypeForErrors If we encounter an error and need to push
        ///                         a message to the driver station logging
        ///                         queue, this parameter determines the error
        ///                         type we report.
        ///
        /// @post connectedAddress() and connectedPort() will be set to the
        ///       first address and ports that we successfully connected to.
        ///       descriptor() will be set to the file descriptor of the
        ///       successfully-connected socket.
        ///
        /// @throws Throws a std::runtime_error if no connection could be made
        ///         to any of the addresses before the timeout period was
        ///         reached.
        void connect(const std::vector<std::string>& addressesToTry, int port, int timeoutInMilliseconds,
                     TransmissionBuffer::LogType logTypeForErrors=TransmissionBuffer::debug);

        /// Is this Connection established?
        ///
        /// This function is rather important because this class models a
        /// potentially unreliable connection that can disconnect at any
        /// time.  You should always check the value of connected() before
        /// attempting a read or write operation.
        ///
        /// @return true if we are connected to a server and false if we are
        ///         not.
        bool connected() const;

        /// The hostname we are connected() to, if any.
        ///
        /// @return The address or DNS name of the host we are connected to if
        ///         connected(), or an empty string otherwise.
        std::string address() const;

        /// The port we are connected() to, if any.
        /// @return The port of the host we are connected to if connected(),
        ///         or -1 otherwise.
        int port() const;

        /// If this Connection has never been connected(), does nothing.
        ///
        /// If this Connection has been connected() at some point in the past,
        /// calls connect() again with the same parameters as before, possibly
        /// calling disconnect() first if currently connected().
        ///
        /// Note that if a connection is lost, this object will attempt to
        /// reconnect automatically.  (TODO: How will we do this?  By checking
        /// every time descriptor() is called?  We'll still have to handle
        /// SIGPIPE.)
        void reconnect();

        /// If connected(), starts to close the underlying connection.  connected()
        /// will return false and descriptor() will return -1 once this is done.
        void disconnect();

        /// Returns the low-level file descriptor that this class manages so
        /// that the caller can perform read() or write() calls on it.
        int descriptor() const;

        /// If we're connected(), writes the given string to the underlying
        /// socket.
        ///
        /// @param s The string to transmit.
        /// @param logTypeForErrors If we cannot write, this is the type of
        ///                         log message we give to our underlying
        ///                         TransmissionBuffer.
        /// @returns True if we were able to write successfully and false if
        ///          the write failed.
        bool write(const std::string& s, TransmissionBuffer::LogType logTypeForErrors=TransmissionBuffer::debug) const;

    private:

        // The low-level thread function that performs the actual connection
        // with a single address.
        static int _createClientSocket(const std::string& addressToTry, int port, TransmissionBuffer& buffer, TransmissionBuffer::LogType logTypeForErrors);


        /// True if we're trying to asynchronously establish connections right
        /// now.
        bool connecting_;

        TransmissionBuffer& buffer_;
        int fd;
        std::vector<std::string> addressesToTry_;
        std::string connectedAddress_;
        int connectedPort_;
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
        bool ignoreRobotConnectionFailure;
        TransmissionBuffer buffer_;

        // The function executed by the transmission thread.
        static void threadFunction(const Config& config,
                                   bool ignoreRobotConnectionFailure,
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
