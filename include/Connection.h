#ifndef CONNECTION_H__
#define CONNECTION_H__

#include "TransmissionBuffer.h"

#include <vector>
#include <string>

#include <signal.h>


namespace robot {

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

        /// Attempts to reestablish a connection according to a specific set
        /// of rules.
        ///
        /// If this Connection has never been connected(), does nothing.
        ///
        /// If we're in the middle of making a connection right now, does
        /// nothing.
        ///
        /// Otherwise, we have connected successfully at least once in the
        /// past, so connect() will be called again regardless of whether
        /// there is an existing connection or not.  (If there is an existing
        /// connection, disconnect() will be called first to sever it.)
        ///
        /// Note that if a connection is lost, this object will attempt to
        /// reconnect automatically.
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
        bool write(const std::string& s, TransmissionBuffer::LogType logTypeForErrors=TransmissionBuffer::debug);

    private:

        // The low-level thread function that performs the actual connection
        // with a single address.
        static int _createClientSocket(const std::string& addressToTry, int port, TransmissionBuffer& buffer, TransmissionBuffer::LogType logTypeForErrors);


        /// True if we're trying to asynchronously establish connections right
        /// now.
        bool connecting_;

        TransmissionBuffer& buffer_;

        // Parameters that represent our current connection (if it was successful.)
        int fd;
        std::string connectedAddress_;
        int connectedPort_;

        /// A default-constructible, copyable, first-class object that retains
        /// the arguments that were passed into the most recent call to
        /// connect().
        ///
        /// Note that parameters.port is not the same as connectedPort_ -- a
        /// disconnection will set connectedPort_ to -1 but will not alter
        /// parameters.port.
        struct MostRecentConnectParameters {
            public:
                std::vector<std::string> addressesToTry;
                int port;
                int timeoutInMilliseconds;
                TransmissionBuffer::LogType logTypeForErrors;
        };

        MostRecentConnectParameters parameters;

        // Save the original SIGPIPE handler (you know, the one we don't want
        // because it exits the program) so that we can reinstall it when
        // we're done.
        static struct sigaction original_sigaction;

        // Increments when Connections are constructed.  When it's 0 again, we
        // uninstall the handler.
        static int connection_count;
};

} // end (namespace robot)

#endif // (#ifndef CONNECTION_H__)
