#include "RemoteTransmitter.h"

#include <mutex>
#include <array>
#include <thread>
#include <iomanip>
#include <sstream>
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <condition_variable>

#include <unistd.h>     // write()
#include <string.h>     // strerror()
#include <netdb.h>      // struct addrinfo
#include <sys/socket.h>

#include <time.h> // For the reentrant and POSIX-standard localtime_r()

using std::ref;
using std::move;
using std::cerr;
using std::copy;
using std::array;
using std::deque;
using std::mutex;
using std::string;
using std::thread;
using std::vector;
using std::cv_status;
using std::exception;
using std::defer_lock;
using std::lock_guard;
using std::unique_lock;
using std::stringstream;
using std::runtime_error;
using std::condition_variable;

using namespace std::chrono;

namespace robot {

////////////////////////////////////
// Allocate static class objects. //
////////////////////////////////////

struct sigaction Connection::original_sigaction;
int Connection::connection_count;

//////////////////////////////////////
// TransmissionBuffer helper class. //
//////////////////////////////////////

// =========================================================================
// Ensures that echoing to the terminal is disabled by default.

TransmissionBuffer::TransmissionBuffer()
    : echoToTerminal_(false), robotTransmissionBuffer(), driverStationTransmissionBuffer() { }

// =========================================================================
// Obtain read-write access to our queues.

deque<string>& TransmissionBuffer::robotQueue() { return robotTransmissionBuffer; }
deque<string>& TransmissionBuffer::driverStationQueue() { return driverStationTransmissionBuffer; }

// =========================================================================
// Control whether debug messages are also printed on the terminal.

bool TransmissionBuffer::echoToTerminal() const { return echoToTerminal_; }
void TransmissionBuffer::echoToTerminal(bool enable) { echoToTerminal_ = enable; }


// =========================================================================
// Logs a message to stderr and enqueue it for later transmission to the
// driver station.

void TransmissionBuffer::logMessage(TransmissionBuffer::LogType logType, const string& messageString) {

    string prefix;
    switch(logType) {
        case camera:                  prefix = "[C --> *]"; break;
        case sentToRobot:             prefix = "[* --> R]"; break;
        case sentToDriverStation:     prefix = "[* --> D]"; break;
        case cantSendToRobot:         prefix = "[* -> R?]"; break;
        case cantSendToDriverStation: prefix = "[* -> D?]"; break;
        case debug:                   prefix = "[ DEBUG ]"; break;
    }

    if (echoToTerminal_) {
        // Sometimes, even the best of us forget our newlines.
        cerr << prefix << " " << messageString <<  (messageString.back() != '\n' ? "\n" : "");
    }

    // Almost all of the messages we log are transmitted to the driver station
    // automatically as XML,which means wrapping them around LogMessage
    // objects.
    //
    // We do not transmit sentToRobot messages.  That's a deliberate decision;
    // mainLoop() already transmits a more useful debug message than the
    // PapasVision XML anyway.

    if (logType != sentToRobot) {
        // Sometimes, even the best of us provide unnecessary newlines.
        string s = (messageString.back() == '\n' ? messageString.substr(0, messageString.size() - 1) : messageString);
        LogMessage logMessage(s);
        driverStationTransmissionBuffer.push_back(static_cast<string>(logMessage));
    }
}



////////////////////////////////////////////////////////////////
// The methods that actually perform the network connections. //
////////////////////////////////////////////////////////////////

// Create a connection that's not connected to anything.
Connection::Connection(TransmissionBuffer& buffer)
    : connecting_(false), buffer_(buffer), fd(-1), connectedAddress_(),
      connectedPort_(-1), parameters() {

    if (++connection_count == 1) {

        // Tell the runtime to ignore SIGPIPE.
        struct sigaction new_sigaction;
        new_sigaction.sa_handler = SIG_IGN;
        sigemptyset(&new_sigaction.sa_mask);
        new_sigaction.sa_flags = 0;
        int result = sigaction(SIGPIPE, &new_sigaction, &original_sigaction);

        if (result < 0) {

            stringstream stream;
            stream << "*** PELIGRO! *** Unable to register a sigaction handler because sigaction() returned "
                   << result
                   << ".  Any remote disconnection will now terminate the program!";

            buffer_.logMessage(TransmissionBuffer::debug, stream.str());

        } else {

            buffer_.logMessage(TransmissionBuffer::debug, "Connection::Connection: sigaction handler for SIGPIPE installed successfully.");

        }
    }
}


// Move construction: *this takes control of other's assets, leaving other as
// an empty husk.
Connection::Connection(Connection&& other)
    : connecting_(other.connecting_), buffer_(other.buffer_), fd(other.fd),
      connectedAddress_(move(other.connectedAddress_)),
      connectedPort_(other.connectedPort_),
      parameters(move(other.parameters)) {

    other.connecting_ = false;
    other.fd = -1;
    other.connectedAddress_ = "";
    other.connectedPort_ = -1;
}


// Move assignment: *this takes control of other's assets, leaving other as an
// empty husk.
Connection& Connection::operator= (Connection&& other) {
    connecting_ = other.connecting_;
    buffer_ = move(other.buffer_);
    fd = other.fd;
    connectedAddress_ = move(other.connectedAddress_);
    connectedPort_ = other.connectedPort_;
    parameters = move(other.parameters);

    other.connecting_ = false;
    other.fd = -1;
    other.connectedAddress_ = "";
    other.connectedPort_ = -1;

    return *this;
}


// The destructor disconnects and unregisters the signal handler.
Connection::~Connection() {
    disconnect();

    if (--connection_count == 0) {
        int result = sigaction(SIGPIPE, &original_sigaction, nullptr);

        if (result < 0) {

            stringstream stream;
            stream << "Connection::~Connection: Warning: sigaction() returned " << result
                   << " while restoring the old SIGPIPE handler.";

            buffer_.logMessage(TransmissionBuffer::debug, stream.str());

        } else {

            buffer_.logMessage(TransmissionBuffer::debug, "Connection::~Connection: original sigaction handler for SIGPIPE restored successfully.");

        }
    }
}


// Writes a string to the underlying socket if the connection is active.
bool Connection::write(const string& s, TransmissionBuffer::LogType logTypeForErrors) {

    if (fd >= 0) {
        ssize_t result = ::write(fd, s.c_str(), s.size());

        if (static_cast<size_t>(result) == s.size()) {

            return true;

        } else if (result >= 0) {

            stringstream stream;
            stream << "write: Warning: Only wrote " << result << " out of "
                   << s.size()
                   << "bytes from last message to socket.  Perhaps there was a network interruption?";
            buffer_.logMessage(logTypeForErrors, stream.str());

        } else {

            int old_errno = errno; // Any subsequent glibc call might change it.

            if (old_errno == EPIPE) {

                // This is what we get when there's a remote disconnection while
                // ignoring the SIGPIPE signal.  We handle it the same way we
                // would handle SIGPIPE: by explicitly disconnecting on our end

                stringstream stream;
                stream << "write: Warning: Received received error code " << EPIPE
                       << " (EPIPE) while trying to write to socket file descriptor "
                       << fd << ".  Connection to " << connectedAddress_ << ":"
                       << connectedPort_ << " must have closed unexpectedly.";

                buffer_.logMessage(logTypeForErrors, stream.str());
                disconnect();

            } else {

                char* message = strerror(old_errno);
                stringstream stream;
                stream << "write: ERROR: Can't write to socket for file descriptor "
                       << fd << ": \"" << message << "\" (errno = "
                       << old_errno << ")";
                buffer_.logMessage(logTypeForErrors, stream.str());
            }
        }
    }
    return false;
}


/// This low-level routine opens a socket to the given address and port and
/// then returns its descriptor.  This is the business part of the thread
/// function run by each of the connection threads in createClientSocket().
///
/// We employ the modern getaddrinfo() approach here, which is much more
/// concise than getprotobyname() et al.
///
/// @param addressToTry A DNS name or IPv4 address string.
/// @param port A port number to connect to on the addressToTry.
/// @param buffer The TransmissionBuffer to use for queuing error messages if
///               we end up having a connection problem.
/// @param logTypeForErrors How we will classify any connection error messages
///                         at the time we log them.
/// @return A file descriptor representing the valid, connected socket, or -1
///         upon failure.
int Connection::_createClientSocket(const string& addressToTry, int port, TransmissionBuffer& buffer, TransmissionBuffer::LogType logTypeForErrors) {

    stringstream stream;
    stream << port;
    string portString = stream.str();

    stream.str("");
    stream << "_createClientSocket [" << std::this_thread::get_id() << "]";
    string name = stream.str();

    stream.str("");
    stream << name << ": Trying to connect to " << addressToTry << ":" << port;
    buffer.logMessage(TransmissionBuffer::debug, stream.str());

    addrinfo hints;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = 0;
    hints.ai_flags = 0;
    addrinfo *result;
    int errorCode = getaddrinfo(addressToTry.c_str(), portString.c_str(), &hints, &result);

    if (errorCode == 0) {

        // DNS resolution worked.  Time to connect.
        int fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        errorCode = ::connect(fd, result->ai_addr, result->ai_addrlen);

        // We don't need the addrinfo data structure that
        // getaddrinfo() allocated anymore, regardless of whether we
        // connected successfully or not.
        freeaddrinfo(result);

        if (errorCode == 0) {

            stream.str("");
            stream << name << ": Successfully connected to " << addressToTry << ":"
                   << port << " with file descriptor " << fd << ".";
            buffer.logMessage(TransmissionBuffer::debug, stream.str());
            return fd;

        } else {

            // If we made it here, we did the DNS resolution, but we couldn't
            // connect to that address.
            int old_errno = errno;    // Any subsequent glibc call might change it.
            char* message = strerror(old_errno);

            stream.str("");
            stream << name << ": WARNING: Can't connect to "
                   << addressToTry << ":" << port << ": \"" << message
                   << "\" (errno = " << old_errno << ")";
            buffer.logMessage(logTypeForErrors, stream.str());
            return -1;
        }

    }

    // If we made it here, DNS resolution failed.
    stream.str("");
    stream << name << ": WARNING: Can't locate host named '"
           << addressToTry << "' on the network: " << gai_strerror(errorCode);
    buffer.logMessage(logTypeForErrors, stream.str());
    return -1;
}


// ==========================================================================
// Attempts to connect to one of the given addresses or DNS names using the
// given port.  Gives up if no connection can be made after
// timeoutInMilliseconds seconds have elapsed.

void Connection::connect(const std::vector<std::string>& addressesToTry,
                         int port,
                         int timeoutInMilliseconds,
                         TransmissionBuffer::LogType logTypeForErrors) {

    // Allows the main thread (i.e., us, right here) to be asynchronously
    // notified whenever one of our child threads is able to successfully
    // connect.  But the _real_ advantage of using a condition variable is
    // that condition_variable::wait_for() allows us to leave early if the
    // timeout has expired.
    //
    // Connections to an address that can't be found on the network take a
    // long time to fail, and we don't want that to block the robot's camera
    // subsystem.
    condition_variable cv;

    // The condition variable uses file_descriptor_lock to create critical
    // sections where needed.  The child connection threads also rely on the
    // underlying mutex to safely modify the shared member data under *this.
    mutex file_descriptor_mutex;
    unique_lock<mutex> file_descriptor_lock(file_descriptor_mutex, defer_lock);

    // This flag is used so that all of the other child threads that have not
    // managed to connect will leave *this alone.
    bool connection_made = false;

    // This is our first official (or most recent) connection attempt; set
    // these variables to let reconnect() know that it will be okay to
    // reconnect should we fail.
    parameters.addressesToTry = addressesToTry;
    parameters.port = port;
    parameters.logTypeForErrors = logTypeForErrors;
    parameters.timeoutInMilliseconds = timeoutInMilliseconds;

    // The function that all child threads run.
    //
    // It attempts to connect to a single address and port.
    auto connectionThreadFunction =
        [this, &cv, &file_descriptor_mutex, &file_descriptor_lock, &connection_made] (const string& addressToTry, int port, TransmissionBuffer::LogType logTypeForErrors) {

        // Perform the potentially-expensive connection, which will block
        // this thread until it completes.
        int my_fd = _createClientSocket(addressToTry, port, this->buffer_, logTypeForErrors);

        if (my_fd > 0) { // We connected!

            if (!connection_made) { // We connected first.

                // Write the shared data safely.
                {
                    lock_guard<mutex> lock(file_descriptor_mutex);
                    this->fd = my_fd;
                    this->connectedAddress_ = addressToTry;
                    this->connectedPort_ = port;
                }

                // Let our calling thread know we're ready.  (It might have
                // already returned if we took too long, though.)
                cv.notify_one();

            } else { // Some other thread beat us to the punch.

                stringstream stream;
                stream << "createClientSocket [" << std::this_thread::get_id()
                       << "]: Successfully connected, but another thread has already connected.  Closing this thread's file descriptor ("
                       << fd << ").";
                lock_guard<mutex> lock(file_descriptor_mutex);
                this->buffer_.logMessage(TransmissionBuffer::debug, stream.str());

                // Our fd is now useless!
                close(my_fd);
            }
        } else {

            // Let's hope some other thread succeeds where we failed.

            // stringstream stream;
            // stream << "createClientSocket [" << std::this_thread::get_id()
            //        << "]: WARNING: Could not connect to " << addressToTry << ":" << port << ".";
            // buffer.logMessage(TransmissionBuffer::debug, stream.str());
        }
    };

    // Spawn multiple parallel threads to connect to all of the addressesToTry
    // at once.
    //
    // There is a _slight_ danger here that one of the threads will notify
    // before we have had a chance to finish spawning the other threads and
    // wait on the condition variable.  If that's a problem, having each
    // thread wait for a handful of milliseconds at the start should help.
    connecting_ = true;
    vector<thread> connectionThreads;
    for (string addressToTry : addressesToTry) {
        connectionThreads.push_back(thread(connectionThreadFunction, addressToTry, port, logTypeForErrors));

        // A thread in C++ that exits without being joined or detached
        // terminate()s, so we need to sever our ties with the connection
        // threads right away.
        if (connectionThreads.back().joinable()) {
            connectionThreads.back().detach();
        }
    }

    // Wait for a thread to notify us, but our time is limited.
    milliseconds timeout(timeoutInMilliseconds);
    if (cv.wait_for(file_descriptor_lock, timeout) == cv_status::no_timeout) {

        connection_made = true;

        // The other detached threads will get the hint eventually, but in the
        // meantime, we have to go.

        stringstream stream;
        stream << "createClientSocket [main]: Returning file descriptor " << fd << ".";
        buffer_.logMessage(TransmissionBuffer::debug, stream.str());
        connecting_ = false;
        return;
    }

    // If control made it here, we obviously timed out.
    connecting_ = false;
    stringstream stream;
    stream << "createClientSocket [main]: ERROR: No connections succeeded within "
           << timeout.count() << " milliseconds.  Giving up.\n";
    throw runtime_error(stream.str());
}


// Try to connect to one of the given addresses using the given port.
void Connection::disconnect() {
    if (connecting_ == true) {
        // We haven't connected yet, and we're still trying.  There's nothing
        // to disconnect, and the threads in connect() would overwrite
        // whatever we did here anyway.
        return;
    }

    if (fd >= 0) {
        close(fd);
        stringstream stream;
        stream << "disconnect: Closed file descriptor " << fd;
        buffer_.logMessage(TransmissionBuffer::debug, stream.str());

        // Let the outside world know we have no connection.
        fd = -1;
        connectedAddress_ = "";
        connectedPort_ = -1;
    }
}


// ==========================================================================
// Reconnect to whatever we successfully connected to last time.

void Connection::reconnect() {

    if (parameters.addressesToTry.size() == 0) {
        // We can't *re*-connect until we've connected at least once.
        return;
    }

    if (connecting_ == true) {
        // A connection is currently in progress.  Let's not interrupt it.
        return;
    }

    // If control makes it here, we connected successfully at least once in
    // the past, and are not making a connection now.

    if (connected()) {
        disconnect();
    }
    connect(parameters.addressesToTry, parameters.port, parameters.timeoutInMilliseconds, parameters.logTypeForErrors);
}


// ==========================================================================
// One-liner methods.

int Connection::descriptor() const { return fd; }
string Connection::address() const { return connectedAddress_; }
int Connection::port() const { return connectedPort_; }
bool Connection::connected() const { return (fd > 0); }


///////////////////////////////////////
// RemoteTransmitter implementation. //
///////////////////////////////////////

// =========================================================================
// Construct the remote transmitter.

RemoteTransmitter::RemoteTransmitter(const Config& config, ConnectionPolicy connectionPolicy)
    : config_(config),
      connectionPolicy_(connectionPolicy),
      buffer_(),
      robotConnection_(buffer_),
      driverStationConnection_(buffer_),
      shutdown(false),
      // This starts the thread!
      transmissionThread(threadFunction,
                         ref(config_),
                         ref(connectionPolicy_),
                         ref(buffer_),
                         ref(robotConnection_),
                         ref(driverStationConnection_),
                         ref(shutdown)) { }

// =========================================================================
// Kill the remote transmitter.

RemoteTransmitter::~RemoteTransmitter() {
    // Tell the thread to shut down and block until that happens.
    shutdown = true;
    transmissionThread.join();
}

// =========================================================================
// Allow the caller to get or set our connection policy.

RemoteTransmitter::ConnectionPolicy RemoteTransmitter::connectionPolicy() const { return connectionPolicy_; }
void RemoteTransmitter::connectionPolicy(RemoteTransmitter::ConnectionPolicy connectionPolicy_) { this->connectionPolicy_ = connectionPolicy_; }

// =========================================================================
// Tell the caller what we're connected to.

Connection& RemoteTransmitter::robotConnection() { return robotConnection_; }
const Connection& RemoteTransmitter::robotConnection() const { return robotConnection_; }
Connection& RemoteTransmitter::driverStationConnection() { return driverStationConnection_; }
const Connection& RemoteTransmitter::driverStationConnection() const { return driverStationConnection_; }


// =========================================================================
// Give the caller something in which they can store messages for us to
// transmit.

TransmissionBuffer& RemoteTransmitter::buffer() { return buffer_; }


// =========================================================================
// Toss a message into the pile of stuff to transmit.

void RemoteTransmitter::enqueueRobotMessage(const CameraMessage& cameraMessage) {
    buffer_.robotQueue().push_back(static_cast<string>(cameraMessage));

    // The driver station will get a more condensed and easily-readable
    // message.
    //
    // driverStationTransmissionBuffer.push_back(static_cast<string>(message));
}

void RemoteTransmitter::enqueueDriverStationMessage(const Message& message) {
    buffer_.driverStationQueue().push_back(static_cast<string>(message));
}


// =========================================================================
/// Connects to the robot and driver station and then transmits whatever
/// messages are in the queue for them.
///
/// @param config The Config that holds our known addresses, ports, and
///               timeouts for the roboRIO and the driver station.
///
/// @param connectionPolicy If the robotConnection cannot succeed in making a
///                         connection to the RoboRIO or the driver station,
///                         should we quit or should we keep trying?
///
/// @param buffer The TransmissionBuffer used for dequeuing the messages that
///               we will transmit.  Feel free to enqueue messages to it
///               asynchronously; we'll consume those as quickly as we can.
///
/// @param robotConnection[out] A Connection to wherever the Config says our
///                             RoboRIO is.  The caller should pass in a
///                             freshly-constructed object that is not
///                             connected to anything; we modify this object
///                             by attempting to make the connection (which
///                             might not succeed, mind you.)
///
///                             The caller can then use this updated object to
///                             perform queries about the connection even as
///                             we use the same object to write data to the
///                             remote server.
///
/// @param driverStationConnection[out] A Connection to wherever the Config
///                                     says the driver station is.  As usual,
///                                     you give us a freshly-constructed
///                                     Connection and we'll take care of
///                                     attempting to make it valid.
///
/// @param shutdown A reference to a boolean that will be set synchronously by
///                 the main thread.  We only continue to transmit while this
///                 value remains false.

void RemoteTransmitter::threadFunction(const Config& config,
                                       const ConnectionPolicy& connectionPolicy,
                                       TransmissionBuffer& buffer,
                                       Connection& robotConnection,
                                       Connection& driverStationConnection,
                                       const bool& shutdown) {

    // Transmit a heartbeat message when this many seconds have passed since
    // the last heartbeat message.
    const double heartbeatThresholdMilliseconds = 10000.0;
    auto lastHeartbeatTransmissionTime = high_resolution_clock::now();

    // Let's connect to the RoboRIO.
    buffer.logMessage(TransmissionBuffer::debug, "threadFunction: Opening connection to robot.");
    try {
        robotConnection.connect(config.robotAddresses(),
                                config.robotPort(),
                                config.robotTimeoutMilliseconds(),
                                TransmissionBuffer::cantSendToRobot);

    } catch (const exception& e) {
        buffer.logMessage(TransmissionBuffer::cantSendToRobot, "threadFunction: ERROR: Robot is unreachable.  Please check the addresses and port in the config file.");
    }


    // Let's connect to the driver station monitor.
    buffer.logMessage(TransmissionBuffer::debug, "threadFunction: Opening connection to driver station monitor.");
    try {

        driverStationConnection.connect(config.driverStationAddresses(),
                                        config.driverStationPort(),
                                        config.driverStationTimeoutMilliseconds(),
                                        TransmissionBuffer::cantSendToDriverStation);

    } catch(const exception& e) {
        buffer.logMessage(TransmissionBuffer::cantSendToDriverStation, "threadFunction: ERROR: Driver station is not reachable.  Please check the addresses and port in the config file.");
    }


    while (shutdown == false) {

        if (robotConnection.connected()) {

            // If there are messages in the queue, read one and transmit it.
            if (buffer.robotQueue().size() > 0) {
                string dataToTransmit = static_cast<string>(buffer.robotQueue().front());
                if (robotConnection.write(dataToTransmit, TransmissionBuffer::cantSendToRobot)) {
                    buffer.logMessage(TransmissionBuffer::sentToRobot, dataToTransmit);
                }
                buffer.robotQueue().pop_front();
            }

        } else if (connectionPolicy == AUTO_RECONNECT_ON_FAILURE) {

            try {
                robotConnection.reconnect();
            } catch(const exception& e) {
                buffer.logMessage(TransmissionBuffer::cantSendToRobot, "threadFunction: ERROR: Robot is still unreachable.  You really ought to check the addresses and port in the config file.");
            }
        }

        if (driverStationConnection.connected()) {

            // If there are messages in the queue, read one and transmit it.
            if (buffer.driverStationQueue().size() > 0) {
                string dataToTransmit = static_cast<string>(buffer.driverStationQueue().front());
                if (driverStationConnection.write(dataToTransmit, TransmissionBuffer::cantSendToDriverStation)) {
                    buffer.driverStationQueue().pop_front();
                }
            }

        } else if (connectionPolicy == AUTO_RECONNECT_ON_FAILURE) {

            try {
                driverStationConnection.reconnect();
            } catch(const exception& e) {
                buffer.logMessage(TransmissionBuffer::cantSendToDriverStation, "threadFunction: ERROR: Driver station is still unreachable.  You really ought to check the addresses and port in the config file.");
            }
        }


        // Send heartbeat messages every now and again.
        auto timeDelta = high_resolution_clock::now() - lastHeartbeatTransmissionTime;
        double elapsedMillisecondsSinceLastHeartbeat = duration<double, std::milli>(timeDelta).count();

        if (elapsedMillisecondsSinceLastHeartbeat > heartbeatThresholdMilliseconds) {
            buffer.driverStationQueue().push_back(static_cast<string>(HeartbeatMessage()));
            lastHeartbeatTransmissionTime = high_resolution_clock::now();
        }

        // Yield so this thread's not consuming 100% of a CPU core.
        std::this_thread::yield();

    } // end (main thread loop)

    cerr << "Shutdown detected.\n";
}

} // end (namespace robot)
