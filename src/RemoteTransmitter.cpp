#include "RemoteTransmitter.h"

#include <mutex>
#include <array>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <condition_variable>

#include <unistd.h>     // write()
#include <string.h>     // strerror()
#include <netdb.h>      // struct addrinfo
#include <sys/socket.h>

#include <time.h> // For the reentrant and POSIX-standard localtime_r()

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

// =========================================================================
// Initialize the RemoteTransmitter static variables.

bool RemoteTransmitter::shutdown = false;
deque<string> RemoteTransmitter::driverStationTransmissionBuffer;
deque<string> RemoteTransmitter::robotTransmissionBuffer;


// =========================================================================
// Construct the remote transmitter.

RemoteTransmitter::RemoteTransmitter(const Config& config, TransmissionMode transmissionMode)
    : config_(config),
      ignoreRobotConnectionFailure(transmissionMode == IGNORE_ROBOT_CONNECTION_FAILURE ? true : false),
      // This starts the thread!
      transmissionThread(threadFunction, config, ignoreRobotConnectionFailure) { }


// =========================================================================
// Kill the remote transmitter.

RemoteTransmitter::~RemoteTransmitter() {
    // Tell the thread to shut down and block until that happens.
    shutdown = true;
    transmissionThread.join();
}

// =========================================================================
// Toss a message into the pile of stuff to transmit.

void RemoteTransmitter::enqueueRobotMessage(const CameraMessage& cameraMessage) const {
    robotTransmissionBuffer.push_back(static_cast<string>(cameraMessage));

    // The driver station will get a more condensed and easily-readable
    // message.
    //
    // driverStationTransmissionBuffer.push_back(static_cast<string>(message));
}

void RemoteTransmitter::enqueueDriverStationMessage(const Message& message) const {
    driverStationTransmissionBuffer.push_back(static_cast<string>(message));
}


// =========================================================================
// Logs a message to stderr.

void RemoteTransmitter::logMessage(RemoteTransmitter::LogType logType, const string& messageString) {

    string prefix;
    switch(logType) {
        case camera:                  prefix = "[C --> *]"; break;
        case sentToRobot:             prefix = "[* --> R]"; break;
        case sentToDriverStation:     prefix = "[* --> D]"; break;
        case cantSendToRobot:         prefix = "[* -> R?]"; break;
        case cantSendToDriverStation: prefix = "[* -> D?]"; break;
        case debug:                   prefix = "[ DEBUG ]"; break;
    }

    // Sometimes, even the best of us forget our newlines.
    cerr << prefix << " " << messageString <<  (messageString.back() != '\n' ? "\n" : "");

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

// =========================================================================
// The methods that actually perform the network connections.

// I wrote this to make it easier to ensure sockets were closed in a timely
// manner even in the face of exceptions.
class SocketWrapper {
    public:
        SocketWrapper() : fd_(-1) { }
        SocketWrapper(int fd) : fd_(fd) { }
        SocketWrapper(SocketWrapper&& other) : fd_(other.fd_) { }
        SocketWrapper(const SocketWrapper&) = delete;
        SocketWrapper& operator=(SocketWrapper&& s) { fd_ = s.fd_; s.fd_ = -1; return *this; }
        SocketWrapper& operator=(const SocketWrapper& s) = delete;
        ~SocketWrapper();
        int descriptor() const { return fd_; }
        bool write(const string& s, RemoteTransmitter::LogType logTypeForErrors=RemoteTransmitter::debug) const;
    private:
        int fd_;
};

SocketWrapper::~SocketWrapper() {
    if (fd_ >= 0) {
        close(fd_);
        stringstream stream;
        stream << "SocketWrapper: Closed file descriptor " << fd_;
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
    }
}

bool SocketWrapper::write(const string& s, RemoteTransmitter::LogType logTypeForErrors) const {
    if (fd_ >= 0) {
        ssize_t result = ::write(fd_, s.c_str(), s.size());

        if (result >= 0) {
            return true;
        }

        int old_errno = errno;    // Any subsequent glibc call might change it.
        char* message = strerror(old_errno);

        stringstream stream;
        stream << "SocketWrapper::write: ERROR: Can't write to socket for file descriptor "
               << fd_ << ": \"" << message << "\" (errno = "
               << old_errno << ")";
        RemoteTransmitter::logMessage(logTypeForErrors, stream.str());
    }
    return false;
}


int _createClientSocket(const string& addressToTry, int port, RemoteTransmitter::LogType logTypeForErrors) {

    stringstream stream;
    stream << port;
    string portString = stream.str();

    stream.str("");
    stream << "_createClientSocket [" << std::this_thread::get_id() << "]";
    string name = stream.str();

    stream.str("");
    stream << name << ": Trying to connect to " << addressToTry << ":" << port;
    RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());

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
        errorCode = connect(fd, result->ai_addr, result->ai_addrlen);

        // We don't need the addrinfo data structure that
        // getaddrinfo() allocated anymore, regardless of whether we
        // connected successfully or not.
        freeaddrinfo(result);

        if (errorCode == 0) {

            stream.str("");
            stream << name << ": Successfully connected to " << addressToTry << ":"
                   << port << " with file descriptor " << fd << ".";
            RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
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
            RemoteTransmitter::logMessage(logTypeForErrors, stream.str());
            return -1;
        }

    }

    // If we made it here, DNS resolution failed.
    stream.str("");
    stream << name << ": WARNING: Can't locate host named '"
           << addressToTry << "' on the network: " << gai_strerror(errorCode);
    RemoteTransmitter::logMessage(logTypeForErrors, stream.str());
    return -1;
}


/// ==========================================================================
/// Given a list of addresses and a port number to try, this function opens
/// multiple simultaneous connections to each of those addresses.  The first
/// one to connect "wins" and is returned.
///
/// @param addressesToTry   A list of hostname or IPv4 address strings.
/// @param port             A port number to connect to.  It will be tried for
///                         each of the addresses.
/// @param timeout          The number of milliseconds to wait for the
///                         connecting threads before giving up.
/// @param logTypeForErrors If we encounter an error and need to push a
///                         message to the driver station logging queue, this
///                         parameter determines the error type we report.
/// @return                 The socket file descriptor for the first address
///                         that successfully connected.
/// @throws                 Throws a std::runtime_error if no connection could
///                         be made to any of the addresses before the timeout
///                         period was reached.
int createClientSocket(const vector<string>& addressesToTry, int port, milliseconds timeout, RemoteTransmitter::LogType logTypeForErrors=RemoteTransmitter::debug) {

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
    // underlying mutex to safely modify the shared file descriptor.
    mutex file_descriptor_mutex;
    unique_lock<mutex> file_descriptor_lock(file_descriptor_mutex, defer_lock);

    // The first child thread to successfully connect will modify this
    // variable atomically and return.
    int fd = -1;

    // This flag is used so that all of the other child threads that have not
    // managed to connect will leave the file descriptor alone.
    bool connection_made = false;

    // The function that all child threads run.
    //
    // It attempts to connect to a single address and port.
    auto connectionThreadFunction =
        [&file_descriptor_lock, &file_descriptor_mutex, &cv, &fd, &connection_made] (const string& addressToTry, int port, RemoteTransmitter::LogType logTypeForErrors) {

        // Perform the potentially-expensive connection, which will block
        // this thread until it completes.
        int my_fd = _createClientSocket(addressToTry, port, logTypeForErrors);

        if (my_fd > 0) { // We connected!

            if (!connection_made) { // We connected first.

                // Write the shared data safely.
                {
                    lock_guard<mutex> lock(file_descriptor_mutex);
                    fd = my_fd;
                }

                // Let our calling thread know we're ready.  (It might have
                // already returned if we took too long, though.)
                cv.notify_one();

            } else { // Some other thread beat us to the punch.

                stringstream stream;
                stream << "createClientSocket [" << std::this_thread::get_id()
                       << "]: Successfully connected, but another thread has already connected.  Closing this thread's file descriptor ("
                       << fd << ").";
                RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());

                // Our fd is now useless!  Ensure that it is closed in an
                // exception-safe manner.
                SocketWrapper socketWrapper(my_fd);
            }
        } else {

            // Let's hope some other thread succeeds where we failed.

            // stringstream stream;
            // stream << "createClientSocket [" << std::this_thread::get_id()
            //        << "]: WARNING: Could not connect to " << addressToTry << ":" << port << ".";
            // RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
        }
    };

    // Spawn multiple parallel threads to connect to all of the addressesToTry
    // at once.
    //
    // There is a _slight_ danger here that one of the threads will notify
    // before we have had a chance to finish spawning the other threads and
    // wait on the condition variable.  If that's a problem, having each
    // thread wait for a handful of milliseconds at the start should help.
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
    if (cv.wait_for(file_descriptor_lock, timeout) == cv_status::no_timeout) {

        connection_made = true;

        // The other detached threads will get the hint eventually, but in the
        // meantime, we have to go.

        stringstream stream;
        stream << "createClientSocket [main]: Returning file descriptor " << fd << ".";
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
        return fd;
    }

    stringstream stream;
    stream << "createClientSocket [main]: ERROR: No connections succeeded within "
           << timeout.count() << " milliseconds.  Giving up.\n";
    throw runtime_error(stream.str());
}


// =========================================================================
// Transmit the messages.
//
// TODO: If the server disconnects suddenly, we'll receive a fatal SIGPIPE.
//       We need to be able to handle that.

void RemoteTransmitter::threadFunction(const Config& config, bool ignoreRobotConnectionFailure) {

    const milliseconds connectionTimeout = milliseconds(5000);

    // Transmit a heartbeat message when this many seconds have passed since
    // the last heartbeat message.

    const double heartbeatThresholdMilliseconds = 5000.0;
    auto lastHeartbeatTransmissionTime = high_resolution_clock::now();

    // Let's connect to the network.
    //
    // The connection to the robot is not optional; if it fails, it'll throw
    // an exception and that will be that.

    logMessage(debug, "threadFunction: Opening connection to robot.");
    SocketWrapper clientSocketToRobot;
    try {
        int fd = createClientSocket(config.robotAddresses(), config.robotPort(), connectionTimeout, cantSendToRobot);
        clientSocketToRobot = SocketWrapper(fd);
    } catch (const exception& e) {
        logMessage(cantSendToRobot, "threadFunction: ERROR: Robot is unreachable.  Please check the addresses and port in the config file.");

        if (!ignoreRobotConnectionFailure) {
            // Not much point in proceeding without a robot connection.
            return;
        }
    }

    // The connection to the driver station monitor /is/ optional.  If it
    // times out, oh well.
    logMessage(debug, "threadFunction: Opening connection to driver station monitor.");
    SocketWrapper clientSocketToDriverStation;
    try {
        int fd = createClientSocket(config.driverStationAddresses(), config.driverStationPort(), connectionTimeout, cantSendToDriverStation);
        clientSocketToDriverStation = SocketWrapper(fd);
    } catch(const exception& e) {
        logMessage(cantSendToDriverStation, "threadFunction: ERROR: Driver station is not reachable.  Please check the addresses and port in the config file.");
    }


    while (shutdown == false) {

        // If there are messages in the queues, read one and transmit it.
        if (robotTransmissionBuffer.size() > 0) {
            string dataToTransmit = static_cast<string>(robotTransmissionBuffer.front());
            if (clientSocketToRobot.write(dataToTransmit)) {
                logMessage(sentToRobot, dataToTransmit);
            }
            robotTransmissionBuffer.pop_front();
        }
        if (driverStationTransmissionBuffer.size() > 0) {
            string dataToTransmit = static_cast<string>(driverStationTransmissionBuffer.front());
            if (clientSocketToDriverStation.write(dataToTransmit)) {
                driverStationTransmissionBuffer.pop_front();
            }
        }

        // Send heartbeat messages every now and again.
        auto timeDelta = high_resolution_clock::now() - lastHeartbeatTransmissionTime;
        double elapsedMillisecondsSinceLastHeartbeat = duration<double, std::milli>(timeDelta).count();

        if (elapsedMillisecondsSinceLastHeartbeat > heartbeatThresholdMilliseconds) {
            driverStationTransmissionBuffer.push_back(static_cast<string>(HeartbeatMessage()));
            lastHeartbeatTransmissionTime = high_resolution_clock::now();
        }

        // Yield so this thread's not consuming 100% of a CPU core.
        std::this_thread::yield();

    } // end (main thread loop)

    cerr << "Shutdown detected.\n";
}

} // end (namespace robot)
