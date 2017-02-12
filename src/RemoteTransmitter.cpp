#include "RemoteTransmitter.h"

#include <array>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>

#include <unistd.h>     // write()
#include <string.h>     // strerror()
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
using std::unique_lock;
using std::stringstream;
using std::runtime_error;
using std::condition_variable;

using namespace std::this_thread;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace robot {

// =========================================================================
// Initialize the RemoteTransmitter static variables.

bool RemoteTransmitter::shutdown = false;
deque<string> RemoteTransmitter::driverStationTransmissionBuffer;
deque<string> RemoteTransmitter::robotTransmissionBuffer;


// =========================================================================
// Construct the remote transmitter.

RemoteTransmitter::RemoteTransmitter(const Config& config)
    : config_(config),
      // This starts the thread!
      transmissionThread(threadFunction, config) { }


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

    if (messageString.back() != '\n') {
        cerr << "\n"; // Sometimes, even the best of us forget our newlines.
    }
    cerr << prefix << " " << messageString;

    // Almost all of the messages we log are transmitted to the driver station
    // automatically as XML,which means wrapping them around LogMessage
    // objects.
    //
    // We do not transmit sentToRobot messages.  That's a deliberate decision;
    // mainLoop() already transmits a more useful debug message than the
    // PapasVision XML anyway.

    if (logType != sentToRobot) {
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
        void write(const string& s, RemoteTransmitter::LogType logTypeForErrors=RemoteTransmitter::debug) const;
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

void SocketWrapper::write(const string& s, RemoteTransmitter::LogType logTypeForErrors) const {
    if (fd_ >= 0) {
        ssize_t result = ::write(fd_, s.c_str(), s.size());

        if (result < 0) {
            int old_errno = errno;    // Any subsequent glibc call might change it.
            char* message = strerror(old_errno);

            stringstream stream;
            stream << "SocketWrapper::write: ERROR: Can't write to socket for file descriptor "
                   << fd_ << ": \"" << message << "\" (errno = "
                   << old_errno << ")";
            RemoteTransmitter::logMessage(logTypeForErrors, stream.str());
        }
    }
}


int createClientSocket(const vector<string>& addressesToTry, int port, std::chrono::milliseconds timeout, RemoteTransmitter::LogType logTypeForErrors=RemoteTransmitter::debug) {

    condition_variable cv;
    mutex file_descriptor_mutex;
    unique_lock<mutex> file_descriptor_lock(file_descriptor_mutex, defer_lock);

    int fd = -1;
    bool connection_made = false;

    // Each thread function attempts to connect to a single address and port.
    auto connectionThreadFunction = [&file_descriptor_lock, &cv, &fd, &connection_made] {

        // Perform the potentially-expensive connection, which will block
        // this thread until it completes.
        int my_fd = _createClientSocket(const vector<string>& addressesToTry, int port, RemoteTransmitter::LogType logTypeForErrors);

        if (connection_made) {

            // Some other thread beat us to the punch.  Our fd is now useless!

            stringstream stream;
            stream << "createClientSocket() [" << this_thread::get_id()
                   << "]: Another thread has already connected.  Closing this thread's file descriptor ("
                   << fd << ").";
            RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());

            if (my_fd >= 0) {
                close(my_fd);
            }

        } else {
            // Let our calling thread know we're ready.  (It might have
            // already returned if we took too long, though.)
            fd = my_fd;
            cv.notify_one();
        }
    };

    // Spawn multiple parallel threads to connect to all of the addressesToTry
    // at once.
    vector<thread> connectionThreads;
    for (string addressToTry : addressesToTry) {
        connectionThreads.push_back(thread(connectionThreadFunction));
    }

    // Wait for a thread to notify us, but our time is limited.
    auto status = ;
    stringstream stream;
    if (cv.wait_for(file_descriptor_lock, timeout) == cv_status::no_timeout) {

        connection_made = true;
        stream << "createClientSocket() [main]: Got a successful connection on file descriptor " << fd << ".";
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
        return fd;
    } else {
        stream << "createClientSocket() [main]: TIMEOUT: No connections succeeded within "
               << timeout.count() << " milliseconds.\n";
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
        return -1;
    }


    /// TODO:
    // As it turns out, select() is only useful for monitoring a bunch of FDs
    // that are already open.  It doesn't help with connection timeouts.
    //
    // Consensus on the Internet is that the best way to handle connect
    // timeouts is to spawn a thread and do the getaddrinfo() and connect() in
    // there, then join() on that thread with a timeout.
    //
    // To make matters worse, there is no timeout for std::thread::join().
    // Instead, we'll have to use condition variables, the programming of
    // which is undeniably painful.  http://stackoverflow.com/a/9949133
    //
    // Here are the changes we'll need:
    // 1. createClientSocket() needs to take a timeout in
    //    std::chrono::milliseconds.
    // 2. createClientSocket() declares a unique_lock<std::mutex>(m)
    //    that can be captured by any lambda functions.  Note that this
    //    acquires the lock immediately.
    // 3. createClientSocket() declares an int fd=-1 that can be captured by
    //    any lambda functions.
    // 4. createClientSocket() declares a std::condition_variable.
    // 5. createClientSocket() forks a thread to do the connection in a
    //    lambda, capturing the unique_lock, the condition_variable, and the fd.
    // 5. createClientSocket() calls
    //    condition_variable.wait_for(std::chrono::seconds(5)) to wait for the
    //    connection to be established.
    // 5a. If wait_for() returned std::cv_status::timeout, then we have waited
    //     too long for the connection and we return -1 for the fd.
    // 5b. If wait_for() returned std::cv_status::no_timeout, then we got
    //     notified!  The fd is ready, so return that.
    // 6. When the lambda has the fd from the connection, it sets the captured
    //    fd variable and it calls cv.notify_one() to unblock the main thread.
    //
    // And there you go.

}


int _createClientSocket(const string& addressToTry, int port, RemoteTransmitter::LogType logTypeForErrors) {

    stringstream stream;
    stream << port;
    string portString = stream.str();

    for (auto iter = addressesToTry.begin(); iter != addressesToTry.end(); ++iter) {

        stream.str("");
        stream << "createClientSocket: Trying to connect to " << *iter << ":" << port;
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());

        addrinfo hints;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = 0;
        hints.ai_flags = 0;
        addrinfo *result;
        int errorCode = getaddrinfo(iter->c_str(), portString.c_str(), &hints, &result);

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
                stream << "createClientSocket: Connected to " << *iter << ":"
                       << port << " with file descriptor " << fd << ".";
                RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());

                return fd;

            } else {

                // If we made it here, we did the DNS resolution, but we couldn't
                // connect to that address.
                int old_errno = errno;    // Any subsequent glibc call might change it.
                char* message = strerror(old_errno);

                stream.str("");
                stream << "createClientSocket: ERROR: Can't connect to "
                       << *iter << ": \"" << message << "\" (errno = "
                       << old_errno << ")";
                RemoteTransmitter::logMessage(logTypeForErrors, stream.str());

            }

        } else {

            // If we made it here, DNS resolution failed.
            stream.str("");
            stream << "createClientSocket: ERROR: Can't locate host named '"
                   << *iter << "' on the network: " << gai_strerror(errorCode);
            RemoteTransmitter::logMessage(logTypeForErrors, stream.str());
        }

        // Move on to the next one.
    }

    // If we're still here, none of the addresses we wanted to try worked.
    throw runtime_error("createClientSocket: ERROR: Can't connect to any server in the list.  Giving up.");
}


// =========================================================================
// Transmit the messages.

void RemoteTransmitter::threadFunction(const Config& config) {

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
        int fd = createClientSocket(config.robotAddresses(), config.robotPort(), cantSendToRobot);
        clientSocketToRobot = SocketWrapper(fd);
    } catch (const exception& e) {
        logMessage(cantSendToRobot, "threadFunction: ERROR: Robot is not reachable.  Please check the addresses and port in the config file.");
        // throw;
    }

    // The connection to the driver station monitor /is/ optional.  If it
    // times out, oh well.

    logMessage(debug, "threadFunction: Opening connection to driver station monitor.");
    SocketWrapper clientSocketToDriverStation;
    try {
        int fd = createClientSocket(config.driverStationAddresses(), config.driverStationPort(), cantSendToDriverStation);
        clientSocketToDriverStation = SocketWrapper(fd);
    } catch(const exception& e) {
        logMessage(cantSendToDriverStation, "threadFunction: ERROR: Driver station is not reachable.  Please check the addresses and port in the config file.");
    }


    while (shutdown == false) {

        // If there are messages in the queues, read one and transmit it.
        if (robotTransmissionBuffer.size() > 0) {
            string dataToTransmit = static_cast<string>(robotTransmissionBuffer.front());
            clientSocketToRobot.write(dataToTransmit);
            logMessage(sentToRobot, dataToTransmit);
            robotTransmissionBuffer.pop_front();
        }
        if (driverStationTransmissionBuffer.size() > 0) {
            string dataToTransmit = static_cast<string>(driverStationTransmissionBuffer.front());
            clientSocketToDriverStation.write(dataToTransmit);
            driverStationTransmissionBuffer.pop_front();
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
