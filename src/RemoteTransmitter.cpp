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
#include <string.h>     // strerror_r() (a gnu-specific function)
#include <netdb.h>      // struct hostent
#include <arpa/inet.h>  // htons()
#include <sys/socket.h>
#include <netinet/in.h> // struct sockaddr_in [and htons() on some systems]

#include <time.h> // For the reentrant and POSIX-standard localtime_r()

using std::cerr;
using std::copy;
using std::array;
using std::deque;
using std::string;
using std::thread;
using std::vector;
using std::exception;
using std::stringstream;
using std::runtime_error;

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
            array<char, 200> buffer;
            char* message = strerror_r(old_errno, buffer.data(), buffer.size());

            stringstream stream;
            stream << "SocketWrapper::write: ERROR: Can't write to socket for file descriptor "
                   << fd_ << ": \"" << message << "\" (errno = "
                   << old_errno << ")";
            RemoteTransmitter::logMessage(logTypeForErrors, stream.str());
        }
    }
}


int createClientSocket(const vector<string>& addressesToTry, int port, RemoteTransmitter::LogType logTypeForErrors=RemoteTransmitter::debug) {

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
                array<char, 200> buffer;
                char* message = strerror_r(old_errno, buffer.data(), buffer.size());

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
