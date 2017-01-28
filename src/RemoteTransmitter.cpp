#include "RemoteTransmitter.h"

#include <array>
#include <ctime>
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

using std::tm;
using std::time;
using std::time_t;
using std::strftime;

using namespace std::chrono;
using namespace std::chrono_literals;

namespace robot {

// =========================================================================
// Get the current timestamp, or an error message string if that fails.

string Message::timestamp() const {
    time_t now = time(nullptr);
    tm localTime;
    localtime_r(&now, &localTime);

    array<char, 100> buffer;
    if (strftime(&buffer[0], buffer.size(), "%c %Z", &localTime)) {
        stringstream stream;
        stream << &buffer[0];
        return stream.str();
    }

    return "<Error: Buffer size too small to hold timestamp>";
}

// =========================================================================
// The Message base class is responsible for constructing the overall XML
// message from its various pieces (including the message payload, which is
// usually also XML.)
//
// We don't use a real XML parser for this -- there's simply no need on the
// transmission side.

Message::operator string() const {
    string payload = str();
    stringstream stream;

    stream << "<message><type>" << name() << "</type><timestamp>" << timestamp() << "</timestamp><data>";
    if (!payload.empty()) {
        stream << payload;
    }
    stream << "</data>";

    return stream.str();
}


// =========================================================================
// Heartbeat messages are empty -- only the timestamp matters.

HeartbeatMessage::HeartbeatMessage() { }
string HeartbeatMessage::str() const { return ""; }
string HeartbeatMessage::name() const { return "heartbeat"; }


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

void RemoteTransmitter::enqueueRobotMessage(const Message& message) const {
    robotTransmissionBuffer.push_back(static_cast<string>(message));
}

void RemoteTransmitter::enqueueDriverStationMessage(const Message& message) const {
    driverStationTransmissionBuffer.push_back(static_cast<string>(message));
}


// =========================================================================
// Logs a message to stderr.

void RemoteTransmitter::logMessage(RemoteTransmitter::LogType logType, const string& message) {

    string prefix;
    switch(logType) {
        case camera:                  prefix = "[C --> *]"; break;
        case sentToRobot:             prefix = "[* --> R]"; break;
        case sentToDriverStation:     prefix = "[* --> D]"; break;
        case cantSendToRobot:         prefix = "[* -> R?]"; break;
        case cantSendToDriverStation: prefix = "[* -> D?]"; break;
        case debug:                   prefix = "[ DEBUG ]"; break;
    }

    cerr << prefix << " " << message << "\n";
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
        void write(const string& s) const;
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

void SocketWrapper::write(const string& s) const {
    if (fd_ >= 0) {
        ssize_t result = ::write(fd_, s.c_str(), s.size());

        if (result < 0) {
            int old_errno = errno;    // Any subsequent glibc call might change it.
            array<char, 200> buffer;
            char* message = strerror_r(old_errno, buffer.data(), buffer.size());

            stringstream stream;
            stream << "Error while writing to socket for file descriptor "
                   << fd_ << ": \"" << message << "\" (errno = "
                   << old_errno << ")";
            RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
        }
    }
}


// Creates a generic TCP/IP streaming socket that can be used for transmitting
// or receiving.
int _createBasicSocket() {

    // AF_INET + SOCK_STREAM + protocol 0 = TCP over IPv4.
    // AF_INET + SOCK_DGRAM  + protocol 0 = UDP over IPv4.
    int fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd == -1) {
        int old_errno = errno;    // Any subsequent glibc call might change it.
        array<char, 200> buffer;
        char* message = strerror_r(old_errno, buffer.data(), buffer.size());

        stringstream stream;
        stream << "Error while creating socket: " << message
               << "\" (errno = " << old_errno << ")";
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
    }

    return fd;
}


int createClientSocket(const vector<string>& addressesToTry, int port) {

    int fd = _createBasicSocket();
    hostent* server = nullptr;
    stringstream stream;

    for (auto iter = addressesToTry.begin(); iter != addressesToTry.end(); ++iter) {

        stream << "createClientSocket: Trying to connect to " << *iter << ":" << port;
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());

        // Perform a DNS lookup on the current hostname.  (If it's an IP
        // address, this should still do the right thing.)
        server = gethostbyname(iter->c_str());

        if (server) {
            sockaddr_in serverAddress;
            serverAddress.sin_family = AF_INET;
            serverAddress.sin_port = htons(port);    // Convert byte order to big-endian.

            // Copy the network address from the DNS lookup.
            copy(reinterpret_cast<char *>(server->h_addr),
                 reinterpret_cast<char *>(server->h_addr) + server->h_length,
                 reinterpret_cast<char *>(&serverAddress.sin_addr.s_addr));

            if (connect(fd, reinterpret_cast<sockaddr*>(&serverAddress), sizeof(serverAddress)) < 0) {
                // Couldn't connect.
                int old_errno = errno;    // Any subsequent glibc call might change it.
                array<char, 200> buffer;
                char* message = strerror_r(old_errno, buffer.data(), buffer.size());

                stream.str("");
                stream << "createClientSocket: Error while connecting to "
                       << *iter << ": \"" << message << "\" (errno = "
                       << old_errno << ")";
                RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
            } else {
                // If we reached this point, we connected successfully.
                stream.str("");
                stream << "createClientSocket: Connected to " << *iter << ":"
                       << port << " with file descriptor " << fd << ".";
                RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
                return fd;
            }

        }

        // Failed.  Keep trying the next one.
        stream.str("");
        stream << "createClientSocket: Connection to " << *iter << ":" << port << " failed.";
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
    }

    // None of them worked.
    if (!server) {
        stringstream stream;
        stream << "createClientSocket: Can't find any server on the network.  Giving up.";
        throw runtime_error(stream.str());
    }

    // Then have the threadFunction write() the data.

    //TODO:
    return -1;
}


// =========================================================================
// Transmit the messages.

void RemoteTransmitter::threadFunction(const Config& config) {

    // Transmit a heartbeat message when this many seconds have passed since
    // the last heartbeat message.

    const double heartbeatThresholdMilliseconds = 2000.0;
    auto lastHeartbeatTransmissionTime = high_resolution_clock::now();

    // Let's connect to the network.
    //
    // The connection to the robot is not optional; if it fails, it'll throw
    // an exception and that will be that.

    logMessage(debug, "threadFunction: Opening connection to robot.");
    SocketWrapper clientSocketToRobot(createClientSocket(config.robotAddresses(), config.robotPort()));

    // The connection to the driver station monitor /is/ optional.  If it
    // times out, oh well.

    logMessage(debug, "threadFunction: Opening connection to driver station monitor.");
    SocketWrapper clientSocketToDriverStation;
    try {
        int fd = createClientSocket(config.driverStationAddresses(), config.driverStationPort());
        clientSocketToDriverStation = SocketWrapper(fd);
    } catch(const exception& e) {
        logMessage(debug, "threadFunction: Driver station is not reachable.  Please check the addresses and port in the config file.");
    }


    while (shutdown == false) {

        // If there are messages in the queues, read one and transmit it.
        if (robotTransmissionBuffer.size() > 0) {
            string dataToTransmit = static_cast<string>(robotTransmissionBuffer.front());
            clientSocketToRobot.write(dataToTransmit);
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
