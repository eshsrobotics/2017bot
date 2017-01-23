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
using std::strerror;
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
// The Message base class is responsible for constructing the overall JSON
// message from its various pieces (including the message payload, which is
// usually also JSON.)
//
// We don't use a real JSON parser for this -- there's simply no need on the
// transmission side.

Message::operator string() const {

    stringstream stream;

    string payload = str();

    string timestamp;
    time_t now = time(nullptr);
    tm localTime;
    localtime_r(&now, &localTime);
    array<char, 100> buffer;
    if (strftime(&buffer[0], buffer.size(), "%c %Z", &localTime)) {
        stream.str("");
        stream << &buffer[0];
        timestamp = stream.str();
    } else {
        timestamp = "<Error: Buffer size too small to hold timestamp>";
    }

    // Construct the JSON message itself.
    stream.str("");
    stream << "{ \"type\": \"" << name() << "\", \"timestamp\": \"" << timestamp << "\", \"data\": ";
    if (payload.empty()) {
        stream << "{ }";
    } else {
        stream << payload;
    }
    stream << " }";
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
deque<string> RemoteTransmitter::transmissionBuffer;


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

void RemoteTransmitter::enqueueMessage(const Message& message) {
    // TODO: Actually feed the message into our deque
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
        SocketWrapper(int fd) : fd_(fd) { }
        SocketWrapper(const SocketWrapper&) = delete;
        SocketWrapper& operator=(const SocketWrapper& s) = delete;
        ~SocketWrapper() { if (fd_ >= 0) { close(fd_); } }
        int descriptor() const { return fd_; }
    private:
        int fd_;
};

// Creates a generic TCP/IP streaming socket that can be used for transmitting
// or receiving.
int _createBasicSocket() {

    // AF_INET + SOCK_STREAM + protocol 0 = TCP over IPv4.
    // AF_INET + SOCK_DGRAM  + protocol 0 = UDP over IPv4.
    int fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd == -1) {
        int old_errno = errno;    // Any subsequent glibc call might change it.
        array<char, 1000> buffer;
        strerror_r(old_errno, &buffer[0], buffer.size());

        stringstream stream;
        stream << "Error while creating socket: " << &buffer[0];
        RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
    }

    return fd;
}

// int createServerSocket(const vector<string>& addressesToTry, int port) {
//
//     int fd = _createBasicSocket();
//
//     // TODO: Bind to a local address with bind()
//     // TODO: Listen for connections with listen()
//     // TODO: Blocking accept for incoming connections with accept()
//     // Then have the camera client monitor function read() the data.
//     return -1;
// }

int createClientSocket(const vector<string>& addressesToTry, int port) {

    int fd = _createBasicSocket();
    hostent* server = nullptr;
    stringstream stream;

    for (auto iter = addressesToTry.begin(); iter != addressesToTry.end(); ++iter) {

        stream << "Trying to connect to " << *iter << ":" << port;
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
                array<char, 1000> buffer;
                strerror_r(old_errno, &buffer[0], buffer.size());

                stream.str("");
                stream << "Error while connecting to " << *iter << ": " << &buffer[0];
                RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
            }

            // If we reached this point, we connected successfully.
            stream.str("");
            stream << "Connected to " << *iter << ":" << port;
            RemoteTransmitter::logMessage(RemoteTransmitter::debug, stream.str());
            return fd;
        }

        // Failed.  Keep trying the next one.
        stream.str("");
        stream << "createClientSocket: Cant locate host named '" << *iter << "' on the network.";
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

    SocketWrapper clientSocket(createClientSocket(config.robotAddresses(), config.robotPort()));

    while (shutdown == false) {

        // If there are messages in the queue, read one and transmit it.

        // Send heartbeat messages every now and again.
        auto timeDelta = high_resolution_clock::now() - lastHeartbeatTransmissionTime;
        double elapsedMillisecondsSinceLastHeartbeat = duration<double, std::milli>(timeDelta).count();

        if (elapsedMillisecondsSinceLastHeartbeat > heartbeatThresholdMilliseconds) {
            // TODO: Actually transmit this.
            cerr << (string)HeartbeatMessage() << "\n";
            lastHeartbeatTransmissionTime = high_resolution_clock::now();
        }

    } // end (main thread loop)

    cerr << "Shutdown detected.\n";
}

} // end (namespace robot)
