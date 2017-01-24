#include "RemoteTransmitter.h"

#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <array>

#include <time.h> // For the reentrant and POSIX-standard localtime_r()

using std::cerr;
using std::array;
using std::string;
using std::stringstream;

using std::tm;
using std::time;
using std::time_t;
using std::strftime;

namespace robot {

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

HeartbeatMessage::HeartbeatMessage() { }
string HeartbeatMessage::str() const { return ""; }
string HeartbeatMessage::name() const { return "heartbeat"; }

RemoteTransmitter::RemoteTransmitter(const Config& config) : config_(config) {
    // Fire up the thread here.
}

RemoteTransmitter::~RemoteTransmitter() {
    // Shut down the thread here.
}

void RemoteTransmitter::enqueueMessage(const Message& message) {
    cerr << (string)message;
}

} // end (namespace robot)
