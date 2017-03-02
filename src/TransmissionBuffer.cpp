#include "Message.h"
#include "TransmissionBuffer.h"

#include <iostream>

using std::cerr;
using std::deque;
using std::string;

namespace robot {

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


} // end (namespace robot)
