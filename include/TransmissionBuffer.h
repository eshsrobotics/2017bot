#ifndef TRANSMISSION_BUFFER_H__
#define TRANSMISSION_BUFFER_H__

#include <string>
#include <deque>

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


} // end (namespace robot)

#endif // (#ifndef TRANSMISSION_BUFFER_H__)
