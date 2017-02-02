// MAIN.CPP
//
// Entry point for the program that the Nvidia Jetson TK1 will be excuting at
// boot time.

#include "Config.h"
#include "RemoteTransmitter.h"
#include "PapasVision.h"

#ifdef __GLIBCXX__
#include <cxxabi.h> // A GCC-specific function useful for demangling std::type_info.name() strings.
#endif // #ifdef __GLIBCXX__

#include <exception>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <random>

#include <opencv2/core/core.hpp>

namespace p = boost::program_options;
using std::uniform_int_distribution;
using std::default_random_engine;
using std::setprecision;
using std::stringstream;
using std::exception;
using std::string;
using std::vector;
using std::copy;
using std::cout;
using std::cerr;
using namespace std::this_thread;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace robot;

void mainLoop(const Config& config);

int main() {

    try {

        Config config;
        mainLoop(config);

    } catch(const exception& e) {

        string exceptionTypeName = typeid(e).name();

#ifdef __GLIBCXX__
        // Translates a mangled g++ type name into something a human being can
        // read.  See
        // https://gcc.gnu.org/onlinedocs/libstdc++/manual/ext_demangling.html
        // for more information.  Other compilers don't seem to do this, so we
        // could leave exceptionTypeName as-is.

        int status;
        char* realname;
        realname = abi::__cxa_demangle(exceptionTypeName.c_str(), 0, 0, &status);
        exceptionTypeName = string(realname);
        free(realname);
#endif // #ifdef __GLIBCXX__

        // Catch-all for any exceptions thrown in the program.
        cerr << "\n\n*** Abnormal termination due to uncaught "
             << exceptionTypeName << " exception.***\n\n";
        cerr << "Exception message: \"" << e.what() << "\"\n";
        return 1;

    }
}


// =========================================================================
// Runs the camera code, constructs messages from it, and transmits those
// messages remotely as long as there are messages to transmit and something
// weird doesn't happen.

void mainLoop(const Config& config) {

    // The RemoteTransmitter will shut the thread down when it goes out of scope.
    RemoteTransmitter transmitter(config);
    PapasVision papasVision(config, 180.0, true);
    auto start = high_resolution_clock::now();
    default_random_engine generator(start.time_since_epoch().count());
    uniform_int_distribution<int> distribution(1, 8);
    bool done = false;

    transmitter.logMessage(RemoteTransmitter::debug, "mainLoop: Camera client ready!");
    while (!done) {

        cerr << "\r";

        // For now, let's run everything for ten seconds.
        double elapsedSeconds = duration<double>(high_resolution_clock::now() - start).count();
        if (elapsedSeconds >= 10.0) {
            done = true;
        } else {
            cerr << "\rWaiting for " << setprecision(2) << (10.0 - elapsedSeconds) << " seconds...";
        }

        // Ensure that the log messages aren't too spammy.
        std::this_thread::sleep_for(0.2s);

        // Run the PapasVision detector.
        //
        // Sample images are ./samples/{1..8}.png.
        int imageNumber = distribution(generator);
        papasVision.findBoiler(imageNumber);

        // Send the PapasVision results out.
        if (papasVision.getSolutionFound()) {

            double papasDistance = papasVision.getDistToGoalInch();
            double papasAngle = papasVision.getAzimuthGoalDeg();

            // TODO: Not all camera messages will be for the boiler.
            // PapasVision needs to tell us the correct solution type.
            CameraMessage::SolutionType solutionType = CameraMessage::Boiler;

            // Print the camera image number for debugging purposes.
            stringstream stream;
            stream << (solutionType == CameraMessage::Boiler ? "Boiler" : "Peg")
                   << " solution found for image #" << imageNumber << ": (distance="
                   << setprecision(5) << papasDistance << " inches, angle="
                   << papasAngle << " degrees)\n";
            transmitter.logMessage(RemoteTransmitter::camera, stream.str());

            // Transmit.
            CameraMessage cameraMessage(true, solutionType, papasDistance, papasAngle);
            transmitter.enqueueRobotMessage(cameraMessage);

        } else {

            // If we can't find a solution then we need to tell the robot
            // that, too, so it can act accordingly.
            CameraMessage cameraMessage(false, CameraMessage::Boiler, -0.0, 0.0);
            transmitter.enqueueRobotMessage(cameraMessage);
        }

    }
    cout << "\n";

}
