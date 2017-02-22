// MAIN.CPP
//
// Entry point for the program that the Nvidia Jetson TK1 will be excuting at
// boot time.

// TODO: Add FPS measurement for Boiler/Peg solution rate.

#include "Config.h"
#include "RemoteTransmitter.h"
#include "PapasVision.h"

#ifdef __GLIBCXX__
#include <cxxabi.h> // A GCC-specific function useful for demangling std::type_info.name() strings.
#endif              // #ifdef __GLIBCXX__

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
using namespace robot;

// The board's 60 feet long, so we're starting with that.
//
// TODO: We need to find a better, more useful value for this
// threshold.  By the time our camera can no longer see the target
// clearly, we should reject the solution.
const double GOAL_REJECTION_THRESHOLD_INCHES = 12.0 * 60; 

void mainLoop(const Config &config);
void testSolutions(const Config& config, const vector<string>& imageFileNames);

void usage(const string& programName) {
  cout << "usage: " << programName << " [interactive|i]\n"
       << "       " << programName << " [test|t] [IMAGEFILE [IMAGEFILE...]]\n"
       << "       " << programName << "\n\n"
       << "With no arguments, runs the main loop, attempting to connect to the robot and driver station in the config file.\n\n"
       << "With the 'test' subcommand, runs the PapasVision solution algorithms on the given image(s).  If the paths are not absolute, they are considered to be relative to the config file's camera-folder setting.\n\n"
       << "Finally, with the 'interactive' subcommand, runs a user-driver version of the main loop, allowing the user to simulate network disconnections and to generate false PapasData.\n\n";
}

int main(int argc, char* argv[])
{
    try
    {
	string programName = argv[0];
        Config config;

	if (argc == 1) {

	    // No arguments.
	    mainLoop(config);

	} else {

	    // We have at least one argument.
	    string subCommand = argv[1];
	    if (subCommand == "test" || subCommand == "t") {

		// All remaining arguments must be file names.  We'll
		// let PapasVision test the file names for existence;
		// all that matters to us is that the user provided at
		// least one.
		if (argc == 2) {
		    cerr << "[ERROR] What image file(s) do you want me to test?\n";
		    usage(programName);
		    return 2;
		}

		vector<string> filenames;
		for (int i = 2; i < argc; ++i) {
		    filenames.push_back(argv[i]);
		}
		testSolutions(config, filenames);

	    } else if (subCommand == "interactive" || subCommand == "i") {

	    } else {
		cerr << "[ERROR] Invalid sub-command '" << subCommand << "'.\n";
		usage(programName);
		return 1;
	    }
	}


    }
    catch (const exception &e)
    {

        string exceptionTypeName = typeid(e).name();

#ifdef __GLIBCXX__
        // Translates a mangled g++ type name into something a human being can
        // read.  See
        // https://gcc.gnu.org/onlinedocs/libstdc++/manual/ext_demangling.html
        // for more information.  Other compilers don't seem to do this, so we
        // could leave exceptionTypeName as-is.

        int status;
        char *realname;
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
// Is there a Peg or Boiler PapasVision solution for any of the sample
// images?  This function's job is to find out.

void testSolutions(const Config& config, const vector<string>& imageFileNames) {
    const bool writeIntermediateFilesToDisk = true;
    PapasVision papasVision(config, GOAL_REJECTION_THRESHOLD_INCHES, writeIntermediateFilesToDisk);

    for (string imageFileName : imageFileNames) {
	cout << "*** " << imageFileName << " ***\n";

	papasVision.findBoiler(imageFileName);
	if (papasVision.getSolutionFound()) {
	    cout << "  Boiler solution found.  Distance: " 
		 << papasVision.getDistToGoalInch()
		 << " inches; angle: "
		 << papasVision.getAzimuthGoalDeg() 
		 << " degrees.\n";
	} else {
	    cout << "  Boiler solution not found.\n";
	}

	papasVision.findPeg(imageFileName);
	if (papasVision.getSolutionFound()) {
	    cout << "  Peg solution found.  Distance: " 
		 << papasVision.getDistToGoalInch()
		 << " inches; angle: "
		 << papasVision.getAzimuthGoalDeg() 
		 << " degrees.\n";
	} else {
	    cout << "  Peg solution not found.\n";
	}
    }

}

// =========================================================================
// Runs the camera code, constructs messages from it, and transmits those
// messages remotely as long as there are messages to transmit and something
// weird doesn't happen.

void mainLoop(const Config &config)
{

    // Only for debugging.  In reality, a failure to connect to the robot
    // before the timeout ought to be fatal.
    RemoteTransmitter::TransmissionMode mode = RemoteTransmitter::IGNORE_ROBOT_CONNECTION_FAILURE;

    // The RemoteTransmitter will shut the thread down when it goes out of scope.
    RemoteTransmitter transmitter(config, mode);

    PapasVision papasVision(config, 180.0, true);
    auto start = high_resolution_clock::now();
    default_random_engine generator(start.time_since_epoch().count());
    uniform_int_distribution<int> distribution(1, 8);
    PapasVision::SolutionType solutionType;
    int counter = 0;
    bool done = false;

    transmitter.logMessage(RemoteTransmitter::debug, "mainLoop: Camera client ready!");

    // We're in the middle of some deep vision debugging, and the rest of the
    // network code is noise at present.
    string sampleImage = "1ftH1ftD0Angle0Brightness.jpg";
    solutionType = PapasVision::Peg;
    papasVision.findPeg(sampleImage);
    cout << "\n\n*** Just ran findPeg(" << sampleImage << "); check the samples folder.  Bye for now. ***\n";
    cout << (solutionType == PapasVision::Boiler ? "Boiler" : "Peg")
           << " solution found for image" << sampleImage << ": (distance="
           << setprecision(5) << papasVision.getDistToGoalInch() << " inches, angle="
           << papasVision.getAzimuthGoalDeg() << " degrees)\n";
    exit(0);

    while (!done)
    {
        cerr << "\r";

        // For now, let's run everything for ten seconds.
        double elapsedSeconds = duration<double>(high_resolution_clock::now() - start).count();
        if (elapsedSeconds >= 10.0)
        {
            done = true;
        }
        else
        {
            cerr << "\rWaiting for " << setprecision(2) << (10.0 - elapsedSeconds) << " seconds...";
        }

        // Ensure that the log messages aren't too spammy.
        std::this_thread::sleep_for(milliseconds(200));

        // Run the PapasVision detector.
        //
        // Sample images are ./samples/{1..8}.png.
        // int imageNumber = distribution(generator);

        int imageNumber = 14; // testing purposes
        if (counter % 2 == 0) {
            solutionType = PapasVision::Boiler;
            papasVision.findBoiler(imageNumber);
        } else {
            solutionType = PapasVision::Peg;
            papasVision.findPeg(imageNumber);
        }

        // Send the PapasVision results out.
        if (papasVision.getSolutionFound())
        {

            double papasDistance = papasVision.getDistToGoalInch();
            double papasAngle = papasVision.getAzimuthGoalDeg();
            PapasVision::SolutionType solutionType = PapasVision::Boiler;

            // Print the camera image number for debugging purposes.
            stringstream stream;
            stream << (solutionType == PapasVision::Boiler ? "Boiler" : "Peg")
                   << " solution found for image #" << imageNumber << ": (distance="
                   << setprecision(5) << papasDistance << " inches, angle="
                   << papasAngle << " degrees)\n";
            transmitter.logMessage(RemoteTransmitter::camera, stream.str());

            // Transmit.
            CameraMessage cameraMessage(true, solutionType, papasDistance, papasAngle);
            transmitter.enqueueRobotMessage(cameraMessage);
        }
        else
        {

            // If we can't find a solution then we need to tell the robot
            // that, too, so it can act accordingly.
            CameraMessage cameraMessage(false, PapasVision::Boiler, -0.0, 0.0);
            transmitter.enqueueRobotMessage(cameraMessage);
        }

        counter++;
    }
    cout << "\n";
}
