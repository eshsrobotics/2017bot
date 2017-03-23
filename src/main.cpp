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
#include <iterator>
#include <iomanip>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <cctype>

#include <opencv2/core/core.hpp>

namespace p = boost::program_options;
using std::back_inserter;
using std::setprecision;
using std::stringstream;
using std::exception;
using std::toupper;
using std::string;
using std::vector;
using std::copy_n;
using std::copy;
using std::cout;
using std::cerr;
using std::cin;
using namespace std::this_thread;
using namespace std::chrono;
using namespace robot;

// The board's 60 feet long, so we're starting with that.
//
// TODO: We need to find a better, more useful value for this
// threshold.  By the time our camera can no longer see the target
// clearly, we should reject the solution.
const double GOAL_REJECTION_THRESHOLD_INCHES = 12.0 * 60;

// The length of time that mainLoop() is permitted to continue.  This number
// is slightly more time than the round length (165 seconds) to make up for
// false starts and what-not.
const double TIME_LIMIT_SECONDS = 200.0 * 1000;


void usage(const string& programName);
string trim(const string& s);
void mainLoop(const Config &config);
void interactiveLoop(const Config &config);
void testSolutions(const Config& config, const vector<string>& imageFileNames);
CameraMessage getPapasDataFromUser(string argument);


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

                vector<string> filenames;
                copy_n(argv + 2, argc - 2, back_inserter(filenames));

                // It's perfectly acceptable for the array to be empty here;
                // that just means to use the camera.
                if (filenames.size() == 0) {
                    filenames.push_back("");
                }

                testSolutions(config, filenames);

            } else if (subCommand == "interactive" || subCommand == "i") {

                interactiveLoop(config);

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
        cerr << "\n[ERROR] " << string(70, '-') << "\n";
        cerr << "[ERROR] Abnormal termination due to uncaught "
             << exceptionTypeName << " exception.\n";
        cerr << "[ERROR] Exception message: \"" << e.what() << "\"\n";
        return 1;
    }
}


// =========================================================================
// Prints a usage message.

void usage(const string& programName) {
  cout << "usage: " << programName << " [interactive|i]\n"
       << "       " << programName << " [test|t] [IMAGEFILE [IMAGEFILE...]]\n"
       << "       " << programName << "\n\n"
       << "With no arguments, runs the main loop, attempting to connect to the robot\n"
       << "and driver station in the config file.\n"
       << "\n"
       << "With the 'test' subcommand, runs the PapasVision solution algorithms on\n"
       << "the given image(s).  If the paths are not absolute, they are considered\n"
       << "to be relative to the config file's camera-folder setting.  If there are\n"
       << "no paths, then PapasVision will capture an image from the first available\n"
       << "camera attached to the system.\n"
       << "\n"
       << "Finally, with the 'interactive' subcommand, runs a user-driven version\n"
       << "of the main loop, allowing the user to simulate network disconnections\n"
       << "and to generate false PapasData.\n"
       << "\n";
}

// =========================================================================
/// Trims whitespace from the beginning and end of a string.

string trim(const string& s) {
    if (s.size() == 0) {
        return s;
    }

    unsigned int left = 0;
    while (isblank(s[left]) && left < s.size()) { left++; }

    unsigned int right = s.size() - 1;
    while (isblank(s[right]) && right > left) { right--; }

    return s.substr(left, right - left + 1);
}


// =========================================================================
// Is there a Peg or Boiler PapasVision solution for any of the given sample
// images?  This function's job is to find out.

void testSolutions(const Config& config, const vector<string>& imageFileNames) {

    const bool writeIntermediateFilesToDisk = true;
    PapasVision papasVision(config, GOAL_REJECTION_THRESHOLD_INCHES, writeIntermediateFilesToDisk);

    for (string imageFileName : imageFileNames) {
        cout << "*** ";
        if (imageFileName == "") {
            cout << "Video Camera";
        } else {
            cout << imageFileName;
        }
        cout << " ***\n";
        cout.precision(4);

        papasVision.findBoiler(imageFileName);
        if (papasVision.getSolutionFound()) {
            cout << "  Boiler solution found in "
                 << papasVision.getCalculationTimeMilliseconds()
                 << " milliseconds.  Distance: "
                 << papasVision.getDistToGoalInch()
                 << " inches; angle: "
                 << papasVision.getAzimuthGoalDeg()
                 << " degrees.\n";
        } else {
            cout << "  Boiler solution not found after "
                 << papasVision.getCalculationTimeMilliseconds()
                 << " milliseconds.\n";
        }

        papasVision.findPeg(imageFileName);
        if (papasVision.getSolutionFound()) {
            cout << "  Peg solution found in "
                 << papasVision.getCalculationTimeMilliseconds()
                 << " milliseconds.  Distance: "
                 << papasVision.getDistToGoalInch()
                 << " inches; angle: "
                 << papasVision.getAzimuthGoalDeg()
                 << " degrees.\n";
        } else {
            cout << "  Peg solution not found after "
                 << papasVision.getCalculationTimeMilliseconds()
                 << " milliseconds.\n";
        }
    }

}

// =========================================================================
// Just like mainLoop(), but each iteration of the loop is interrupted with a
// menu of sorts.

void interactiveLoop(const Config& config) {

    // Returns true if s starts with prefix, and false otherwise.
    auto startsWith = [] (const string& s, const string& prefix) -> bool {
        if (prefix.size() > s.size()) {
            return false;
        }
        return (s.substr(0, prefix.size()) == prefix);
    };

    // Spawns the thread and attempts to connect right away.
    RemoteTransmitter::ConnectionPolicy connectionPolicy = RemoteTransmitter::STOP_CONNECTING_ON_FAILURE;
    RemoteTransmitter transmitter(config, connectionPolicy);

    bool done = false;

    while(!done) {

        string robotAddressAndPort = "No robot.";
        if (transmitter.robotConnection().connected()) {
            stringstream stream;
            stream << "Robot: " << transmitter.robotConnection().address() << ":" << transmitter.robotConnection().port();
            robotAddressAndPort = stream.str();
        }

        string driverStationAddressAndPort = "No driver station.";
        if (transmitter.driverStationConnection().connected()) {
            stringstream stream;
            stream << "Driver station: " << transmitter.driverStationConnection().address() << ":" << transmitter.driverStationConnection().port();
            driverStationAddressAndPort = stream.str();
        }

        string connectionPolicy = "automatically reconnect when failure occurs";
        if (transmitter.connectionPolicy() == RemoteTransmitter::STOP_CONNECTING_ON_FAILURE) {
            connectionPolicy = "stop trying to reconnect when failure occurs";
        }

        cout << "\n"
             << " ,----------------.\n"
             << "( Choose an Option )  * " << robotAddressAndPort << "\n"
             << " `----------------'   * " << driverStationAddressAndPort << "\n"
             << "\n"
             << " DD) Disconnect from driver station\n"
             << " DR) Disconnect from robot\n"
             << " RD) Reconnect to driver station\n"
             << " RR) Reconnect to robot\n"
             << "SSD) Send string to driver station\n"
             << "SSR) Send string to robot\n"
             << "SXR) Send PapasVision XML to robot\n"
             << "  P) Change connection policy (current policy is to " << connectionPolicy << ")\n"
             << "  T) Toggle echoing debug messages to terminal (currently " << (transmitter.buffer().echoToTerminal() == true ? "enabled" : "disabled") << ")\n"
             << "  Q) Quit\n"
             << "\n"
             << "Your choice (DD,DR,RD,RR,SSD,P,T,Q)? ";

        string inputString, inputStringUpper;
        getline(cin, inputString);
        transform(inputString.cbegin(),
                  inputString.cend(),
                  back_inserter<string>(inputStringUpper),
                  static_cast<int (*)(int)>(toupper)); // Why is calling toupper() from transform() so hard?
        inputStringUpper = trim(inputStringUpper);


        if (startsWith(inputStringUpper, "DD")) {

            transmitter.driverStationConnection().disconnect();

        } else if (startsWith(inputStringUpper, "DR")) {

            transmitter.robotConnection().disconnect();

        } else if (startsWith(inputStringUpper, "RD")) {

            transmitter.driverStationConnection().reconnect();

        } else if (startsWith(inputStringUpper, "RR")) {

            transmitter.robotConnection().reconnect();

        } else if (startsWith(inputStringUpper, "SSD")) {

            string argument = trim(inputString.substr(3));
            if (argument.size() > 0) {
                cout << "Adding \"" << argument << "\" to message queue.\n";
            } else {
                cout << "String to send to driver station? ";
                getline(cin, argument);
                cout << "Added to message queue.\n";
            }
            transmitter.buffer().logMessage(TransmissionBuffer::debug, argument);

        } else if (startsWith(inputStringUpper, "SSR")) {

            string argument = trim(inputString.substr(3));
            if (argument.size() > 0) {
                cout << "Adding \"" << argument << "\" to message queue.\n";
            } else {
                cout << "String to send to robot? ";
                getline(cin, argument);
                cout << "Added to message queue.\n";
            }
            // Awkward, but I recognize that this is not a normal operation.
            transmitter.buffer().robotQueue().push_back(argument + "\n");

        } else if (startsWith(inputStringUpper, "SXR")) {

            string argument = trim(inputString.substr(3));
            CameraMessage cameraMessage = getPapasDataFromUser(argument);
            transmitter.enqueueRobotMessage(cameraMessage);

        } else if (inputStringUpper == "P") {

            if (transmitter.connectionPolicy() == RemoteTransmitter::AUTO_RECONNECT_ON_FAILURE) {
                transmitter.connectionPolicy(RemoteTransmitter::STOP_CONNECTING_ON_FAILURE);
            } else {
                transmitter.connectionPolicy(RemoteTransmitter::AUTO_RECONNECT_ON_FAILURE);
            }

        } else if (inputStringUpper == "T") {

            transmitter.buffer().echoToTerminal(!transmitter.buffer().echoToTerminal());

        } else if (inputStringUpper == "Q") {

            cout << "Goodbye for now.\n";
            done = true;

        }

    } // end (while not done)
}

// =========================================================================
/// Asks the user for the key pieces of data that make up PapasVision XML
/// data using a flexible parser (of sorts.)
///
/// @param argument An existing string the user has already entered from the
///                 menu (such as by typing "sxr true peg" instead of "sxr".)
///                 We incorporate this into the final result.  The string is
///                 allowed to be empty.
/// @return A valid PapasVision XML message.

CameraMessage getPapasDataFromUser(string argument) {

    bool solutionFound;
    double papasDistance;
    double papasAngle;
    PapasVision::SolutionType solutionType;

    bool done              = false;
    bool haveSolutionFound = false;
    bool haveSolutionType  = false;
    bool havePapasDistance = false;
    bool havePapasAngle    = false;
    string s               = argument;

    // Parse whatever we have until we have enough.
    while (!done) {

        // Tokenize the user's input.  The default whitespace delimiters are
        // just fine for us.
        stringstream stream(s);
        string token;
        while (stream >> token) {

            // Is this a boolean?  If so, it satisfies our requirements for
            // having SolutionFound.
            string tokenInUppercase;
            transform(token.cbegin(),
                      token.cend(),
                      back_inserter<string>(tokenInUppercase),
                      static_cast<int (*)(int)>(toupper));

            if (tokenInUppercase == "TRUE") {
                haveSolutionFound = true;
                solutionFound = true;
                continue;
            } else if (tokenInUppercase == "FALSE") {
                haveSolutionFound = true;
                solutionFound = false;
                continue;
            }

            // If this the string "peg" or the string "boiler"?  If so, it
            // satisfies our requirements for SolutionType.
            if (tokenInUppercase == "BOILER") {
                haveSolutionType = true;
                solutionType = PapasVision::Boiler;
                continue;
            } else if (tokenInUppercase == "PEG") {
                haveSolutionType = true;
                solutionType = PapasVision::Peg;
                continue;
            }

            stringstream tokenStream(token);
            double d;
            if (tokenStream >> d) {
                if (!havePapasDistance) {
                    // If the token parses as a double and we don't have a
                    // PapasDistance yet, then it satisfies our requirements
                    // for PapasDistance.
                    papasDistance = d;
                    havePapasDistance = true;
                } else if (!havePapasAngle) {
                    // If the token parses as a double and we don't have a
                    // PapasAngle yet, then it satisfies our requirements for
                    // PapasAngle.
                    papasAngle = d;
                    havePapasAngle = true;
                } else {
                    cout << "Discarding unnecessary floating-point token: " << token << "\n";
                }
                continue;
            }

            // A token that made it here is junk.
            cout << "Discarding unrecognized token: \"" << token << "\"\n";
        }

        // Have we met our requirements?
        if (haveSolutionFound) {
            if (solutionFound == false) {
                // A single boolean for SolutionFound is enough...if it's
                // false.  We'll fill in the missing details.
                if (!haveSolutionType) {
                    solutionType = PapasVision::Boiler;
                }
                if (!havePapasDistance) { papasDistance = -1.0; }
                if (!havePapasAngle) { papasAngle = 0.0; }
                done = true;
            } else {
                // If a solution's found, we expect at a minimum the solution
                // type and the distance.
                if (haveSolutionType && havePapasDistance) {
                    if (!havePapasAngle) { papasAngle = 0.0; }
                    done = true;
                }
            }
        } else /* haveSolutionFound == false */ {
            // We can infer that a solution was found if we have a solution
            // type and a PapasDistance.
            if (haveSolutionType && havePapasDistance) {
                if (!havePapasAngle) { papasAngle = 0.0; }
                done = true;
            }
        }

        if (!done) {
            // Prompt the user for the remaining missing information, storing
            // their input into s.
            cout << "Please enter ";

            vector<string> missingOptions;
            if (!haveSolutionFound) {
                missingOptions.push_back("SolutionFound (a boolean)");
            }
            if (!haveSolutionType) {
                missingOptions.push_back("SolutionType (either 'boiler' or 'peg')");
            }
            if (!havePapasDistance) {
                missingOptions.push_back("PapasDistance (a double)");
            }
            if (!havePapasAngle) {
                missingOptions.push_back("PapasAngle (a double)");
            }

            if (missingOptions.size() == 1) {
                cout << missingOptions[0];
            } else if (missingOptions.size() == 2) {
                cout << missingOptions[0] << " and/or " << missingOptions[1];
            } else {
                for (unsigned i = 0; i < missingOptions.size(); ++i) {
                    cout << missingOptions[i];
                    if (i < missingOptions.size() - 2) {
                        cout << ", ";
                    } else if (i == missingOptions.size() - 2) {
                        cout << ", and/or ";
                    }
                }
            }

            cout << ": ";
            getline(cin, s);
        }
    } // end (while not done)

    return CameraMessage(solutionFound, solutionType, papasDistance, papasAngle);
}


// =========================================================================
// Runs the camera code, constructs messages from it, and transmits those
// messages remotely as long as there are messages to transmit and something
// weird doesn't happen.

void mainLoop(const Config &config)
{
    // The RemoteTransmitter will shut the thread down when it goes out of scope.
    RemoteTransmitter transmitter(config);

    // The object that will find computer vision solutions for us.
    const bool writeIntermediateFilesToDisk = false;
    PapasVision papasVision(config, GOAL_REJECTION_THRESHOLD_INCHES, writeIntermediateFilesToDisk);
    PapasVision::SolutionType solutionType;

    stringstream stream;
    stream << "mainLoop: Camera client ready!  Will operate for "
           << TIME_LIMIT_SECONDS << " seconds.";
    transmitter.buffer().logMessage(TransmissionBuffer::debug, stream.str());

    // TODO: Get the average time for peg and boiler solutions.
    auto start = high_resolution_clock::now();
    int counter = 0;
    int boilerSolutionsFound = 0, pegSolutionsFound = 0;
    bool done = false;

    while (!done)
    {
        double elapsedSeconds = duration<double>(high_resolution_clock::now() - start).count();
        if (elapsedSeconds >= TIME_LIMIT_SECONDS)
        {
            done = true;
        }

        // Ensure that the log messages aren't too spammy.
        std::this_thread::sleep_for(milliseconds(200));

        // Run the PapasVision detector.
        if (counter++ % 2 == 0) {
            solutionType = PapasVision::Boiler;
            papasVision.findBoiler();
        } else {
            solutionType = PapasVision::Peg;
            papasVision.findPeg();
        }

        // Send the PapasVision results out.
        if (papasVision.getSolutionFound())
        {
            if (solutionType == PapasVision::Peg) {
                ++pegSolutionsFound;
            } else {
                ++boilerSolutionsFound;
            }

            double papasDistance = papasVision.getDistToGoalInch();
            double papasAngle = papasVision.getAzimuthGoalDeg();
            PapasVision::SolutionType solutionType = PapasVision::Boiler;

            // CC successful solutions to the driver station for debugging
            // purposes.
            stringstream stream;
            stream << (solutionType == PapasVision::Boiler ? "Boiler" : "Peg")
                   << " solution found: (distance="
                   << setprecision(5) << papasDistance << " inches, angle="
                   << papasAngle << " degrees)\n";
            transmitter.buffer().logMessage(TransmissionBuffer::camera, stream.str());

            // Transmit to the robot.
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
    } // end (while not done)


    transmitter.buffer().logMessage(TransmissionBuffer::debug, "mainLoop: Time's up!  Shutting down the camera client.");

    stream << "mainLoop: " << boilerSolutionsFound << " Boiler and "
           << pegSolutionsFound << "Peg solution(s) found.";
    transmitter.buffer().logMessage(TransmissionBuffer::debug, stream.str());
}
