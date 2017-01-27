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
#include <iterator>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace p = boost::program_options;
using std::ostream_iterator;
using std::exception;
using std::string;
using std::vector;
using std::copy;
using std::cout;
using std::cerr;

using namespace robot;

int main() {
    cv::Mat matrix;
    cout << "OpenCV version " << CV_VERSION << " ready!\n";

    try {
        Config config;
        PapasVision papasVision(config, 0.0, true);

        cout << "RoboRIO IP address list from config file \""
             << config.path() << "\": ";

        vector<string> address_list = config.robotAddresses();
        copy(address_list.begin(), address_list.end(), ostream_iterator<string>(cout, ", "));
        cout << "\n";

        RemoteTransmitter transmitter(config);
        HeartbeatMessage message;
        transmitter.enqueueMessage(message);
        cout << "\n";

        papasVision.findGoal(1); // Sample images are ./samples/{1..8}.png

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
