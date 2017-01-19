// MAIN.CPP
//
// Entry point for the program that the Nvidia Jetson TK1 will be excuting at
// boot time.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

namespace p = boost::program_options;
using std::string;
using std::vector;
using std::ifstream;
using std::cout;
using std::cerr;

int main() {
    cv::Mat matrix;
    cout << "OpenCV version " << CV_VERSION << " ready!\n";

    // Read the config file to test Boost.Program-Options.
    // TODO: Wrap this into a ConfigFile class so no one else is exposed to
    // these horrors.
    string config_file_path = "config/camera-client.ini";
    ifstream config_file(config_file_path);
    if (!config_file) {
        cerr << "Error: config file \"" << config_file_path << "\" could not be opened.  It should be part of the Git repository.\n";
        return 1;
    }

    auto options = p::options_description("Config settings");

    // Note that if we want to use the variables_map (and trust me, you do),
    // then we have no choice but to register the expected config file options
    // first.  I'm not too happy about that.
    options.add_options()("network.roborio_addresses",
                          p::value<vector<string> >(),
                          "This is the comma-separated list of IP addresses and DNS names that we try until we get a match for a live RoboRIO.  If we can't find any of these, camera-client is a client without a server and it gives up.");

    auto parser = p::parse_config_file(config_file, options, /* allow_unregistered */ true);
    auto vm = p::variables_map();
    p::store(parser, vm);
    p::notify(vm);

    // BUG: It's not parsing the commas -- the size of this vector<string>
    // should be 2, not 1.  Will fix layer.
    const string expected_setting_name = "network.roborio_addresses";
    if (vm.count(expected_setting_name) > 0) {
        vector<string> address_list = vm[expected_setting_name].as<vector<string> >();
        cout << "RoboRIO IP address list from config file \""
             << config_file_path << "\": [";
        for (auto iter = address_list.begin(); iter != address_list.end(); ++iter) {
            cout << *iter << ", ";
        }
        cout << "]\n";
    }
}
