#include "Config.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>

namespace p = boost::program_options;
using std::runtime_error;
using std::stringstream;
using std::ifstream;
using std::getline;
using std::string;
using std::vector;
using std::cout;
using std::cerr;

namespace robot {


// Boost.ProgramOptions has a convention of using "foo.bar" for config
// options, where "foo" is the (optional) INI section header, and "bar" is the
// actual option name.

const char* robotAddressesKey         = "network.roborio_addresses";
const char* robotPortKey              = "network.roborio_port";
const char* driverStationAddressesKey = "network.driver_station_addresses";
const char* driverStationPortKey      = "network.driver_station_port";

// =========================================================================
// Constructor: Read from the config file and prepare the variables_map.

Config::Config(string configFilePath) :
    vm(p::variables_map()), path_(configFilePath) {

    ifstream configFileStream(configFilePath);
    if (!configFileStream) {
        stringstream message;
        message << "Config file \"" << configFilePath << "\" could not be opened for reading.";
        throw runtime_error(message.str());
    }

    auto options = p::options_description("Config settings");

    // Register the options we expect.
    //
    // This seems counter-intuitive, doesn't it?  After all, config files are
    // a runtime thing, not a hard-coded compile-time thing.  The only reason
    // we're doing things this way is because using the variables_map is
    // easier than not using it (and subsequently having to deal with the
    // collect_unrecognized() method and its nonsense.)

    options.add_options()
        (robotAddressesKey, p::value<vector<string> >())
        (robotPortKey, p::value<int>());

    // We don't actually use any unregistered options, but we don't want
    // Boost.ProgramOptions throwing exceptions just because someone added
    // something to the config file that we weren't expecting to see.
    bool allowUnregistered = true;

    auto parser = p::parse_config_file(configFileStream, options, allowUnregistered);
    p::store(parser, vm);
    p::notify(vm);
}


// =========================================================================
// Returns the path we read the config file from.  We would have thrown an
// exception if we weren't able to read from it, so this path is likely to
// exist.

string Config::path() const { return path_; }


// =========================================================================
// Returns the server list from the config file.  We'll only connect to one of
// them, but the C++ codebase has no way of knowing which one is correct (is
// the robot connected via Wi-Fi?  mDNS?  USB-A cable?) so we try them one
// after the other.

vector<string> Config::robotAddresses() const {

    vector<string> addressList;

    if (vm.count(robotAddressesKey) > 0) {

        // BUG: It's not parsing the commas -- the size of this vector<string>
        // should be 2, not 1.  Will fix later; for now, we split it
        // ourselves.

        // addressList = vm[robotAddressesKey].as<vector<string> >();

        stringstream stream(vm[robotAddressesKey].as<vector<string> >()[0]);
        string address;
        while (getline(stream, address, ',')) {
            addressList.push_back(address);
        }

    } else {

        // If control makes it here, the key is present, but the value is
        // missing.
        stringstream message;
        message << "Expected the \"" << robotAddressesKey << "\" option from \""
                << path_
                << "\" to contain a comma-separated list of DNS names or IP addresses, but there was nothing there.";
        throw runtime_error(message.str());
    }

    if (addressList.size() == 0) {
        // I'm not sure how control could make it here, but just in case.
        stringstream message;
        message << "Expected the \"" << robotAddressesKey << "\" option from \""
                << path_
                << "\" to be a comma-separated list of one or more DNS names or IP addresses, not \""
                << vm[robotAddressesKey].as<vector<string> >()[0] << "\".";
        throw runtime_error(message.str());
    }

    return addressList;
}


// =========================================================================
// Returns the RoboRIO port from the config file.

int Config::robotPort() const {

    if (vm.count(robotPortKey) > 0) {
        return vm[robotAddressesKey].as<int>();
    }

    // If control makes it here, the key is present, but the value is missing.
    stringstream message;
    message << "Expected the \"" << robotPortKey << "\" option from \"" << path_
            << "\" to contain an integer, but there was nothing there.";
    throw runtime_error(message.str());
}


} // end (namespace robot)
