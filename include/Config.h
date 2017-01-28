#ifndef CONFIG_H__
#define CONFIG_H__

// Header file for our simple config file wrapper.  There are only two or
// three settings that matter here.

#include <string>
#include <vector>

#include <boost/program_options/variables_map.hpp>

namespace robot {

class Config {

    public:

        // Initializes this object by reading from a config file.
        // If the file doesn't exist, we throw an exception and the program
        // will exit.
        Config(std::string configFilePath = "./config/camera-client.ini");

        // Returns the path we loaded the file from.
        std::string path() const;

        // -----------------------------------------
        // Accessors for the configuration settings.
        // -----------------------------------------

        // Returns the list of IP addresses and/or hostnames that we should
        // try when connecting to the Java listening thread on the RoboRIO.
        std::vector<std::string> robotAddresses() const;

        // Returns the port that the RoboRIO's Java listening thread is
        // expected to be listening on.
        int robotPort() const;

        // Returns the list of IP addresses and?or host names that we should
        // try when connecting to the driver station monitor running on the
        // driver station laptop.
        std::vector<std::string> driverStationAddresses() const;

        // Returns the port that the driver station monitor is expected to be
        // listening on.
        int driverStationPort() const;

        // Returns the port that we expect the driver station monitor to be
        // listening on.
        std::string cameraFolder() const;

    private:
        // This object "receives" all of the settings that the
        // config_file_parser reads from the config file.  The accessor
        // functions then make sense of what's in here.
        boost::program_options::variables_map vm;

        // Remembers the name of the config file we read for the purposes of
        // generating error messages.
        std::string path_;

        // Helper functions to save myself some typing.
        std::vector<std::string> getCommaSeparatedListFromConfig(const std::string& key) const;
        int getIntFromConfig(const std::string& key) const;
};

} // end (namespace robot)

#endif // (#ifndef CONFIG_H__)
