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
        // try when connecting to the server.
        std::vector<std::string> serverAddresses() const;

        // Returns the port that the server is expected to be listening on.
        int serverPort() const;

    private:
        // This object "receives" all of the settings that the
        // config_file_parser reads from the config file.  The accessor
        // functions then make sense of what's in here.
        boost::program_options::variables_map vm;

        // Remembers the name of the config file we read for the purposes of
        // generating error messages.
        std::string path_;
};

} // end (namespace robot)

#endif // (#ifndef CONFIG_H__)
