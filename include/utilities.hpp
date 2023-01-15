#pragma once
#include <string>
#include <carla/client/Client.h>

namespace sdv {
    struct cmdline_args_s {
        cmdline_args_s(int _argc, char **_argv) {
            argc = _argc;
            argv = new char*[argc];
            argv = _argv;
        }
        int argc;
        char **argv;
    };

    struct world_config_s {
        std::string map_name;
        carla::rpc::WeatherParameters weather = carla::rpc::WeatherParameters::Default;
    };
};