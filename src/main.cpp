#include <commons.hpp>
#include <carla_client.hpp>
#include <dds_channel.hpp>

#include <csignal>

std::atomic<bool> terminate_flag{false};

void signal_handler(int _sigid) {
    terminate_flag = true;
}
int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);

    char *local_argv[3] = {__FILE__,"-DCPSConfigFile","../rtps.ini"};
    cmdline_args_s clargs(3, local_argv);   

    world_config_s config;
    config.map_name = "Town10";

    carla_client cc(config);
    cc.setup_world();
    dds_channel channel1(cc,clargs);

    while(terminate_flag == false) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
