#include "cli_options.hpp"

namespace GooBalls {

namespace op = boost::program_options;

op::options_description cli_options() {
    op::options_description desc{"Options"};
    auto ad = desc.add_options();

    ad("help,h", "Help screen");
    ad("src,s", op::value<std::string>(), "Scene to display");
    ad("log,l", op::value<std::string>()->default_value("info"), "Set lowest log level to show. Possible options: trace, debug, info, warning, error, fatal, none. Default: info");
    ad("fluid-solver", op::value<std::string>()->default_value("viscoElastic"), "Set the fluid solver: ssph, viscoElastic");
    ad("time-step", op::value<double>()->default_value(0.001), "Set dt used for one integration step. Default: 0.001");
    ad("pause,p", "True: start the simulation in pause mode, false: run as expected, default: false");
    ad("styler", op::value<std::string>()->default_value("noStyle"), "Should a special styler be applied to change the appearance of scene objects dynamically? default: noStyle, options: pressureDensity");
    ad("scene-max-frames", op::value<int>(), "Maximum number of frames to animate.");
    ad("scene-max-seconds", op::value<double>(), "Maximum number of seconds of physic to animate.");

    return desc;
}

} // GooBalls
