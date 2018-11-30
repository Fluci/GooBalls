#include "cli_options.hpp"

namespace GooBalls {

namespace op = boost::program_options;

op::options_description cli_options() {
    op::options_description desc{"Options"};
    auto ad = desc.add_options();

    ad("help,h", "Help screen");
    ad("src,s", op::value<std::string>(), "Scene to display");
    ad("log,l", op::value<std::string>()->default_value("info"), "Set lowest log level to show. Possible options: trace, debug, info, warning, error, fatal, none. Default: info");

    return desc;
}

} // GooBalls
