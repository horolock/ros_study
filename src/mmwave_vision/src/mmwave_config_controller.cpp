#include <string>
#include <fstream>
#include <regex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class MMWaveConfigControllerNode: public rclcpp::Node
{
public:
    MMWaveConfigControllerNode(): Node("mmwave_config_controller") {
        this->declare_parameter("/ti_mmwave/startFreq", "0");
        this->declare_parameter("/ti_mmwave/idleTime", "0");
        this->declare_parameter("/ti_mmwave/adcStartTime", "0");
        this->declare_parameter("/ti_mmwave/rampEndTime", "0");
        this->declare_parameter("/ti_mmwave/freqSlopeConst", "0");
        this->declare_parameter("/ti_mmwave/numAdcSamples", "0");
        this->declare_parameter("/ti_mmwave/digOutSampleRate", "0");
        this->declare_parameter("/ti_mmwave/rxGain", "0");
        this->declare_parameter("/ti_mmwave/chirpStartIdx", "0");
        this->declare_parameter("/ti_mmwave/chirpEndIdx", "0");
        this->declare_parameter("/ti_mmwave/numLoops", "0");
        this->declare_parameter("/ti_mmwave/numFrames", "0");
        this->declare_parameter("/ti_mmwave/framePeriodicity", "0");
        this->declare_parameter("/ti_mmwave/zoneMinX", "0");
        this->declare_parameter("/ti_mmwave/zoneMaxX", "0");
        this->declare_parameter("/ti_mmwave/zoneMinY", "0");
        this->declare_parameter("/ti_mmwave/zoneMaxY", "0");
        this->declare_parameter("/ti_mmwave/zoneMinZ", "0");
        this->declare_parameter("/ti_mmwave/zoneMaxZ", "0");

        std::ifstream config_file;
        std::string config_file_path = "/home/hojoon/ros_study/src/mmwave_vision/config/6843ISK_Tracking.cfg";
        std::string config_line;
        std::string token;
        
        config_file.open(config_file_path);

        if (config_file.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Successfully opened config file.");

            /* read line by line */
            while (std::getline(config_file, config_line)) {
                config_line.erase(std::remove(config_line.begin(), config_line.end(), '\r'), config_line.end());
                if (!(std::regex_match(config_line, std::regex("^\\s*%.*")) || std::regex_match(config_line, std::regex("^\\s*")))) {
                    std::istringstream ss(config_line);
                    std::vector<std::string> splited_config;

                    while (std::getline(ss, token, ' ')) { splited_config.emplace_back(token); }

                    /**
                     * profileCfg
                     * frameCfg
                     * zoneDef
                     */
                    if (!splited_config[0].compare("profileCfg")) {
                        std::vector<rclcpp::Parameter> params {
                            rclcpp::Parameter("/ti_mmwave/startFreq", splited_config[2]),
                            rclcpp::Parameter("/ti_mmwave/idleTime", splited_config[3]),
                            rclcpp::Parameter("/ti_mmwave/adcStartTime", splited_config[4]),
                            rclcpp::Parameter("/ti_mmwave/rampEndTime", splited_config[5]),
                            rclcpp::Parameter("/ti_mmwave/freqSlopeConst", splited_config[8]),
                            rclcpp::Parameter("/ti_mmwave/numAdcSamples", splited_config[10]),
                            rclcpp::Parameter("/ti_mmwave/digOutSampleRate", splited_config[11]),
                            rclcpp::Parameter("/ti_mmwave/rxGain", splited_config[14])
                        };

                        this->set_parameters(params);
                    } else if (!splited_config[0].compare("frameCfg")) {
                        std::vector<rclcpp::Parameter> params {
                            rclcpp::Parameter("/ti_mmwave/chirpStartIdx", splited_config[1]),
                            rclcpp::Parameter("/ti_mmwave/chirpEndIdx", splited_config[2]),
                            rclcpp::Parameter("/ti_mmwave/numLoops", splited_config[3]),
                            rclcpp::Parameter("/ti_mmwave/numFrames", splited_config[4]),
                            rclcpp::Parameter("/ti_mmwave/framePeriodicity", splited_config[5])
                        };

                        this->set_parameters(params);
                    } else if (!splited_config[0].compare("zoneDef")) {
                        std::vector<rclcpp::Parameter> params {
                            rclcpp::Parameter("/ti_mmwave/zoneMinX", splited_config[2]),
                            rclcpp::Parameter("/ti_mmwave/zoneMaxX", splited_config[3]),
                            rclcpp::Parameter("/ti_mmwave/zoneMinY", splited_config[4]),
                            rclcpp::Parameter("/ti_mmwave/zoneMaxY", splited_config[5]),
                            rclcpp::Parameter("/ti_mmwave/zoneMinZ", splited_config[6]),
                            rclcpp::Parameter("/ti_mmwave/zoneMaxZ", splited_config[7])
                        };

                        this->set_parameters(params);
                    }
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open config file...");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        config_file.close();
    }

private:
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MMWaveConfigControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}