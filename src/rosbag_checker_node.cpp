#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/info.hpp"
#include "yaml-cpp/yaml.h"
#include <regex>

#define GREEN "\033[1;32;48m"
#define YELLOW "\033[1;33;48m"
#define RED "\033[1;31;48m"
#define COLOR_END "\033[1;37;0m"

class RosbagCheckerNode: public rclcpp::Node 
{
public:
    RosbagCheckerNode(): Node("rosbag_checker")
    {
        this->declare_parameter("bag_file", rclcpp::PARAMETER_STRING);
        this->declare_parameter("topic_list", rclcpp::PARAMETER_STRING);
        this->declare_parameter("topics", rclcpp::PARAMETER_STRING);
        this->declare_parameter("check_frequency", true);
        this->declare_parameter("default_frequency_requirements", std::vector<double>({-1, std::numeric_limits<double>::max()}));

        try{
            bag_ = this->get_parameter("bag_file").as_string();
        }   
        catch(...){
            RCLCPP_ERROR(this->get_logger(), "Please set parameter bag_file to path of rosbag file");
        }    
        try{
            topic_list_ = this->get_parameter("topic_list").as_string();
            use_yaml_ = true;
            RCLCPP_INFO(this->get_logger(), "Using input yaml file as topic list");
        }   
        catch(...){
            use_yaml_ = false;
            RCLCPP_WARN(this->get_logger(), "No input yaml file specified");
            try{
                topic_re_ = this->get_parameter("topics").as_string();
                RCLCPP_INFO(this->get_logger(), ("Checking the topic name or the topics that match the regular expression" + topic_re_).c_str()); // TODO: Do I need to change type of topic_re_? for example, .c_str()?
            }   
            catch(...){
                RCLCPP_ERROR(this->get_logger(), "Please give input yaml file or specify topics to check manually");
            }    
        }     

        check_frequency_ = this->get_parameter("check_frequency").as_bool();
        if (check_frequency_){
            RCLCPP_INFO(this->get_logger(), "Including check for frequency requirements");
        }

        frequency_requirements_ = this->get_parameter("default_frequency_requirements").as_double_array();

        check_bag();
    }

    void check_bag(){
        std::map<std::string, std::vector<double>> topics_to_rate;
        if (use_yaml_){ // 1a. Get list of topics to check and their rate requirements from yaml file
            YAML::Node topic_list = YAML::LoadFile(topic_list_);

            const YAML::Node& topics = topic_list["topics"];
            
            for (YAML::const_iterator it = topics.begin(); it != topics.end(); ++it) {
                const YAML::Node& topic = *it;
                // std::cout << "name: " << topic["name"].as<std::string>() << "\n";
                // std::cout << "hz_range: " << topic["hz_range"].as<std::string>() << "\n";

                auto topic_name = topic["name"].as<std::string>();
                std::vector<double> hz_range;
                try {
                    hz_range = topic["hz_range"].as<std::vector<double>>();
                }
                catch(...) {
                    hz_range = frequency_requirements_;
                }
                // topics_to_rate.insert({"testing insert", std::vector<double>({0.0, 0.0})});
                topics_to_rate.insert({topic_name, hz_range});
            }
            // RCLCPP_INFO(this->get_logger(), "Finished constructing topics_to_rate");

        }
        else{ // 1b. Get regex cli parameter
            topics_to_rate.insert({topic_re_, frequency_requirements_});
        }

        // 2. Read rosbag and get duration of rosbag GOOD
        auto info_obj = rosbag2_cpp::Info();

        rosbag2_storage::BagMetadata metadata;
        if (ends_with(bag_, ".db3")) {
            metadata = info_obj.read_metadata(bag_, "sqlite3");
        }
        else if (ends_with(bag_, ".mcap")) {
            metadata = info_obj.read_metadata(bag_, "mcap");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Please submit a rosbag in sqlite3 or mcap format");
        }

        auto duration = metadata.duration.count()/1000000000.0;
        RCLCPP_INFO(this->get_logger(), "rosbag duration = %f", duration);

        // 3. Loop through topic data and gather output string
        std::stringstream output_stream;
        for (std::map<std::string, std::vector<double>>::iterator it = topics_to_rate.begin(); it != topics_to_rate.end(); ++it){
            std::string topic = it->first;
            auto hz_range = it->second;
            // RCLCPP_INFO(this->get_logger(), "Iterating...now on topic %s", topic.c_str());
            bool found_match = false;
            for (auto topic_info : metadata.topics_with_message_count){
                std::regex re(topic);
                if (std::regex_match(topic_info.topic_metadata.name, re)) {
                    // RCLCPP_INFO(this->get_logger(), "Found match!");
                    found_match = true;
                    auto color_to_use = GREEN;
                    auto msg_count = topic_info.message_count;
                    auto msg_rate = msg_count/duration;
                    
                    if (msg_count == 0) {
                        color_to_use = RED;
                    }
                    else if (check_frequency_) {
                        auto min_rate = hz_range[0];
                        auto max_rate = hz_range[1];
                        if (msg_rate < min_rate || msg_rate > max_rate){
                            color_to_use = YELLOW;
                        }
                    }
                    
                    output_stream << color_to_use << "Statistics for topic " << topic_info.topic_metadata.name << "\n" << "Message count = " << msg_count << ", Message frequency = " << msg_rate << COLOR_END << "\n\n";
                }
            }

            if (!found_match) {
                output_stream << RED << "Statistics for topic " << topic << "\n" << "Message count = " << 0 << ", Message frequency = " << 0 << COLOR_END << "\n\n";
            }
        }

        // 4. Output final results
        RCLCPP_INFO(this->get_logger(), "Results: \n%s", output_stream.str().c_str());
    }

    bool ends_with(std::string s, std::string suffix) {
        return s.length() >= suffix.length() && !s.compare(s.length() - suffix.length(), suffix.length(), suffix);
    }
    

private:
    std::string bag_;
    std::string topic_list_;
    bool use_yaml_;
    std::string topic_re_;
    bool check_frequency_;
    std::vector<double> frequency_requirements_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosbagCheckerNode>(); 
    rclcpp::shutdown();
    return 0;
}