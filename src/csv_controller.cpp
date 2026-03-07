#include "csv_controller/csv_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "rclcpp/parameter.hpp"

#include <iomanip> // For setting precision

namespace csv_controller
{
    CSVController::CSVController()
        : controller_interface::ControllerInterface()
    {
    }

    void CSVController::declare_parameters()
    {
        param_listener_ = std::make_shared<ParamListener>(get_node());
    }

    controller_interface::CallbackReturn CSVController::read_parameters()
    {
        if (!param_listener_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
            return controller_interface::CallbackReturn::ERROR;
        }
        params_ = param_listener_->get_params();
        RCLCPP_INFO(get_node()->get_logger(), "read_parameters() csv_name: %s", params_.csv_name.c_str());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CSVController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces;
        command_interfaces.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces.names.push_back("shoulder_pan_joint/position");
        command_interfaces.names.push_back("shoulder_lift_joint/position");
        command_interfaces.names.push_back("elbow_joint/position");
        command_interfaces.names.push_back("wrist_1_joint/position");
        command_interfaces.names.push_back("wrist_2_joint/position");
        command_interfaces.names.push_back("wrist_3_joint/position");
        return command_interfaces;
    }

    controller_interface::InterfaceConfiguration CSVController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces;
        state_interfaces.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        state_interfaces.names.push_back("shoulder_pan_joint/position");
        state_interfaces.names.push_back("shoulder_lift_joint/position");
        state_interfaces.names.push_back("elbow_joint/position");
        state_interfaces.names.push_back("wrist_1_joint/position");
        state_interfaces.names.push_back("wrist_2_joint/position");
        state_interfaces.names.push_back("wrist_3_joint/position");
        return state_interfaces;
    }

    controller_interface::CallbackReturn CSVController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_init() beginning...");

        urscript_publisher = get_node()->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
        io_client = get_node()->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

        try
        {
            declare_parameters();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        auto ret = this->read_parameters();
        if (ret != controller_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }

        std::string csv_name = params_.csv_name;

        // read the CSV file and store into an array
        std::string filename = ament_index_cpp::get_package_share_directory("csv_controller") +
                               std::string("/csv_files/") + csv_name + ".csv";
        if (!readCSVFile(filename))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error reading CSV file");
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "on_init() successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CSVController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_node()->get_logger(), "on_configure() successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CSVController::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        startTime = 0.0;
        index = 0;

        // turn off LED
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun = 1;     // Set the function (1 to set digital output)
        request->pin = 16;    // Set the pin number
        request->state = 0.0; // Set the state (0 for OFF)
        io_client->async_send_request(request);

        RCLCPP_INFO(get_node()->get_logger(), "on_activate() successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CSVController::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate() successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CSVController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // (void)time;
        (void)period;

        // set initial start time
        if (startTime == 0.0)
        {
            startTime = time.seconds();
        }

        // ensure that the joints are "close enough" to the start of the next position
        double distance = distanceToCSVPosition(index);
        if (distance < 0.2)
        {
            double t = time.seconds() - startTime;
            if (t < waitTime)
            {
                RCLCPP_INFO(get_node()->get_logger(), "\033[31m time left: %1.2f \033[0m", waitTime - t);
                return controller_interface::return_type::OK;
            }

            command_interfaces_[0].set_value(csvData[index].shoulderPan);
            command_interfaces_[1].set_value(csvData[index].shoulderLift);
            command_interfaces_[2].set_value(csvData[index].elbow);
            command_interfaces_[3].set_value(csvData[index].wrist1);
            command_interfaces_[4].set_value(csvData[index].wrist2);
            command_interfaces_[5].set_value(csvData[index].wrist3);

            // if the LED value has changed this index, send a request to the IO service
            if (index == csvData.size() - 1)
            {
                auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
                request->fun = 1;     // Set the function (1 to set digital output)
                request->pin = 16;    // Set the pin number
                request->state = 0.0; // Set the state (0 for OFF)
                io_client->async_send_request(request);
            }
            else if (index == 0 || csvData[index].led != csvData[index - 1].led)
            {
                auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
                request->fun = 1;  // Set the function (1 to set digital output)
                request->pin = 16; // Set the pin number
                request->state = (float)csvData[index].led;
                io_client->async_send_request(request);
            }

            // update the index
            index = std::min(index + 1, csvData.size() - 1);
        }
        else
        {
            if (haveSentMoveJCommand)
            {
                return controller_interface::return_type::ERROR;
            }
            // the distance between current joint position and the requested joint position is too great
            // we will deactivate the controller and wait for the robot to reach the desired position
            RCLCPP_INFO(get_node()->get_logger(), "distance %f too great, remaining at index %li", distance, index);
            //
            auto message = std::make_shared<std_msgs::msg::String>();
            std::ostringstream ur_script_command;
            ur_script_command << "movej(["
                              << std::fixed << std::setprecision(12) << csvData[index].shoulderPan << ", "
                              << std::fixed << std::setprecision(12) << csvData[index].shoulderLift << ", "
                              << std::fixed << std::setprecision(12) << csvData[index].elbow << ", "
                              << std::fixed << std::setprecision(12) << csvData[index].wrist1 << ", "
                              << std::fixed << std::setprecision(12) << csvData[index].wrist2 << ", "
                              << std::fixed << std::setprecision(12) << csvData[index].wrist3 << "])";

            // Set the data field of the message with the formatted string
            message->data = ur_script_command.str();
            urscript_publisher->publish(*message);
            waitTime = 10.0;
            startTime = time.seconds();

            // turn off LED
            auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
            request->fun = 1;     // Set the function (1 to set digital output)
            request->pin = 16;    // Set the pin number
            request->state = 0.0; // Set the state (0 for OFF)
            io_client->async_send_request(request);

            haveSentMoveJCommand = true;

            return controller_interface::return_type::ERROR;
        }

        return controller_interface::return_type::OK;
    }

    bool CSVController::readCSVFile(const std::string &filename)
    {
        std::ifstream file(filename);

        if (!file)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error opening file %s", filename.c_str());
            return false;
        }

        std::string line, word;
        // discard the first line
        std::getline(file, line);

        while (std::getline(file, line))
        {

            CSVLine csvLine;
            std::stringstream s(line); // Convert the line into a stringstream

            // std::cout << s.str() << std::endl;

            std::getline(s, word, ',');
            csvLine.time = std::stod(word);

            std::getline(s, word, ',');
            csvLine.shoulderPan = std::stod(word);

            std::getline(s, word, ',');
            csvLine.shoulderLift = std::stod(word);

            std::getline(s, word, ',');
            csvLine.elbow = std::stod(word);

            std::getline(s, word, ',');
            csvLine.wrist1 = std::stod(word);

            std::getline(s, word, ',');
            csvLine.wrist2 = std::stod(word);

            std::getline(s, word, ',');
            csvLine.wrist3 = std::stod(word);

            std::getline(s, word, ',');
            try
            {
                csvLine.led = std::stoul(word);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "foobar %s", e.what());
                csvLine.led = (unsigned long)std::stod(word);
            }

            csvData.push_back(csvLine);
        }

        return true;
    }

    // return the distance from the current position
    // to the start of the next position
    double CSVController::distanceToCSVPosition(size_t index)
    {
        Eigen::Matrix<double, 6, 1> currentPos;
        Eigen::Matrix<double, 6, 1> startPos;

        currentPos << state_interfaces_[0].get_value(),
            state_interfaces_[1].get_value(),
            state_interfaces_[2].get_value(),
            state_interfaces_[3].get_value(),
            state_interfaces_[4].get_value(),
            state_interfaces_[5].get_value();

        startPos << csvData[index].shoulderPan,
            csvData[index].shoulderLift,
            csvData[index].elbow,
            csvData[index].wrist1,
            csvData[index].wrist2,
            csvData[index].wrist3;

        return (currentPos - startPos).norm();
    }

} // namespace csv_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    csv_controller::CSVController, controller_interface::ControllerInterface)