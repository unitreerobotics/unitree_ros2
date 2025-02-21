/**
 * This example demonstrates how to use ROS2 to call loco client api of unitree g1 robot
 **/
#include "client/g1/g1_loco_client.hpp" 

using namespace unitree::ros2::g1;

void ParseArgv(int argc, char **argv, std::map<std::string, std::string> &args)
{
    std::map<std::string, std::string> values;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.substr(0, 2) == "--")
        {
            size_t pos = arg.find("=");
            std::string key, value;
            if (pos != std::string::npos)
            {
                key = arg.substr(2, pos - 2);
                value = arg.substr(pos + 1);

                if (value.front() == '"' && value.back() == '"')
                {
                    value = value.substr(1, value.length() - 2);
                }
            }
            else
            {
                key = arg.substr(2);
                value = "";
            }
            if (args.find(key) != args.end())
            {
                args[key] = value;
            }
            else
            {
                args.insert({{key, value}});
            }
        }
    }
}

std::vector<float> StringToFloatVector(const std::string &str)
{
    std::vector<float> result;
    std::stringstream ss(str);
    float num;
    while (ss >> num)
    {
        result.push_back(num);
        // ignore any trailing whitespace
        ss.ignore();
    }
    return result;
}

int32_t DoWork(const std::shared_ptr<LocoClient> &client, const std::map<std::string, std::string> &args)
{
    // return 0 if success
    int32_t ret = 0;
    for (const auto &arg_pair : args)
    {
        std::cout << "Processing command: [" << arg_pair.first << "] with param: ["
                  << arg_pair.second << "] ..." << std::endl;

        if (arg_pair.first == "get_fsm_id")
        {
            int32_t fsm_id;
            ret = client->GetFsmId(fsm_id);
            std::cout << "ret : " << ret << " , current fsm_id: " << fsm_id << std::endl;
        }
        else if (arg_pair.first == "get_fsm_mode")
        {
            int32_t fsm_mode;
            ret = client->GetFsmMode(fsm_mode);
            std::cout << "ret : " << ret << " , current fsm_mode: " << fsm_mode << std::endl;
        }
        else if (arg_pair.first == "get_balance_mode")
        {
            int32_t balance_mode;
            ret = client->GetBalanceMode(balance_mode);
            std::cout << "ret : " << ret << " , current balance_mode: " << balance_mode << std::endl;
        }
        else if (arg_pair.first == "get_swing_height")
        {
            float swing_height;
            ret = client->GetSwingHeight(swing_height);
            std::cout << "ret : " << ret << " , current swing_height: " << swing_height << std::endl;
        }
        else if (arg_pair.first == "get_stand_height")
        {
            float stand_height;
            ret = client->GetStandHeight(stand_height);
            std::cout << "ret : " << ret << " , current stand_height: " << stand_height << std::endl;
        }
        else if (arg_pair.first == "get_phase")
        {
            std::vector<float> phase;
            ret = client->GetPhase(phase);

            std::cout << "ret : " << ret << " , current phase: (";
            for (const auto &p : phase)
            {
                std::cout << p << ", ";
            }
            std::cout << ")" << std::endl;
        }
        else if (arg_pair.first == "set_fsm_id")
        {
            int32_t fsm_id = std::stoi(arg_pair.second);
            ret = client->SetFsmId(fsm_id);
            std::cout << "ret : " << ret << " , set fsm_id to " << fsm_id << std::endl;
        }
        else if (arg_pair.first == "set_balance_mode")
        {
            int32_t balance_mode = std::stoi(arg_pair.second);
            ret = client->SetBalanceMode(balance_mode);
            std::cout << "ret : " << ret << " , set balance_mode to " << balance_mode << std::endl;
        }
        else if (arg_pair.first == "set_swing_height")
        {
            float swing_height = std::stof(arg_pair.second);
            ret = client->SetSwingHeight(swing_height);
            std::cout << "ret : " << ret << " , set swing_height to " << swing_height << std::endl;
        }
        else if (arg_pair.first == "set_stand_height")
        {
            float stand_height = std::stof(arg_pair.second);
            ret = client->SetStandHeight(stand_height);
            std::cout << "ret : " << ret << " , set stand_height to " << stand_height << std::endl;
        }
        else if (arg_pair.first == "set_velocity")
        {
            std::vector<float> param = StringToFloatVector(arg_pair.second);
            auto param_size = param.size();
            float vx, vy, omega, duration;
            if (param_size == 3)
            {
                vx = param.at(0);
                vy = param.at(1);
                omega = param.at(2);
                duration = 1.f;
            }
            else if (param_size == 4)
            {
                vx = param.at(0);
                vy = param.at(1);
                omega = param.at(2);
                duration = param.at(3);
            }
            else
            {
                std::cerr << "Invalid param size for method SetVelocity: " << param_size
                          << std::endl;
                return 1;
            }

            ret = client->SetVelocity(vx, vy, omega, duration);
            std::cout << "ret : " << ret << " , set velocity to " << arg_pair.second << std::endl;
        }
        else if (arg_pair.first == "damp")
        {
            ret = client->Damp();
            std::cout << "Damp ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "start")
        {
            ret = client->Start();
            std::cout << "Start ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "squat")
        {
            ret = client->Squat();
            std::cout << "Squat ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "sit")
        {
            ret = client->Sit();
            std::cout << "Sit ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "stand_up")
        {
            ret = client->StandUp();
            std::cout << "StandUp ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "zero_torque")
        {
            ret = client->ZeroTorque();
            std::cout << "ZeroTorque ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "stop_move")
        {
            ret = client->StopMove();
            std::cout << "StopMove ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "high_stand")
        {
            ret = client->HighStand();
            std::cout << "HighStand ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "low_stand")
        {
            ret = client->LowStand();
            std::cout << "LowStand ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "balance_stand")
        {
            ret = client->BalanceStand();
            std::cout << "BalanceStand ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "continous_gait")
        {
            bool flag;
            if (arg_pair.second == "true")
            {
                flag = true;
            }
            else if (arg_pair.second == "false")
            {
                flag = false;
            }
            else
            {
                std::cerr << "invalid argument: " << arg_pair.second << std::endl;
                return 1;
            }
            ret = client->ContinuousGait(flag);
            std::cout << "ContinuousGait ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "switch_move_mode")
        {
            bool flag;
            if (arg_pair.second == "true")
            {
                flag = true;
            }
            else if (arg_pair.second == "false")
            {
                flag = false;
            }
            else
            {
                std::cerr << "invalid argument: " << arg_pair.second << std::endl;
                return 1;
            }
            ret = client->SwitchMoveMode(flag);
            std::cout << "SwitchMoveMode ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "move")
        {
            std::vector<float> param = StringToFloatVector(arg_pair.second);
            auto param_size = param.size();
            float vx, vy, omega;
            if (param_size == 3)
            {
                vx = param.at(0);
                vy = param.at(1);
                omega = param.at(2);
            }
            else
            {
                std::cerr << "Invalid param size for method SetVelocity: " << param_size
                          << std::endl;
                return 1;
            }
            ret = client->Move(vx, vy, omega);
            std::cout << "Move ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "set_task_id")
        {
            int task_id = std::stoi(arg_pair.second);
            ret = client->SetTaskId(task_id);
            std::cout << "ret : " << ret << " , set task_id to " << task_id << std::endl;
        }
        else if (arg_pair.first == "shake_hand")
        {
            client->ShakeHand(0);
            std::cout << "Shake hand starts! Waiting for 10 s for ending"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(10));
            std::cout << "Shake hand ends!" << std::endl;
            ret = client->ShakeHand(1);
            std::cout << "ShakeHand ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "wave_hand")
        {
            ret = client->WaveHand();
            std::cout << "wave hand ret : " << ret << std::endl;
        }
        else if (arg_pair.first == "wave_hand_with_turn")
        {
            ret = client->WaveHand(true);
            std::cout << "wave hand with turn" << ret << std::endl;
        }

        std::cout << "Done!" << std::endl;
    }
    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        auto client = std::make_shared<LocoClient>();
        std::map<std::string, std::string> args;
        ParseArgv(argc, argv, args);
        DoWork(client, args);
        rclcpp::shutdown();
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        std::cerr << "RCLError caught: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
