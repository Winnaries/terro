#include <iostream>
#include <fstream> 
#include <ctello.h> 

#include <ORB_SLAM3/System.h>
#include <ORB_SLAM3/ImuTypes.h>
#include <ORB_SLAM3/Optimizer.h>

void ShowStatus(const std::string& state)
{
    system("clear");
    int begin{0};
    std::cout << "+-----------+-----------+" << std::endl;
    const int padding{10};
    while (begin < state.size())
    {
        const auto split{state.find(':', begin)};
        const auto name{state.substr(begin, split - begin)};
        const auto end{state.find(';', split)};
        const auto value{state.substr(split + 1, end - split - 1)};
        begin = end + 1;

        std::cout << "|  " << name;
        std::cout << std::setw(padding - name.size()) << "|";
        std::cout << "  " << value;
        std::cout << std::setw(padding - value.size()) << "|";
        std::cout << std::endl;
    }
    std::cout << "+-----------+-----------+" << std::endl;
}

int main(int argc, char **argv)
{
    ctello::Tello tello; 

    if (argc < 3) 
    {
        std::cerr << std::endl << "Usage: ./terro path_to_vocabulary path_to_settings"; 
        return 1; 
    }
    
    if (!tello.Bind())
    {
        return 0;
    }

    auto result = tello.SendCommand("command");

    while (true)
    {
        if (const auto state = tello.GetState())
        {
            ShowStatus(*state); 
        }
    }

    return 0;
}