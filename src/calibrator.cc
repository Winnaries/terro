#include <sstream>
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <format>
#include <utility>
#include <cmath>
#include <chrono>
#include <atomic> 
#include <cstdlib> 
#include <cstdio> 
#include <cstdint> 
#include <unordered_map>

#include <fmt/core.h>
#include <ctello.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

const char *const LOG_PATTERN = "[%D %T] [terro] [%^%l%$] %v";
const double BURN_PERIOD = 10 * 1e9;

struct IMU
{
    double pitch;
    double roll;
    double yaw;
    double agx;
    double agy;
    double agz;
};

IMU parse_imu_state(const std::string &state)
{
    int begin{0};
    IMU imu{};

    std::unordered_map<std::string, std::function<void(double)>> funcMap = {
        {"pitch", [&imu](const double value)
         { imu.pitch = value * M_PI / 180.0; }},
        {"roll", [&imu](const double value)
         { imu.roll = value * M_PI / 180.0; }},
        {"yaw", [&imu](const double value)
         { imu.yaw = value * M_PI / 180.0; }},
        {"agx", [&imu](const double value)
         { imu.agx = value / 100.0; }},
        {"agy", [&imu](const double value)
         { imu.agy = value / 100.0; }},
        {"agz", [&imu](const double value)
         { imu.agz = value / 100.0; }},
    };

    while (begin < state.size())
    {
        const auto split{state.find(':', begin)};
        const auto name{state.substr(begin, split - begin)};
        const auto end{state.find(';', split)};
        const auto value{state.substr(split + 1, end - split - 1)};
        begin = end + 1;

        if (funcMap.count(name))
        {
            funcMap[name](std::stod(value));
        }
    }

    return imu;
}

int main()
{
    spdlog::set_pattern(LOG_PATTERN);
    std::optional<std::string> response;
    ctello::Tello tello{};

    if (!tello.Bind())
    {
        return 0;
    }

    tello.SendCommand("streamon");
    while (!(response = tello.ReceiveResponse()))
        ;

    spdlog::info("Stream:         {0}", *response);

    std::string video_source = "udp://0.0.0.0:11111";
    cv::VideoCapture capture(video_source, cv::CAP_FFMPEG);

    if (!capture.isOpened())
    {
        spdlog::error("Unable to open video source");
        return -1;
    }

    capture.set(cv::CAP_PROP_BUFFERSIZE, 3);
    spdlog::info("Capture:        {0}", *response);

    double frameRate = capture.get(cv::CAP_PROP_FPS);
    spdlog::info("FPS: {}", frameRate);

    std::string prefix = "./calibration/";
    std::string imuPath = prefix + "imu0.csv";
    std::string camPrefix = prefix + "cam0/";
    system(("mkdir -p " + camPrefix).c_str());

    spdlog::info("Dataset Dir: {0}", prefix);
    spdlog::info("CAM0 Dir: {0}", camPrefix);
    spdlog::info("IMU0 Path: {0}", imuPath);

    std::atomic<bool> exit(false); 
    std::vector<std::pair<std::uint64_t, IMU>> imus;
    std::vector<std::pair<std::uint64_t, cv::Mat>> frames;

    spdlog::info("Starting receive_video thread");
    std::thread receive_video([capture, &frames, &exit]() mutable
                              {
        cv::Mat frame; 
        while (!exit) 
        {
            capture >> frame; 
            
            if (!frame.empty())
            {
                auto now = std::chrono::high_resolution_clock::now();
                auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                if (nanos < BURN_PERIOD) 
                    continue; 
                frames.push_back(std::make_pair(nanos, frame.clone())); 
            }
        } });

    spdlog::info("Starting receive_imu thread"); 
    std::thread receive_imu([&tello, &imus, &exit]()
                            {
        while (!exit) 
        {
            if (const auto resp = tello.GetState())
            {
                auto imu = parse_imu_state(*resp);
                auto now = std::chrono::high_resolution_clock::now();
                auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                if (nanos < BURN_PERIOD)
                    continue;
                imus.push_back(std::make_pair(nanos, imu)); 
            }
        } });

    spdlog::info("Press `Enter` to exit"); 
    while (true) {
        int key = std::getchar(); 
        if (key == 10)
        {
            exit = true; 
            break;
        }
    }

    receive_video.join(); 
    receive_imu.join(); 

    spdlog::info("Saving the IMU readings..."); 

    std::ofstream imuFile(imuPath, std::ios::app);
    imuFile << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;

    auto [prev_timestamp, prev_imu] = imus[0]; 

    for (int i = 1; i < imus.size(); i += 1)
    {
        auto [timestamp, imu] = imus[i]; 

        if (timestamp - prev_timestamp < 90000000)
        {
            continue; 
        }

        auto dt = (double)(timestamp - prev_timestamp) / 1e9; 

        auto dpitch = imu.pitch - prev_imu.pitch; 
        auto droll = imu.roll - prev_imu.roll; 
        auto dyaw = imu.yaw - prev_imu.yaw; 

        auto ang_x = droll / dt; 
        auto ang_y = dpitch / dt; 
        auto ang_z = dyaw / dt;

        imuFile << fmt::format("{},{:.f4f},{:.f4f},{:.f4f},{},{},{}", 
            timestamp, ang_x, ang_y, ang_z, imu.agx, imu.agy, imu.agz) << std::endl; 

        prev_timestamp = timestamp;
        prev_imu = imu; 
    }

    spdlog::info("Saving the frames readings..."); 

    for (const auto [timestamp, image] : frames) 
    {
        std::string filename = fmt::format("{}.png", timestamp);
        cv::imwrite(camPrefix + filename, image);
    }

    capture.release();
    cv::destroyAllWindows();
}