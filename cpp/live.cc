#include <cstdint>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <set>
#include <sstream>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace fs = std::filesystem;
volatile std::sig_atomic_t interrupted = 0;
void interrupt_handler(int) { interrupted = 1; }

uint64_t be(const uint8_t* data, int count) {
    uint64_t value = 0;
    for (int i = 0; i < count; ++i) value = (value << 8) | data[i];
    return value;
}

struct Frame {
    uint64_t sequence = 0, timestamp = 0, receipt = 0;
    cv::Mat image;
    std::vector<uint8_t> jpeg;
};

uint64_t monotonic_ns() {
    timespec value{};
    clock_gettime(CLOCK_MONOTONIC, &value);
    return uint64_t(value.tv_sec)*1000000000ULL + value.tv_nsec;
}

class Stream {
    int socket_ = -1;
    std::atomic<bool> closed_{false};
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable ready_;
    Frame latest_;
    bool available_ = false;
    std::string error_;

    void receive(void* output, size_t length) {
        auto* data = static_cast<uint8_t*>(output);
        while (length) {
            const auto count = recv(socket_, data, length, 0);
            if (count <= 0) throw std::runtime_error("Pi camera stream disconnected or timed out");
            data += count;
            length -= count;
        }
    }
    void run() {
        uint64_t previous = 0;
        try {
            while (!closed_) {
                std::array<uint8_t, 32> header{};
                receive(header.data(), header.size());
                if (std::string(reinterpret_cast<char*>(header.data()), 4) != "PSL1")
                    throw std::runtime_error("Invalid Pi stream magic");
                Frame frame;
                frame.sequence = be(header.data()+4, 8);
                frame.timestamp = be(header.data()+12, 8);
                size_t length = be(header.data()+20, 4);
                int width = be(header.data()+24, 2), height = be(header.data()+26, 2);
                uint32_t flags = be(header.data()+28, 4);
                if (!length || length > 16*1024*1024 || frame.timestamp <= previous ||
                    (flags & 0xE000) != 0x2000)
                    throw std::runtime_error("Invalid frame size or capture timestamp");
                frame.jpeg.resize(length);
                receive(frame.jpeg.data(), frame.jpeg.size());
                frame.receipt = monotonic_ns();
                frame.image = cv::imdecode(frame.jpeg, cv::IMREAD_COLOR);
                if (frame.image.empty() || frame.image.cols != width || frame.image.rows != height)
                    throw std::runtime_error("Invalid JPEG frame");
                previous = frame.timestamp;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = std::move(frame);
                    available_ = true;
                }
                ready_.notify_all();
            }
        } catch (const std::exception& exc) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!closed_) error_ = exc.what();
            ready_.notify_all();
        }
    }
public:
    explicit Stream(int port) {
        socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0) throw std::runtime_error("Cannot create camera socket");
        timeval timeout{10, 0};
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
        if (connect(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address))) {
            close(socket_);
            socket_ = -1;
            throw std::runtime_error("Cannot connect to Pi camera; run start-camera.sh first");
        }
        worker_ = std::thread(&Stream::run, this);
    }
    bool read(Frame& frame, uint64_t previous) {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait_for(lock, std::chrono::milliseconds(250), [&] {
            return !error_.empty() || (available_ && latest_.sequence != previous);
        });
        if (!error_.empty()) throw std::runtime_error(error_);
        if (!available_ || latest_.sequence == previous) return false;
        frame = latest_;
        return true;
    }
    ~Stream() {
        closed_ = true;
        shutdown(socket_, SHUT_RDWR);
        if (worker_.joinable()) worker_.join();
        close(socket_);
    }
};

void write_pose(std::ostream& out, double stamp, const Sophus::SE3f& Twc) {
    auto t = Twc.translation();
    auto q = Twc.unit_quaternion();
    out << std::fixed << std::setprecision(9) << stamp << ' ' << t.x() << ' ' << t.y() << ' '
        << t.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
}

void write_pose_event(ORB_SLAM3::System& slam, const Frame& frame, int state,
                      const Sophus::SE3f& Tcw, const fs::path& output, std::ostream& log) {
    const auto result_time = monotonic_ns();
    std::set<ORB_SLAM3::MapPoint*> unique;
    for (auto* point : slam.GetTrackedMapPoints())
        if (point && !point->isBad() && point->Observations() > 0) unique.insert(point);
    std::ostringstream event;
    event << std::setprecision(12) << "{\"capture_timestamp_ns\":" << frame.timestamp
          << ",\"result_monotonic_ns\":" << result_time
          << ",\"receipt_to_result_ms\":" << double(result_time-frame.receipt)/1e6
          << ",\"sequence\":" << frame.sequence << ",\"tracking_state\":" << state
          << ",\"map_id\":" << slam.GetAtlasForExport()->GetCurrentMap()->GetId()
          << ",\"tracked_landmarks\":" << unique.size()
          << ",\"metric_scale\":false,\"time_sync_verified\":false"
             ",\"body_extrinsics_validated\":false,\"continuous_local_frame\":false"
             ",\"end_to_end_latency_ms\":null,\"flight_output_enabled\":false"
             ",\"pose_camera_world_xyzw\":";
    if (state == 2 && Tcw.matrix().allFinite()) {
        const auto Twc = Tcw.inverse();
        const auto t = Twc.translation();
        const auto q = Twc.unit_quaternion();
        event << '[' << t.x() << ',' << t.y() << ',' << t.z() << ',' << q.x() << ','
              << q.y() << ',' << q.z() << ',' << q.w() << ']';
    } else event << "null";
    event << '}';
    const auto temporary = output / "latest_pose.json.tmp";
    { std::ofstream file(temporary); file << event.str() << '\n'; }
    fs::rename(temporary, output / "latest_pose.json");
    log << event.str() << '\n';
}

size_t export_maps(ORB_SLAM3::System& slam, const fs::path& output) {
    size_t total = 0;
    for (auto* map : slam.GetAtlasForExport()->GetAllMaps()) {
        auto points = map->GetAllMapPoints();
        std::vector<Eigen::Vector3f> valid;
        for (auto* point : points)
            if (point && !point->isBad() && point->Observations() >= 3 && point->GetWorldPos().allFinite())
                valid.push_back(point->GetWorldPos());
        auto name = "map_" + std::to_string(map->GetId());
        std::ofstream ply(output / (name + ".ply"));
        ply << "ply\nformat ascii 1.0\ncomment coordinates have monocular arbitrary scale\n"
            << "element vertex " << valid.size() << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
        for (const auto& point : valid) ply << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
        auto keyframes = map->GetAllKeyFrames();
        std::sort(keyframes.begin(), keyframes.end(), [](auto* a, auto* b) { return a->mTimeStamp < b->mTimeStamp; });
        std::ofstream trajectory(output / (name + "_keyframes.tum"));
        for (auto* keyframe : keyframes)
            if (!keyframe->isBad()) write_pose(trajectory, keyframe->mTimeStamp, keyframe->GetPoseInverse());
        std::cout << name << ": " << valid.size() << " reliable points, " << keyframes.size() << " keyframes\n";
        total += valid.size();
    }
    return total;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: pi_slam_live VOCABULARY CAMERA_YAML OUTPUT_DIR [--headless] [--pose-events] [--record-frames] [--max-frames N] [--seconds N] [--port N]\n";
        return 2;
    }
    bool viewer = true;
    bool pose_events = false;
    bool record_frames = false;
    size_t max_frames = 0;
    double seconds = 0;
    int port = 8765;
    for (int i=4; i<argc; ++i) {
        if (std::string(argv[i]) == "--headless") viewer = false;
        else if (std::string(argv[i]) == "--pose-events") pose_events = true;
        else if (std::string(argv[i]) == "--record-frames") record_frames = true;
        else if (std::string(argv[i]) == "--max-frames" && i+1<argc) max_frames = std::stoull(argv[++i]);
        else if (std::string(argv[i]) == "--seconds" && i+1<argc) seconds = std::stod(argv[++i]);
        else if (std::string(argv[i]) == "--port" && i+1<argc) port = std::stoi(argv[++i]);
        else { std::cerr << "Unknown argument: " << argv[i] << '\n'; return 2; }
    }
    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);
    std::unique_ptr<ORB_SLAM3::System> slam;
    fs::path output(argv[3]);
    bool shut_down = false;
    try {
        cv::FileStorage settings(argv[2], cv::FileStorage::READ);
        if (!settings.isOpened() || int(settings["Calibration.validated"]) != 1)
            throw std::runtime_error("Validated calibration is required: run calibrate.py first");
        int width = settings["Camera.width"], height = settings["Camera.height"];
        settings.release();
        fs::create_directories(output);
        if (fs::exists(output / "poses_live.tum")) throw std::runtime_error("Output directory already contains a run");
        // Load vocabulary before opening the receiver so camera frames don't queue during startup.
        slam = std::make_unique<ORB_SLAM3::System>(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, viewer);
        Stream stream(port);
        std::ofstream poses(output / "poses_live.tum");
        std::ofstream events;
        if (pose_events) events.open(output / "pose_events.jsonl");
        std::ofstream frame_index;
        if (record_frames) {
            fs::create_directories(output / "frames");
            frame_index.open(output / "frames.csv");
            frame_index << "time_s,capture_timestamp_ns,sequence,image\n";
            frame_index << std::fixed << std::setprecision(9);
        }
        poses << "# Online poses before loop corrections; arbitrary monocular scale\n";
        uint64_t previous = UINT64_MAX, origin = 0, dropped = 0;
        size_t processed = 0, tracked = 0;
        double total_ms = 0;
        auto start = std::chrono::steady_clock::now();
        while (!interrupted && !slam->ViewerFinished() && (!max_frames || processed < max_frames) &&
               (!seconds || std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < seconds)) {
            Frame frame;
            if (!stream.read(frame, previous)) continue;
            if (frame.image.cols != width || frame.image.rows != height)
                throw std::runtime_error("Camera resolution differs from calibration; recalibrate this capture mode");
            if (!origin) origin = frame.timestamp;
            if (previous != UINT64_MAX && frame.sequence > previous+1) dropped += frame.sequence-previous-1;
            previous = frame.sequence;
            double timestamp = double(frame.timestamp-origin)/1e9;
            if (record_frames) {
                auto name = "frames/" + std::to_string(frame.sequence) + ".jpg";
                std::ofstream image_file(output / name, std::ios::binary);
                image_file.write(reinterpret_cast<const char*>(frame.jpeg.data()), frame.jpeg.size());
                if (!image_file) throw std::runtime_error("Cannot record camera image (disk full?)");
                frame_index << timestamp << ',' << frame.timestamp << ',' << frame.sequence << ',' << name << '\n';
                if (!frame_index) throw std::runtime_error("Cannot record frame index");
            }
            auto before = std::chrono::steady_clock::now();
            auto Tcw = slam->TrackMonocular(frame.image, timestamp);
            total_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-before).count();
            int state = slam->GetTrackingState();
            if (pose_events) write_pose_event(*slam, frame, state, Tcw, output, events);
            ++processed;
            if (state == 2 && Tcw.matrix().allFinite()) { ++tracked; write_pose(poses, timestamp, Tcw.inverse()); }
            if (processed % 20 == 0) std::cout << "Frames " << processed << " | state " << state
                << " | mean tracking " << total_ms/processed << "ms | skipped " << dropped << '\n';
            if (viewer) {
                cv::Mat display = frame.image.clone();
                std::string status = state == 2 ? "TRACKING" : state == 1 ? "MOVE CAMERA SIDEWAYS" : "TRACKING LOST";
                cv::putText(display, status + " | Q: save and quit", cv::Point(10,25),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
                cv::imshow("Pi camera - ORB-SLAM3", display);
                int key = cv::waitKey(1) & 0xFF;
                if (key == 27 || key == 'q') break;
            }
        }
        slam->Shutdown();
        shut_down = true;
        auto points = export_maps(*slam, output);
        std::ofstream stats(output / "stats.json");
        stats << "{\"processed_frames\":" << processed << ",\"tracked_frames\":" << tracked
              << ",\"skipped_frames\":" << dropped << ",\"exported_points\":" << points
              << ",\"mean_tracking_ms\":" << (processed ? total_ms/processed : 0)
              << ",\"capture_clock_origin_ns\":" << origin << ",\"metric_scale\":false}\n";
        std::cout << "Saved run to " << output << '\n';
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "ERROR: " << exc.what() << '\n';
        if (slam && !shut_down) { slam->Shutdown(); export_maps(*slam, output); }
        return 1;
    }
}
