#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {
    auto last_pub_time_ = std::chrono::system_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto now_time = std::chrono::system_clock::now();
    uint64_t publish_interval_ = 100000000; //100 ms
    if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) {
        auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(now_time - last_pub_time_);
        std::cout << "Delta time " << delta.count() << " ns is less than publish interval " << publish_interval_ << std::endl;
    }

    last_pub_time_ += std::chrono::microseconds(publish_interval_ / 1000);
    // last_pub_time_ += std::chrono::nanoseconds(publish_interval_); // This causes error in android NDK.
    //  candidate function not viable: no known conversion from 'duration<[...], ratio<[...], 1000000000>>' to 'const duration<[...], ratio<[...], 1000000>>' for 1st argument
    uint64_t currenttime = std::chrono::duration_cast<std::chrono::nanoseconds>(last_pub_time_.time_since_epoch()).count();
    uint64_t currenttime2 = last_pub_time_.time_since_epoch().count() * 1000;
    std::cout << "currenttime " << currenttime << " ns currenttime2 " << currenttime2 << " ns" << std::endl;
    return 0;
}
