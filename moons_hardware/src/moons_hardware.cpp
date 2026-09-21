#include "moons_hardware/moons_hardware.hpp"
#include <thread>
#include <chrono>

int main()
{
    // Create object with your settings
    MoonsMotorEnable motor(
        "/dev/ttyUSB0",  // port
        9600,            // baudrate
        1,               // left slave
        2,               // right slave
        40125,           // enable register
        40343            // speed register
    );

    // Connect
    if (!motor.connect())
    {
        return -1;
    }

    // Wait 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Enable motors
    if (!motor.enableMotors())
    {
        return -1;
    }

    // Enable jog mode
    if (!motor.enableJogMode())
    {
        return -1;
    }

    // Set speed to 500 RPM on both motors
    if (!motor.setJogSpeed(500, 500))
    {
        return -1;
    }

    // Run for 15 seconds
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // Stop motors (set speed to 0)
    motor.setJogSpeed(0, 0);

    // Disable motors
    motor.disableMotors();

    return 0;
}



// #include "rclcpp/rclcpp.hpp"
// #include "moons_hardware/moons_hardware.hpp"
// #include <thread>

// int main()
// {

//     MoonsMVDBDriver driver("/dev/ttyUSB0");

//     driver.init();

//     driver.enableAxis(AXIS_1);
//     driver.enableAxis(AXIS_2);

//     sleep(1);
//     printf("shailesh");

//     driver.setVelocityRPM(AXIS_1, 500);
//     driver.setVelocityRPM(AXIS_2, -500);

//     sleep(2);

//     driver.stopAxis(AXIS_1);
//     driver.stopAxis(AXIS_2);

// }