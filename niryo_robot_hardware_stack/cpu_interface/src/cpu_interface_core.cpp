#include "cpu_interface/cpu_interface_core.hpp"

CpuInterfaceCore::CpuInterfaceCore()
{
    _cpu_temperature = 0;
    initParams();
    startReadingData();
    ROS_INFO("CPU Interface - Started");
}

void CpuInterfaceCore::initParams()
{
    if (ros::param::has("read_rpi_diagnostics_frequency/read_rpi_diagnostics_frequency"))
    {
        ros::param::get("read_rpi_diagnostics_frequency/read_rpi_diagnostics_frequency", _read_cpu_frequency);
    }
    else
    {
        ROS_INFO("CPU Interface - No params found. Init with default value");
        _nh.getParam("/niryo_robot_hardware_interface/read_rpi_diagnostics_frequency", _read_cpu_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/temperature_warn_threshold", _temperature_warn_threshold);
        _nh.getParam("/niryo_robot_hardware_interface/temperature_shutdown_threshold", _temperature_shutdown_threshold);
    }
    ROS_DEBUG("CPU Interface - Read temperature frequency %f", _read_cpu_frequency);
}


int CpuInterfaceCore::getCpuTemperature()
{
    return _cpu_temperature;
}

void CpuInterfaceCore::_readCpuTemperature()
{
#if defined __arm__ || defined __aarch64__
    std::fstream cpu_temp_file("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);
    
    int read_temp;
    cpu_temp_file >> read_temp;
    if (read_temp > 0) {
        _cpu_temperature = read_temp / 1000;
    }
#endif
}

void CpuInterfaceCore::startReadingData()
{
   _read_hardware_data_thread.reset(new std::thread(boost::bind(&CpuInterfaceCore::_readHardwareDataLoop, this))); 
}

void CpuInterfaceCore::_readHardwareDataLoop()
{
    ros::Rate read_rpi_diagnostics_rate = ros::Rate(_read_cpu_frequency);

    while (ros::ok()) {
        _readCpuTemperature();

        // check if Rpi is too hot
        if (_cpu_temperature > _temperature_warn_threshold) {
            ROS_WARN("CPU Interface - Rpi temperature is really high !");
        }
        if (_cpu_temperature > _temperature_shutdown_threshold) {
            ROS_ERROR("CPU Interface - Rpi is too hot, shutdown to avoid any damage");
            std::system("sudo shutdown now");
        }

        read_rpi_diagnostics_rate.sleep();
    }
}
