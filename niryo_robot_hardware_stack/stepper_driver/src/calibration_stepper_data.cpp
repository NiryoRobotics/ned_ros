
#include "stepper_driver/calibration_stepper_data.hpp"

using namespace common::model;

namespace StepperDriver
{

    CalibrationStepperData::CalibrationStepperData()
    {
        reset();
    }

    CalibrationStepperData::~CalibrationStepperData()
    {

    }

    CalibrationStepperData::CalibrationStepperData(unsigned long irxId, uint8_t ilen, std::array<uint8_t, 8> irxBuf) :
        _rxId(irxId),
        _len(ilen),
        _rxBuf(irxBuf)
    {

    }

    bool CalibrationStepperData::isValid() const {
        return CAN_DATA_CALIBRATION_RESULT == _rxBuf[0];
    }

    uint8_t CalibrationStepperData::getId() const {
        return _rxId & 0x0F;
    }

    EStepperCalibrationStatus CalibrationStepperData::getStatus() const{
        if(!isValid())
            return common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
        else
            return static_cast<common::model::EStepperCalibrationStatus>(_rxBuf[1]);
    }

    int32_t CalibrationStepperData::getCalibrationResult() const{
        return (_rxBuf[2] << 8) + _rxBuf[3];
    }

    void CalibrationStepperData::reset()
    {
        _rxId = 0;
        _len = 0;
        _rxBuf.fill(0);
        _time = ros::Time::now();
    }

    std::string CalibrationStepperData::str() const
    {
        std::ostringstream ss;
        ss << "Calibration Stepper Data" << "\n";

        return ss.str();
    }

}
