#include "mt6701/MagneticSensorMT6701SSI.h"

#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "config\config.h"

MagneticSensorMT6701SSI::MagneticSensorMT6701SSI(int nCS, SPISettings settings)
    : settings(settings), nCS(nCS)
{}

MagneticSensorMT6701SSI::~MagneticSensorMT6701SSI() {}

void MagneticSensorMT6701SSI::init(SPIClass *_spi, bool init)
{
    this->spi = _spi;
    if (nCS >= 0)
    {
        pinMode(nCS, OUTPUT);
        digitalWrite(nCS, HIGH);
    }
    if (!init)
    {
        this->spi->begin(MOTOR_SSI_SCK, MOTOR_SSI_DO);
    }
    this->Sensor::init();
}

// check 40us delay between each read?
float MagneticSensorMT6701SSI::getSensorAngle()
{
    float angle_data = readRawAngleSSI();
    angle_data       = (angle_data / (float)MT6701_CPR) * _2PI;
    // return the shaft angle
    return angle_data;
}

uint16_t MagneticSensorMT6701SSI::readRawAngleSSI()
{
    if (nCS >= 0)
        digitalWrite(nCS, LOW);
    spi->beginTransaction(settings);
    uint16_t value = spi->transfer16(0x0000);
    spi->endTransaction();
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);
    return (value >> MT6701_DATA_POS) & 0x3FFF;
};
