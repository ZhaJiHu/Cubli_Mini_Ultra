#include "control/param.h"

namespace CubliMini {
namespace Control {

bool Param::Load()
{
    EepromInit();
    uint32_t flag = 0xffffffff;
    ReadFirstDownload(flag);
    if (flag != 0)
    {
        flag = 0;
        SaveFirstDownload(flag);
        FirstWriteParamToEeprom();
        printf("first write\r\n");
    }
    LoadParam();
    printf("WIFI_PARAM_ADDR:%d\n", WIFI_PARAM_ADDR);
    return 0;
}

void Param::LoadParam()
{
    ReadPBalanceParam(p_axis_param_);
    ReadUBalanceParam(u_param_);
}
void Param::FirstWriteParamToEeprom()
{
    PBalanceControl::PAxisParam p_axis_param {
        {P_BALANCE_X_P, P_BALANCE_X_V, P_BALANCE_X_S, P_BALANCE_X_A},
        {P_BALANCE_Y_P, P_BALANCE_Y_V, P_BALANCE_Y_S, P_BALANCE_Y_A},
        {P_BALANCE_Z_P, P_BALANCE_Z_V, P_BALANCE_Z_S, P_BALANCE_Z_A}
    };

    AxisParam_t u_param {U_BALANCE_CH3_P, U_BALANCE_CH3_V, U_BALANCE_CH3_S, U_BALANCE_CH3_A};
    SavePBalanceParam(p_axis_param);
    SaveUBalanceParam(u_param);
}

void Param::SaveAllParam()
{
    SavePBalanceParam(p_axis_param_);
    SaveUBalanceParam(u_param_);
}

bool Param::SaveFirstDownload(uint32_t first)
{
    return Write(FIRST_DOWNLOAD_ADDR, (uint8_t *)&first, sizeof(first));
}

void Param::ReadFirstDownload(uint32_t &first)
{
    Read(FIRST_DOWNLOAD_ADDR, (uint8_t *)&first, sizeof(first));
}

bool Param::SavePBalanceParam(const PBalanceControl::PAxisParam &_param)
{
    return Write(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void Param::ReadPBalanceParam(PBalanceControl::PAxisParam &_param)
{
    Read(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool Param::SaveUBalanceParam(const AxisParam_t &_param)
{
    return Write(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void Param::ReadUBalanceParam(AxisParam_t &_param)
{
    Read(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}
}  // namespace Control
}  // namespace CubliMini