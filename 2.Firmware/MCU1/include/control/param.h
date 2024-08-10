#pragma once
#include "bsp/eeprom.h"
#include "config/config.h"
#include "control/p_balance_control.h"
#include "control/control_base.h"

namespace CubliMini {
namespace Control {

#define FIRST_DOWNLOAD_ADDR 0
#define P_PARAM_ADDR        4
#define U_PARAM_ADDR        P_PARAM_ADDR + sizeof(PBalanceControl::PAxisParam)
#define WIFI_PARAM_ADDR     U_PARAM_ADDR + sizeof(AxisParam_t)

class Param
{
   public:
    static Param *GetInstance()
    {
        static Param param;
        return &param;
    }

    void SaveAllParam();
    void SavePBalanceParam()
    {
        SavePBalanceParam(p_axis_param_);
    }
    void SaveUBalanceParam()
    {
        SaveUBalanceParam(u_param_);
    }

    bool Load();
    PBalanceControl::PAxisParam &GetPAxisParam() { return p_axis_param_; }
    AxisParam_t &GetUAxisParam() { return u_param_; }
    void FirstWriteParamToEeprom();
    void LoadParam();
    
   private:
    bool SavePBalanceParam(const PBalanceControl::PAxisParam &_param);
    bool SaveUBalanceParam(const AxisParam_t &_param);
    bool SaveFirstDownload(uint32_t first);
    void ReadFirstDownload(uint32_t &first);
    void ReadPBalanceParam(PBalanceControl::PAxisParam &_param);
    void ReadUBalanceParam(AxisParam_t &_param);

   private:
    PBalanceControl::PAxisParam p_axis_param_;
    AxisParam_t u_param_;
};

}  // namespace Control
}  // namespace CubliMini