#include "control/serial_commander.h"
namespace CubliMini {
namespace Control {

void SerialCommander::Register(const std::string &cmd, Callback callback)
{
    if (cmds_.count(cmd) > 0)
    {
        CmdPrintf("register fail, cmd(%s) is exist!!", cmd.c_str());
        return;
    }
    cmds_.emplace(cmd, callback);
}

void SerialCommander::DecodeCmd(const std::string &str)
{
    size_t pos = str.find(g_split);
    if (pos == 0)
    {
        return;
    }
    if (pos == std::string::npos)
    {
        return;
    }
    if (pos >= str.size() - 1)
    {
        return;
    }
    std::string before = str.substr(0, pos);
    std::string after  = str.substr(pos + 1);
    if(cmds_.count(before) > 0)
        cmds_[before](after);
    else
    {
        CmdPrintf("error cmd!!!\n");
    }
}

void SerialCommander::Run(Stream &_serial)
{
    if (_serial.available() > 0)
    {
        int ch                    = _serial.read();
        received_chars_[rec_cnt_] = (char)ch;
        rec_cnt_++;
        if (rec_cnt_ >= MAX_RECEIVER_SIZE)
        {
            rec_cnt_           = 0;
            received_chars_[0] = 0;
        }
        if (IsSentinel(ch))
        {
            DecodeCmd(std::string(received_chars_, rec_cnt_));
            received_chars_[0] = 0;
            rec_cnt_           = 0;
        }
    }
}

bool SerialCommander::IsSentinel(char ch)
{
    if (ch == g_keol)
    {
        return true;
    }
    else if (ch == '\n')
    {
        CmdPrintf("Warn: \\r detected! \n");
    }
    return false;
}

void SerialCommander::CmdPrintf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vprintf(fmt, args);
    va_end(args);
}

}  // namespace Control
}  // namespace CubliMini