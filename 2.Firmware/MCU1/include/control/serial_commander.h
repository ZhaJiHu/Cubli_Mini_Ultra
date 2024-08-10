#pragma once
#include <Arduino.h>

#include <map>

namespace CubliMini {
namespace Control {
const char g_keol  = '\n';
const char g_split = ':';

#define MAX_RECEIVER_SIZE 50

#define REGISTER_COMMAND_FLOAT(commander, cmdChar, modeVariable)                                   \
    commander.Register(cmdChar, [&](const std::string &value) {                                    \
        modeVariable = std::stof(value);                                                           \
        printf("%s value: %f\r\n", cmdChar, modeVariable);                                      \
    });

#define REGISTER_COMMAND_INT(commander, cmdChar, modeVariable)                                     \
    commander.Register(cmdChar, [&](const std::string &value) {                                    \
        modeVariable = std::stoi(value);                                                           \
        printf("%s value: %d\r\n", cmdChar, modeVariable);                                         \
    });

class SerialCommander
{
   public:
    using Callback = std::function<void(const std::string &)>;
    virtual void Run(Stream &_serial);
    void Register(const std::string &cmd, Callback callback);
    const std::map<std::string, Callback> & GetCmds() const
    {
        return cmds_;
    }
    
   protected:
    virtual void CmdPrintf(const char *fmt, ...);
    void DecodeCmd(const std::string &str);
    bool IsSentinel(char ch);
    std::map<std::string, Callback> cmds_;

   private:
    int rec_cnt_;
    char received_chars_[MAX_RECEIVER_SIZE];
};

}  // namespace Control
}  // namespace CubliMini