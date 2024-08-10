#include "control/wifi_commander.h"

#include <esp_wifi.h>
#include "comm/time.h"

namespace CubliMini {
namespace Control {
using namespace Comm;

WifiConnectStatus_e WifiCommander::ConnectWifi()
{
    WiFi.begin(ssid_.c_str(), password_.c_str());
    CumulativeTime time_out_;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        printf("WIFI: connecting...\r\n");
        // 20s连接失败则退出连接
        if (time_out_.GetTimeS() > 10)
        {
            printf("WIFI: connected fail!\r\n");
            esp_wifi_disconnect();
            esp_wifi_stop();
            wifi_connect_status_ = WifiConnectStatus_e::eWIFI_FAIL;
            return wifi_connect_status_;
        }
    }
    wifi_connect_status_ = WifiConnectStatus_e::eWIFI_SUCCESS;

    // 打印模块IP
    printf(
        "WIFI: connected, cubli server ip address: %s port: %d\r\n",
        WiFi.localIP().toString(),
        server_port_);

    return wifi_connect_status_;
}

void WifiCommander::WaitClientConnect()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        server_.stop();
        return;
    }
    else
    {
        server_.begin(server_port_);
    }

    client_ = server_.available();
    printf("Waiting for tcp client connection ...\n");
    while (!client_)
    {
        delay(1000);
        client_ = server_.available();
        if (client_)
        {
            printf("client connect, remote IP(): %s \n", client_.remoteIP().toString().c_str());
        }
    }
}

bool WifiCommander::TcpClientStatus()
{
    return client_.connected();
}

void WifiCommander::TcpClientUnpack()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        return;
    }

    CmdPrintf("this is cubli_mini, enjoy !!!!\r\n");
    while (client_.connected())
    {
        if (client_.available())
        {
            // ;结束符
            String line = client_.readStringUntil('\n');
            DecodeCmd(std::string(line.c_str(), line.length()));
        }
        delay(50);
    }
    client_.stop();
    printf("tcp client close this connect\r\n");
}

void WifiCommander::CmdPrintf(const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);
    if (len < 0)
    {
        va_end(arg);
        return;
    };
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            va_end(arg);
            return;
        }
        len = vsnprintf(temp, len + 1, format, arg);
    }
    va_end(arg);
    len = client_.write((uint8_t *)temp, len);
    if (temp != loc_buf)
    {
        free(temp);
    }
}

}  // namespace Control
}  // namespace CubliMini
