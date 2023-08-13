#include <Arduino.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>

WiFiMulti wifiMulti; // 建立ESP8266WiFiMulti对象,对象名称是'wifiMulti'

/*************MQTT*********************/
#include "PubSubClient.h"                          //默认，加载MQTT库文件
#define ID_MQTT "XXXXXXXXXXXXXXXXXXXX" // 用户私钥，控制台获取
const char *topic = "HomeLight002";                // 主题名字，可在巴法云控制台自行创建，名称随意
const int B_led = 27;                              // 单片机LED引脚值，D系列是NodeMcu引脚命名方式，其他esp8266型号将D2改为自己的引脚
const char *mqtt_server = "bemfa.com";             // 默认，MQTT服务器
const int mqtt_server_port = 9501;                 // 默认，MQTT服务器
WiFiClient espClient;
PubSubClient client(espClient);
//**************************************************//

/**************GPIO SET***************/
#define WORK_LED 33
#define PWM_LED 25
#define SW_D0 26
#define SW_D1 12
#define SW_D2 27
#define SW_D3 14
volatile bool LED_PWM_State = false;
/****************************************************/

void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void MQTTinit(void);
void MQTTloop(void);
void handleInterrupt();

void setup()
{
    // GPIO INITIAL
    pinMode(WORK_LED, OUTPUT);
    pinMode(PWM_LED, OUTPUT);
    pinMode(SW_D1, INPUT_PULLDOWN);
    // pinMode(SW_D1, INPUT_PULLDOWN);
    // pinMode(SW_D2, INPUT_PULLDOWN);
    // pinMode(SW_D3, INPUT_PULLDOWN);
    // 设置 SW_D0 管脚的中断触发方式为上升沿和下降沿触发
    attachInterrupt(digitalPinToInterrupt(SW_D1), handleInterrupt, RISING | FALLING);
    // attachInterrupt(digitalPinToInterrupt(SW_D1), handleInterrupt, RISING | FALLING);
    // attachInterrupt(digitalPinToInterrupt(SW_D2), handleInterrupt, RISING | FALLING);
    // attachInterrupt(digitalPinToInterrupt(SW_D3), handleInterrupt, RISING | FALLING);
    // put your setup code here, to run once:
    Serial.begin(115200); // 启动串口通讯
    // pinMode(2, OUTPUT);   // 设置内置LED引脚为输出模式以便控制LED
    //  ticker.attach(blinkInterval, tickerCount);  // 设置Ticker对象
    //  通过addAp函数存储  WiFi名称       WiFi密码
    wifiMulti.addAP("XXXXX", "XXXXXX"); // 这三条语句通过调用函数addAP来记录3个不同的WiFi网络信息。
                                             // wifiMulti.addAP("ssid2", "passphrase2"); // 这三条语句通过调用函数addAP来记录3个不同的WiFi网络信息。
                                             //  wifiMulti.addAP("taichi-maker2", "87654321"); // 这3个WiFi网络名称分别是taichi-maker, taichi-maker2, taichi-maker3。
                                             // wifiMulti.addAP("taichi-maker3", "13572468"); // 这3个网络的密码分别是123456789，87654321，13572468。
                                             // 此处WiFi信息只是示例，请在使用时将需要连接的WiFi信息填入相应位置。
                                             // 另外这里只存储了3个WiFi信息，您可以存储更多的WiFi信息在此处。

    int i = 0;
    while (wifiMulti.run() != WL_CONNECTED)
    {                // 此处的wifiMulti.run()是重点。通过wifiMulti.run()，NodeMCU将会在当前
        delay(1000); // 环境中搜索addAP函数所存储的WiFi。如果搜到多个存储的WiFi那么NodeMCU
        Serial.print(i++);
        Serial.print(' '); // 将会连接信号最强的那一个WiFi信号。
    }
    Serial.print("Connected to ");  // NodeMCU将通过串口监视器输出。
    Serial.println(WiFi.SSID());    // 连接的WiFI名称
    Serial.print("IP address:");    // 以及
    Serial.println(WiFi.localIP()); // NodeMCU的IP地址

    MQTTinit();
}

void loop()
{

    // put your main code here, to run repeatedly:
    MQTTloop();
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Topic:");
    Serial.println(topic);
    String msg = "";
    for (unsigned int i = 0; i < length; i++)
    {
        msg += (char)payload[i];
    }
    Serial.print("Msg:");
    Serial.println(msg);
    if (msg.substring(0, 2) == "on")
    {
        LED_PWM_State = true;
        Serial.print("LED_PWM_State:");
        Serial.println(int(LED_PWM_State));
        if (msg.substring(0, 3) == "on#")
        { // 亮度调节
            Serial.println(msg.substring(0, 3));
            Serial.println(msg.substring(3, 5));
            analogWrite(PWM_LED, 255 / 100.0 * atoi((msg.substring(3, 5)).c_str()));
        }
        else
        { // 如果接收字符on，亮灯
            Serial.println(msg.substring(0, 2));
            analogWrite(PWM_LED, 255);
        }
    }
    else if (msg == "off")
    {
        LED_PWM_State = false;
        Serial.print("LED_PWM_State:");
        Serial.println(int(LED_PWM_State));
        // 如果接收字符off，亮灯
        analogWrite(PWM_LED, 0);
    }
    msg = "";
}
void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(ID_MQTT))
        {
            Serial.println("connected");
            Serial.print("subscribe:");
            Serial.println(topic);
            // 订阅主题，如果需要订阅多个主题，可发送多条订阅指令client.subscribe(topic2);client.subscribe(topic3);
            client.subscribe(topic);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void MQTTinit(void)
{
    client.setServer(mqtt_server, mqtt_server_port); // 设置mqtt服务器
    client.setCallback(callback);                    // mqtt消息处理
}
void MQTTloop(void)
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
}

void handleInterrupt()
{
    // 中断处理函数，中断触发时会执行这里的代码
    // 反转输入状态
    LED_PWM_State = !LED_PWM_State;
    analogWrite(PWM_LED, 255 * int(LED_PWM_State));
}
