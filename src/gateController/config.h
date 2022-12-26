

// Connection
// DEBUG LED
#define LED_D0_PIN 27
// H-BRIDGE
#define CH_A_PIN 5
#define CH_B_PIN 0
#define EN_PIN 18

// REMOTE COMMANDS
#define COMMAND_ENABLE 1
#define COMMAND_DISABLE 2

#define COMMAND_SET_HOME_DIRECTION 10
#define COMMAND_SET_CURRENT_LIMIT 11
#define COMMAND_SET_INVERT_DIRECTION 12

#define COMMAND_START_CALIBRATION 13
#define COMMAND_FINISH_CALIBRATION 14

#define COMMAND_MOVE_ABSOLUTE 20
#define COMMAND_MOVE_RELATIVE 21
#define COMMAND_MOVE_HOME 22

/*
Note: ADC2 pins cannot be used when Wi-Fi is used.
So, if you’re using Wi-Fi and you’re having trouble
getting the value from an ADC2 GPIO, you may consider
using an ADC1 GPIO instead, that should solve your problem.
*/
#define CS_PIN 33
#define PWM_PIN 13

// ENCODER
#define ENC_A_PIN 2
#define ENC_B_PIN 15

//OTA
// USER: admin
// PASSWORD: secret

// WIFI SETTINGS
extern const char *ssid = "SSID";
extern const char *password = "PSK";

// MQTT SETTINGS
extern const char *mqttURL = "armbian";
extern const int mqttPort = 1883;      
extern const char *mqttUser = "gateController";
extern const char *mqttPassword = "secret";

extern "C" {
uint8_t temprature_sens_read();
}
