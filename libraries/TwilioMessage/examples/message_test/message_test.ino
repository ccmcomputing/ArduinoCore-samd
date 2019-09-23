
#include "atmelsamd21_gprs.h"
#include "TwilioMessage.h"


#define TEMBOO_ACCOUNT "???"
#define TEMBOO_APP_KEY_NAME "???"
#define TEMBOO_APP_KEY_VALUE "???"
#define TWILIO_ACCOUNT_SID "???"
#define TWILIO_AUTH_TOKEN "???"
#define TWILIO_NUMBER "+???"


TwilioMessageClass Message(TEMBOO_ACCOUNT, TEMBOO_APP_KEY_NAME, TEMBOO_APP_KEY_VALUE, TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN);
int i = 0;


void setup()
{
    SerialUSB.begin(115200);
    while(!SerialUSB);
    SerialUSB.println("Temboo Message Test ...");
    
    Message.Init("cmwap");  //change "cmnet" to your own APN
    
    //Message.SendSMS(TWILIO_NUMBER, "+???", "Twili send sms test: Hello World!");
    //delay(3000);
}

void loop()
{
    SerialUSB.print("i = ");
    SerialUSB.println(i ++);
    //Message.GetLastMessageFromNumber("+???");
    Message.ListMessages(TWILIO_NUMBER);
    //Message.GetMessage("???");
    //Message.GetMessagesThatContain("Hello");
    //Message.GetLastMessageThatContains("Hello");
    delay(5000);
} 