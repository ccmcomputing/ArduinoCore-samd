
#ifndef _TWILIO_MESSAGE_H_
#define _TWILIO_MESSAGE_H_


#include "utility/tmbhmac.h"


#define M_BUFFER_SIZE    1024


class TwilioMessageClass
{
public:
    
    TwilioMessageClass(char *accountName, char *appKeyName, char *appKeyValue, char *accountSid, char *authToken);
    
    void Init(char *apn);
    
    void GetLastMessageFromNumber(char *from);
    void SendSMS(char *from, char *to, char *body);
    void ListMessages(char *number);    
    void GetMessage(char *smsMessageSID);
    void GetLastMessageThatContains(char *filter);
    void GetMessagesThatContain(char *filter);

private:

    void Run(char *comand, char *content, int time);
    int GetAuth(char *appKey, char *content, char *result);    
    
    unsigned long get_temboo_time(char *buffer);
    unsigned int get_http_cmd(char *buffer);
    char * uint32toa(unsigned long value, char* dest);   
    
    
    char *m_accountName;
    char *m_appKeyName;
    char *m_appKeyValue;
    
    char *m_accountSid;
    char *m_authToken;

    GPRS gprs;
    
    char m_buffer[M_BUFFER_SIZE];
    
};



#endif
