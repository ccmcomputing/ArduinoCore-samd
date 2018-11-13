
#include <arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <malloc.h>
#include "atmelsamd21_gprs.h"
#include "TwilioMessage.h"


unsigned long x_temboo_time = 0;
unsigned int content_length = 0; 


TwilioMessageClass::TwilioMessageClass(char *accountName, char *appKeyName, char *appKeyValue, char *accountSid, char *authToken)
{    
    m_accountName = accountName;
    m_appKeyName = appKeyName;
    m_appKeyValue = appKeyValue;
    
    m_accountSid = accountSid;
    m_authToken = authToken;
    
    x_temboo_time = 0;
}

void TwilioMessageClass::Init(char *apn)
{
    SerialUSB.println("GPRS preInit...");  
    gprs.preInit();
    
    while(0 != gprs.init())
    {
        SerialUSB.println("init error");
        delay(1000);
    }
    
    while(!gprs.join(apn))
    {
        SerialUSB.println("GPRS join network error");
        delay(2000);
    }
    
    SerialUSB.print("IP Address is ");
    SerialUSB.println(gprs.getIPAddress());
}

void TwilioMessageClass::GetLastMessageFromNumber(char *from)
{ 
    char content[strlen(m_accountSid) + strlen(m_authToken) + strlen(from) + 54];
    snprintf(content,  sizeof(content),"{\"inputs\":{\"AuthToken\":\"%s\",\"From\":\"%s\",\"AccountSID\":\"%s\"}}", m_authToken, from, m_accountSid);
    //SerialUSB.println(content);
    Run("GetLastMessageFromNumber", content, 5);

}

void TwilioMessageClass::SendSMS(char *from, char *to, char *body)
{
    char content[strlen(m_accountSid) + strlen(m_authToken) + strlen(to) + strlen(from)  + strlen(body) + 73];
    snprintf(content,  sizeof(content),"{\"inputs\":{\"AuthToken\":\"%s\",\"To\":\"%s\",\"From\":\"%s\",\"Body\":\"%s\",\"AccountSID\":\"%s\"}}", 
    m_authToken, to, from, body, m_accountSid);
    //SerialUSB.println(content);
    Run("SendSMS", content, 5);
}

void TwilioMessageClass::ListMessages(char *number)
{
    char content[strlen(m_accountSid) + strlen(m_authToken) + strlen(number) + 78];
    snprintf(content,  sizeof(content),"{\"inputs\":{\"AuthToken\":\"%s\",\"AccountSID\":\"%s\",\"PageSize\":\"1\",\"Page\":\"0\",\"To\":\"%s\"}}", m_authToken, m_accountSid, number);
    //SerialUSB.println(content);
    Run("ListMessages", content, 30);
}

void TwilioMessageClass::GetMessage(char *smsMessageSID)
{
    char content[strlen(m_accountSid) + strlen(m_authToken) + strlen(smsMessageSID) + 63];
    snprintf(content,  sizeof(content),"{\"inputs\":{\"AuthToken\":\"%s\",\"SMSMessageSID\":\"%s\",\"AccountSID\":\"%s\"}}", m_authToken, smsMessageSID, m_accountSid);
    //SerialUSB.println(content);
    Run("GetMessage", content, 10);
}

void TwilioMessageClass::GetLastMessageThatContains(char *filter)
{
    char content[strlen(m_accountSid) + strlen(m_authToken) + strlen(filter) + 56];
    snprintf(content,  sizeof(content),"{\"inputs\":{\"AuthToken\":\"%s\",\"AccountSID\":\"%s\",\"Filter\":\"%s\"}}", m_authToken, m_accountSid, filter);
    //SerialUSB.println(content);
    Run("GetLastMessageThatContains", content, 5);
}

void TwilioMessageClass::GetMessagesThatContain(char *filter)
{
    char content[strlen(m_accountSid) + strlen(m_authToken) + strlen(filter) + 56];
    snprintf(content,  sizeof(content),"{\"inputs\":{\"AuthToken\":\"%s\",\"AccountSID\":\"%s\",\"Filter\":\"%s\"}}", m_authToken, m_accountSid, filter);
    //SerialUSB.println(content);
    Run("GetMessagesThatContain", content, 5);
}

void TwilioMessageClass::Run(char *comand, char *content, int time)
{    
    gprs.closeTCP();
    delay(1000);
    
    int len = strlen(m_accountName);
    char m_host[len + 16];
    snprintf(m_host, sizeof(m_host), "%s.temboolive.com", m_accountName);
    //SerialUSB.println(m_host);
    
    for(int i = 0; i < 2; i ++)
    {      
        int connect = gprs.connectTCP(m_host, 80);
        
        if(connect == 0)
        {
            SerialUSB.println("connect success");
            delay(1000);
            
            char m_auth[HMAC_HEX_SIZE_BYTES + 1];
            memset(m_auth, 0, sizeof(m_auth));
            len = GetAuth(m_appKeyValue, content, m_auth);
            
            memset(m_buffer, 0, M_BUFFER_SIZE);
            snprintf(m_buffer, M_BUFFER_SIZE,
            "POST /arcturus-web/api-1.0/ar/Library/Twilio/SMSMessages/%s?source_id=arduinoSDK1 HTTP/1.0\r\n\
x-temboo-authentication: %s:%s\r\n\
Host: %s.temboolive.com\r\n\
Accept: application/xml\r\n\
x-temboo-domain: /%s/master\r\n\
Content-Type: text/plain\r\n\
x-temboo-time: %ld\r\n\
Content-Length: %d\r\n\r\n%s\r\n", 
            comand, m_appKeyName, m_auth, m_accountName, m_accountName, x_temboo_time, len, content);
            //SerialUSB.println(m_buffer);
            
            SerialUSB.println("waiting to fetch ...");
            if(0 == gprs.sendTCPData(m_buffer))
            {
                memset(m_buffer, 0, M_BUFFER_SIZE);
                gprs.serialDebug(m_buffer, time); 

                int command = get_http_cmd(m_buffer);
                if(command == 200)break;
                else if(command == 401 && i == 0)x_temboo_time = get_temboo_time(m_buffer);
                
                //SerialUSB.println(m_buffer);
            }
            else
            {
                SerialUSB.println("send TCP data error");
                return;
            }
        }    
        else
        {
            SerialUSB.println("connect error");
        }
    }
}

int TwilioMessageClass::GetAuth(char *appKey, char *content, char *result)
{   
    HMAC hmac;
    int len = 0;

    char timeBuffer[11];
    uint32toa(x_temboo_time, timeBuffer);
    
    int keyLength = strlen(appKey) + strlen(timeBuffer);
    char key[keyLength + 1];
    char *ptr = content;
    
    strcpy(key, timeBuffer);
    strcat(key, appKey);
    hmac.init((unsigned char *)key, keyLength);
    
    unsigned char buffer[HMAC_BLOCK_SIZE_BYTES];
    int blockCount = 0;
    
    while(*ptr != '\0')
    {
        len++;
        buffer[blockCount++] = *ptr ++;

        if(blockCount == HMAC_BLOCK_SIZE_BYTES)
        {
            hmac.process(buffer, blockCount);
            blockCount = 0;
        }
    }
    hmac.process(buffer, blockCount);
    hmac.finishHex(result);
    
    //SerialUSB.print("len is ");
    //SerialUSB.println(len);
    //SerialUSB.print("result is ");
    //SerialUSB.println(result);
    
    return len;
}

unsigned long TwilioMessageClass::get_temboo_time(char *buffer)
{
    char time[11] = {'\0'};
    char *ptr_str = strstr(buffer, "x-temboo-time: ");
    char *ptr_end = strstr(ptr_str, "\r\n");
    
    if(ptr_str != NULL && ptr_end != NULL)
    {
        int len = ptr_end - ptr_str- strlen("x-temboo-time: ");
        for(int i = 0; i < len; i++)time[i] = *(ptr_str + strlen("x-temboo-time: ") + i);
        return atol(time);
    }
    else 
    {
        return 0;  
    } 
}

unsigned int TwilioMessageClass::get_http_cmd(char *buffer)
{
    char data[4] = {'\0'};
    char *ptr_str = strstr(buffer, "HTTP/1.");
    
    if(ptr_str != NULL)
    {
        for(int i = 0; i < 3; i++)data[i] = *(ptr_str + strlen("HTTP/1.") + i + 2);
        return atol(data);
    }
    else 
    {
        return 0;  
    } 
}

char *TwilioMessageClass::uint32toa(unsigned long value, char* dest)
{
    char* end = dest;
    do
    {
        *end++ = (value % 10) + '0';
    }while(value /= 10);
    *end = '\0';
    end--;

    char c;
    char* begin = dest;
    while(end > begin)
    {
        c = *end;
        *end = *begin;
        *begin = c;
        end--;
        begin++;
    }

    return dest;
}
