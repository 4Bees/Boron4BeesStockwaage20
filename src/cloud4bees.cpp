#include "cloud4bees.h"

Cloud4BeesLibrary::Cloud4Bees::Cloud4Bees(String apiKey)
{
    key = apiKey;
    timeout= DEFAULT_RESPONSE_WAIT_TIME;
}

void Cloud4BeesLibrary::Cloud4Bees::setConnectionTimeout(uint32_t milliseconds)
{
    this->timeout = milliseconds;   
}

bool Cloud4BeesLibrary::Cloud4Bees::recordValue(int fieldId, String fieldValue)
{
    if(fieldId < 1 || fieldId > NUMBER_OF_FIELDS) {
        return false;
    }
    values[fieldId-1] = fieldValue;
    return true;
}

String Cloud4BeesLibrary::Cloud4Bees::composeQuery()
{
    String result = String ("/update?key=" + key);
    for(int ct = 0; ct < NUMBER_OF_FIELDS; ct++) 
    {
        if(values[ct] != NULL) 
        {
            String fieldParameter = String("&field" + String((ct+1), DEC) + "=" + values[ct]);
            result.concat(fieldParameter);
            //values[ct] = NULL;
            values[ct] = "0";
        }
    }
    return result;
}

bool Cloud4BeesLibrary::Cloud4Bees::sendValues()
{
    if (client.connect(CLOUD4BEES_API, 80))
    {
        bool ret = false;
        client.println(String("GET " + this->composeQuery() + " HTTP/1.0"));
        client.println(String("Host: " + String(CLOUD4BEES_API)));
        client.println("Content-Length: 0");
        client.println();
               
        uint32_t lastRead = millis();       
        //while (!ret && (millis() - lastRead) < this->timeout) {
            while (client.available() > 0) {
                ret = true;
                client.flush();
            }
           
        //}
        
        client.flush();
        client.stop();
        return ret;
    }
    else
    {
        Serial.println("Werte konnten nicht gesendet werden:");
        return false;
    }
}