#ifndef Cloud4Bees_h
#define Cloud4Bees_h

#include "application.h"

static const int NUMBER_OF_FIELDS = 9;
static const char CLOUD4BEES_API [] = "cloud.4bees.at";
static const uint32_t DEFAULT_RESPONSE_WAIT_TIME = 1500;


namespace Cloud4BeesLibrary
{
    class Cloud4Bees
    {
        private:
            TCPClient client;
            String key;
            String values [NUMBER_OF_FIELDS];
            String composeQuery();
            uint32_t timeout;
        public:
            Cloud4Bees(String apiKey);
            bool recordValue(int fieldId, String fieldValue);
            bool sendValues();
            void setConnectionTimeout(uint32_t milliseconds);

    };
}

#endif