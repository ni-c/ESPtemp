#include <unity.h>

void runBackoffTests();
void runFirmwareTests();

int main()
{
    UNITY_BEGIN();
    runBackoffTests();
    runFirmwareTests();
    return UNITY_END();
}
