#include <Arduino.h>
#include <timeout.h> 

unsigned char msTimeout(unsigned long time, unsigned long timeout)
{
  return (millis() - time >= timeout);
}

unsigned char secTimeout(unsigned long time, unsigned long timeout)
{
  return (millis() - time >= (timeout * 1000L) );
}

unsigned char csTimeout(unsigned long time, unsigned long timeout)
{
  return (millis() - time >= (timeout * 100L) );
}