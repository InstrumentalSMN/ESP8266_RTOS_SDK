#include "common.h"

//nombre del tag para ESPLOG
char *TAG = "main";

//implementacion ftoa()
static const double rounders[MAX_PRECISION + 1] = {
   0.5,            // 0
   0.05,           // 1
   0.005,          // 2
   0.0005,         // 3
   0.00005,        // 4
   0.000005,       // 5
   0.0000005,      // 6
   0.00000005,     // 7
   0.000000005,    // 8
   0.0000000005,   // 9
   0.00000000005   // 10
};


char* floatToString( float value, char* result, int32_t precision )
{
   char * ptr = result;
   char * p = ptr;
   char * p1;
   char c;
   long intPart;

   // check precision bounds
   if (precision > MAX_PRECISION)
      precision = MAX_PRECISION;

   // sign stuff
   if (value < 0) {
      value = -value;
      *ptr++ = '-';
   }

   if (precision < 0) { // negative precision == automatic precision guess
      if (value < 1.0) precision = 6;
      else if (value < 10.0) precision = 5;
      else if (value < 100.0) precision = 4;
      else if (value < 1000.0) precision = 3;
      else if (value < 10000.0) precision = 2;
      else if (value < 100000.0) precision = 1;
      else precision = 0;
   }

   // round value according the precision
   if (precision)
      value += rounders[precision];

   // integer part...
   intPart = value;
   value -= intPart;

   if (!intPart)
      *ptr++ = '0';
   else {
      // save start pointer
      p = ptr;

      // convert (reverse order)
      while (intPart) {
         *p++ = '0' + intPart % 10;
         intPart /= 10;
      }

      // save end pos
      p1 = p;

      // reverse result
      while (p > ptr) {
         c = *--p;
         *p = *ptr;
         *ptr++ = c;
      }

      // restore end pos
      ptr = p1;
   }

   // decimal part
   if (precision) {
      // place decimal point
      *ptr++ = '.';

      // convert
      while (precision--) {
         value *= 10.0;
         c = value;
         *ptr++ = '0' + c;
         value -= c;
      }
   }

   // terminating zero
   *ptr = 0;

   return result;
}
