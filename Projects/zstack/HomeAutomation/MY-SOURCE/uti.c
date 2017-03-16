#include "uti.h"
#include "ZComDef.h"

uint8 * Int16ToString(uint16 value)
{
  uint8 t = (uint8)(value&0x00FF);
  uint8 t1 = (uint8)(value>>8);
  static uint8 a[4];
  a[3] = (t&0x0F);
  a[2] = (t>>4);
  a[1] = (t1&0x0F);
  a[0] = (t1>>4); 
  for (uint8 i = 0; i < 4; i++){
    if (a[i] > 9){
      a[i] = a[i] + 55;
    }
    else{
      a[i] = a[i] + 48;
    }
  }
  return a;
}

uint8* Int8ToString(uint8 value)
{
  static uint8 a[2];
  a[1] = value&0x0F;
  a[0] = value>>4;
  for (uint8 i = 0; i < 2; i++){
    if (a[i] > 9){
      a[i] = a[i] + 55;
    }
    else{
      a[i] = a[i] + 48;
    }
  }
  return a;
}