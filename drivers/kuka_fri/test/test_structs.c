
#include "friComm.h"


int main()
{
  FRI_PREPARE_CHECK_BYTE_ORDER
  return (FRI_CHECK_SIZES_OK && FRI_CHECK_BYTE_ORDER_OK)- 1;
}
