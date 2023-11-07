//#include "fdcandriver.h"
//
//int main(int argc, char *argv[])
//{
//
//  /* Add callback for address 0x11 */
//  FDCAN_addCallback(&hfdcan1, 0x11, &processData1);
//
//  /* Start FDCAN */
//  FDCAN_Start(&hfdcan1);
//
//  while (1)
//  {
//    /* Infinite loop */
//    double current = 3.12321;
//    uint8_t *TxData = &data;
//
//    FDCAN_sendData(&hfdcan1, 0x33, TxData);
//
//    HAL_Delay(1000);
//  }
//
//}
