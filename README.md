# mmWave sensor frames encoding / decoding library
## LD2410B
- Initialization
    ```cpp
        uint8_t txBuffer[60], length;
        length=LD2410B_buildSetConfigOn(txBuffer, 60);
        if(length>0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        LD2410B_Status_t status;
        status = LD2410B_checkSetACK(txBuffer, txSize, rxBuffer, rxSize)
        if (status != LD2410B_SUCCESS) {
            return status
        }


        length= LD2410B_buildSetAuxControl(txBuffer, 60, LD2410B_LIGHT_SENSE_OFF, 60, LD2410B_AUX_OUT_DEFAULT_LOW);
        if(length>0){
            HAL_UART_Transmit(&huart1, txBuffer,length, portMAX_DELAY);
        }
        status = LD2410B_checkSetACK(txBuffer, txSize, rxBuffer, rxSize)
        if (status != LD2410B_SUCCESS) {
            return status
        }

        length=LD2410B_buildSetConfigOff(txBuffer, 60);
        if(length>0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        LD2410B_Status_t status;
        status = LD2410B_checkSetACK(txBuffer, txSize, rxBuffer, rxSize)
        if (status != LD2410B_SUCCESS) {
            return status
        }
    ```
- Read data
    ```cpp
        LD2410B_Status_t status;
        LD2410B_Target_t target;
        status = LD2410B_parseRadarData(rxBuffer, size, &target);
        if (status != LD2410B_SUCCESS) {
            return status
        }
    ```
