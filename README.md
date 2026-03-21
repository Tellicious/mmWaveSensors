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

## LD2410S
- Initialization
    ```cpp
        uint8_t txBuffer[128], length;
        
        // Enable configuration mode
        length = LD2410S_buildSetConfigOn(txBuffer, 128);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        LD2410S_Status_t status;
        status = LD2410S_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2410S_SUCCESS) {
            return status;
        }
        
        // Set output mode
        length = LD2410S_buildSetOutputMode(txBuffer, 128, LD2410S_OUTPUT_STANDARD);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        status = LD2410S_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2410S_SUCCESS) {
            return status;
        }
        
        // Set common parameters
        LD2410S_CommonParams_t params = {
            .farthest_gate = 15,
            .nearest_gate = 0,
            .unattended_delay_s = 60,
            .status_report_hz_x10 = 10,  // 1.0 Hz
            .distance_report_hz_x10 = 10, // 1.0 Hz
            .response = LD2410S_RESPONSE_NORMAL
        };
        length = LD2410S_buildSetCommonParams(txBuffer, 128, &params);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        status = LD2410S_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2410S_SUCCESS) {
            return status;
        }
        
        // End configuration mode
        length = LD2410S_buildSetConfigOff(txBuffer, 128);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        status = LD2410S_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2410S_SUCCESS) {
            return status;
        }
    ```
- Read data
    ```cpp
        LD2410S_Status_t status;
        LD2410S_Report_t report;
        status = LD2410S_parseRadarData(rxBuffer, size, &report);
        if (status != LD2410S_SUCCESS) {
            return status;
        }
    ```

## LD2420
- Initialization
    ```cpp
        uint8_t txBuffer[128], length;
        
        // Enable configuration mode
        length = LD2420_buildSetConfigOn(txBuffer, 128);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        LD2420_Status_t status;
        status = LD2420_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2420_SUCCESS) {
            return status;
        }
        
        // Set common parameters
        LD2420_CommonParams_t params = {
            .min_gate = 0,
            .max_gate = 15,
            .disappear_delay_s = 60
        };
        length = LD2420_buildSetCommonParams(txBuffer, 128, &params);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        status = LD2420_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2420_SUCCESS) {
            return status;
        }
        
        // End configuration mode
        length = LD2420_buildSetConfigOff(txBuffer, 128);
        if(length > 0){
            HAL_UART_Transmit(&huart1, txBuffer, length, portMAX_DELAY);
        }
        status = LD2420_checkSetACK(txBuffer, length, rxBuffer, rxSize);
        if (status != LD2420_SUCCESS) {
            return status;
        }
    ```
- Read data
    ```cpp
        LD2420_Status_t status;
        LD2420_Target_t target;
        status = LD2420_parseRadarData(rxBuffer, size, &target);
        if (status != LD2420_SUCCESS) {
            return status;
        }
    ```
