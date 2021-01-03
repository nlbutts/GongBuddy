#ifndef COMMON_STRUCTS_H_
#define COMMON_STRUCTS_H_

// Data to pass to the Lora Thread
typedef struct LoraData
{
    uint8_t * buf;
    uint32_t bytesWritten;
} LoraData_t;


#endif //COMMON_STRUCTS_H_