#ifndef W25Q64_FLASH_H
#define W25Q64_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int W25Q64_Init(void);

int W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);
int W25Q64_Write(uint32_t addr, const uint8_t *buf, uint32_t len);
int W25Q64_EraseSector4K(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif
