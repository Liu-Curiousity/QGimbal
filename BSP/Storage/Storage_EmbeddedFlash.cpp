//
// Created by 26757 on 25-4-30.
//

#include "Storage_EmbeddedFlash.h"
#include <algorithm>

// FLASH中存放数据的区域,也用于烧录时擦除这些数据,注意:只读!
// __attribute__((section(".flash_data"))) uint8_t storage_buffer_to_erase[0x4000];

Storage_EmbeddedFlash storage{0x8004000, 0x4000}; // 开头16KB作为储存空间

void Storage_EmbeddedFlash::write(const uint32_t addr, void *buff, uint32_t count) {
    // 限制写入范围
    count = std::min(count, storage_size - addr);

    // 读出当前页数据
    for (uint32_t i = 0; i < storage_size; i++) {
        page_buffer[i] = *reinterpret_cast<volatile uint8_t *>(STORAGE_ADDRESS_BASE + i);
    }

    for (uint32_t i = 0; i < count; i++) {
        page_buffer[addr + i] = static_cast<const uint8_t *>(buff)[i];
    }

    HAL_FLASH_Unlock(); //解锁Flash

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; // 标明Flash执行页面只做擦除操作
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Sector = FLASH_SECTOR_1; // 声明要擦除的地址
    EraseInitStruct.NbSectors = 1;           // 说明要擦除的扇区数
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t PageError = 0;                                              // 设置PageError,如果出现错误这个变量会被设置为出错的FLASH地址
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) {} // 调用擦除函数擦除

    for (uint32_t i = 0; i < storage_size / 4; i++) {
        while (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STORAGE_ADDRESS_BASE + i * 4,
                                           *reinterpret_cast<uint64_t *>(page_buffer + i * 4))) {}
    }

    HAL_FLASH_Lock(); //锁住Flash
}

void Storage_EmbeddedFlash::read(const uint32_t addr, void *buff, uint32_t count) {
    // 限制读取范围
    count = std::min(count, storage_size - addr);

    auto s = reinterpret_cast<uint8_t *>(STORAGE_ADDRESS_BASE + addr);
    auto buff_ = static_cast<uint8_t *>(buff);
    for (uint32_t i = 0; i < count; i++) {
        *(buff_++) = *(s++);
    }
}
