#ifndef DATAPACK_HPP
#define DATAPACK_HPP

#include "uart.hpp"
#include "uart_data_pack.hpp"

/**
 * @brief 通讯类，负责通过串口发送反馈和通过串口接受指令
 * 
 * @param uartClass 使用的串口对象
 */
class COMM {
    public:
        COMM(Uart& uartClass);
        ~COMM();

        bool uartSendPack(RoBoFeedBack& pack);
        bool uartRecePack(RoBoCmd& packBuf);
    private:
        Uart& m_uart; // 传入的串口对象
        uint8_t packStatus; // 解包标志位
        size_t rx_cnt; // 记录已接受的字节数
        static const uint16_t crc16Table[256]; // CRC-16/MODBUS查找表
        
        // 缓冲区
        static constexpr size_t RX_TEMP_BUFFER_SIZE = 64; // 从串口单次读取的临时缓冲区大小
        static constexpr size_t RX_PACKET_BUFFER_SIZE = 256; // 组包缓冲区大小
        uint8_t m_rx_temp_buf[RX_TEMP_BUFFER_SIZE]; // 接受缓冲区
        uint8_t m_rx_packet_buf[RX_PACKET_BUFFER_SIZE]; // 组包缓冲区

        // 解包标志位定义
        static constexpr uint8_t WAITING_FOR_HEADER_1 = 0; // 准备接受包头部分1
        static constexpr uint8_t WAITING_FOR_HEADER_2 = 1; // 准备接受包头部分2
        static constexpr uint8_t RECEIVING_DATA = 2; // 正在接受数据和校验和

        uint16_t _createCRC16(const void* bytesBuf, size_t len) const; //计算16位CRC
};

#endif
