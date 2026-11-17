#ifndef BSP_USB_DEBUG_H
#define BSP_USB_DEBUG_H
#include <stdint.h>

/**
  * @brief          USB 连接初始化或释放时清空调试接收缓存
  * @retval         无
  */
void bsp_usb_debug_rx_reset(void);
/**
  * @brief          在 USB 接收回调中复制调试字节，不执行命令解析或控制
  * @param[in]      data: 接收数据
  * @param[in]      length: 数据字节数
  * @retval         无；缓冲区满时报告溢出，等待任务重新同步行边界
  */
void bsp_usb_debug_rx_push(const uint8_t *data, uint32_t length);
/**
  * @brief          在任务中非阻塞读取一个 USB 调试字节
  * @param[out]     byte: 接收字节
  * @retval         1 读到字节；0 无数据；-1 溢出；-2 连接缓存已重置
  */
int bsp_usb_debug_get_byte(uint8_t *byte);
#endif
