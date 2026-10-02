// BXI 板卡输入输出示例，可复制到自己的 .cpp；不提供 main，不加入默认编译。
// 链接 chassis_bxi_transport；安装后为 chassis::chassis_bxi_transport。
// board 由调用方长期持有；先停止/销毁电机和主站，再断电/释放板卡。

#include "bxi/board.h"

#include <memory>
#include <utility>

std::shared_ptr<chassis::BxiPciTransport> board_open(unsigned int bus)
// 输入：bus 为板卡 CAN 通道号，从 0 开始，必须小于 CANFD_DEVICE_NUM。
// 输出：已打开通道的共享指针；不主动上电。通道已占用或设备初始化失败会抛异常。
// 最后一个板卡通道析构时会关闭共享电源，不能在发送一次指令后立即销毁。
{
  return std::make_shared<chassis::BxiPciTransport>(bus);
}

bxi::Result<void> board_power(chassis::BxiPciTransport & board, bool enabled)
// 输入：已打开的 board；enabled=true 上电，false 断电，作用于共享电机电源。
// 输出：Result 成功/失败；失败时读取 result.error()。上电后需由调用方等待设备就绪。
{
  return board.set_motor_power(enabled);
}

bxi::Result<void> board_send(
  chassis::BxiPciTransport & board, const bxi::CanFrame & frame)
// 输入：已打开的 board 和包含 ID、长度、数据、CAN/FD 标志的 frame。
// 输出：Result 成功/失败；成功仅代表发送接口接受报文，不是设备动作完成或应答。
{
  return board.send(frame);
}

void board_on_receive(chassis::BxiPciTransport & board, bxi::ReceiveHandler on_frame)
// 输入：board 和收到帧时执行的回调；空回调清除接收处理。
// 输出：无同步返回值；收到的 CanFrame 通过回调输出，回调应快速返回。
// 若 board 已交给 CANopen 主站管理，不要替换它注册的回调；共享数据需自行同步。
{
  board.set_receive_handler(std::move(on_frame));
}

// 调用片段：
// auto board = board_open(1);
// const auto powered = board_power(*board, true);
// if (!powered) { /* 用 powered.error() 处理失败。 */ }
// 之后复用 board；结束时先停止电机，再检查 board_power(*board, false) 的结果。
