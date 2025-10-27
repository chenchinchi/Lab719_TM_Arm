#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetIO,SetPositions
import math


class TMRobot(Node):
    '''
    Techman Robot ROS2 通訊節點
    '''
    def __init__(self, node_name='tm_robot_node'):
        super().__init__(node_name)
        self.get_logger().info(f'✅ {node_name} initialized')

        # 建立 service client
        self.set_io = self.create_client(SetIO, 'set_io')
        self.set_pos = self.create_client(SetPositions, 'set_positions')

        # 等待 service 可用
        self._wait_for_service(self.set_io, 'set_io')
        self._wait_for_service(self.set_pos, 'set_positions')
        # 定義常數
        self.motion_types = {
                'PTP_J': 1,
                'PTP_T': 2,
                'LINE_T': 4,
                'CIRC_T': 6,
                'PLINE_T': 8
            }

    # ===============================
    # 🕹️ 公用函式
    # ===============================
    def _wait_for_service(self, client, name: str):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'⏳ Waiting for service [{name}] to be available...')

    def _deg_to_rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    def _mm_to_m(self, mm: float) -> float:
        return mm / 1000.0

    # ===============================
    # 💡 IO 控制函式
    # ===============================
    def IO_Set(self, module: int, io_type: int, pin: int, state: float) -> bool:
        '''
        設定 TM 機械手臂的 IO 狀態
        '''
        req = SetIO.Request()
        req.module = module
        req.type = io_type
        req.pin = pin
        req.state = state

        self.get_logger().info(
            f'⚙️ [SetIO] module={module}, type={io_type}, pin={pin}, state={state}'
        )

        future = self.set_io.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'✅ SetIO response: ok={result.ok}')
            return result.ok
        else:
            self.get_logger().error('❌ SetIO service call failed')
            return False

    # ===============================
    # 🦾 位置控制函式
    # ===============================
    def Pos_Set(
        self,
        motion_type: str,
        position: list[float],
        velocity: float = 1.0,
        acc_time: float = 0.5,
        blend_percentage: int = 0,
        fine_goal: bool = True
    ) -> bool:
        '''
        設定 TM 機械手臂目標位置
        支援 motion_type: PTP_J, PTP_T, LINE_T, CIRC_T, PLINE_T
        '''
        req = SetPositions.Request()

        # 單位轉換
        position = [
            self._mm_to_m(position[0]),
            self._mm_to_m(position[1]),
            self._mm_to_m(position[2]),
            self._deg_to_rad(position[3]),
            self._deg_to_rad(position[4]),
            self._deg_to_rad(position[5]),
        ]

        req.motion_type = self.motion_types.get(motion_type.upper(), 1)
        req.positions = position
        req.velocity = velocity
        req.acc_time = acc_time
        req.blend_percentage = blend_percentage
        req.fine_goal = fine_goal

        self.get_logger().info(
            f'⚙️ [SetPositions] type={motion_type}, pos={position}, vel={velocity}, acc={acc_time}, blend={blend_percentage}, fine={fine_goal}'
        )

        future = self.set_pos.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'✅ SetPositions response: ok={result.ok}')
            return result.ok
        else:
            self.get_logger().error('❌ SetPositions service call failed')
            return False

    # ===============================
    # 🧭 測試函式
    # ===============================
    def test_io(self):
        try:
            val = float(input("Enter IO state (0=OFF, 1=ON): "))
            success = self.IO_Set(
                module=1,
                io_type=val,
                pin=0,
                state=val
            )
            msg = "✅ IO successfully set" if success else "❌ IO setting failed"
            self.get_logger().info(msg)
        except ValueError:
            self.get_logger().error("Invalid input, please enter 0 or 1")

    def test_move(self):
        input("Press Enter to move robot to specified position...")
        success = self.Pos_Set(
            motion_type='LINE_T',
            position=[-98.56, 299.5, 416.84, 170.90, -2.08, -176.25],
            velocity=1.0,
            acc_time=0.5,
            blend_percentage=0,
            fine_goal=True
        )
        msg = "✅ Robot successfully moved" if success else "❌ Move command failed"
        self.get_logger().info(msg)




def main(args=None):
    rclpy.init(args=args)
    tm_robot = TMRobot()
    tm_robot.test_io()
    tm_robot.test_move()
    tm_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
