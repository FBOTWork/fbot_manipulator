#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from fbot_manipulator_msgs.action import ManipulationTask

class ArmPatrolNode(Node):
    def __init__(self):
        super().__init__('arm_patrol_node')
        self._action_client = ActionClient(self, ManipulationTask, '/fbot_manipulator/manipulation_task')
        
        # Posições de teste seguras (alcance ~0.20m a 0.28m, Z seguro)
        self.targets = [
            {'x': 0.22, 'y':  0.10, 'z': 0.12},
            {'x': 0.25, 'y':  0.00, 'z': 0.15},
            {'x': 0.22, 'y': -0.10, 'z': 0.12},
            {'x': 0.20, 'y':  0.00, 'z': 0.10}
        ]
        self.current_idx = 0
        
        self.get_logger().info('Aguardando servidor de ação...')
        self._action_client.wait_for_server()
        self.get_logger().info('Servidor conectado! Iniciando ciclo de posições.')
        
        self.send_next_goal()

    def send_next_goal(self):
        target = self.targets[self.current_idx]
        self.get_logger().info(f"Enviando para Posição [{self.current_idx + 1}]: x={target['x']}, y={target['y']}, z={target['z']}")
        
        goal_msg = ManipulationTask.Goal()
        goal_msg.task_type = 4
        goal_msg.arm_name = 'interbotix_arm'
        goal_msg.object_id = 'cup'
        goal_msg.object_pose.position.x = target['x']
        goal_msg.object_pose.position.y = target['y']
        goal_msg.object_pose.position.z = target['z']
        goal_msg.object_pose.orientation.w = 1.0
        goal_msg.object_size.x = 0.05
        goal_msg.object_size.y = 0.05
        goal_msg.object_size.z = 0.08

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejeitado! Tentando a próxima em 3s...')
            self.create_timer(3.0, self.timer_next)
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Chegou na posição! Aguardando 3 segundos...')
        # Atualiza o índice para a próxima posição
        self.current_idx = (self.current_idx + 1) % len(self.targets)
        # Timer de 3 segundos de pausa antes de enviar a próxima
        self.create_timer(3.0, self.timer_next)

    def timer_next(self):
        # Garante que o timer rode apenas uma vez por ciclo
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = ArmPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()