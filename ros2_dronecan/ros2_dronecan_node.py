#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import dronecan
import os
from ament_index_python.packages import get_package_share_directory
from ros2_dronecan_interfaces.msg import MotorControllerStatus, MotorStatus, BinStatus
from ros2_dronecan_interfaces.srv import SetBin
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class Ros2DronecanNode(Node):
    def __init__(self):
        super().__init__("ros2_dronecan_node")
        self.declare_parameter("can_interface", "vcan1")
        self.declare_parameter("node_id", 127)
        self.declare_parameter("bitrate", 1000000)
        self.declare_parameter("enable_nodeid_server", True)
        self.declare_parameter("battery_current_offset", 0.0)
        self.declare_parameter("battery_voltage_offset", 0.0)
        self.declare_parameter("battery_calculate_percentage", True)
        self.declare_parameter("battery_negative_charge", True)
        self.declare_parameter("battery_cell_empty", 3.3)
        self.declare_parameter("battery_cell_full", 4.2)
        self.declare_parameter("battery_cell_num", 6)
        self.declare_parameter("battery_report_temperature", False)
        self.declare_parameter("enable_battery", True)
        self.declare_parameter("enable_motor_controller_status", True)
        self.can_interface_ = self.get_parameter("can_interface").value
        self.node_id_ = self.get_parameter("node_id").value
        self.bitrate_ = self.get_parameter("bitrate").value
        self.enable_nodeid_server_ = self.get_parameter("enable_nodeid_server").value
        self.battery_current_offset_ = self.get_parameter("battery_current_offset").value
        self.battery_voltage_offset_ = self.get_parameter("battery_voltage_offset").value
        self.battery_calculate_percentage_ = self.get_parameter("battery_calculate_percentage").value
        self.battery_negative_charge_ = self.get_parameter("battery_negative_charge").value
        self.battery_cell_empty_ = self.get_parameter("battery_cell_empty").value
        self.battery_cell_full_ = self.get_parameter("battery_cell_full").value
        self.battery_cell_num_ = self.get_parameter("battery_cell_num").value
        self.battery_report_temperature_ = self.get_parameter("battery_report_temperature").value
        self.battery_empty_ = self.battery_cell_empty_ * self.battery_cell_num_
        self.battery_full_ = self.battery_cell_full_ * self.battery_cell_num_
        self.enable_battery_ = self.get_parameter("enable_battery").value
        self.enable_motor_controller_ = self.get_parameter("enable_motor_controller_status").value
        self.node_monitor_ = None
        self.dynamic_node_id_allocator_ = None

        self.client_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = None

        self.get_logger().info("ROS2 dronecan node has been started")

        self.node_info_ = dronecan.uavcan.protocol.GetNodeInfo.Response()
        self.node_info_.name = "org.dronecan.rosdronecan"
        self.node_info_.software_version.major = 0
        self.node_info_.software_version.minor = 1
        self.node_info_.hardware_version.unique_id = b'12345'
        self.dronecan_node_ = dronecan.make_node(self.can_interface_, node_id=self.node_id_, bitrate=self.bitrate_,
                                                node_info=self.node_info_, mode=dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL)
        self.node_monitor_ = dronecan.app.node_monitor.NodeMonitor(self.dronecan_node_)
        if self.enable_nodeid_server_:
            self.get_logger().info("Starting dronecan dynamic id allocator")
            self.dynamic_node_id_allocator_ = dronecan.app.dynamic_node_id.CentralizedServer(self.dronecan_node_, self.node_monitor_)
        self.dronecan_node_.health = dronecan.uavcan.protocol.NodeStatus().HEALTH_OK
        if self.enable_battery_:
            self.battery_state_publisher_ = self.create_publisher(BatteryState, "/battery_state", 10)
            self.dronecan_node_.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.node_battery_status_callback)
        if self.enable_motor_controller_:
            self.motor_controller_status_publisher_ = self.create_publisher(MotorControllerStatus, "/motor_controller_status", 10)
            self.dronecan_node_.add_handler(dronecan.com.zxdynamics.controller.MotorControllerStatus, self.node_motor_controller_status_callback)
            self.set_bin_service_ = self.create_service(SetBin, 'set_bin', self.set_bin_callback, callback_group=self.client_cb_group)
        self.run_once_ = True
        self.timer = self.create_timer(0.004, self.spin_dronecan, callback_group=self.timer_cb_group)
        self.get_logger().info("ROS2 Dronecan finished initializing")

    def node_motor_controller_status_callback(self, event):
        # self.get_logger().info(dronecan.to_yaml(event))
        message = MotorControllerStatus()
        message.node_id = event.transfer.source_node_id
        for i in range(0, len(event.message.motor_status)):
            motor_status = MotorStatus()
            motor_status.motor_current = event.message.motor_status[i].motor_current
            motor_status.motor_voltage = event.message.motor_status[i].motor_voltage
            message.motor_status.append(motor_status)
        for i in range(0, len(event.message.bin_status)):
            bin_status = BinStatus()
            bin_status.bin_state = event.message.bin_status[i].bin_state
            message.bin_status.append(bin_status)
        self.motor_controller_status_publisher_.publish(message)
        # if self.run_once_:
        #     self.send_setbin_command(17, 0, 0)
        #     self.send_setbin_command(17, 1, 0)
        #     self.run_once_ = False
        pass

    def node_battery_status_callback(self, event):
        # self.get_logger().info(dronecan.to_yaml(event))
        battery_status = BatteryState()
        battery_status.voltage = event.message.voltage + self.battery_voltage_offset_
        battery_status.current = event.message.current + self.battery_current_offset_
        if self.battery_report_temperature_:
            battery_status.temperature_ = event.message.temperature - 273.15
        else:
            battery_status.temperature = 0.0
        if battery_status.current > 0:
            if self.battery_negative_charge_:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            else:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            if self.battery_negative_charge_:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if self.battery_calculate_percentage_:
            battery_status.percentage = (battery_status.voltage - self.battery_empty_) / (self.battery_full_ - self.battery_empty_) * 100
            if battery_status.percentage > 100:
                battery_status.percentage = 100.0
            if battery_status.percentage < 0:
                battery_status.percentage = 0.0
        else:
            battery_status.percentage = event.message.state_of_charge_pct
        self.battery_state_publisher_.publish(battery_status)
    
    def send_setbin_command(self, node_id, bin_id, state):
        message = dronecan.com.zxdynamics.controller.SetBin.Request()
        message.bin_id = bin_id
        message.state = state
        self.dronecan_node_.request(message, node_id, self.send_command_callback)
    
    def send_command_callback(self, arg):
        if arg is not None:
            self.get_logger().info("Dronecan command sent")
        else:
            self.get_logger().info("Dronecan command timeout")

    def set_bin_callback(self, request, response):
        self.get_logger().info(f"set_bin to node_id = {request.node_id}, bin_id = {request.bin_id}, command = {request.bin_command}")
        try:
            self.send_setbin_command(request.node_id, request.bin_id, request.bin_command)
            # self.send_setbin_command(17, 1, 1)
        except:
            self.get_logger().error("Error sending dronecan command")
            response.success = False
            return response
        response.success = True
        return response

    def spin_dronecan(self):
        # self.get_logger().info("dronecan spin")
        self.dronecan_node_.spin(timeout=0.2)
        # self.dronecan_node_.spin(0)


def main(args=None):
    dronecan.load_dsdl(os.path.join(get_package_share_directory('ros2_dronecan'), 'dsdl', 'com'))
    rclpy.init(args=args)
    node = Ros2DronecanNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok():
        try:
            # node.spin_dronecan()
            # rclpy.spin(node)
            executor.spin()
        except:
            pass
    # self.dronecan_node_.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
