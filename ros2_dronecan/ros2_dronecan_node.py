#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import dronecan

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
        self.node_monitor_ = None
        self.dynamic_node_id_allocator_ = None

        self.battery_state_publisher_ = self.create_publisher(BatteryState, "/battery_state", 10)
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
        self.dronecan_node_.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.node_battery_status_callback)
        while rclpy.ok():
            try:
                self.dronecan_node_.spin(timeout=0.2)
            except:
                pass
        self.dronecan_node_.close()

    def node_battery_status_callback(self, event):
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


def main(args=None):
    rclpy.init(args=args)
    node = Ros2DronecanNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
