import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy.context
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class StudentFacingServer(Node):
    def __init__(self, drone_facing_node, cmd_srv_name, context):
        super().__init__('custom_arming_student_server', context=context)
        self.drone_node = drone_facing_node
        
        # Must separate service callbacks to handle synchronous backend calls
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.arm_srv = self.create_service(
            CommandBool, 
            cmd_srv_name, 
            self.arm_callback,
            callback_group=self.cb_group
        )
        self.mode_srv = self.create_service(
            SetMode,
            f'/drones/edu11/set_mode',
            self.mode_callback,
            callback_group=self.cb_group
        )
        self.raw_arm_srv = self.create_service(
            CommandBool,
            f'/drones/edu11/mavros_node/arming',
            self.raw_arm_callback,
            callback_group=self.cb_group
        )
        self.takeoff_srv = self.create_service(
            CommandTOL,
            f'/drones/edu11/mavros_node/takeoff',
            self.takeoff_callback,
            callback_group=self.cb_group
        )
        self.land_srv = self.create_service(
            CommandTOL,
            f'/drones/edu11/mavros_node/land',
            self.land_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info(f"Student-facing Arming Service natively exposed on Domain 0 -> {cmd_srv_name}")
        self.get_logger().info(f"Student-facing SetMode Service natively exposed on Domain 0 -> /drones/edu11/set_mode")

    def raw_arm_callback(self, request, response):
        self.get_logger().info(f"GUI requested raw arm (value={request.value})")
        success = self.drone_node.call_arm(request.value)
        response.success = success
        response.result = 0 if success else 1
        return response

    def takeoff_callback(self, request, response):
        self.get_logger().info(f"GUI requested takeoff")
        success = self.drone_node.call_takeoff(request)
        response.success = success
        response.result = 0 if success else 1
        return response

    def land_callback(self, request, response):
        self.get_logger().info(f"GUI requested land")
        success = self.drone_node.call_land(request)
        response.success = success
        response.result = 0 if success else 1
        return response

    def mode_callback(self, request, response):
        self.get_logger().info(f"Student requested mode (mode={request.custom_mode})")
        success = self.drone_node.call_set_mode(request.custom_mode)
        response.mode_sent = success
        return response

    def arm_callback(self, request, response):
        self.get_logger().info(f"Student requested arming (value={request.value})")
        
        # Pass the request DIRECTLY to the drone-facing node memory instance! No domain bridge!
        success = self.drone_node.execute_sequence(request.value)
        response.success = success
        response.result = 0 if success else 1
        
        if success:
            self.get_logger().info("Successfully reported arming complete to Student.")
        else:
            self.get_logger().error("Failed to arm Drone sequence. Reporting to Student.")

        return response

class DroneFacingClient(Node):
    def __init__(self, mavros_ns, context):
        super().__init__('custom_arming_drone_client', context=context)
        self.mavros_ns = mavros_ns
        
        self.cb_group = MutuallyExclusiveCallbackGroup()
        
        self.set_mode_cli = self.create_client(
            SetMode, 
            f'{mavros_ns}/set_mode', 
            callback_group=self.cb_group
        )
        self.arm_cli = self.create_client(
            CommandBool, 
            f'{mavros_ns}/mavros_node/arming',
            callback_group=self.cb_group
        )
        self.takeoff_cli = self.create_client(
            CommandTOL, 
            f'{mavros_ns}/mavros_node/takeoff',
            callback_group=self.cb_group
        )
        self.land_cli = self.create_client(
            CommandTOL, 
            f'{mavros_ns}/mavros_node/land',
            callback_group=self.cb_group
        )

        self.get_logger().info(f"Drone-facing MAVROS backend locked on Domain 11 -> {mavros_ns}")

    def execute_sequence(self, arm_value):
        if arm_value is True:
            # 1. LOITER
            self.get_logger().info("Setting mode to LOITER...")
            if not self.call_set_mode("LOITER"): return False
            
            # 2. ARM
            self.get_logger().info("Sending Arming command...")
            if not self.call_arm(True): return False
            
            # 3. GUIDED
            self.get_logger().info("Setting mode to GUIDED...")
            if not self.call_set_mode("GUIDED"): return False
            
            self.get_logger().info("Full arm sequence confirmed: LOITER -> ARM -> GUIDED")
            return True
        else:
            self.get_logger().info("Sending Disarm command...")
            return self.call_arm(False)

    def call_set_mode(self, mode):
        if not self.set_mode_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service {self.set_mode_cli.srv_name} unreachable on Domain 11.")
            return False
            
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_cli.call(req)
        return future.mode_sent

    def call_arm(self, value):
        if not self.arm_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service {self.arm_cli.srv_name} unreachable on Domain 11.")
            return False
            
        req = CommandBool.Request()
        req.value = value
        future = self.arm_cli.call(req)
        return future.success

    def call_takeoff(self, req):
        if not self.takeoff_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service {self.takeoff_cli.srv_name} unreachable on Domain 11.")
            return False
            
        future = self.takeoff_cli.call(req)
        return future.success

    def call_land(self, req):
        if not self.land_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service {self.land_cli.srv_name} unreachable on Domain 11.")
            return False
            
        future = self.land_cli.call(req)
        return future.success

def main(args=None):
    rclpy.init(args=args)

    # 1. DUAL-DOMAIN ARCHITECTURE! We completely skip the broken domain_bridge services array !
    # By initializing two totally isolated Contexts, this python script can simultaneously exist on two networks physically!
    
    student_context = rclpy.context.Context()
    student_context.init(domain_id=0)  # Bound to Student Network natively
    
    drone_context = rclpy.context.Context()
    drone_context.init(domain_id=11)   # Bound to Drone Network natively
    
    mavros_ns = '/drones/edu11'
    cmd_srv_name = '/drone11/cmd/arming'

    # 2. Spin up exactly one node on Domain 11 and one on Domain 0 using zero Serialization between them!
    drone_client = DroneFacingClient(mavros_ns, context=drone_context)
    student_server = StudentFacingServer(drone_client, cmd_srv_name, context=student_context)

    # 3. Run BOTH on a shared fast pool
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(student_server)
    executor.add_node(drone_client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        student_server.destroy_node()
        drone_client.destroy_node()
        rclpy.shutdown()
        student_context.try_shutdown()
        drone_context.try_shutdown()

if __name__ == '__main__':
    main()
