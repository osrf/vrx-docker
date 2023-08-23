import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import ParamVec
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped

class GhostShip(Node):
    def __init__(self):
        print('Starting node...')
        super().__init__('ghost_ship')
        self.task_type = ""
        self.task_state = ""
        self.task_sub = self.create_subscription(ParamVec, '/vrx/task/info', 
                                                 self.taskCB, 10)
        
        pub_qos = QoSProfile(depth=1, 
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.right_thrust_pub = self.create_publisher(Float64, 
                                                      '/wamv/thrusters/right/thrust', 
                                                      qos_profile=pub_qos)
        self.left_thrust_pub = self.create_publisher(Float64, 
                                                     '/wamv/thrusters/left/thrust', 
                                                     qos_profile=pub_qos)
        self.thrust_msg = Float64()
        self.thrust_msg.data = 30.0
        self.loopCount = 0
        
        self.landmark_pub = self.create_publisher(PoseStamped, 
                                                  '/vrx/perception/landmark', 
                                                  qos_profile=pub_qos)
        self.landmark_msg = PoseStamped()
        self.landmark_msg.header.stamp = self.get_clock().now().to_msg()
        self.landmark_msg.header.frame_id = 'mb_marker_buoy_red'
        self.landmark_msg.pose.position.x = -33.7227024
        self.landmark_msg.pose.position.y = 150.67402097
        self.landmark_msg.pose.position.z = 0.0

        self.create_timer(0.5, self.sendCmds)

    def taskCB(self, msg):
        task_info = msg.params
        for p in task_info:
            if p.name == 'name':
                self.task_type = p.value .string_value
            if p.name == 'state':
                self.task_state = p.value.string_value

    def moveForward(self):
        self.right_thrust_pub.publish(self.thrust_msg)
        self.left_thrust_pub.publish(self.thrust_msg)
        self.loopCount += 1

    def stop(self):
        self.thrust_msg.data = 0.0
        self.right_thrust_pub.publish(self.thrust_msg)
        self.left_thrust_pub.publish(self.thrust_msg)

    def publishBuoyLoc(self):
        if self.loopCount > 10:
            self.landmark_pub.publish(self.landmark_msg)
            self.loopCount = 0
        self.loopCount += 1



    def sendCmds(self):
        if rclpy.ok():
            match self.task_type:
                case "perception":
                    if self.task_state == ("initial" or "ready"):
                        print("Waiting for perception task to start...")
                    elif self.task_state == "running":
                        self.publishBuoyLoc()
                    elif self.task_state == "finished":
                        print("Task ended...")
                        rclpy.shutdown()
                case "stationkeeping":
                    if self.task_state == ("initial" or "ready"):
                        print("Waiting for stationkeeping task to start...")
                    elif self.task_state == "running":
                        if self.loopCount < 10:
                            self.moveForward()
                        else:
                            self.stop()
                    elif self.task_state == "finished":
                        print("Task ended...")
                        rclpy.shutdown()
                case _:
                    print(self.task_state)
                    if self.task_state == ("initial" or "ready"):
                        print("Waiting for default task to start...")
                    elif self.task_state == "running":
                        self.right_thrust_pub.publish(self.thrust_msg)
                        self.left_thrust_pub.publish(self.thrust_msg)
                    elif self.task_state == "finished":
                        print("Task ended...")
                        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    ghost_ship = GhostShip()
    rclpy.spin(ghost_ship)


if __name__ == '__main__':
    main()
