import rospy
from vrx_gazebo.msg import Task
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geographic_msgs.msg import GeoPoseStamped


class SimpleNode:
    def __init__(self):
        print("Starting node...")
        rospy.init_node('send_commands')
        self.rate = rospy.Rate(10)
        self.loopCount = 0

        self.imageWAMV = 0

        self.taskInfo = 0
        self.taskType = Task()

        # Send a thrust command
        self.thrustVal = 1.0

        # Send a buoy location
        self.buoyLoc = GeoPoseStamped()

        # Basic subscribers
        self.taskSub = rospy.Subscriber('/vrx/task/info', Task, self.taskCB)
        self.imageSub = rospy.Subscriber('/wamv/sensors/cameras/middle_right_camera/image_raw', Image, self.imageCB)

        # Basic publishers
        self.rightThrustPub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)
        self.leftThrustPub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)

        self.buoyLocationPub = rospy.Publisher('/vrx/perception/landmark', GeoPoseStamped, queue_size=1)

    def taskCB(self, _data):
        self.taskInfo = _data

    def imageCB(self, _data):
        self.imageWAMV = _data

    def calcBuoyLoc(self):
        print("Publish buoy location")

        # Populate message with basic info
        self.buoyLoc.header.frame_id = "red_0"
        self.buoyLoc.header.stamp = rospy.get_rostime()
        self.buoyLoc.pose.position.latitude = -33.7227024
        self.buoyLoc.pose.position.longitude = 150.67402097
        self.buoyLoc.pose.position.altitude = 0.0

        # Time to calculate the buoy positions
        rospy.sleep(3)
        while self.buoyLocationPub.get_num_connections() < 1:
            # Make sure there is a connection
            a = 0
        self.buoyLocationPub.publish(self.buoyLoc)
        print("Published buoy location...")
        rospy.sleep(2)

    def briefMoveForward(self):
        self.rightThrustPub.publish(self.thrustVal)
        self.leftThrustPub.publish(self.thrustVal)
        self.loopCount += 1

    def sendCmds(self):
        while not rospy.is_shutdown():
            try:
                if isinstance(self.taskInfo, type(self.taskType)):
                    # print(self.taskInfo.name, self.taskInfo.state)
                    if self.taskInfo.name == "stationkeeping" or self.taskInfo.name == "wayfinding" or \
                            self.taskInfo.name == "gymkhana" or self.taskInfo.name == "navigation_course":
                        if self.taskInfo.state == "initial":
                            print("Waiting for task to start...")
                        elif self.taskInfo.state == "running":
                            print("Task runnning")
                            self.rightThrustPub.publish(self.thrustVal)
                            self.leftThrustPub.publish(self.thrustVal)
                        elif self.taskInfo.state == "finished":
                            print("Task ended...")
                            break
                    elif self.taskInfo.name == "perception":
                        if self.taskInfo.state == "initial":
                            print("Waiting for task to start...")
                        elif self.taskInfo.state == "running":
                            self.calcBuoyLoc()
                        elif self.taskInfo.state == "finished":
                            print("Task ended...")
                            break
                    elif self.taskInfo.name == "scan":
                        if self.taskInfo.state == "initial":
                            print("Waiting for task to start...")
                        elif self.taskInfo.state == "running":
                            self.briefMoveForward()
                            if self.loopCount > 10:
                                break
                        elif self.taskInfo.state == "finished":
                            print("Task ended...")
                            break

                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

sn = SimpleNode()
sn.sendCmds()
print('Complete')

