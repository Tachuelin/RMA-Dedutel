import rospy
from package_arm.Controlled_motor import MotorDriver
#Subscriber
from std_msgs.msg import Int32
#Service
from std_srvs.srv import Trigger
#Publisher
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

class MotorDriverROSWrapper:
    
    #Here we are getting the max_speed setting from ROS param
    #and we pass this value to the MotorDriver init function
    def __init__(self):
        max_speed = rospy.get_param("~max_speed", 8)
        
        #To get status and the current speed, we'll use 2 rospy Publishers
        #Each publishers will be run with a rospy TImer, in order to publish at fixed rate
        #They will wrap the get_speed() and get_status() methods from the driver
        publish_current_speed_frequency = rospy.get_param("~publish_current_speed_frequency", 5.0)
        publish_motor_status_frequency = rospy.get_param("~publish_motor_status_frequency", 1.0)

        self.motor = MotorDriver(max_speed=max_speed)

        rospy.Subscriber("speed_command", Int32, self.callback_speed_command)    
        rospy.Service("stop_motor", Trigger, self.callback_stop)

        self.current_speed_pub = rospy.Publisher("current_speed", Int32, queue_size=10)
        self.motor_status_pub = rospy.Publisher("motor_status", DiagnosticStatus, queue_size=1)

        rospy.timer(rospy.Duration(1.0/publish_current_speed_frequency), self.publish_current_status)
        rospy.timer(rospy.Duration(1.0/publish_motor_status_frequency), self.publish_motor_status)

    def publish_current_speed(self, event=None):
        self.current_speed_pub.publish(self.motor.get_speed())

    def publish_motor_status(self, event=None):
        status = self.motor.get_status()
        data_list = []
        for key in status:
            data_list.append(KeyValue(key, str(status[key])))

            msg = DiagnosticStatus()
            msg.values = data_list

            self.motor_status_pub.publish(msg)
            #We use the DiagnosticStatus message from the diagnostic_msgs package.
            #This message definition is quite appropriate for sending a hardware status
            #of your hardware.
            
     

    def stop(self):
        self.motor.stop()

    #When you send an integer value to the speed_command topic
    #this value will be passed to the motor driver to set the actual motor speed
    def callback_speed_command(self, msg):
        self.motor.set_speed(msg.data)

    #Sometimes you'll nedd to create custom message and service definitions
     #if you can't find and existing one that does the work. For this example
     #the Trigger service definitios is just what we need.
    def callback_stop(self, req):
        self.stop()
        return {"succes": True, "message": "Motor has been stoped"}    

if __name__ == "__main__":
    rospy.init_node("motor_driver")

    motor_driver_wrapper = MotorDriverROSWrapper()
    rospy.on_shutdown(motor_driver_wrapper.stop)

    rospy.loginfo("MOtor driver is now started, ready to get commands.")
    rospy.spin()


     