import numpy as np
import openvr 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class VivePub(Node):
    def __init__(self):
        super().__init__("vive_pub")
        self.pose_pub = self.create_publisher(Float64MultiArray, "/vive/pose", 10)
        self.btn_pub = self.create_publisher(Float64MultiArray, "/vive/buttons", 10)
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.dev = None
        self.create_timer(1.0/250.0, self.tick)
        self.get_logger().info("vive_pub up")

    def tick(self):
        vr = self.vr
        UNIVERSE = openvr.TrackingUniverseRawAndUncalibrated
        PAD = 1 << openvr.k_EButton_SteamVR_Touchpad
        MENU = 1 << openvr.k_EButton_ApplicationMenu
        poses = vr.getDeviceToAbsoluteTrackingPose(UNIVERSE,0,openvr.k_unMaxTrackedDeviceCount)
        if self.dev is None or vr.getTrackedDeviceClass(self.dev) != openvr.TrackedDeviceClass_Controller:
            self.dev = next((i for i in range(openvr.k_unMaxTrackedDeviceCount) if  vr.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller), None)
        if self.dev is None:
            return
        p = poses[self.dev]
        if p.bPoseIsValid:
            m = p.mDeviceToAbsoluteTracking
            msg = Float64MultiArray()
            msg.data = [float(m[0][3]), float(m[1][3]), float(m[2][3]), 
                        float(m[0][0]), float(m[0][1]), float(m[0][2]), 
                        float(m[1][0]), float(m[1][1]), float(m[1][2]), 
                        float(m[2][0]), float(m[2][1]), float(m[2][2])] 
            self.pose_pub.publish(msg)
        res,state = vr.getControllerState(self.dev)
        if res:
            pad = 1.0 if (state.ulButtonPressed & PAD) else 0.0
            menu= 1.0 if (state.ulButtonPressed & MENU) else 0.0
            trig = float(state.rAxis[1].x)
            bmsg = Float64MultiArray()
            bmsg.data = [trig, pad, menu]
            self.btn_pub.publish(bmsg)
    
def main():
    rclpy.init()
    node = VivePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        openvr.shutdown()
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()
    
if __name__ == "__main__":
    main()