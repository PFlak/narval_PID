import rclpy
from narval_pid.depthPID import DepthPID

def main():
    rclpy.init()
    
    depthPID = DepthPID()

    rclpy.spin(depthPID)

    depthPID.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()