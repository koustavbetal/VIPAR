import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import smbus
import math
import numpy as np
# from time import sleep

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # Initialize I2C bus and MPU6050 registers (unchanged)
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68
        
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        
        # Initialize MPU6050
        self.mpu_init()
        
        # Create publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        # Initialize EKF parameters
        self.dt = 0.1  # 10Hz sampling rate
        
        # State vector [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
        self.x = np.zeros(6)
        
        # State covariance matrix
        self.P = np.eye(6) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001])
        
        # Measurement noise covariance
        self.R = np.diag([0.1, 0.1, 0.1])
        
        # Previous timestamp
        self.last_time = self.get_clock().now()
        
        # Create timer
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info('MPU6050 Node with EKF has been started')

    def mpu_init(self):
        # Same initialization as before
        self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.device_address, self.CONFIG, 0)
        self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.device_address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        # Same as before
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
            value = value - 65536
        return value

    def predict_state(self, gyro_x, gyro_y, gyro_z, dt):
        """EKF prediction step using gyroscope data"""
        # Remove estimated bias
        wx = gyro_x - self.x[3]
        wy = gyro_y - self.x[4]
        wz = gyro_z - self.x[5]
        
        # Predict next state
        roll_pred = self.x[0] + dt * (wx + np.sin(self.x[0]) * np.tan(self.x[1]) * wy + 
                                     np.cos(self.x[0]) * np.tan(self.x[1]) * wz)
        pitch_pred = self.x[1] + dt * (np.cos(self.x[0]) * wy - np.sin(self.x[0]) * wz)
        yaw_pred = self.x[2] + dt * (np.sin(self.x[0])/np.cos(self.x[1]) * wy + 
                                    np.cos(self.x[0])/np.cos(self.x[1]) * wz)
        
        # Bias terms remain constant in prediction
        self.x = np.array([roll_pred, pitch_pred, yaw_pred, self.x[3], self.x[4], self.x[5]])
        
        # Calculate Jacobian of state transition
        F = np.eye(6)
        F[0:3, 0:3] = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        F[0:3, 3:6] = -dt * np.eye(3)
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_state(self, acc_x, acc_y, acc_z):
        """EKF update step using accelerometer data"""
        # Calculate roll and pitch from accelerometer
        roll_meas = math.atan2(acc_y, math.sqrt(acc_x*acc_x + acc_z*acc_z))
        pitch_meas = math.atan2(-acc_x, math.sqrt(acc_y*acc_y + acc_z*acc_z))
        
        # Measurement vector
        z = np.array([roll_meas, pitch_meas, self.x[2]])  # Using previous yaw as we can't measure it
        
        # Measurement matrix (we only measure roll and pitch)
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)
        
        # Calculate Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        y = z - H @ self.x
        self.x = self.x + K @ y
        
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Same as before
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def timer_callback(self):
        # Read sensor data
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_XOUT_H + 2)
        acc_z = self.read_raw_data(self.ACCEL_XOUT_H + 4)
        
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(self.GYRO_XOUT_H + 4)
        
        # Convert to physical values
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        
        Gx = gyro_x/131.0  # deg/s
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        
        # Convert gyro to rad/s
        Gx_rad = math.radians(Gx)
        Gy_rad = math.radians(Gy)
        Gz_rad = math.radians(Gz)
        
        # Calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # EKF predict step
        self.predict_state(Gx_rad, Gy_rad, Gz_rad, dt)
        
        # EKF update step
        self.update_state(Ax, Ay, Az)
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # Set orientation using filtered values
        imu_msg.orientation = self.euler_to_quaternion(self.x[0], self.x[1], self.x[2])
        
        # Set angular velocity (bias-corrected)
        imu_msg.angular_velocity.x = float(Gx_rad - self.x[3])
        imu_msg.angular_velocity.y = float(Gy_rad - self.x[4])
        imu_msg.angular_velocity.z = float(Gz_rad - self.x[5])
        
        # Set linear acceleration
        imu_msg.linear_acceleration.x = float(Ax)
        imu_msg.linear_acceleration.y = float(Ay)
        imu_msg.linear_acceleration.z = float(Az)
        
        # Add covariance matrices
        # Convert 6x6 covariance to 9x9 for orientation
        orientation_cov = np.zeros((9,9))
        orientation_cov[0:3, 0:3] = self.P[0:3, 0:3]
        imu_msg.orientation_covariance = orientation_cov.flatten().tolist()
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()