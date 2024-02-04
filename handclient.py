import rclpy
from rclpy.node  import Node
from std_srvs.srv import SetBool 
from garment_msgs.srv import GraspPoints
import time



class handclient(Node):
     def __init__(self, name):
          super().__init__(name)
          self.client = self.create_client(SetBool, '/dual/motor')
          self.client1 = self.create_client(SetBool, '/dual/pump')
          self.clientca= self.create_client(GraspPoints, '/grasp_points')
          while not self.client.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('service not available, waiting again...')
          while not self.client1.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('service not available, waiting again...')
          while not self.clientca.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('service not available, waiting again...')
          self.request = SetBool.Request()
          self.requestca=GraspPoints.Request()

     def send_request1(self):
          self.request.data = True
          self.future = self.client.call_async(self.request)
          rclpy.spin_until_future_complete(self, self.future)
          return self.future.result()
     def send_request2(self):
          self.request.data = False
          self.future = self.client.call_async(self.request)
          rclpy.spin_until_future_complete(self, self.future)
          return self.future.result()
     def send_request3(self):
          self.request.data = True
          self.future = self.client1.call_async(self.request)
          rclpy.spin_until_future_complete(self, self.future)
          return self.future.result()
     def send_request4(self):
          self.request.data = False
          self.future = self.client1.call_async(self.request)
          rclpy.spin_until_future_complete(self, self.future)
          return self.future.result()
     def send_request5(self):
          self.requestca = None
          self.future = self.clientca.call_async(self.requestca)
          rclpy.spin_until_future_complete(self, self.future)
          return self.future.result()

def main(args=None):
   rclpy.init(args=args)
   node = handclient('handclient')
   response=node.send_request5()
   if response:
          node.get_logger().info('Grasp Points: %s' % response.grasp_points)
          node.send_request1()
          node.send_request3()
          time.sleep(6)
          node.send_request2()
          node.send_request4()
          node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
     main()
 