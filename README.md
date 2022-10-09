# week5-4824
# 1. creating package
*Run the package creation command by going to ros2 ws/src:*
```
ros2 pkg create --build-type ament_python py_pubsub
```
# 2.Create custom definitions
## 2.1 MSG definition
*In the tutorial interfaces/msg directory that you just created, make a new file named Num.msg, and then add a single line of code describing the data structure in Num.msg:*
```
int64 num
```
*This custom message sends a single value, the 64-bit integer num.*

*In the directory you just created for the tutorial interfaces/msg, make a new file called Sphere.msg and put the following information in it:*
```
geometry_msgs/Point center
float64 radius
```
*A message from a different message package—in this case, geometry msgs/Point—is used in this custom message.*
## 2.2srv definition
*In the instructional interfaces/srv directory that you just created, add a new file with the name AddThreeInts.srv with the following request and response structure:*
```
int64 a
int64 b
int64 c
---
int64 sum
```
# 3.CMakeLists.txt
*Add the following lines to CMakeLists.txt to translate the interfaces you defined into language-specific code (such C++ and Python) so they may be utilized in those languages:*
```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```
# 4.package.xml
*These lines should be added to package.xml.*
```
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

# 5.Build the tutorial_interfaces package
*You may construct your custom interfaces package now that all of its components are in place. Run the command below in the workspace's root (~/ros2_ws):*
```
colcon build --packages-select tutorial_interfaces
```
*Other ROS 2 programs will now be able to find the interfaces.*

# 6.Confirm msg and srv creation
*Run the following command from within your workspace (ros2 ws) to source it in a new terminal:*
```
. install/setup.bash
```
*should return:*
```
int64 num
```
*and:*
```
ros2 interface show tutorial_interfaces/msg/Sphere
```
*should return:*
```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```
*and:*
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
*should return:*
```
int64 a
int64 b
int64 c
---
int64 sum
```
# 7.Test the new interfaces
*You can utilize the packages you made in earlier instructions for this step. You may use your new interfaces by making a few straightforward changes to the nodes, CMakeLists, and package files.*
## 7.1 Testing Num.msg with pub/sub
*For this step, you can use the packages you created in the prior stages. By making a few simple adjustments to the nodes, CMakeLists, and package files, you can use your new interfaces.*
*Publisher:*
```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 ```
*Suscriber:*
```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
*CMakeLists.txt:*

*Add the following lines (C++ only):*
```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

*package.xml:*

*Add the following line:*
```
<exec_depend>tutorial_interfaces</exec_depend>
```
*Build the package after making the aforementioned alterations and saving all the modifications:*
```
colcon build --packages-select py_pubsub
```
*on windows:*
```
colcon build --merge-install --packages-select py_pubsub
```
*Then open two new terminals, source ros2_ws in each, and run:*
```
ros2 run py_pubsub talker
```
*Instead of broadcasting strings as Num.msg only relays an integer, the talker should only be publishing integer values:*
```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```
## 7.2 Testing AddThreeInts.srv with service/client
*You may use AddThreeInts.srv by making a few minor adjustments to the service/client package developed in a prior tutorial (in C++ or Python). The output will alter significantly because you'll be switching from the initial two integer request srv to a three integer request srv.*

*Service:*
```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
 ```
 *client:*
 ```
 from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
 *CMakeLists.txt:*

*Add the following lines (C++ only):*
```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
*package.xml:*

*Add the following line:*
```
<exec_depend>tutorial_interfaces</exec_depend>
```
*After making the above edits and saving all the changes, build the package:*
```
colcon build --packages-select py_srvcli
```
*On Windows:*
```
colcon build --merge-install --packages-select py_srvcli
```
*Then open two new terminals, source ros2_ws in each, and run:*
```
ros2 run py_srvcli service
```
```
ros2 run py_srvcli client 2 3 1
```
