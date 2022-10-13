# week5-4824

# A Simple Publisher and Subscriber
## Create pakages
*Run the package creation command by going to ros2 ws/src:*
```
ros2 pkg create --build-type ament_python py_pubsub
```
![image](https://user-images.githubusercontent.com/92029196/195634124-d09146ef-b6ae-482a-8286-8ea10507d12d.png)

*Your terminal will display a message confirming the creation of your package py pubsub and all of its necessary files and folders.*
## 2. Write the publisher node

*You can download the example talker code by going to ros2 ws/src/py pubsub/py pubsub and typing the following command:*
```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
![image](https://user-images.githubusercontent.com/92029196/195634751-3449e77f-896e-44db-b8a9-f457d228bb7c.png)


*The __init.py file will now be followed by a new one called publisher member function.py. Open the file in your preferred text editor.*
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 2.1 Add dependencies
*Return one level up to the setup.py, setup.cfg, and package.xml files that have been produced for you in the ros2 ws/src/py pubsub directory.*
*Open package.xml in your text editor, and make sure the description>, maintainer>, and license> tags are full.*
```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
*After the lines above that correspond to the import declarations for your node, add the following dependencies:*
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
<img width="413" alt="Screenshot_7" src="https://user-images.githubusercontent.com/92029196/195635287-a2a36901-8bc8-43c2-ba04-353dfed754a3.png">


*This states that rclpy and std msgs are necessary for the package's code to run.*
*Remember to save the file.*

### 2.2 Add an entry point
 *Take a look at setup.py. Make sure to check your package.xml one more time to make sure the maintainer, maintainer email, description, and license columns match:*
 ```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
*Add the next line to the entry points field between the console scripts brackets:*
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
<img width="406" alt="Screenshot_8" src="https://user-images.githubusercontent.com/92029196/195635587-5b293e3a-cedf-46c7-b6ae-3a5b2d76fbbb.png">


*Remember to save.*
### 2.3 Check setup.cfg
*The following details should be included by default in the setup.cfg file:*
```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```
*Tell setuptools to put your executables in the lib directory so that ros2 run knows where to find them.*

*You could create your package right now, source the local setup files, and launch it if you wanted to see the full system in operation. Let's first, though, establish the subscriber node.*
## 3 Write the subscriber node
*The next node can be created by going back to ros2 ws/src/py pubsub/py pubsub. Fill out your terminal with the following code:*
```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
<img width="366" alt="Screenshot_11" src="https://user-images.githubusercontent.com/92029196/195636023-67247b32-c893-4c49-bbc9-e481426fd110.png">

*Now, the directory must include the following files:*
```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```
*Now, Open the subscriber_member_function.py with your text editor.*
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 3.1 Add an entry point
*Reopen setup.py and place the subscriber node's entry point beneath the publisher's entry point. Now, the entry points field should be as follows:*
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
<img width="428" alt="Screenshot_12" src="https://user-images.githubusercontent.com/92029196/195635866-a8052547-c91d-431e-ba85-c1ee3b434488.png">


*Once the file has been saved, your pub/sub system should be operational.*

## 4 Build and Run
*The `rclpy` and `std msgs` packages are probably already installed on your ROS 2 system. Before building, it's best practice to run `rosdep` in the workspace's root directory (`ros2 ws`) to check for any missing dependencies:*
```
rosdep install -i --from-path src --rosdistro foxy -y
```
<img width="353" alt="Screenshot_13" src="https://user-images.githubusercontent.com/92029196/195639348-065d9849-220c-425b-8e86-b38952370b23.png">


*Still in the root of your workspace, ros2_ws, build your new package:*
```
colcon build --packages-select py_pubsub
```

<img width="323" alt="Screenshot_14" src="https://user-images.githubusercontent.com/92029196/195639411-29ffa209-3b09-4f18-b679-40ed01f0bbd0.png">


*Open a new terminal, navigate to ros2_ws, and source the setup files:*
```
. install/setup.bash
```
*Now run the talker node:*
```
ros2 run py_pubsub talker
```

*Starting in 0.5 seconds, the terminal should begin sending out info messages as follows:*
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```
<img width="364" alt="Screenshot_17" src="https://user-images.githubusercontent.com/92029196/195639788-76076bba-c626-4150-b9ad-a71267789669.png">

*Launch a new terminal, once more source the setup files from ros2 ws, and then launch the listener node:*
```
ros2 run py_pubsub listener
```

*Starting at the publisher's current message count, the listener will begin writing messages to the console as follows:*
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```
<img width="350" alt="Screenshot_16" src="https://user-images.githubusercontent.com/92029196/195639859-f8b83138-9be7-4992-acaa-9f22a59313bf.png">


*Enter Ctrl+C in each terminal to stop the nodes from spinning*



# A Simple Service and Client
# 1.Create a package

*Run the package creation command by going to ros2 ws/src:*
```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```
![image](https://user-images.githubusercontent.com/92029196/195643634-11e179e3-3eae-4875-94cc-0c0f4aed8b06.png)

*You will see confirmation from your terminal that your package py srvcli and all of its necessary files and folders have been created.*

## 1.1 Update (package.xml)
*Because you used the —dependencies option when generating the package, you don't need to manually add dependencies to package.xml.*

*But as always, don't forget to include the description, license information, and the name and email of the maintainer in package.xml.*
```
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
![image](https://user-images.githubusercontent.com/92029196/195644483-a360ed50-73c8-430f-a2a5-28569db747a6.png)



## 1.2 Update (setup.py)
*The following details should be added to the setup.py file's description, maintainer, maintainer email, and license fields:*
```
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
![image](https://user-images.githubusercontent.com/92029196/195644972-6dd981dc-e9ee-46da-85f7-4aa81ab4c861.png)


# 2.Write the service node
*In the ros2 ws/src/py srvcli/py srvcli directory, make a new file called service member function.py, and then paste the following code inside:*
```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2.1 Add an entry point
*For the ros2 run command to be able to run your node, the entry point must be added to setup.py (located in the ros2 ws/src/py srvcli directory).*

*In between the "console scripts" brackets, the following line should be added:*
```
'service = py_srvcli.service_member_function:main',
```
![image](https://user-images.githubusercontent.com/92029196/195645590-0f408c5b-7748-472b-83dc-3da81a21a177.png)

# 3.Write the client node
 *In the ros2 ws/src/py srvcli/py srvcli directory, make a new file called client member function.py, and then paste the following code inside:*
 ```python
 import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## 3.1 Add an entry point
*Similar to how the service node needs an entry point, the client node also needs one.*

*The entry points column in your setup.py file must be formatted as follows*
```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```
![image](https://user-images.githubusercontent.com/92029196/195646514-18df638f-3852-4fd9-bdbb-03ddfdcee3f8.png)


# 4.Build and Run
*Running rosdep in the workspace's root directory (ros2 ws) is a good idea to see if any dependencies are missing before building:*
```
rosdep install -i --from-path src --rosdistro foxy -y
```
*Go back to ros2 ws, the workspace's root, and create your new package:*

```
colcon build --packages-select py_srvcli
```
*Open a new terminal, navigate to ros2_ws, and source the setup files:*
```
. install/setup.bash
```
*Run the service node right now:*
```
ros2 run py_srvcli service
```
*The node will hold off until the client makes a request.*

*Re-source the setup files from ros2 ws in a new terminal. The client node, any two integers, and a space between them.*
```
ros2 run py_srvcli client 2 3
```
*If you chose options 2 and 3 as an illustration, the customer would receive a response similar to this:*
```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```
*You should return to the terminal where your service node is running. As you can see, it published the following log statements after receiving the request:*
```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```
*Ctrl+C will stop the nodes from rotating in each terminal.*




# Creating custom msg and srv files

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
