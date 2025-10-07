#### What is ROS

ROS2 is basically a software that helps us have different processes talk to each other. You can have python scripts, C++ scripts, different external devices like cameras, sensors or motor all talk to each other using this framework.

#### The basic ROS2 architecture

We have nodes that talk to each other using interfaces. There is a number of different interfaces like messages, services and actions. Nodes can also be parameterized.

Each device or algorithm can be a different node and they can use the ROS architecture to talk to each other.

#### The first steps, meet Jerry the robot

Hello, this is Jerry. Jerry is a very simple robot living in a very simple 2D world. It likes walking around. Jerry is also very cooperative. It can listen to very basic commands:

- Forward
- Left
- Right
- Backward

Using these commands we can tell Jerry exactly what to do. In order to do this though, we need a way to publish commands to him. Jerry is also very social. It likes sharing where it is at every moment and what it is about to do. We can simulate this whole process using ROS2. We can use a publisher *node* that publishes *messages* on a certain *topic* and a subscriber *node* that listens to the *messages* and then prints what action it is currently doing. Additionally, we will implement a *service* for status updates; whenever Jerry is asked what its current status is, it will respond with the coordinates that it is currently in.

Let's take this step by step.

#### Making the interfaces

Before we implement the nodes that talk to each other, we need the interfaces that they use when communicating. In ROS2 interfaces are files written in ROS2 Interface Definition and Language (IDL). There are tree types of interfaces: *messages*, *services* and *actions*. We can use these interfaces to communicate between *nodes* that are even written in different languages (like Python and C++).

##### The command messages

*Messages* are the most typical way to communicate between nodes. They are used for continuous data exchange, where a *publisher* sends data to one or more *subscribers*. Publisher and subscribers communicate over a specific *topic*.

In our case we want a *publisher node* to send messages over a *topic* called **/cmd** to a *subscriber* that will then "execute" the commands. Since Jerry takes very simple words as orders, our message will consist of simple string data.

It is in general good practice to use interfaces that already exist in ROS2 when dealing with common robotic concepts such as sensor messages, instead of creating new ones that serve the exact same purpose. Let's see an example to understand the basic structure of a ROS2 message. We can run the command:

```
ros2 interface packages
```

This shows us all the interface packages that are pre-built as well as any that we have written ourselves. Now we can see what each interface package contains. For example let's check out the *geometry_msgs* package:

```
ros2 interface package geometry_msgs
```

As we can see there are many different types of messages for different functionalities. Let's take a look at the *Point* message. We can do so with the **ros2 interface show** command in the terminal:

```
ros2 interface show geometry_msgs/msg/Point
```

The lines that begin with "#" are obviously comments so we can see that this message contains three float type variables called *x, y, z* representing a 3D point in space. We can make our own message containing any data type using the same structure.

In the ***interfaces_package/msg*** directory, we will create a new *.msg* file. We can call it *JerryCommand.msg*. It will consist of a string for the command and an integer for the number of steps to take. It should have the following layout:

```
# This is a command message for Jerry.
# String data represents theverbal command and can be "forward",
# "backward", "left", "right"
# Integer data is the number of steps that Jerry will take

#### TO BE COMPLETED BY THE STUDENTS ####


```

##### The status service

*Services* are used to exchange information in a *request-answer* form of communication. In this type of interface we have *services* not *topics*. A *client* requests information and a *server* provides the requested information. This type of communication is not continuous.

We will create a service so Jerry can give us status updates whenever we need it.  We will use this service to ask Jerry how far away he currently is from obstacles. The service will be called **/distance**. The request will be in the form of a point in 2D space and the answer will be in the form of a float number, representing the Euclidean distance from this point/obstacle.

To see how we can structure a service file, let's take a look at an example one. We can use the **ros2 interface show** command again on the terminal:

```
ros2 interface show std_srvs/srv/SetBool
```

We can see that the *request* is separated from the *response* with ---. What we do with the information of the service must be implemented within a *node*, the *service* it self is just a protocol for bidirectional communication.

In the ***interfaces_package/srv*** directory, we will create a new *.srv* file. Let's call it *DistanceFromObstacle.srv*. It should have the following layout:

```
# This service asks Jerry to tells us how far away he is from
# a certain point/obstacle. The request is in the form of x and y
# coordinates for the obstacle and the response is the Euclidean
# distance between Jerry and the obstacle

### TO BE COMPLETED BY THE STUDENTS ###


```

##### Building and sourcing

Now that we have made our interfaces, we must build and source the project. We can do that from our main project directory, like this:

```
colcon build
source install/setup.bash
```

#### Publisher of Commands, the first Node

Now that we have our interfaces, we can create nodes that communicate using these interfaces. The first node that we will make is the *CommandPublisher* node. We will use this node to periodically publish commands to Jerry.

We use the **create_publisher()** method to create a publisher that publishes *jerry_command* over the topic */jerry_command*.

```
self.jerry_command_publisher = self.create_publisher(
    #### ENTER CODE HERE ###########


	################################
        )
```

We will also create a timer that will call a *publish_command* method every 2 seconds. We can do that with the **create_timer()** method.

```
self.timer = self.create_timer(timer_period, self.publish_command)
```

In the *publish_command()* method we must first create a message of type *jerry_command* and then fill it with a random command (forward, backward, left, right) and a random number of steps. We can then use the **publish(msg)** method to publish our message to the topic.

```
def publish_command(self):
	# Method that is periodically called by the timer

	jerry_command = JerryCommand()

	#### ENTER CODE HERE ###########


	################################

	self.incremental_id += 1
```

##### Test the publisher

Now that our publisher is ready, we can test it using the **ros2 run** command. First we build and source:

```
colcon build
source install/setup.bash
```

and then we can run our publisher:

```
ros2 run node_package command_publisher_node
```

We should see the command that it is publishing every two seconds.
#### Jerry, the Subscriber

To be able to control our robot, we need to create a subscriber node that subscribes to the topic */jerry_command* and receives the commands that our publisher sends every two seconds.

We will subscribe to the topic with the **create_subscription()** method. We need to specify what type of message we are expecting to receive, what topic we are subscribing at and what the callback function will be. The callback function is the one that will be called automatically when a message is received from the publisher.

```
self.jerry_subscriber = self.create_subscription(
	#### ENTER CODE HERE ###########


	################################
)
```

The callback function is a function that expects the message type as an argument and does not return anything. In our case we want to create a callback function that receives the message that is the command that we published and then prints it (to simulate taking the action).

```
def execute_command(self, msg: JerryCommand):
	# Method that is called when a new command is received by Jerry.
	self.get_logger().info(f"Aye aye captain! Moving {msg.cmd} for {msg.steps} steps.")

```

##### Test the subscriber

First we must build and source:

```
colcon build
source install/setup.bash
```

and then while the publisher is running, we can run the subscriber on another terminal:

```
ros2 run node_package jerry_robot_node
```

#### Jerry shall serve you

If you remember our initial design we want to have a service going on through which we can ask about Jerry's status. For this we will use the **DistanceFromObstacle** service that we created earlier. This means that we must extend the capabilities of the **jerry_robot_node** to also be a server for the **DistanceFromObstacle** service. Remember this node knows exactly where Jerry is and will answer the client call with a Euclidean distance from the given obstacle.

In the *__init__(self)*  method of the node we create the server with the *self.create_service()* command.

```
self.jerry_server = self.create_service(
	######## ENTER CODE HERE ###########


	################################
	)
```

The *create_service()* function expects three arguments: the type of service, the name of the particular service and what function to call when a service request is made a client.

The callback function's API dictates that we must give it both the request and the response as an argument which seems to be a bit weird but we must follow it. The function must return the response. The function is defined as follows:

```
def service_callback(self, request: DistanceFromObstacle.Request,
                          response: DistanceFromObstacle.Response) -> DistanceFromObstacle.Response :
	"""
	This function is called to respond to a DistanceFromObstacle request.
	It takes the x and y coordinates of the obstalce as an input and calculates
	the Euclidean distance from Jerry's current wherabouts.
	"""
	#### ENTER CODE HERE ###########


	################################

	self.get_logger().info(f"Distance from the obstacle is: {response.dist}. Responding to the client...")

	return response
```

After this is done we must build and source as always.

```
colcon build
source install/setup.bash
```

#### The final step: the client

Finally we will create one more node to serve as the client. The client's job is to send a **/distance** service request with the coordinates of an obstacle. The implementation seems straightforward enough but there are some things to consider when implementing the client. Let's take it step by step.

We will make a new node, like we did before, called **ClientNode** and create the client in the *__init__(self)* method with the *self.create_client()* method.

```
self.service_client = self.create_client(
	#### ENTER CODE HERE ###########


	################################
)
```

Now that we have done that, let's think about how services work. Unlike messages the server needs to be available for the client to make a request. But we cannot expect the server node to always be available before the client, even if it was executed first! The order as well as the speed of execution of the various nodes depends on a lot of reasons, including ROS2 itself, the operating system etc.

So before trying to make a service request to the server we can use the *self.service_client.wait_for_service()* method to wait until the server becomes available.

```
while not self.service_server.wait_for_service(timeout_sec = 1.0):
	self.get_logger().info(f"Server not available...")
```

With this done, we can create a timer that periodically makes a request after the service server has become available.

```
timer_period: float = 0.5
	self.timer = self.create_timer(
		timer_period_sec=timer_period,
		#### ENTER CODE HERE (add callback)###########


		################################
	)
```

This will call the *timer_callback()* method every 0.5 seconds. Within this method we can make a service call with the *call_async()* method. This method will make a call to the server sending a request. First we must define a *Future* object to handle the servers response. After the answer comes back, we will use the *add_done_callback()* method on our future object to process the response.

```
self.future: Future = None
self.future = self.service_client.call_async(request)
self.future.add_done_callback(self.process_response)
```

Now we must implement the *process_response()* method to handle the server's response. Let's just print the response we got.

```
def process_response(self, future: Future):
	# Callback for the future, that will be called when it is done
	response = future.result()
	if response is not None:
		#### ENTER CODE HERE ###########
		# Print the servers response

		################################
	else:
		#### ENTER CODE HERE ###########
		# Prin the servers response

		################################
```

As usual we build and source.