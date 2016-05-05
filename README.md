# mongo_msg_db
[![Build Status](https://travis-ci.org/jstnhuang/mongo_msg_db.svg?branch=master)](https://travis-ci.org/jstnhuang/mongo_msg_db)

A simple MongoDB database for flat collections of ROS messages.

## Usage
Once the database node is running, it provides services for adding messages to a MongoDB database, using the default MongoDB instance.
The messages are saved and retrieved in JSON format, which is convenient for web purposes.
We recommend using [rospy_message_converter](http://wiki.ros.org/rospy_message_converter) to do the conversion between ROS messages and JSON.

### Services
See the [service files](https://github.com/jstnhuang/mongo_msg_db_msgs/tree/master/srv) for the full definitions.
- `mongo_msg_db/delete`: delete a message from a collection by ID
- `mongo_msg_db/find`: get a single message from a collection by ID
- `mongo_msg_db/insert`: insert a message to a collection and get its ID
- `mongo_msg_db/list`: get all messages from a collection
- `mongo_msg_db/update`: replace a message in a collection

## Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/jstnhuang/mongo_msg_db_msgs.git
git clone https://github.com/jstnhuang/mongo_msg_db.git
catkin build
```

## Running
```bash
rosrun mongo_msg_db db_node.py
```
