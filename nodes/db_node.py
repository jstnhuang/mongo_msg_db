#! /usr/bin/env python
import rospy
from pymongo import MongoClient
from mongo_msg_db import MessageDb
from mongo_msg_db import RosMessageDb
from mongo_msg_db_msgs.srv import Delete
from mongo_msg_db_msgs.srv import Find
from mongo_msg_db_msgs.srv import Insert
from mongo_msg_db_msgs.srv import List
from mongo_msg_db_msgs.srv import Subscribe
from mongo_msg_db_msgs.srv import SubscribeToList
from mongo_msg_db_msgs.srv import Update

def main():
    rospy.init_node('mongo_msg_db')
    mongo_client = MongoClient()
    message_database = MessageDb(mongo_client)
    database = RosMessageDb(message_database)
    rospy.Service('~delete', Delete, database.delete)
    rospy.Service('~find', Find, database.find)
    rospy.Service('~insert', Insert, database.insert)
    rospy.Service('~list', List, database.list)
    rospy.Service('~update', Update, database.update)
    rospy.Service('~subscribe', Subscribe, database.subscribe)
    rospy.Service('~subscribe_to_list', SubscribeToList, database.subscribe_to_list)
    rospy.spin()


if __name__ == '__main__':
    main()
