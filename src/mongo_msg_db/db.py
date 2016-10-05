#!/usr/bin/env python
from bson.objectid import ObjectId
from pymongo import MongoClient
from rospy_message_converter import json_message_converter as jmc

from mongo_msg_db_msgs.msg import Collection
from mongo_msg_db_msgs.msg import Message
from mongo_msg_db_msgs.msg import MessageEvent
from mongo_msg_db_msgs.msg import MessageList
from mongo_msg_db_msgs.srv import DeleteResponse
from mongo_msg_db_msgs.srv import FindResponse
from mongo_msg_db_msgs.srv import InsertResponse
from mongo_msg_db_msgs.srv import ListResponse
from mongo_msg_db_msgs.srv import SubscribeResponse
from mongo_msg_db_msgs.srv import SubscribeToListResponse
from mongo_msg_db_msgs.srv import UpdateResponse

import rospy


class MessageDb(object):
    """A simple JSON message database.
    """

    def __init__(self, mongo_client):
        self._mongo_client = mongo_client
        self._publishers = {}
        self._list_publishers = {}

    def _collection(self, collection_msg):
        db = getattr(self._mongo_client, collection_msg.db)
        collection = getattr(db, collection_msg.collection)
        return collection

    def _collection_key(self, collection):
        return collection.db, collection.collection

    def delete(self, collection, id):
        """Deletes a message from a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        id: The ObjectId of the message to delete, as a string.

        Returns: number of messages deleted.
        """
        mongo_collection = self._collection(collection)
        result = mongo_collection.delete_one({'_id': ObjectId(id)})
        if id in self._publishers:
            event = MessageEvent(event=MessageEvent.DELETE)
            self._publishers[id].publish(event)

        key = self._collection_key(collection)
        if key in self._list_publishers:
            messages = self.list(collection)
            message_list = MessageList(messages=messages)
            self._list_publishers[key].publish(message_list)

        return result.deleted_count

    def find_msg(self, collection, id):
        matched_count, message = self.find(collection, id)
        msg = jmc.convert_json_to_ros_message(message.msg_type, message.json)
        return matched_count, msg

    def find(self, collection, id):
        """Finds a message in a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        id: The ObjectId of the message to find, as a string.

        Returns: (match_count, message). match_count is 1 if found, 0 
            otherwise. message is None if the message was not found, otherwise
            it's a mongo_msg_db_msgs.msg.Message
        """
        collection = self._collection(collection)
        result = collection.find_one({'_id': ObjectId(id)})
        matched_count = 0
        message = None
        if result is not None:
            matched_count = 1
            message = Message()
            message.id = id
            message.msg_type = result['msg_type']
            message.json = result['json']
        return matched_count, message

    def insert_msg(self, collection, msg):
        return self.insert(collection, jmc.convert_ros_message_to_json(msg),
                           msg._type)

    def insert(self, collection, json, msg_type):
        """Inserts a message into a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        json: The JSON string representation of the message to insert.
        msg_type: The name of the message type, e.g., std_msgs/String

        Returns: The ObjectId of the inserted item, as a string.
        """
        mongo_collection = self._collection(collection)
        result = mongo_collection.insert_one(
            {'msg_type': msg_type,
             'json': json})

        key = self._collection_key(collection)
        if key in self._list_publishers:
            messages = self.list(collection)
            message_list = MessageList(messages=messages)
            self._list_publishers[key].publish(message_list)

        return str(result.inserted_id)

    def list_msgs(self, collection):
        response = self.list(collection)
        return [jmc.convert_json_to_ros_message(x.msg_type, x.json)
                for x in response]

    def list(self, collection):
        """Lists all messages in a collection.

        collection: A mongo_msg_db_msgs.msg.Collection

        Returns: A list of mongo_msg_db_msgs.msg.Message
        """
        collection = self._collection(collection)
        result = collection.find()
        response = []
        for msg in result:
            message = Message()
            message.id = str(msg['_id'])
            message.msg_type = msg['msg_type']
            message.json = msg['json']
            response.append(message)
        return response

    def subscribe(self, collection, id):
        """Tells the DB to publish changes to the given message.

        Changes will be published to a topic of type
        mongo_msg_db_msgs/MessageEvent.
        Note that to actually subscribe, you should create a subscriber
        yourself. The name of the topic to subscribe to is returned.

        Also publishes the current value of the message.

        Args:
            collection: A mongo_msg_db_msgs.msg.Collection.
            id: The MongoDB ID of the message.

        Returns: topic, error. topic is the name of the topic to subscribe to.
            topic is None if an error occurs. error is an error message, namely
            if the message you're subscribing to doesn't exist. error is None
            on success.
        """
        matched_count, message = self.find(collection, id)
        if matched_count > 0:
            topic = 'mongo_msg_db/{}/{}/{}'.format(collection.db,
                                                   collection.collection, id)
            if id not in self._publishers:
                self._publishers[id] = rospy.Publisher(topic, MessageEvent,
                                                       latch=True)
            event = MessageEvent(event=MessageEvent.UPDATE, message=message)
            self._publishers[id].publish(event)
            return topic, None
        else:
            return None, 'ID {} not found'.format(id)

    def subscribe_to_list(self, collection):
        """Tells the DB to publish changes to the given collection.

        Changes will be published to a topic of type
        mongo_msg_db_msgs/MessageList.
        Note that to actually subscribe, you should create a subscriber
        yourself. The name of the topic to subscribe to is returned.

        Args:
            collection: A mongo_msg_db_msgs.msg.Collection.

        Returns: topic, error. topic is the name of the topic to subscribe to.
            topic is None if an error occurs. error is an error message, namely
            if the message you're subscribing to doesn't exist. error is None
            on success.
        """
        key = self._collection_key(collection)
        topic = 'mongo_msg_db/{}/{}'.format(collection.db,
                                            collection.collection)
        if key not in self._list_publishers:
            self._list_publishers[key] = rospy.Publisher(topic, MessageList,
                                                         latch=True)
        messages = self.list(collection)
        message_list = MessageList(messages=messages)
        self._list_publishers[key].publish(message_list)
        return topic, None

    def update(self, collection, message):
        """Updates a message in a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        message: A mongo_msg_db_msgs.msg.Message. The ID of the message gives
            the message to replace, while the JSON and msg_type fields are the
            content to replace.

        Returns: 1 if the message was updated, 0 if it was not found.
        """
        mongo_collection = self._collection(collection)
        msg = {'msg_type': message.msg_type, 'json': message.json}
        result = mongo_collection.replace_one({'_id': ObjectId(message.id)},
                                              msg)
        if message.id in self._publishers:
            event = MessageEvent(event=MessageEvent.UPDATE, message=message)
            self._publishers[message.id].publish(event)

        key = self._collection_key(collection)
        if key in self._list_publishers:
            messages = self.list(collection)
            message_list = MessageList(messages=messages)
            self._list_publishers[key].publish(message_list)
        return result.matched_count


class RosMessageDb(object):
    """A wrapper around MessageDb for ROS services.
    """

    def __init__(self, db):
        """Constructor

        db: A MessageDb
        """
        self._db = db

    def delete(self, request):
        response = DeleteResponse()
        response.deleted_count = self._db.delete(request.collection,
                                                 request.id)
        return response

    def find(self, request):
        matched_count, message = self._db.find(request.collection, request.id)
        response = FindResponse()
        if matched_count == 1:
            response.matched_count = 1
            response.message = message
        return response

    def insert(self, request):
        inserted_id = self._db.insert(request.collection, request.json,
                                      request.msg_type)
        response = InsertResponse()
        response.id = str(inserted_id)
        return response

    def list(self, request):
        response = ListResponse()
        response.messages = self._db.list(request.collection)
        return response

    def subscribe(self, request):
        response = SubscribeResponse()
        response.topic, response.error = self._db.subscribe(request.collection,
                                                            request.id)
        return response

    def subscribe_to_list(self, request):
        response = SubscribeToListResponse()
        response.topic, response.error = self._db.subscribe_to_list(
            request.collection)
        return response

    def update(self, request):
        response = UpdateResponse()
        response.matched_count = self._db.update(request.collection,
                                                 request.message)
        return response
