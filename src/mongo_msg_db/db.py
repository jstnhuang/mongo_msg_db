#!/usr/bin/env python
from bson.objectid import ObjectId
from pymongo import MongoClient

from mongo_msg_db_msgs.msg import Collection
from mongo_msg_db_msgs.msg import Message
from mongo_msg_db_msgs.srv import DeleteResponse
from mongo_msg_db_msgs.srv import FindResponse
from mongo_msg_db_msgs.srv import InsertResponse
from mongo_msg_db_msgs.srv import ListResponse
from mongo_msg_db_msgs.srv import UpdateResponse


class MessageDb(object):
    """A simple JSON message database.
    """

    def __init__(self, mongo_client):
        self._mongo_client = mongo_client

    def _collection(self, collection_msg):
        db = getattr(self._mongo_client, collection_msg.db)
        collection = getattr(db, collection_msg.collection)
        return collection

    def delete(self, collection, id):
        """Deletes a message from a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        id: The ObjectId of the message to delete, as a string.

        Returns: number of messages deleted.
        """
        collection = self._collection(collection)
        result = collection.delete_one({'_id': ObjectId(id)})
        return result.deleted_count

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

    def insert(self, collection, json, msg_type):
        """Inserts a message into a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        json: The JSON string representation of the message to insert.
        msg_type: The name of the message type, e.g., std_msgs/String

        Returns: The ObjectId of the inserted item, as a string.
        """
        collection = self._collection(collection)
        result = collection.insert_one({'msg_type': msg_type, 'json': json})
        return str(result.inserted_id)

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

    def update(self, collection, message):
        """Updates a message in a collection.

        collection: A mongo_msg_db_msgs.msg.Collection
        message: A mongo_msg_db_msgs.msg.Message. The ID of the message gives
            the message to replace, while the JSON and msg_type fields are the
            content to replace.

        Returns: 1 if the message was updated, 0 if it was not found.
        """
        collection = self._collection(collection)
        msg = {'msg_type': message.msg_type, 'json': message.json}
        result = collection.replace_one({'_id': ObjectId(message.id)}, msg)
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

    def update(self, request):
        response = UpdateResponse()
        response.matched_count = self._db.update(request.collection,
                                                 request.message)
        return response
