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
    def __init__(self, mongo_client):
        self._mongo_client = mongo_client

    def _collection(self, collection_msg):
        db = getattr(self._mongo_client, collection_msg.db)
        collection = getattr(db, collection_msg.collection)
        return collection

    def delete(self, request):
        collection = self._collection(request.collection)
        result = collection.delete_one({'_id': ObjectId(request.id)})
        response = DeleteResponse()
        response.deleted_count = result.deleted_count
        return response

    def find(self, request):
        collection = self._collection(request.collection)
        result = collection.find_one({'_id': ObjectId(request.id)})
        response = FindResponse()
        if result is None:
            response.matched_count = 0
        else:
            response.matched_count = 1
            response.message.id = request.id
            response.message.msg_type = result['msg_type']
            response.message.json = result['json']
        return response

    def insert(self, request):
        collection = self._collection(request.collection)
        result = collection.insert_one(
            {'msg_type': request.msg_type,
             'json': request.json})
        response = InsertResponse()
        response.id = str(result.inserted_id)
        return response

    def list(self, request):
        collection = self._collection(request.collection)
        result = collection.find()
        response = ListResponse()
        for msg in result:
            message = Message()
            message.id = str(msg['_id'])
            message.msg_type = msg['msg_type']
            message.json = msg['json']
            response.messages.append(message)
        return response

    def update(self, request):
        collection = self._collection(request.collection)
        msg = {
            'msg_type': request.message.msg_type,
            'json': request.message.json
        }
        result = collection.replace_one({'_id': ObjectId(request.message.id)},
                                        msg)
        response = UpdateResponse()
        response.matched_count = result.matched_count
        return response
