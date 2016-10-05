#! /usr/bin/env python
from bson.objectid import ObjectId
from mongo_msg_db import MessageDb
from mongo_msg_db import RosMessageDb
from mongo_msg_db_msgs.msg import Collection
from mongo_msg_db_msgs.msg import Message
from mongo_msg_db_msgs.msg import MessageEvent
from mongo_msg_db_msgs.msg import MessageList
from mongo_msg_db_msgs.srv import DeleteRequest
from mongo_msg_db_msgs.srv import FindRequest
from mongo_msg_db_msgs.srv import InsertRequest
from mongo_msg_db_msgs.srv import ListRequest
from mongo_msg_db_msgs.srv import SubscribeRequest
from mongo_msg_db_msgs.srv import SubscribeToListRequest
from mongo_msg_db_msgs.srv import UpdateRequest
from pymongo import MongoClient
import json
import mock
import nose
from nose.tools import assert_equals


def testCollection():
    mongo_client = mock.Mock()
    db = MessageDb(mongo_client)
    db_p = mock.PropertyMock()
    coll_p = mock.PropertyMock()
    type(mongo_client).test = db_p
    type(mongo_client.test).collection = coll_p

    collection = Collection()
    collection.db = 'test'
    collection.collection = 'collection'
    db._collection(collection)
    db_p.assert_has_calls([mock.call(), mock.call()])
    coll_p.assert_called_once_with()


def testDelete():
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    request = InsertRequest()
    request.collection.db = 'test'
    request.collection.collection = 'commands'
    request.msg_type = 'std_msgs/String'
    request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(request)

    request = DeleteRequest()
    request.collection.db = 'test'
    request.collection.collection = 'commands'
    request.id = 'non-existent'
    response = db.delete(request)
    response.deleted_count == 0
    assert_equals(mongo_client.test.commands.count(), 1)

    request.id = insert_response.id
    response = db.delete(request)
    assert_equals(mongo_client.test.commands.count(), 0)
    assert_equals(response.deleted_count, 1)


def testFind():
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    request = FindRequest()
    request.collection.db = 'test'
    request.collection.collection = 'commands'
    request.id = 'non-existent'
    response = db.find(request)
    assert_equals(response.matched_count, 0)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(insert_request)

    request = FindRequest()
    request.collection.db = 'test'
    request.collection.collection = 'commands'
    request.id = 'non-existent'
    response = db.find(request)
    assert_equals(response.matched_count, 0)

    request.id = insert_response.id
    response = db.find(request)
    assert_equals(response.matched_count, 1)
    nose.tools.assert_is_not_none(response.message)
    assert_equals(response.message.msg_type, insert_request.msg_type)
    assert_equals(response.message.json, insert_request.json)


def testInsert():
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    request = InsertRequest()
    request.collection.db = 'test'
    request.collection.collection = 'commands'
    request.msg_type = 'std_msgs/String'
    request.json = json.dumps({'data': 'Hello'})
    response = db.insert(request)
    assert_equals(mongo_client.test.commands.count(), 1)
    nose.tools.assert_is_not_none(
        mongo_client.test.commands.find_one({'_id': ObjectId(response.id)}))


def testList():
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    request = ListRequest()
    request.collection.db = 'test'
    request.collection.collection = 'commands'
    response = db.list(request)
    assert_equals(len(response.messages), 0)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(insert_request)
    response = db.list(request)
    assert_equals(len(response.messages), 1)
    assert_equals(response.messages[0].id, insert_response.id)

    insert_request.json = json.dumps({'data': 'World'})
    insert_response = db.insert(insert_request)
    response = db.list(request)
    assert_equals(len(response.messages), 2)


def testUpdate():
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    request1 = UpdateRequest()
    request1.collection.db = 'test'
    request1.collection.collection = 'commands'
    request1.message.id = 'non-existent'
    response1 = db.update(request1)
    assert_equals(response1.matched_count, 0)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(insert_request)

    request2 = UpdateRequest()
    request2.collection.db = 'test'
    request2.collection.collection = 'commands'
    request2.message.id = insert_response.id
    request2.message.msg_type = 'std_msgs/String'
    request2.message.json = json.dumps({'data': 'World'})
    response2 = db.update(request2)
    assert_equals(response2.matched_count, 1)
    assert_equals(mongo_client.test.commands.count(), 1)
    doc = mongo_client.test.commands.find_one(
        {'_id': ObjectId(insert_response.id)})
    assert_equals(doc['json'], request2.message.json)


def testMultipleCollections():
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(insert_request)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands2'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'World'})
    insert_response = db.insert(insert_request)

    assert_equals(mongo_client.test.commands.count(), 1)
    assert_equals(mongo_client.test.commands2.count(), 1)


@mock.patch('rospy.Publisher')
def testSubscribe(Publisher):
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(insert_request)

    subscribe_request = SubscribeRequest()
    subscribe_request.collection.db = 'test'
    subscribe_request.collection.collection = 'commands'
    subscribe_request.id = insert_response.id
    subscribe_response = db.subscribe(subscribe_request)

    topic = 'mongo_msg_db/test/commands/{}'.format(insert_response.id)
    assert subscribe_response.topic == topic
    assert subscribe_response.error is None

    inserted_message = Message(id=insert_response.id,
                               json=insert_request.json,
                               msg_type='std_msgs/String')
    event = MessageEvent(event=MessageEvent.UPDATE, message=inserted_message)
    mdb._publishers[insert_response.id].publish.assert_called_with(event)

    update_request = UpdateRequest()
    update_request.collection.db = 'test'
    update_request.collection.collection = 'commands'
    update_request.message.id = insert_response.id
    update_request.message.json = json.dumps({'data': 'World'})
    update_request.message.msg_type = 'std_msgs/String'
    update_response = db.update(update_request)

    # Expected update event to be published
    event = MessageEvent(event=MessageEvent.UPDATE,
                         message=update_request.message)
    mdb._publishers[insert_response.id].publish.assert_called_with(event)

    delete_request = DeleteRequest()
    delete_request.collection.db = 'test'
    delete_request.collection.collection = 'commands'
    delete_request.id = insert_response.id
    delete_response = db.delete(delete_request)

    # Expected delete event to be published
    empty = Message()
    event = MessageEvent(event=MessageEvent.DELETE, message=empty)
    mdb._publishers[insert_response.id].publish.assert_called_with(event)


@mock.patch('rospy.Publisher')
def testSubscribeList(Publisher):
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    insert_request = InsertRequest()
    insert_request.collection.db = 'test'
    insert_request.collection.collection = 'commands'
    insert_request.msg_type = 'std_msgs/String'
    insert_request.json = json.dumps({'data': 'Hello'})
    insert_response = db.insert(insert_request)

    subscribe_request = SubscribeToListRequest()
    subscribe_request.collection.db = 'test'
    subscribe_request.collection.collection = 'commands'
    subscribe_response = db.subscribe_to_list(subscribe_request)

    topic = 'mongo_msg_db/test/commands'
    assert subscribe_response.topic == topic
    assert subscribe_response.error is None

    inserted_message = Message(id=insert_response.id,
                               json=insert_request.json,
                               msg_type='std_msgs/String')
    msg_list = MessageList(messages=[inserted_message])
    mdb._list_publishers['test', 'commands'].publish.assert_called_with(
        msg_list)

    update_request = UpdateRequest()
    update_request.collection.db = 'test'
    update_request.collection.collection = 'commands'
    update_request.message.id = insert_response.id
    update_request.message.json = json.dumps({'data': 'World'})
    update_request.message.msg_type = 'std_msgs/String'
    update_response = db.update(update_request)

    # Expected updated list to be published
    msg_list = MessageList(messages=[update_request.message])
    mdb._list_publishers['test', 'commands'].publish.assert_called_with(
        msg_list)

    delete_request = DeleteRequest()
    delete_request.collection.db = 'test'
    delete_request.collection.collection = 'commands'
    delete_request.id = insert_response.id
    delete_response = db.delete(delete_request)

    # Expected empty list to the published
    msg_list = MessageList()
    mdb._list_publishers['test', 'commands'].publish.assert_called_with(
        msg_list)

@mock.patch('rospy.Publisher')
def testSubscribeFailure(Publisher):
    mongo_client = MongoClient()
    mongo_client.drop_database('test')
    mdb = MessageDb(mongo_client)
    db = RosMessageDb(mdb)

    subscribe_request = SubscribeRequest()
    subscribe_request.collection.db = 'test'
    subscribe_request.collection.collection = 'commands'
    subscribe_request.id = str(ObjectId())
    subscribe_response = db.subscribe(subscribe_request)

    assert subscribe_response.topic is None
    assert subscribe_response.error is not None
