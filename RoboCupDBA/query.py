import pymongo
import json
from bson.json_util import dumps
##function##
def query(host= '10.26.11.13', db="robocup", collection="", data={}):
    link= f"mongodb://{host}:27017/"
    try:
        client = pymongo.MongoClient(link)
        db = client[db]
        
        # Access a specific collection within the database
        collection = db[collection]
        query_result = collection.find_one(data)
        # Close the MongoDB connection
        client.close()
        return json.loads(dumps(query_result))
    except:
        return False

def getAll(host= '10.26.11.13', db="robocup", collection=""):
    try:
        client = pymongo.MongoClient(f'mongodb://{host}:27017/')
        # Access a specific database
        db = client[db]
        # Access a specific collection within the database
        collection = db[collection]
        dataList = collection.find({})
        
        res = []
        for data in dataList:
            res.append(data)

        client.close()
        return res
    except:
        return False