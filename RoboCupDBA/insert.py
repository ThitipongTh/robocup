import pymongo
##function##
# blind functions
def blindInsert(host= '10.26.11.13', collection= '', data= None):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup'][collection]

    try:
        result = collection.insert_one(data)
        client.close()
        return True
    except:
        client.close()
        return False
    
def blindInsertList(host= '10.26.11.13', collection= '', data= None):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup'][collection]

    try:
        result = collection.insert_many(data)
        client.close()
        return True
    except:
        client.close()
        return False

def directUpdate(host= '10.26.11.13', collection= '', key= '', data= ''):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup'][collection]
    
    res = collection.update_one(key, {'$set': data})
    
    client.close()
    if res.modified_count:
        return True
    else:
        return False
    
    
# Map
def updateMap(host= '10.26.11.13', zone='', oren='', stat='', teamColor= 'MAGENTA'):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup']['explore_map']
    
    _zone = f'{teamColor[0]}_Z{str(zone)}'
    existing_doc = collection.find_one({"observing_zone": _zone})
    
    if existing_doc:
        collection.update_one(
            {"observing_zone": _zone},
            { "$set": {f'env.{str(oren)}': stat}}
        )
        res = True
    else:
        res = False
    
    client.close()
    return res

def resetMap(host= '10.26.11.13'):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup']['explore_map']
    
    dataList = []
    for i in range(1, 6):
        for j in range(1, 6):
            data = {}
            data['observing_zone'] = f'M_Z{i}{j}'
            data['env'] = {str(i * 45) : False for i in range(0, 8)}
            
            if i == 1:
                data['env']['0'] = True
                data['env']['45'] = True
                data['env']['315'] = True
            elif i == 5:
                data['env']['135'] = True
                data['env']['180'] = True
                data['env']['225'] = True
            
            if j == 1:
                data['env']['225'] = True
                data['env']['270'] = True
                data['env']['315'] = True
            elif j == 5:
                data['env']['45'] = True
                data['env']['90'] = True
                data['env']['135'] = True
            
            if i == 3 and j == 2:
                data['env']['225'] = True
                data['env']['270'] = True
            elif i > 3 and j == 2:
                data['env']['225'] = True
                data['env']['270'] = True
                data['env']['315'] = True
                
            dataList.append(data.copy())
    
    try: 
        for doc in dataList:
            collection.update_one({'observing_zone': doc['observing_zone']}, {"$set": {'env': doc['env']}})
        res = True
    except:
        res = None
        
    client.close()
    return res


# Station
def updateStation(host= '10.26.11.13', name= '', position= []):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup']['station_info']
    
    existing_doc = collection.find_one({"name": name})
    print(existing_doc)
    
    if existing_doc:
        ress = collection.update_one(
            {"name": name},
            { "$set": {'pose.center': position,'present': True}}
        )
        print(ress)
        res = True
    else:
        res = False
    
    client.close()
    return res


def resetStation(host= '10.26.11.13'):
    client = pymongo.MongoClient(f'mongodb://{host}:27017/')
    collection = client['robocup']['station_info']
    
    data = {
        'zone': None,
        'rotation:': 0,
        'present': False,
        'pose': {
            'center': [],
            'input': [],
            'output': []
            }
        }
    
    try: 
        collection.update_many({}, {"$set": data})
        res = True
    except:
        res = None
        
    client.close()
    return res

