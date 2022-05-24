#!/usr/bin/env python

import json
import os
import sys
from enum import Enum

class ErrorCode(Enum):
    COM    = 1
    CARLA  = 2
    FILTER = 3


def load_CAN_ID(levelToUse, filename='CAN_ID.json', field='key'):
    data = ''
    carlaIDMap = {}
    try:
        with open(os.path.join(sys.path[0],filename), 'r') as file:
            data = file.read()
    except Exception as e:
        print("Could not read CAN_ID.json file please specify path to it with --can_id or make sure specified file exists")
        print("Failed to read file: " + filename)
        quit()
    initDict = json.loads(data)
    candIDsDict = initDict['can_id']
    # try and find IDs witha not None carlaVar value for a given ID
    for key, val in candIDsDict.items():
        try:
            carlaVar = val['carlaVar']
            canLevel = val['level']
            if carlaVar is not None and canLevel == levelToUse:
                if field == 'key':
                    carlaIDMap[carlaVar] = int(key)
                else:
                    carlaIDMap[carlaVar] = val[field]
        except Exception as e:
            print("Failed to parse CAN_ID json file")
            print(e)
            print("key, val", key, ",", val)
    return carlaIDMap
