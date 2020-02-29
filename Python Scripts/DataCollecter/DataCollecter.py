import threading
from networktables import NetworkTables
import json
import time
import mpu.io

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='172.22.11.2')
# NetworkTables.initialize(server='10.21.80.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

table = NetworkTables.getTable('SmartDashboard')
allData = {}
initTime = time.time()
while True:
    try:
        timeStamp = time.time() - initTime
        for i in table.getKeys():
            if i in allData:
                allData[i][str(timeStamp)] = table.getValue(i, 0)
            else:
                allData[i] = {str(timeStamp):table.getValue(i, 0)}
    except KeyboardInterrupt:
       mpu.io.write('JSONs\\' + str(initTime) + '.json', allData)
       exit()