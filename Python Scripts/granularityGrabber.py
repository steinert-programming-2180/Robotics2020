import threading
from networktables import NetworkTables
import csv

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='10.21.80.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

diffs = []
prediff = 0
while True:
    currDiff = NetworkTables.getNumber("Difference", 0)
    if currDiff != prediff:
        diffs.append(currDiff)
    
    if diffs.length() > 500:
        break


print("Connected!")