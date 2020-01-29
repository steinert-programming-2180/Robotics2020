import threading
from networktables import NetworkTables

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='172.22.11.1')
# NetworkTables.initialize(server='10.21.80.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

table = NetworkTables.getTable('SmartDashboard')

while True:
	velocity = table.getNumber('Velocity')
	appliedPercentage = table.getNumber('Applied')
	voltage = table.getNumber('BusVoltage')

print("Connected!")