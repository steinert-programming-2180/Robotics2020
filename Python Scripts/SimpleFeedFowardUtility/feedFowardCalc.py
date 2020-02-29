import threading
from networktables import NetworkTables

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

#NetworkTables.initialize(server='172.22.11.2')
NetworkTables.initialize(server='10.21.80.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

table = NetworkTables.getTable('SmartDashboard')

while True:
	leftvelocity = table.getNumber('LeftVelocity', 0)
	#appliedPercentage = table.getNumber('Applied')
	leftvoltage = table.getNumber('LeftVoltage', 0)
	rightvelocity = table.getNumber('RightVelocity', 0)
	rightvoltage = table.getNumber('Right Voltage', 0)
	if ((leftvelocity > 0) and (leftvoltage > 0)):
		leftratio = leftvoltage / leftvelocity
	else:
		leftratio = 0
	
	if ((rightvelocity > 0) and (rightvoltage > 0)):
		rightratio = rightvoltage / rightvelocity
	else:
		rightratio = 0
	
	print("Left: " + str(leftratio))
	print("Right: " + str(rightratio))

print("Connected!")