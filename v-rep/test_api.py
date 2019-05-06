#-*- coding:utf-8 -*-
import vrep

# close the previous connection
vrep.simxFinish(-1) 
#connect to server
clientId = vrep.simxStart("127.0.0.1", 19999, True, True, 5000, 5) 
if clientId != -1:  # successfully connected
    print('connect successfully')
else:
    print('connect failed')
vrep.simxFinish(clientId)
print('program ended')   