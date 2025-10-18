from coppeliasim_zmqremoteapi_client import *
import math
import time
import threading

class PioneerP3DX:

    def __init__(self, robotName, client=None) -> None:

        if client == None:
            client = RemoteAPIClient()
        self.sim = client.require('sim')
        
        self.sim_actuator = client.require('sim')
        self.sim_sensor = client.require('sim')
        
        # Get robot and motors handle
        self.simrobot = self.sim.getObject(f'/{robotName}')
        self.motorL = self.sim.getObject(f'/{robotName}/leftMotor')
        self.motorR = self.sim.getObject(f'/{robotName}/rightMotor')    
        # Get sensor handle:
        self.ultrasonic_sensor = [self.sim.getObject(f'/{robotName}/ultrasonicSensor')]
        self.ultrasonic_data = [0] * 12
        # Get vision sensor handle:
        #self.camera_sensor = self.sim.getObject(f'/{robotName}/camera')
        self.stop_sensing_thread_event = threading.Event()

    def Move(self, leftVel, rightVel):
        self.sim_actuator.setJointTargetVelocity(self.motorL, leftVel * math.pi/180)
        self.sim_actuator.setJointTargetVelocity(self.motorR, rightVel* math.pi/180)

    def Rotate(self, tetha, speed):
        initOri = self.sim_actuator.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
        targetOri = initOri - tetha
        
        if targetOri > 180:
                targetOri = (targetOri - 180) - 180
        elif targetOri < -180:
            targetOri = (targetOri + 180) + 180  

        if tetha < 0:
            self.Move(-speed, speed)
        elif tetha > 0:
            self.Move(speed, -speed)

        while True:
            currentOri = self.sim_actuator.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
            if abs(currentOri - targetOri) < 5.0:
                break
            time.sleep(0.05)
        self.Move(0, 0) 

    def ReadUltrasonicSensors(self):
        distance = []
        for i in range(12):
            ret = self.sim_sensor.readProximitySensor(self.ultrasonic_sensor[i])
            dist = ret[1]
            if dist == 0:
                dust = 3.00;           
            distance.append(dist * 100) # convert to cm
        return distance
    
    def sys_sensing(self):
        client_sensing = RemoteAPIClient()
        sim = client_sensing.require('sim')
        while not self.stop_sensing_thread_event.is_set() or sim.getSimulationState() != sim.simulation_stopped:
            for i in range(12):
                self.ultrasonic_data[i] = sim.readProximitySensor(self.ultrasonic_sensor[i])[1] * 100
            time.sleep(0.05)        
        print('Sensing thread stopped')
    
    def start_sensing(self):
        self.stop_sensing_thread_event.clear()
        self.sensing_thread = threading.Thread(target=self.sys_sensing)
        self.sensing_thread.start()
    
    def stop_sensing(self):
        self.stop_sensing_thread_event.set()
        self.sensing_thread.join()
    

if __name__ == '__main__':
    pass
