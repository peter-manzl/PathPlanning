#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Simple vehicle model with 'rotating' laser scanner
#
# Author:   Johannes Gerstmayr
# Date:     2023-04-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn
import exudyn as exu
from exudyn.utilities import *
from exudyn.robotics.utilities import AddLidar

import numpy as np
from math import sin, cos, tan
import matplotlib.pyplot as plt

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
useGraphics=True

SC = exu.SystemContainer()
mbs = SC.AddSystem()

g = [0,0,-9.81]     #gravity in m/s^2

doBreaking = False

def Rot2D(phi): 
    return np.array([[np.cos(phi),-np.sin(phi)],
                      [np.sin(phi), np.cos(phi)]])

#++++++++++++++++++++++++++++++
#wheel parameters:
rhoWheel = 500      #density kg/m^3
rWheel = 0.4            #radius of disc in m
wWheel = 0.2             #width of disc in m, just for drawing
p0Wheel = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
initialRotationCar = RotationMatrixZ(0)

v0 = -5*0 #initial car velocity in y-direction
omega0Wheel = [v0/rWheel,0,0]                   #initial angular velocity around z-axis

#v0 = [0,0,0]                                   #initial translational velocity
#exu.Print("v0Car=",v0)

#++++++++++++++++++++++++++++++
#car parameters:
p0Car = [0,0,rWheel]    #origin of disc center point at reference, such that initial contact point is at [0,0,0]
lCar = 2                #y-direction
wCar = 1.5                #x-direction
hCar = rWheel           #z-direction
mCar = 500
omega0Car = [0,0,0]                   #initial angular velocity around z-axis
v0Car = [0,-v0,0]                  #initial velocity of car center point

#inertia for infinitely small ring:
inertiaWheel = InertiaCylinder(density=rhoWheel, length=wWheel, outerRadius=rWheel, axis=0)
#exu.Print(inertiaWheel)

inertiaCar = InertiaCuboid(density=mCar/(lCar*wCar*hCar),sideLengths=[wCar, lCar, hCar])
#exu.Print(inertiaCar)
# 
rLidar = 0.5*rWheel
pLidar1 = [(-wCar*0.5-rLidar)*0, 0*(lCar*0.5+rWheel+rLidar), hCar*0.8]
# pLidar2 = [ wCar*0.5+rLidar,-lCar*0.5-rWheel-rLidar,hCar*0.5]
graphicsCar = [GraphicsDataOrthoCubePoint(centerPoint=[0,0,0],size=[wCar-1.1*wWheel, lCar+2*rWheel, hCar], 
                                         color=color4steelblue)]


graphicsCar += [GraphicsDataCylinder(pAxis=pLidar1, vAxis=[0,0,0.5*rLidar], radius=rLidar, clor=color4darkgrey)]
graphicsCar += [GraphicsDataBasis(headFactor = 4, length=2)]
# graphicsCar += [GraphicsDataCylinder(pAxis=pLidar2, vAxis=[0,0,0.5*rLidar], radius=rLidar, clor=color4darkgrey)]

[nCar,bCar]=AddRigidBody(mainSys = mbs, 
                         inertia = inertiaCar, 
                         nodeType = str(exu.NodeType.RotationEulerParameters), 
                         position = p0Car, 
                         rotationMatrix = initialRotationCar,
                         angularVelocity = omega0Car,
                         velocity=v0Car,
                         gravity = g, 
                         graphicsDataList = graphicsCar)

markerCar = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=[0,0,hCar*0.5]))


markerCar1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pLidar1))
# markerCar2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pLidar2))


nWheels = 4
markerWheels=[]
markerCarAxles=[]
oRollingDiscs=[]
sAngularVelWheels=[]

# car setup:
# ^Y, lCar
# | W2 +---+ W3
# |    |   |
# |    | + | car center point
# |    |   |
# | W0 +---+ W1
# +---->X, wCar

#ground body and marker
LL = 8
gGround = GraphicsDataCheckerBoard(point=[0.25*LL,0.25*LL,0],size=2*LL)

#obstacles:
zz=1
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[0,8,0.5*zz],size=[2*zz,zz,1*zz], color=color4dodgerblue), gGround)
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[8,6,1.5*zz],size=[zz,2*zz,3*zz], color=color4dodgerblue), gGround)
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[4,-4,0.5*zz],size=[2*zz,zz,1*zz], color=color4dodgerblue), gGround)
gGround = MergeGraphicsDataTriangleList(GraphicsDataCylinder(pAxis=[8,0,0],vAxis=[0,0,zz], radius=1.5, color=color4dodgerblue, nTiles=64), gGround)

#walls:
tt=0.2
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[0.25*LL,0.25*LL-LL,0.5*zz],size=[2*LL,tt,zz], color=color4dodgerblue), gGround)
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[0.25*LL,0.25*LL+LL,0.5*zz],size=[2*LL,tt,zz], color=color4dodgerblue), gGround)
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[0.25*LL-LL,0.25*LL,0.5*zz],size=[tt,2*LL,zz], color=color4dodgerblue), gGround)
gGround = MergeGraphicsDataTriangleList(GraphicsDataOrthoCubePoint(centerPoint=[0.25*LL+LL,0.25*LL,0.5*zz],size=[tt,2*LL,zz], color=color4dodgerblue), gGround)


oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#set up general contact geometry where sensors measure
[meshPoints, meshTrigs] = GraphicsData2PointsAndTrigs(gGround)

ngc = mbs.CreateDistanceSensorGeometry(meshPoints, meshTrigs, rigidBodyMarkerIndex=mGround, searchTreeCellSize=[8,8,1])

#single sensor:
# sDistanceSphere = mbs.CreateDistanceSensor(ngc, positionOrMarker=markerCar2, dirSensor=dirSensor2,
#                                     minDistance=0, maxDistance=maxDistance, measureVelocity=True,
#                                     cylinderRadius=0, storeInternal=True, addGraphicsObject=True, 
#                                     selectedTypeIndex=exu.ContactTypeIndex.IndexTrigsRigidBodyBased,
#                                     color=color4red)

maxDistance = 100 #max. distance of sensors; just large enough to reach everything; take care, in zoom all it will show this large area


# AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar2, minDistance=0, maxDistance=maxDistance, 
          # numberOfSensors=100,angleStart=0, angleEnd=1.5*pi, inclination=0,
          # lineLength=1, storeInternal=True, color=color4lawngreen )
mbs.variables['Lidar'] = [-pi*0.25, pi*0.25, 100]
mbs.variables['LidarAngles'] = np.linspace(mbs.variables['Lidar'][1], mbs.variables['Lidar'][0], mbs.variables['Lidar'] [2])
mbs.variables['R'] = []
for phi in mbs.variables['LidarAngles']: 
    mbs.variables['R']  += [Rot2D(phi+np.pi/2)]


mbs.variables['sLidarList'] = AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar1, minDistance=0, maxDistance=maxDistance, 
          numberOfSensors=mbs.variables['Lidar'][2],angleStart=mbs.variables['Lidar'][0], angleEnd=mbs.variables['Lidar'][1], # 1.5*pi-pi,
          lineLength=1, storeInternal=True, color=color4red, inclination=0)

if 0: 
    AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar2, minDistance=0, maxDistance=maxDistance, 
              numberOfSensors=100,angleStart=0, angleEnd=1.5*pi, inclination=-4/180*pi,
              lineLength=1, storeInternal=True, color=color4grey )
    
    AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar2, minDistance=0, maxDistance=maxDistance, 
              numberOfSensors=100,angleStart=0, angleEnd=1.5*pi, inclination= 4/180*pi,
              lineLength=1, storeInternal=True, color=color4grey )
    
    AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar2, minDistance=0, maxDistance=maxDistance, 
              numberOfSensors=100,angleStart=0, angleEnd=1.5*pi, inclination= 8/180*pi,
              lineLength=1, storeInternal=True, color=color4grey )
    
    AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar2, minDistance=0, maxDistance=maxDistance, 
              numberOfSensors=100,angleStart=0, angleEnd=1.5*pi, inclination=12/180*pi,
              lineLength=1, storeInternal=True, color=color4grey )
    
    

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if useGraphics:
    sCarVel = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, #fileName='solution/rollingDiscCarVel.txt', 
                                outputVariableType = exu.OutputVariableType.Velocity))

mbs.variables['sRot'] = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
mbs.variables['sPos'] = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sPos = []
sTrail=[]
sForce=[]


for iWheel in range(nWheels):
    frictionAngle = 0.25*np.pi #45°
    if iWheel == 0 or iWheel == 3: #difference in diagonal
        frictionAngle *= -1

    #additional graphics for visualization of rotation (JUST FOR DRAWING!):
    graphicsWheel = [GraphicsDataOrthoCubePoint(centerPoint=[0,0,0],size=[wWheel*1.1,0.7*rWheel,0.7*rWheel], color=color4lightred)]
    nCyl = 12
    rCyl = 0.1*rWheel
    for i in range(nCyl): #draw cylinders on wheels
        iPhi = i/nCyl*2*np.pi
        pAxis = np.array([0,rWheel*np.sin(iPhi),-rWheel*np.cos(iPhi)])
        vAxis = [0.5*wWheel*np.cos(frictionAngle),0.5*wWheel*np.sin(frictionAngle),0]
        vAxis2 = RotationMatrixX(iPhi)@vAxis
        rColor = color4grey
        if i >= nCyl/2: rColor = color4darkgrey
        graphicsWheel += [GraphicsDataCylinder(pAxis=pAxis-vAxis2, vAxis=2*vAxis2, radius=rCyl, 
                                               color=rColor)]
        graphicsWheel+= [GraphicsDataBasis()]

    dx = -0.5*wCar
    dy = -0.5*lCar
    if iWheel > 1: dy *= -1
    if iWheel == 1 or iWheel == 3: dx *= -1

    kRolling = 1e5
    dRolling = kRolling*0.01

    initialRotation = RotationMatrixZ(0)

    #v0Wheel = Skew(omega0Wheel) @ initialRotationWheel @ [0,0,rWheel]   #initial angular velocity of center point
    v0Wheel = v0Car #approx.

    pOff = [dx,dy,0]


    #add wheel body
    [n0,b0]=AddRigidBody(mainSys = mbs, 
                         inertia = inertiaWheel, 
                         nodeType = str(exu.NodeType.RotationEulerParameters), 
                         position = VAdd(p0Wheel,pOff), 
                         rotationMatrix = initialRotation, #np.diag([1,1,1]),
                         angularVelocity = omega0Wheel,
                         velocity=v0Wheel,
                         gravity = g, 
                         graphicsDataList = graphicsWheel)

    #markers for rigid body:
    mWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))
    markerWheels += [mWheel]

    mCarAxle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pOff))
    markerCarAxles += [mCarAxle]

    lockedAxis0 = 0
    if doBreaking: lockedAxis0 = 1
    #if iWheel==0 or iWheel==1: freeAxis = 1 #lock rotation
    mbs.AddObject(GenericJoint(markerNumbers=[mWheel,mCarAxle],rotationMarker1=initialRotation,
                               constrainedAxes=[1,1,1,lockedAxis0,1,1])) #revolute joint for wheel

    #does not work, because revolute joint does not accept off-axis
    #kSuspension = 1e4
    #dSuspension = kSuspension*0.01
    #mbs.AddObject(CartesianSpringDamper(markerNumbers=[mWheel,mCarAxle],stiffness=[0,0,kSuspension],damping=[0,0,dSuspension]))

    nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
    oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[mGround, mWheel], nodeNumber = nGeneric,
                                                  discRadius=rWheel, dryFriction=[1.,0.], dryFrictionAngle=frictionAngle,
                                                  dryFrictionProportionalZone=1e-1, 
                                                  rollingFrictionViscous=0.2*0,
                                                  contactStiffness=kRolling, contactDamping=dRolling,
                                                  visualization=VObjectConnectorRollingDiscPenalty(discWidth=wWheel, color=color4blue)))
    oRollingDiscs += [oRolling]

    strNum = str(iWheel)
    sAngularVelWheels += [mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,#fileName='solution/rollingDiscAngVelLocal'+strNum+'.txt', 
                               outputVariableType = exu.OutputVariableType.AngularVelocityLocal))]

    if useGraphics:
        sPos+=[mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,#fileName='solution/rollingDiscPos'+strNum+'.txt', 
                                   outputVariableType = exu.OutputVariableType.Position))]
    
        sTrail+=[mbs.AddSensor(SensorObject(name='Trail'+strNum,objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrail'+strNum+'.txt', 
                                   outputVariableType = exu.OutputVariableType.Position))]
    
        sForce+=[mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscForce'+strNum+'.txt', 
                                   outputVariableType = exu.OutputVariableType.ForceLocal))]



#takes as input the translational and angular velocity and outputs the velocities for all 4 wheels
#wheel axis is mounted at x-axis; positive angVel rotates CCW in x/y plane viewed from top
# car setup:
# ^Y, lCar
# | W2 +---+ W3
# |    |   |
# |    | + | car center point
# |    |   |
# | W0 +---+ W1
# +---->X, wCar
#values given for wheel0/3: frictionAngle=-pi/4, wheel 1/2: frictionAngle=pi/4; dryFriction=[1,0] (looks in lateral (x) direction)
#==>direction of axis of roll on ground of wheel0: [1,-1] and of wheel1: [1,1]
def MecanumXYphi2WheelVelocities(xVel, yVel, angVel, R, Lx, Ly):
    LxLy2 = (Lx+Ly)/2
    mat = (1/R)*np.array([[ 1,-1, LxLy2],
                          [-1,-1,-LxLy2],
                          [-1,-1, LxLy2],
                          [ 1,-1,-LxLy2]])    
    return mat @ [xVel, yVel, angVel]

def WheelVelocities2MecanumXYphi(w, R, Lx, Ly):
    LxLy2 = (Lx+Ly)/2
    mat = (1/R)*np.array([[ 1,-1, LxLy2],
                          [-1,-1,-LxLy2],
                          [-1,-1, LxLy2],
                          [ 1,-1,-LxLy2]])    
    # return mat @ [xVel, yVel, angVel]
    return np.linalg.pinv(mat) @ w


#compute velocity trajectory
def ComputeVelocity(t):
    vel = [0,0,0] #vx, vy, angVel; these are the local velocities!!!
    f=1
    if t < 3:
      # f = SmoothStep(t, 0, 1, 0, 1)
      vel = [f,0,0]
    elif t < 6:
        
      vel = [-f*0,f,0]
    elif t < 20:
      vel = [0,0,-0.125*np.pi]
    elif t < 20:
      vel = [-f,0,0]
    return vel



# ^Y, lCar
# | W2 +---+ W3
# |    |   |
# |    | + | car center point
# |    |   |
# | W0 +---+ W1
# +---->X, wCar
pControl = 5000
mbs.variables['wheelMotor'] = []
mbs.variables['loadWheel'] = []
posWheel = [[-wCar/2, -lCar/2], [wCar/2, -lCar/2], [-wCar/2, lCar/2], [wCar/2, lCar/2]]
for i in range(4):
    mbs.variables['loadWheel'] += [mbs.AddLoad(Torque(markerNumber=markerWheels[i],
                                   loadVector=[0,0,0], bodyFixed = True))]
    # mbs.variables['loadWheel'] += [mbs.AddObject(CoordinateSpringDamperExt(markerNumbers=[markerCar, markerWheels[i]], 
                                                    # ))]
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates = 1, initialCoordinates=[0]))
    # marerCarAxle = mbs.Add
    # RM0 = RotationMatrixY(-np.pi/2)
    RM0 = RotXYZ2RotationMatrix([-np.pi/2, 0, -np.pi/2])
    RM1 = RotationMatrixY(-np.pi/2) # np.eye(3)
    mAxle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition = posWheel[i] + [0]))
    # mbs.variables['wheelMotor'] += [mbs.AddObject(TorsionalSpringDamper(name='Wheel{}Motor'.format(i), 
    #                                         # mobileRobotBackDic['mAxlesList'][i]
    #                                         markerNumbers=[mAxle, markerWheels[i]],
    #                                         nodeNumber= nData, # for continuous Rotation
    #                                         stiffness = pControl, damping = pControl*0.05, 
    #                                         rotationMarker0=RM0, 
    #                                         rotationMarker1=RM1))]
   
def GetCurrentData(mbs, Rot, pos): 
    data = np.zeros([mbs.variables['nLidar'] , 2])
    # RotGL = RotationMatrixZ(np.pi/2*0)[0:2, 0:2]
    for i, sensor in enumerate(mbs.variables['sLidarList']): 
        data[i,:] =  pos[0:2] + Rot[0:2,0:2] @ mbs.variables['R'][i] @ mbs.GetSensorValues(sensor).tolist() #  + [0.32]
        
        
    return data

#%% 
flagReadPosRot = False
flagOdometry = True
flagLidarNoise = True

mbs.variables['wWheels'] = np.zeros([4])
mbs.variables['posOdom'], mbs.variables['rotOdom'], mbs.variables['tLast'] = np.array([0,0], dtype=np.float64), 0, 0
mbs.variables['phiWheels'] = np.zeros(4)

def PreStepUF(mbs, t):
    # using Prestep instead of UFLoad reduced simulation time fopr 24 seconds from 6.11887 to 4.02554 seconds (~ 34%)
    v = ComputeVelocity(t)
    wDesired = MecanumXYphi2WheelVelocities(v[0],v[1],v[2],rWheel,wCar,lCar)
    
    # wheel control
    for iWheel in range(4):
        # mbs.AddLoad(Torque(markerNumber=markerWheels[i],loadVector=[ i,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))
        wCurrent = mbs.GetSensorValues(sAngularVelWheels[iWheel])[0] #local x-axis = wheel axis
        cTorque = pControl * (wDesired[iWheel] - wCurrent)
        # phiCurrent = mbs.GetObjectParameter(mbs.variables['wheelMotor'][i], 'offset')
        # mbs.variables['phiWheels'][iWheel] += wDesired[i] * x(t - mbs.variables['tLast'])
        # mbs.SetObjectParameter(mbs.variables['wheelMotor'][i], 'offset', mbs.variables['phiWheels'][iWheel])
        # mbs.SetObjectParameter(mbs.variables['wheelMotor'][i], 'velocityOffset', wDesired[i])
        # mbs.SetObjectParameter(mbs.variables['wheelMotor'][i], 'torque', 50)
        mbs.SetLoadParameter(mbs.variables['loadWheel'][iWheel], 'loadVector', [cTorque, 0, 0])
        mbs.variables['wWheels'][iWheel] = wCurrent 
        # print("i = {}, wCurrent: {}, wDesired: {}".format(iWheel, round(wCurrent, 3), round(wDesired[iWheel], 3)))
    # calculate odometry
     
    if flagOdometry: 
        # odometry: vOdom = pinv(J) @ wWheels
        dt = mbs.sys['dynamicSolver'].it.currentStepSize 
        vOdom = WheelVelocities2MecanumXYphi(mbs.variables['wWheels'], rWheel, wCar, lCar)
        mbs.variables['rotOdom'] += vOdom[-1] * dt  # (t - mbs.variables['tLast'])
        mbs.variables['posOdom'] += Rot2D(mbs.variables['rotOdom']) @ vOdom[0:2] * dt
        # print('pos: ', mbs.variables['posOdom'])
        
    if (t - mbs.variables['tLast']) > mbs.variables['dtLidar']: 
        mbs.variables['tLast'] += mbs.variables['dtLidar']
        
        if flagReadPosRot: 
            # position and rotation taken from the gloabl data --> accurate! 
            Rot = mbs.GetSensorValues(mbs.variables['sRot']).reshape([3,3])
            pos = mbs.GetSensorValues(mbs.variables['sPos'])
        elif flagOdometry: 
            Rot = Rot2D(mbs.variables['rotOdom'])
            pos = mbs.variables['posOdom']
        data = GetCurrentData(mbs, Rot, pos)
        k = int(t/mbs.variables['dtLidar'])
        print('data {} at t: {}'.format(k, round(t, 2)))
        mbs.variables['lidarDataHistory'][k,:,:] = data
        mbs.variables['posHistory'][k] = pos
        mbs.variables['RotHistory'][k] = Rot
        # plt.plot(data[:,0], data[:,1], 'x', label='data at t=' + str(round(t, 2)))
        # plt.plot(pos[0], pos[1], 'o')
    return True


h=0.002
tEnd = 0.5
if useGraphics:
    tEnd = 14 + h


mbs.variables['tLast'] = 0
mbs.variables['dtLidar'] = 1 #50e-3
mbs.variables['nLidar'] = len(mbs.variables['sLidarList'])
nMeasure = int(tEnd/mbs.variables['dtLidar']) + 1
mbs.variables['lidarDataHistory'] = np.zeros([nMeasure, mbs.variables['nLidar'], 2])
mbs.variables['RotHistory'] = np.zeros([nMeasure, 2,2])
mbs.variables['RotHistory'][0] = np.eye(2)

mbs.variables['posHistory'] = np.zeros([nMeasure, 2])
mbs.SetPreStepUserFunction(PreStepUF)
mbs.Assemble()
data0 = GetCurrentData(mbs, mbs.GetSensorValues(mbs.variables['sRot']).reshape([3,3]), mbs.GetSensorValues(mbs.variables['sPos']))
mbs.variables['lidarDataHistory'][0] = data0
#%% 

# 
# import sys
# sys.exit()

#%% 
simulationSettings = exu.SimulationSettings() #takes currently set values or default values



simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = 0.1
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = False
simulationSettings.displayStatistics = False

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5#0.5
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #reduce step size for contact switching
simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse

speedup=True
if speedup:
    simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #reduce step size for contact switching
    simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
    
SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015

SC.visualizationSettings.openGL.lineWidth = 2
SC.visualizationSettings.openGL.shadow = 0.3
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.perspective = 0.7

#create animation:
if useGraphics:
    SC.visualizationSettings.window.renderWindowSize=[1920,1080]
    SC.visualizationSettings.openGL.multiSampling = 4

    if False: #save images
        simulationSettings.solutionSettings.sensorsWritePeriod = 0.01 #to avoid laggy visualization
        simulationSettings.solutionSettings.recordImagesInterval = 0.04
        SC.visualizationSettings.exportImages.saveImageFileName = "images/frame"

if useGraphics:
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

mbs.SolveDynamic(simulationSettings)

p0=mbs.GetObjectOutputBody(bCar, exu.OutputVariableType.Position, localPosition=[0,0,0])

plt.legend()

if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


#%% 
if True: 
    plt.close('all')
    plt.figure('Lidar')
    from matplotlib import colors as mcolors
    myColors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
    col1 = mcolors.to_rgb(myColors['red'])
    col2 = mcolors.to_rgb(myColors['green'])    
    for i in range(0, mbs.variables['lidarDataHistory'].shape[0]):   
        col_i = np.array(col1)* (1 - i/(nMeasure-1)) + np.array(col2)* (i/(nMeasure-1))
        plt.plot(mbs.variables['lidarDataHistory'][i,:,0], mbs.variables['lidarDataHistory'][i,:,1], 
                             'x', label='lidar m' + str(i), color=col_i.tolist())
        e1 = mbs.variables['RotHistory'][i][:,1]
        p = mbs.variables['posHistory'][i]
        plt.plot(p[0], p[1], 'o', color=col_i)
        plt.arrow(p[0], p[1], e1[0], e1[1], color=col_i, head_width=0.2)
        
    plt.title('lidar data: using ' + 'accurate data' * bool(flagReadPosRot) + 'inaccurate Odometry' * bool(not(flagReadPosRot)))
    plt.grid()
    plt.axis('equal')


##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
#plot results
# if useGraphics and False:
#     mbs.PlotSensor(sTrail, componentsX=[0]*4, components=[1]*4, title='wheel trails', closeAll=True,
#                markerStyles=['x ','o ','^ ','D '], markerSizes=12)
#     mbs.PlotSensor(sForce, components=[1]*4, title='wheel forces')
    
