import numpy as np
import pinocchio as pin
import example_robot_data as robex
from scipy.optimize import fmin_bfgs
from numpy.linalg import inv,pinv,eig,norm,svd,det
import time
pin.seed(int(time.time()))  # change me :)

np.set_printoptions(precision=4, linewidth=200, suppress=True)

# --- Load robot model
robot = robex.loadTiagoNoHand()
robot.model = pin.buildReducedModel(robot.model,[11,12],robot.q0)
robot.q0 = robot.q0[:-4]
robot.rebuildData()

#robot = robex.loadSolo()
#robot = robex.loadUR()
robot = robex.loadTalos()

robot.initViewer(loadModel=True)
robot.display(robot.q0)
NQ = robot.model.nq
NV = robot.model.nv
#robot.q0[:] = [0, -3.14/3, -3.14/2, 0, 0, 0]

    
# --- Add box to represent target
gv=robot.viewer.gui
gv.addBox("world/box",  .05,.1,.2, [1., .2, .2, .5])
gv.addSphere("world/ball", .05, [.2, .2, 1., .5])
gv.applyConfiguration('world/box', [0.5,.2,.2,1,0,0,0])
gv.applyConfiguration('world/ball', [-0.5,.2,.2,1,0,0,0])


robotEffectorMap = { 'tiago': 'wrist_tool_joint',
                     'solo': 'FL_FOOT',
                     'ur5': 'tool0',
                     'talos': 'wrist_right_ft_tool_link',
}
assert(robot.model.existFrame(robotEffectorMap[robot.model.name]))
effectorFrameIndex = robot.model.getFrameId(robotEffectorMap[robot.model.name])

#
# COST FUNCTIONS  #########################################################
#
# First, we define a bunch of cost function. Each cost function is defined as a class
# with a "calc" function to evaluate the cost, and a calcDiff function to evaluate the gradient.
# The cost that are implemented so far are:
#   - reaching cost, in 3d and in 6d
#   - postural cost
#   - gravity cost, with and without normalization by the mass matrix
#   - manipulability cost


### Task costs
class CostReaching:
    def __init__(self,frameIndex=None, Mtarget=None,dim = 3):
        self.Mtarget = Mtarget if Mtarget is not None \
                       else pin.SE3(pin.utils.rotate('x',3.14/4), np.matrix([0.5, 0.1, 0.72]).T)  # x,y,z
        self.frameIndex = frameIndex if frameIndex is not None else robot.model.nframes-1
        assert(dim == 3 or dim == 6)
        self.dim=dim
        
    ###
    def calc(self,q):
        if self.dim==3: return self.calc3d(q)
        else: return self.calc6d(q)
    def calcDiff(self,q):
        if self.dim==3: return self.calcDiff3d(q)
        else: return self.calcDiff6d(q)
        
    ### -- 6D
    def residual6d(self,q):
        '''Compute score from a configuration'''
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        self.deltaM = self.Mtarget.inverse() * M
        return pin.log(self.deltaM).vector
    def calc6d(self,q):
        return sum(self.residual6d(q)**2)
    def calcDiff6d(self,q):
        J = pin.computeFrameJacobian(robot.model,robot.data,q,self.frameIndex)
        r = self.residual6d(q)
        Jlog = pin.Jlog6(self.deltaM)
        return 2 * J.T @ Jlog.T @ r

    ### -- 3D
    def residual3d(self,q):
        '''Compute score from a configuration'''
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        return (M.translation-self.Mtarget.translation)
    def calc3d(self,q):
        return sum(self.residual3d(q)**2)
    def calcDiff3d(self,q):
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        pin.computeJointJacobians(robot.model,robot.data,q)
        J = pin.getFrameJacobian(robot.model,robot.data,self.frameIndex,pin.LOCAL_WORLD_ALIGNED)[:3,:]
        return 2*J.T@(M.translation-self.Mtarget.translation)
        
    ### --- Callback
    def callback(self,q):
        print(1)
        q = np.matrix(q).T
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        gv.applyConfiguration('world/blue', pin.SE3ToXYZQUATtuple(M))
        gv.applyConfiguration('world/box', pin.SE3ToXYZQUATtuple(self.Mtarget))
        robot.display(q)
        time.sleep(1e-2)
        
costReaching = CostReaching(frameIndex=effectorFrameIndex)
    
### Postural costs
class CostPosture:
    def __init__(self):
        self.qref = robot.q0.copy()
        self.removeFreeFlyer = robot.model.joints[1].nq == 7
    def residual(self,q):
        if self.removeFreeFlyer:
            return (q-self.qref)[7:]
        else:
            return q-self.qref
    def calc(self,q):
        return sum(self.residual(q)**2)
    def calcDiff(self,q):
        if self.removeFreeFlyer:
            g = np.zeros(robot.model.nv)
            g[6:] = 2*self.residual(q)
            return g
        else:
            return 2*self.residual(q)
costPosture = CostPosture()
    
class CostManipulability:
    def __init__(self,jointIndex=None,frameIndex=None):
        if frameIndex is not None:
            jointIndex = robot.model.frames[frameIndex].parent
        self.jointIndex = jointIndex if jointIndex is not None else robot.model.njoints-1
    def calc(self,q):
        J = self.J=pin.computeJointJacobian(robot.model,robot.data,q,self.jointIndex)
        return np.sqrt(det(J@J.T))
    def calcDiff(self,q):
        Jp = pinv(pin.computeJointJacobian(robot.model,robot.data,q,self.jointIndex))
        res = np.zeros(robot.model.nv)
        v0 = np.zeros(robot.model.nv)
        for k in range(6):
            pin.computeForwardKinematicsDerivatives(robot.model,robot.data,q,Jp[:,k],v0)
            JqJpk = pin.getJointVelocityDerivatives(robot.model,robot.data,self.jointIndex,pin.LOCAL)[0]
            res += JqJpk[k,:]
        res *= self.calc(q)
        return res

costManipulability = CostManipulability(frameIndex=effectorFrameIndex)

class CostGrav:
    def residual(self,q):
        return pin.computeGeneralizedGravity(robot.model,robot.data,q)
    def calc(self,q):
        return sum(self.residual(q)**2)
    def calcDiff(self,q):
        g = self.residual(q)
        G = pin.computeGeneralizedGravityDerivatives(robot.model,robot.data,q)
        return 2*G.T@g

costGrav = CostGrav()

class CostWeightedGrav:
    """
    return g.T*inv(M)*g = g(q).T * aba(q,0,0) = rnea(q,0,0).T*aba(q,0,0)
    """
    def __init__(self):
        self.v0 = np.zeros(robot.model.nv)
    def calc(self,q):
        g = pin.computeGeneralizedGravity(robot.model,robot.data,q)
        taugrav = -pin.aba(robot.model,robot.data,q,self.v0,self.v0)
        return np.dot(taugrav,g)
    def calcDiff(self,q):
        pin.computeABADerivatives (robot.model,robot.data,q,self.v0,self.v0)
        pin.computeRNEADerivatives(robot.model,robot.data,q,self.v0,self.v0)
        return -robot.data.dtau_dq.T@robot.data.ddq - robot.data.ddq_dq.T@robot.data.tau

costWeightedGrav = CostWeightedGrav()


#
# DERIVATIVE CHECKING #########################################
#
# We now define so num-diff routines. The basic numdiff does not know anything
# about lie groups, and approximate the vector "coefficient-wise" derivatives.
# The two Tdiff functions are to properly derivate when the input or when the output
# are Lie groups (hence the jacobian is indeed a tangent application).

import copy
def numdiff(func,x,eps=1e-6,normalize=None):
    f0 = copy.copy(func(x))
    xe = x.copy()
    fs = []
    for k in range(len(x)):
        xe[k] += eps
        xen = xe if normalize is None else normalize(xe)
        fs.append( (func(xen)-f0)/eps )
        xe[k] -= eps
    if isinstance(fs[0],np.ndarray) and len(f0)>1: return np.stack(fs,axis=1)
    else: return np.array(fs)
    #return np.stack(fs,axis=1 if isinstance(f0,np.ndarray) and len(f0)>1 else 0)

def Tdiff1(func,exp,nv,q,eps=1e-6):
    f0 = copy.copy(func(q))
    fs = []
    v = np.zeros(nv)
    for k in range(nv):
        v[k] = eps
        qk = exp(q,v)
        fs.append( (func(qk)-f0)/eps )
        v[k] -= eps
    if isinstance(fs[0],np.ndarray) and len(fs[0])>1: return np.stack(fs,axis=1)
    else: return np.array(fs)

def Tdiff2(func,log,q,eps=1e-6):
    f0 = copy.copy(func(q))
    fs = []
    qk = q.copy()
    for k in range(len(qk)):
        qk[k] += eps
        fs.append( log(func(qk),f0)/eps )
        qk[k] -= eps
    if isinstance(fs[0],np.ndarray) and len(fs[0])>1: return np.stack(fs,axis=1)
    else: return np.array(fs)

# Tdiffq is used to compute the tangent application in the configuration space.
Tdiffq = lambda f,q: Tdiff1(f,lambda q,v:pin.integrate(robot.model,q,v),robot.model.nv,q)

### Num diff checking, for each cost.
q = pin.randomConfiguration(robot.model)

costReaching.dim = 6
Tg6 = costReaching.calcDiff6d(q)
g6n = numdiff(costReaching.calc6d,q,normalize=lambda q: pin.normalize(robot.model,q))
Tg6n = Tdiffq(costReaching.calc6d,q)
from dexp import dExpQ_inv
g6 = Tg6@dExpQ_inv(robot.model,q)
assert( norm(Tg6-Tg6n)<1e-4)
assert( norm(g6-g6n)<1e-4)

costReaching.dim = 3
Tg3 = costReaching.calcDiff3d(q)
g3n = numdiff(costReaching.calc3d,q,normalize=lambda q: pin.normalize(robot.model,q))
g3 = Tg3@dExpQ_inv(robot.model,q)
assert( norm(g3-g3n)<1e-4)
Tg = costReaching.calcDiff3d(q)
Tgn = Tdiffq(costReaching.calc3d,q)
assert( norm(Tg-Tgn)<1e-4)

Tg = costPosture.calcDiff(q)
Tgn = Tdiffq(costPosture.calc,q)
assert( norm(Tg-Tgn)<1e-4)

Tg = costGrav.calcDiff(q)
Tgn = Tdiffq(costGrav.calc,q)
assert( norm(Tg-Tgn)/costGrav.calc(q)<1e-4)

Tg = costWeightedGrav.calcDiff(q)
Tgn = Tdiffq(costWeightedGrav.calc,q)
assert( norm(Tg-Tgn)/costWeightedGrav.calc(q)<1e-4)

c=costManipulability
Tg = costManipulability.calcDiff(q)
Tgn = Tdiffq(costManipulability.calc,q)
#assert( norm(Tg-Tgn)<1e-4)


#
# Simple Optimization Program #####################################################
#
# A first simple optimization program is implemented: reach a target, optimize the posture
#

### Mixures of costs
class MyCost:
    def calc(self,q):
        q = pin.normalize(robot.model,q)
        costReaching.dim = 3
        return costReaching.calc(q) + 1e-1* costWeightedGrav.calc(q) 
    def calcDiff(self,q):
        return
    def callback(self,q):
        q = pin.normalize(robot.model,q)
        costReaching.callback(q)

qguess = pin.randomConfiguration(robot.model)
cost = MyCost()
#qopt = fmin_bfgs(cost.calc, qguess.copy(), callback=cost.callback
#q = qopt.copy()
q = pin.normalize(robot.model,q)


#
# Complete Optimization Program #####################################################
#
#
class CostFull:
    def __init__(self):
        self.reachLL = CostReaching( robot.model.getFrameId('leg_left_sole_fix_joint'),
                                     pin.SE3( np.eye(3), np.array([ 0, 0.3, 0])) ,dim=6)
        self.reachRL = CostReaching( robot.model.getFrameId('leg_right_sole_fix_joint'),
                                     pin.SE3( np.eye(3), np.array([ 0, -0.3, 0])) ,dim=6)
        self.reachTool = CostReaching( robot.model.getFrameId('wrist_right_ft_tool_link'),
                                       pin.SE3( np.eye(3), np.array([ .3, -0.3, 1.5])) ,dim=3)
        self.posture = CostWeightedGrav() # CostPosture()
        
    def calc(self,q):
        q = pin.normalize(robot.model,q)
        return self.reachTool.calc(q) \
            + self.reachLL.calc(q)*10 \
            + self.reachRL.calc(q)*10 \
            + self.posture.calc(q)*1e-2
    
    def calcDiff(self,q):
        q = pin.normalize(robot.model,q)
        Tg = self.reachTool.calcDiff(q) \
             + self.reachLL.calcDiff(q)*10 \
             + self.reachRL.calcDiff(q)*10 \
             + self.posture.calcDiff(q)*1e-2
        Q = dExpQ_inv(robot.model,q)
        return Tg@Q
    
    def calcT(self,q):
        q = pin.normalize(robot.model,q)
        Tg = self.reachTool.calcDiff(q) \
             + self.reachLL.calcDiff(q)*10 \
             + self.reachRL.calcDiff(q)*10 \
             + self.posture.calcDiff(q)*1e-2
        return Tg
        
    def callback(self,q):
        q = pin.normalize(robot.model,q)
        self.reachTool.callback(q)

assert(robot.model.name == "talos")
cost = CostFull()

q = pin.randomConfiguration(robot.model)
g = cost.calcDiff(q)
gn = numdiff(cost.calc,q)
assert(norm(g-gn)/robot.model.nv<1e-4)
        
qopt = fmin_bfgs(cost.calc, fprime = cost.calcDiff, x0=robot.q0, callback=cost.callback)
q=pin.normalize(robot.model,qopt)
