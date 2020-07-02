import numpy as np
import pinocchio as pin
import example_robot_data as robex
from scipy.optimize import fmin_bfgs
from numpy.linalg import inv,pinv,eig,norm,svd,det
import time
pin.seed(int(time.time()))  # change me :)

np.set_printoptions(precision=2, linewidth=200, suppress=True)

# --- Load robot model
robot = robex.loadTiagoNoHand()
robot.model = pin.buildReducedModel(robot.model,[11,12],robot.q0)
robot.q0 = robot.q0[:-4]
#robot.data = robot.model.createData()
robot.rebuildData()
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

#
# OPTIM 6D #########################################################
#

### Task costs
class CostReaching:
    def __init__(self,frameIndex=None, Mtarget=None):
        self.Mtarget = Mtarget if Mtarget is not None \
                       else pin.SE3(pin.utils.rotate('x',3.14/4), np.matrix([0.5, 0.1, 0.72]).T)  # x,y,z
        self.frameIndex = frameIndex if frameIndex is not None else robot.model.nframes-1

    def cost6d(self,q):
        '''Compute score from a configuration'''
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        return sum(pin.log(M.inverse() * self.Mtarget).vector**2)

    def cost3d(self,q):
        '''Compute score from a configuration'''
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        return sum((M.translation-self.Mtarget.translation)**2)

    def callback(self,q):
        print(1)
        q = np.matrix(q).T
        pin.forwardKinematics(robot.model,robot.data,q)
        M = pin.updateFramePlacement(robot.model,robot.data,self.frameIndex)
        gv.applyConfiguration('world/blue', pin.SE3ToXYZQUATtuple(M))
        gv.applyConfiguration('world/box', pin.SE3ToXYZQUATtuple(self.Mtarget))
        robot.display(q)
        time.sleep(1e-2)

reaching = CostReaching(frameIndex=53)
    
### Postural costs
class CostPosture:
    def __init__(self):
        self.qref = robot.q0.copy()

    def costPosture(self,q):
        return sum((q-self.qref)**2)
posture = CostPosture()
    
def costManipulability(q):
    J = pin.computeJointJacobian(robot.model,robot.data,q,6)
    return np.sqrt(det(J.T@J))

def costGrav(q):
    return sum(pin.computeGeneralizedGravity(robot.model,robot.data,q)**2)

def costWeightedGrav(q):
    """
    return g.T*inv(M)*g = g(q).T * aba(q,0,0) = rnea(q,0,0).T*aba(q,0,0)
    """
    #return np.dot(pin.computeGeneralizedGravity(robot.model,robot.data,q),inv(pin.crba(robot.model,robot.data,q))@pin.computeGeneralizedGravity(robot.model,robot.data,q))
    return np.dot(pin.computeGeneralizedGravity(robot.model,robot.data,q),
                  pin.aba(robot.model,robot.data,q,np.zeros(robot.model.nv),np.zeros(robot.model.nv)))

### Mixures of costs

def cost(q):
    return reaching.cost3d(q) + 1e-1* costManipulability(q) 
    #    return reaching.cost3d(q) + 1e-3 * posture.costPosture(q) #+ 1e-5* costWeightedGrav(q)


qguess = pin.randomConfiguration(robot.model)
qopt = fmin_bfgs(cost, qguess.copy(), callback=reaching.callback)

