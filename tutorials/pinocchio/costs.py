import pinocchio as pin
import numpy as np    
from numpy.linalg import inv,pinv,eig,norm,svd,det

### COST 3D #####################################################################
class Cost3d:
    def __init__(self,rmodel,rdata,frameIndex=None, ptarget=None,viz=None):
        self.rmodel = rmodel
        self.rdata = rdata
        self.ptarget = ptarget if ptarget is not None \
                       else  np.array([0.5, 0.1, 0.27])
        self.frameIndex = frameIndex if frameIndex is not None else robot.model.nframes-1
        self.viz = viz
        
    def residual(self,q):
        pin.framesForwardKinematics(self.rmodel,self.rdata,q)
        M = self.rdata.oMf[self.frameIndex]
        return (M.translation-self.ptarget)

    def calc(self,q):
        return sum(self.residual(q)**2)

    ### --- Callback
    def callback(self,q):
        if self.viz is None: return
        pin.framesForwardKinematics(self.rmodel,self.rdata,q)
        M = self.rdata.oMf[self.frameIndex]
        #TODO Guilhem self.viz.applyConfiguration('world/blue', pin.SE3ToXYZQUATtuple(M))
        #TODO Guilhem self.viz.applyConfiguration('world/box', self.ptarget.tolist()+[1,0,0,0])
        self.viz.display(q)
        time.sleep(1e-2)

    def calcDiff(self,q):
        pin.framesForwardKinematics(self.rmodel,self.rdata,q)
        pin.computeJointJacobians(self.rmodel,self.rdata,q)
        M = self.rdata.oMf[self.frameIndex]
        J = pin.getFrameJacobian(self.rmodel,self.rdata,self.frameIndex,pin.LOCAL_WORLD_ALIGNED)[:3,:]
        return 2*J.T@(M.translation-self.ptarget)


### COST 6D #####################################################################
class Cost6d:
    def __init__(self,rmodel,rdata,frameIndex=None, Mtarget=None,viz=None):
        self.rmodel = rmodel
        self.rdata = rdata
        self.Mtarget = Mtarget if Mtarget is not None \
                       else pin.SE3(pin.utils.rotate('x',3.14/4), np.array([0.5, 0.1, 0.27]))  # x,y,z
        self.frameIndex = frameIndex if frameIndex is not None else robot.model.nframes-1
        self.viz = viz

    def residual(self,q):
        '''Compute score from a configuration'''
        pin.forwardKinematics(self.rmodel,self.rdata,q)
        M = pin.updateFramePlacement(self.rmodel,self.rdata,self.frameIndex)
        self.deltaM = self.Mtarget.inverse() * M
        return pin.log(self.deltaM).vector

    def calc(self,q):
        return sum(self.residual(q)**2)

    def callback(self,q):
        if self.viz is None: return
        self.viz.applyConfiguration('world/blue', pin.SE3ToXYZQUATtuple(M))
        self.viz.applyConfiguration('world/box', pin.SE3ToXYZQUATtuple(self.Mtarget))
        self.viz.display(q)
        time.sleep(1e-2)

    def calcDiff(self,q):
        J = pin.computeFrameJacobian(self.rmodel,self.rdata,q,self.frameIndex)
        r = self.residual(q)
        Jlog = pin.Jlog6(self.deltaM)
        return 2 * J.T @ Jlog.T @ r

### COST Posture #####################################################################
class CostPosture:
    def __init__(self,rmodel,rdata,qref=None, viz=None):
        # viz, rmodel and rdata are taken to respect the API but are not useful.
        self.qref = qref if qref is not None else pin.randomConfiguration(rmodel)
        self.removeFreeFlyer = robot.model.joints[1].nq == 7  # Don't penalize the free flyer if any.

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

### COST Gravity #####################################################################
class CostGravity:
    def __init__(self,rmodel,rdata,viz=None):
        self.rmodel = rmodel
        self.rdata = rdata
        self.viz = viz
    def residual(self,q):
        return pin.computeGeneralizedGravity(self.rmodel,self.rdata,q)
    def calc(self,q):
        return sum(self.residual(q)**2)
    def calcDiff(self,q):
        g = self.residual(q)
        G = pin.computeGeneralizedGravityDerivatives(self.rmodel,self.rdata,q)
        return 2*G.T@g

### COST Weighted Gravity #############################################################
class CostWeightedGravity:
    """
    return g.T*inv(M)*g = g(q).T * aba(q,0,0) = rnea(q,0,0).T*aba(q,0,0)
    """
    def __init__(self,rmodel,rdata,viz=None):
        self.rmodel = rmodel
        self.rdata = rdata
        self.viz = viz
        self.v0 = np.zeros(self.rmodel.nv)  # for convenience in the evaluation
    def calc(self,q):
        g = pin.computeGeneralizedGravity(self.rmodel,self.rdata,q)
        taugrav = -pin.aba(self.rmodel,self.rdata,q,self.v0,self.v0)
        return np.dot(taugrav,g)
    def calcDiff(self,q):
        pin.computeABADerivatives (self.rmodel,self.rdata,q,self.v0,self.v0)
        pin.computeRNEADerivatives(self.rmodel,self.rdata,q,self.v0,self.v0)
        return -self.rdata.dtau_dq.T@self.rdata.ddq - self.rdata.ddq_dq.T@self.rdata.tau

### TESTS ###
### TESTS ###
### TESTS ###

if __name__ == "__main__":
    import example_robot_data as robex
    import copy
    
    def Tdiff1(func,exp,nv,q,eps=1e-6):
        '''
        Num diff for a function whose input is in a manifold
        '''
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
            

    ### Num diff checking, for each cost.
    robot = robex.loadTalos()
    
    q = pin.randomConfiguration(robot.model)
    # Tdiffq is used to compute the tangent application in the configuration space.
    Tdiffq = lambda f,q: Tdiff1(f,lambda q,v:pin.integrate(robot.model,q,v),robot.model.nv,q)

    ### Test Cost3d
    CostClass = Cost3d
    cost = CostClass(robot.model,robot.data)
    Tg = cost.calcDiff(q)
    Tgn = Tdiffq(cost.calc,q)
    assert( norm(Tg-Tgn)<1e-4)

    ### Test Cost6d
    CostClass = Cost6d
    cost = CostClass(robot.model,robot.data)
    Tg = cost.calcDiff(q)
    Tgn = Tdiffq(cost.calc,q)
    assert( norm(Tg-Tgn)<1e-4)

    ### Test CostPosture
    CostClass = CostPosture
    cost = CostClass(robot.model,robot.data)
    Tg = cost.calcDiff(q)
    Tgn = Tdiffq(cost.calc,q)
    assert( norm(Tg-Tgn)<1e-4)

    ### Test CostGravity
    CostClass = CostGravity
    cost = CostClass(robot.model,robot.data)
    Tg = cost.calcDiff(q)
    Tgn = Tdiffq(cost.calc,q)
    assert( norm(Tg-Tgn)/cost.calc(q)<1e-4)

    ### Test CostWeightedGravity
    CostClass = CostWeightedGravity
    cost = CostClass(robot.model,robot.data)
    Tg = cost.calcDiff(q)
    Tgn = Tdiffq(cost.calc,q)
    assert( norm(Tg-Tgn)/cost.calc(q)<1e-4)
