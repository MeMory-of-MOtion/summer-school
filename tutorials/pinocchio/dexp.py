'''
Compute the vector derivative of the configuration-space integrale.
'''

import numpy as np
import pinocchio as pin

def dExpSO3(quat):
    """
    Return the vector "coefficient-wise" jacobian of w->quat(x)Exp(w), which is a 4x3 matrix,
    where Exp: w->quat2 is the exponential for quaternion, i.e. returning the quaternion quat2
    so that quat2.matrix() is the same as pin.exp3(w); and (x) is the quaternion product.
    The quaternion quat is supposed to have the order (x,y,z,w), and the jacobian rows
    are ordered in the same way.
    """
    x,y,z,w = quat/2
    return  np.array([[  w,  -z,   y],
                      [  z,   w,  -x],
                      [ -y,   x,   w],
                      [ -x,  -y,  -z]])

def dExpQ(rmodel,q):
    """
    Return the vector "coefficient-wise" jacobian of vq -> rmodel.integrate(q,vq), which is a 
    NQ x NV matrix.
    The function is ad-hoc and only works for a robot model whose first joint is a free flyer
    and other joints are 1d (euclidean) joints.
    """
    if np.all([ j.nq==1 for j in rmodel.joints]): return np.eye(rmodel.nv)
    assert(rmodel.joints[1].nq==7 and rmodel.joints[1].nv==6)
    for j in rmodel.joints[2:]:
        assert(j.nq==1 and j.nv==1)
    Q = np.zeros([rmodel.nq,rmodel.nv])
    Q[:3,:3] = pin.Quaternion(q[6],q[3],q[4],q[5]).matrix()
    Q[3:7,3:6] = dExpSO3(q[3:7])
    np.fill_diagonal(Q[7:,6:],1)
    return Q

def dExpQ_inv(rmodel,q):
    """
    Return the pseudo inverse of dExpQ (only works for free-flyer+revolute robots).
    """
    if np.all([ j.nq==1 for j in rmodel.joints]): return np.eye(rmodel.nv)
    assert(rmodel.joints[1].nq==7 and rmodel.joints[1].nv==6)
    for j in rmodel.joints[2:]:
        assert(j.nq==1 and j.nv==1)
    Q = np.zeros([rmodel.nv,rmodel.nq])
    Q[:3,:3] = pin.Quaternion(q[6],q[3],q[4],q[5]).matrix().T
    # dExpSO3 is half of a projector Q=.5*P, hence its inverse is twice the projector
    # transposed, i.e. inv(Q) = 2*P.T = 4*Q.T
    Q[3:6,3:7] = 4*dExpSO3(q[3:7]).T
    np.fill_diagonal(Q[6:,7:],1)
    return Q


### --- UNIT TEST ------------------------------------------------------------------------
### --- UNIT TEST ------------------------------------------------------------------------
### --- UNIT TEST ------------------------------------------------------------------------
if __name__ == "__main__":
    import example_robot_data as robex
    from numpy.linalg import inv,pinv,eig,norm,svd,det
    import time
    import copy
    pin.seed(int(time.time()))  # change me :)

    np.set_printoptions(precision=2, linewidth=200, suppress=True)

    # --- Load robot model
    robot = robex.loadTalos()
    NQ = robot.model.nq
    NV = robot.model.nv
    q = pin.randomConfiguration(robot.model)
    
    def exampleFunction(q):
        # This is important, as we are going to finite diff on q, i.e. the function
        # will receive non-normalized inputs.
        q = pin.normalize(robot.model,q)
        # The function returns the log of a robot frame placement.
        Mtarget = pin.SE3(pin.utils.rotate('x',3.14/4), np.matrix([0.5, 0.1, 0.72]).T)  # x,y,z
        idx =  robot.model.getFrameId('wrist_right_ft_tool_link') #('FL_FOOT')
        pin.forwardKinematics(robot.model,robot.data,q)
        pin.updateFramePlacements(robot.model,robot.data)
        M = robot.data.oMf[idx]
        return pin.log(M).vector
    def exampleTangent(q):
        # Tangent application to exampleFunction, i.e. Lie derivatives of exampleFunction.
        idx =  robot.model.getFrameId('wrist_right_ft_tool_link') #('FL_FOOT')
        M = robot.data.oMf[idx]
        pin.computeJointJacobians(robot.model,robot.data,q)
        T=pin.Jlog6(M) @pin.getFrameJacobian(robot.model,robot.data,idx,pin.LOCAL)
        return T

    #########################################
    def numdiff(func,x,eps=1e-6):
        f0 = copy.copy(func(x))
        xe = x.copy()
        fs = []
        for k in range(len(x)):
            xe[k] += eps
            fs.append( (func(xe)-f0)/eps )
            xe[k] -= eps
        if isinstance(fs[0],np.ndarray) and len(f0)>1: return np.stack(fs,axis=1)
        else: return np.array(fs)

    def Tdiff(func,exp,nv,q,eps=1e-6):
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

    # Tangent map of exampleFunction
    exampleFunction(q)
    T = exampleTangent(q)
    # Finite-diff approx of the tangent map of the example function
    Tn = Tdiff(exampleFunction,lambda q,v: pin.integrate(robot.model,q,v),robot.model.nv,q)

    # Finite-diff of the vector "coefficient-wise" jacobian of exampleFunction.
    Jn = numdiff(exampleFunction,q)
    assert(norm(Jn[:,7:]-Tn[:,6:])<1e-4)

    # Jacobian and finite-diff approx of the jacobian of the integrator.
    Jexpn = numdiff(lambda v:pin.integrate(robot.model,q,v),np.zeros(robot.model.nv))
    Jexp = dExpQ(robot.model,q)
    assert(norm(Jexp-Jexpn)<1e-4)

    assert(norm(Jn@Jexp-T)<1e-4)
    assert(norm(Jn-T@dExpQ_inv(robot.model,q))<1e-4)

