import random
import time
# --- BASIS DEFINITIONS -------------------------------
# --- BASIS DEFINITIONS -------------------------------
# --- BASIS DEFINITIONS -------------------------------
from enum import Enum

import example_robot_data as robex
import hppfcl
import numpy as np
import pinocchio as pin
from numpy.linalg import det, eig, inv, norm, pinv, svd
from scipy.optimize import fmin_bfgs

pin.seed(int(time.time()))  # change me :)

np.set_printoptions(precision=4, linewidth=200, suppress=True)

# --- Load robot model
robot = robex.loadTalos()
NQ = robot.model.nq
NV = robot.model.nv

# --- ENVIRONMENT DEFINITION ----------------------------------------
# --- ENVIRONMENT DEFINITION ----------------------------------------
# --- ENVIRONMENT DEFINITION ----------------------------------------


def debris(i, j, altitude, axis, angle):
    """
    Minimal helper function: return the SE3 configuration of a stepstone, with some
    ad-hoc configuration.
    """
    # i,j are integers
    # altitude is the altitude in centimer
    # axis is 2D, rotation axis in the plane
    # angle is the angle of inclination, in radian
    STEP = .5
    axis = np.array(axis, np.float64)
    axis /= norm(axis)
    return pin.SE3(pin.AngleAxis(angle, np.concatenate([axis, [0]])).matrix(),
                   np.array([i * STEP, j * STEP, altitude]))


# Terrain is a list of stepstone. They are defined by the SE3 placement.
terrain = [

    debris(-1, -1, .4, [1, 1], -.1),
    debris(0, -1, .2, [1, 2], .1),
    debris(1, -1, .3, [1, 0], -.2),

    debris(-1, 0, .1, [0, 1], .1),
    debris(0, 0, .1, [1, 1], -.1),
    debris(1, 0, .3, [1, 2], .4),

    debris(-1, 1, .2, [1, 0], -.2),
    debris(0, 1, .3, [0, 1], .3),
    debris(1, 1, .4, [1, 1], -.2),

    ]

# Obstacle is a list of 3D sphere obstacle. They are defined by their sphere centers.
obstacles = [
    np.array([.5, .3, 1.1]),
    np.array([-.3, -.6, 1.4]),
    ]


def addTerrainsToGeomModel(gmodel, terrain, obstacles):
    """
    Add a list of stepstones and obstacles to the robot geometry object.
    Each step stone is defined by its SE3 placement. It is added as a red disk of 20cm radius.
    Each obstacles is defined by its 3d position. It is added as a white sphere of radius 20cm.
    - gmodel is a pinocchio geometry model
    - terrain is a list of SE3 placement of the step stones
    - obstacles is a list of 3d position of the sphere centers.
    """
    # Create pinocchio 3d objects for each step stone
    for i, d in enumerate(terrain):
        # The step stones have name "debris0X" and are attached to the world (jointId=0).
        g2 = pin.GeometryObject("debris%02d" % i, 0, hppfcl.Cylinder(.2, .01), d)
        g2.meshColor = np.array([1, 0, 0, 1.])
        gmodel.addGeometryObject(g2)

    # Create Pinocchio 3d objects for the obstacles.
    for i, obs in enumerate(obstacles):
        # The obstacles have name "obs0X" and are attached to the world (jointId=0).
        g2 = pin.GeometryObject("obs%02d" % i, 0, hppfcl.Sphere(.2), pin.SE3(np.eye(3), obs))
        g2.meshColor = np.array([1, 1, 1, 1.])
        gmodel.addGeometryObject(g2)

    # Add the collision pair to check the robot collision against the step stones and the obstacles.
    # For simplicity, does not activate the self-collision pairs.
    ngeomRobot = len(gmodel.geometryObjects) - len(terrain) - len(obstacles)
    for irobot in range(ngeomRobot):
        for ienv in range(len(terrain) + len(obstacles)):
            gmodel.addCollisionPair(pin.CollisionPair(irobot, ngeomRobot + ienv))


addTerrainsToGeomModel(robot.collision_model, terrain, obstacles)
addTerrainsToGeomModel(robot.visual_model, terrain, obstacles)


# --- VIEWER -------------------------------------------
# --- VIEWER -------------------------------------------
# --- VIEWER -------------------------------------------

# The viewer should be initialized after adding the terrain to the robot
# otherwise, the terrain will not be displayed.

# Viewer = pin.visualize.GepettoVisualizer
Viewer = pin.visualize.MeshcatVisualizer
viz = Viewer(robot.model, robot.collision_model, robot.visual_model)
viz.initViewer(loadModel=True)

viz.display(robot.q0)

rmodel = robot.model
rdata = rmodel.createData()


class ContactType(Enum):
    C3D = 3
    C6D = 6


class Constraint:
    def __init__(self, rmodel, frameIndex, placement, contactType):
        self.frameIndex = frameIndex  # Index of the robot frame that we want to bring to contact
        self.placement = placement  # SE3 placement on the environment
        self.contactType = contactType  # Either 3d or 6d
        assert 0 < frameIndex < len(rmodel.frames)
        assert isinstance(placement, pin.SE3)
        assert isinstance(contactType, ContactType)


# --- CONTACT PLANNER ---------------------------------
# --- CONTACT PLANNER ---------------------------------
# --- CONTACT PLANNER ---------------------------------


class ContactPlanner:
    def __init__(self, rmodel, collision_model, terrain):
        """
        Initialize the contact planner.
        - rmodel is the pinocchio robot model
        - collision_model is the pinocchio collision model
        - terrain is a list of SE3 placements where the robot can make contact
        The planner is initialize with an empty list of candidate contact, that should be
        set later with self.addContact.
        """
        self.rmodel = rmodel
        self.collision_model = collision_model
        self.terrain = terrain
        self.contactCandidates = {}
        self.rdata = self.rmodel.createData()
        self.collision_data = self.collision_model.createData()

    def addContact(self, name, frameIndex, contactType):
        """
        Add a possible contact (for example the right foot).
        - name: arbitrary name you want to use to represent this contact (e.g. "rightfoot")
        - frameIndex: index of the frame corresponding to this contact on the robot kinematic
        tree (eg. rmodel.getFrameId("right_sole_link")
        - contactType, either 3D or 6D, for example ContactType.C6D.
        """
        # We use the type Constraint to also store contact candidates.
        self.contactCandidates[name] = Constraint(self.rmodel, frameIndex, pin.SE3.Identity(), contactType)

    def setPostureGenerator(self, postureGenerator):
        self.postureGenerator = postureGenerator

    def searchContactPosture(self, numberOfContacts, qguess=None, ntrial=100):
        """
        Search (randomly) a contact configuration with a given number of contacts.
        - number of contacts: between 1 and len(self.contactCandidates).
        - qguess is a configuration, of dim rmodel.nq. If None, a random configuration is first
        sampled.
        - ntrial: number of random trial.

        If successfull, set self.success to True, and store the contact posture found in self.qcontact.
        If not sucessfull, set self.success to False.
        Returns self.success.
        """
        assert 0 < numberOfContacts < len(self.contactCandidates)
        if qguess is None:
            qguess = pin.randomConfiguration(self.rmodel)

        for itrial in range(ntrial):
            limbs = random.sample(list(self.contactCandidates.values()), numberOfContacts)
            stones = random.sample(self.terrain, numberOfContacts)

            constraints = [Constraint(self.rmodel, limb.frameIndex, stone, limb.contactType)
                           for limb, stone in zip(limbs, stones)]

            self.postureGenerator.run(qguess, constraints)
            if self.postureGenerator.sucess:
                # We found a candidate posture, let' s check that it is acceptable.
                qcandidate = self.postureGenerator.qopt.copy()

                # computeCollisions check collision among the collision pairs.
                collision = pin.computeCollisions(self.rmodel, self.rdata,
                                                  self.collision_model, self.collision_data, qcandidate, True)

                if not collision:
                    self.qcontact = qcandidate
                    self.success = True
                    return True

        self.success = False
        return False


# --- POSTURE GENERATOR PROTOTYPE --------------------
# --- POSTURE GENERATOR PROTOTYPE --------------------
# --- POSTURE GENERATOR PROTOTYPE --------------------

class PostureGenerator:
    def __init__(self, rmodel, rdata, viz):  # Add any arguments you want here
        self.rmodel = rmodel
        self.rdata = rdata
        self.viz = viz

    def run(self, qguess, constraints):
        '''
        From an initial guess qguess, the function should compute a new configuration
        with the constraints satisfy if possible. If a contact configuration is found,
        the function stores it in self.qopt, set self.success to True and returns True.
        Otherwise, it set self.sucess to False and return False.

        Arguments:
        - qguess is a configuration that can be used as an initial guess in the optimization.
        - contraints is a list of contraints of class Constraint.

        Process:
        1. The function should first optimize a cost function corresponding to penalizing the distance
        of the limbs to the contact placement in the environment.
        2. After the optimization, the function should evaluate the resulting optimum and
        decide if it satisfy the contact constraint. In that case it is a success and
        the optimum is returned by self.qopt. Otherwise, it is a failure and the function should
        only set self.sucess to false and return false.

        '''
        # PUT YOUR CODE HERE
        print('Some code is missing here')
        self.sucess = False
        return False


# --- RUNNING YOUR PLANNER ----------------------------

planner = ContactPlanner(robot.model, robot.collision_model, terrain)
planner.addContact("rightfoot", rmodel.getFrameId("right_sole_link"), ContactType.C6D)
planner.addContact("leftfoot", rmodel.getFrameId("left_sole_link"), ContactType.C6D)

planner.setPostureGenerator(PostureGenerator(rmodel, rdata, viz))

planner.searchContactPosture(1, ntrial=5)
