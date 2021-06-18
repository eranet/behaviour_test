from SrdPy.URDFUtils import getLinkArrayFromURDF

from SrdPy.TableGenerators import generateConstraiedLinearModelTable
from SrdPy.TableGenerators import generateLinearModelTable
from SrdPy.LinksAndJoints import *
from SrdPy.Handlers import *
from SrdPy.InverseKinematics import *
from SrdPy.SymbolicUtils import *
from SrdPy.Loggers import *
from SrdPy.DynamicSolvers import *
from SrdPy.Controllers import *

from SrdPy.Visuals import Visualizer
from SrdPy import SymbolicEngine
from SrdPy import plotGeneric
from copy import deepcopy
from casadi import *
from SrdPy import save, get
from SrdPy.SrdMath import *
from SrdPy.TableGenerators import *
from SrdPy import Chain
from SrdPy import Profiler
import numpy as np
from scipy.integrate import solve_ivp
import os
from control import lqr

class Dot:

    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_


class IIWASimulator:

    def __init__(self):
        pass

    def my_generateLQRTable(self, A_table, B_table, Q_table, R_table):
        count = A_table.shape[0]
        n = A_table.shape[2]
        m = B_table.shape[2]

        K_table = np.zeros((count, m, n))

        for i in range(count):
            K, S, CLP = lqr(A_table[i], B_table[i], Q_table[i], R_table[i])
            K_table[i] = K

        return K_table

    def setup(self):
        iiwaLinks = getLinkArrayFromURDF(os.path.abspath(os.path.dirname(sys.argv[0]) + "/SrdPy/examples/iiwa/iiwa14.urdf"), True)

        iiwaChain = Chain(iiwaLinks)

        import pickle

        print(iiwaChain)
        self.initialPosition = np.zeros(7)
        blank_chain = deepcopy(iiwaChain)
        blank_chain.update(self.initialPosition)

        engine = SymbolicEngine(iiwaChain.linkArray)

        deriveJacobiansForlinkArray(engine)
        H = deriveJSIM(engine)

        C = deriveCmatrixViaChristoffel(engine, H)

        iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
        g = deriveGeneralizedGravitationalForces(engine)
        d = deriveGeneralizedDissipativeForcesUniform(engine, 1)
        T = deriveControlMap(engine)

        description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                                  H=H,
                                                                                  c=(iN + g + d),
                                                                                  T=T,
                                                                                  functionName_H="g_dynamics_H",
                                                                                  functionName_c="g_dynamics_c",
                                                                                  functionName_T="g_dynamics_T",
                                                                                  casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                                  path="./iiwa/Dynamics")

        self.handlerGeneralizedCoordinatesModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

        save(self.handlerGeneralizedCoordinatesModel, "gcModel")

        description_linearization = generateDynamicsLinearization(engine,
                                                                  H=H,
                                                                  c=(iN + g + d),
                                                                  T=T,
                                                                  functionName_A="g_linearization_A",
                                                                  functionName_B="g_linearization_B",
                                                                  functionName_c="g_linearization_c",
                                                                  casadi_cCodeFilename="g_dynamics_linearization",
                                                                  path="./iiwa/Linearization")

        self.handlerLinearizedModel = LinearizedModelHandler(description_linearization)

        save(self.handlerLinearizedModel, "linearizedModel")

        constraint6 = engine.links["iiwa_link_6"].absoluteFollower[0]

        task = constraint6[:2]
        print("task size is: ", task.size)

        description_IK = generateSecondDerivativeJacobians(engine,
                                                           task=task,
                                                           functionName_Task="g_InverseKinematics_Task",
                                                           functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                           functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                           casadi_cCodeFilename="g_InverseKinematics",
                                                           path="./iiwa/InverseKinematics")

        self.ikModelHandler = IKModelHandler(description_IK, engine.dof, task.shape[0])

        save(self.ikModelHandler, "ikModelHandler")

        IC_task = self.ikModelHandler.getTask(self.initialPosition)

        self.IC_task = np.reshape(IC_task, [1, 2])

        self.tasks = [self.IC_task.T]

    def addTrajectory(self, coords: [Dot]):
        for dot in coords:
            self.tasks.append(np.array([[dot.x], [dot.y]]))
        return True

    def startSimulation(self):
        zeroOrderDerivativeNodes = np.hstack(self.tasks)

        firstOrderDerivativeNodes = np.zeros(zeroOrderDerivativeNodes.shape)

        secondOrderDerivativeNodes = np.zeros(zeroOrderDerivativeNodes.shape)

        timeOfOneStage = 2
        timeEnd = (len(zeroOrderDerivativeNodes[1]) - 1) * timeOfOneStage + 1
        nodeTimes = np.arange(start=0, stop=timeEnd, step=timeOfOneStage)

        handlerIK_taskSplines = IKtaskSplinesHandler(nodeTimes,
                                                     zeroOrderDerivativeNodes,
                                                     firstOrderDerivativeNodes,
                                                     secondOrderDerivativeNodes)

        save(handlerIK_taskSplines, "handlerIK_taskSplines")

        timeTable = np.arange(handlerIK_taskSplines.timeStart, handlerIK_taskSplines.timeExpiration + 0.01, 0.01)

        IKTable = generateIKTable(self.ikModelHandler, handlerIK_taskSplines, self.initialPosition, timeTable, method="lsqnonlin")
        plotIKTable(self.ikModelHandler, timeTable, IKTable)

        ikSolutionHandler = IKSolutionHandler(self.ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

        save(ikSolutionHandler, "ikSolutionHandler")

        tf = ikSolutionHandler.timeExpiration

        n = self.handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

        A_table, B_table, c_table, x_table, u_table, dx_table = generateLinearModelTable(self.handlerGeneralizedCoordinatesModel,
                                                                                         self.handlerLinearizedModel,
                                                                                         ikSolutionHandler, timeTable)

        print("Started experiment")

        Q = 10 * np.eye(2 * n)
        R = 0.1 * np.eye(self.handlerGeneralizedCoordinatesModel.dofControl)
        count = A_table.shape[0]

        K_table = self.my_generateLQRTable(A_table, B_table, np.tile(Q, [count,1, 1]), np.tile(R, [ count, 1, 1]))
        AA_table, cc_table = generateCloseLoopTable(A_table, B_table, c_table, K_table, x_table, u_table)
        ode_fnc_handle = ClosedLoopLinearSystemOdeFunctionHandler(AA_table, cc_table, timeTable)

        x0 = np.hstack((self.initialPosition, np.zeros(self.initialPosition.shape[0])))

        sol = solve_ivp(ode_fnc_handle, [0, tf], x0, t_eval=timeTable, method="RK45")

        time_table_0 = sol.t
        solution_tape = sol.y.T

        ax = plotGeneric(time_table_0, solution_tape, figureTitle="", ylabel="ODE")
        ax = plotGeneric(timeTable, x_table, ylabel="linearmodel", old_ax=ax, plot=True)

        ax = plotGeneric(timeTable, solution_tape[:, :n], figureTitle="position", ylabel="q", plot=True)
        ax = plotGeneric(timeTable, solution_tape[:, n:2 * n], figureTitle="velocity", ylabel="v", plot=True)

        with open('anim_array.npy', 'wb') as f:
            np.save(f, solution_tape[:, :n])

        chainLinks = getLinkArrayFromURDF(os.path.abspath(os.path.dirname(sys.argv[0]) + "/SrdPy/examples/iiwa/iiwa14.urdf"), True)

        chain = Chain(chainLinks)

        print(chain)
        blank_chain = deepcopy(chain)
        blank_chain.update(self.initialPosition)
        with open('anim_array.npy', 'rb') as f:
            q = np.load(f)

        blank_chain.update(q[0])
        plotGeneric(np.arange(q.shape[0]), q, plot=True)
        vis = Visualizer()
        vis.animate(blank_chain, q, framerate=0.1, showMeshes=True)

        input()
        return True