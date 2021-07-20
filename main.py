import py_trees
from SImulationTest import IIWASimulator, Dot
import matplotlib.pyplot as plt
from PIL import Image


# Setting up the simulator
class setup(py_trees.behaviour.Behaviour):
    def __init__(self, name, simulator: IIWASimulator, rootOftree):
        super(setup, self).__init__(name)
        self.simulator = simulator
        self.tree = py_trees.trees.BehaviourTree(root = rootOftree)

    def update(self):
        print_tree(self.tree)
        self.simulator.setup()
        self.simulator.startSimulation()
        return py_trees.common.Status.SUCCESS


# Adding trajectory to the simulator
class moveByTrajectory(py_trees.behaviour.Behaviour):
    def __init__(self, name, trajectory: [Dot], simulator: IIWASimulator, rootOftree):
        super(moveByTrajectory, self).__init__(name)
        self.trajectory = trajectory
        self.simulator = simulator
        self.tree = py_trees.trees.BehaviourTree(root=rootOftree)

    def update(self):
        print_tree(self.tree)
        self.simulator.addTrajectory(self.trajectory)
        self.simulator.calculateSolution()
        self.simulator.executeTrajectory()
        return py_trees.common.Status.SUCCESS


# Launching simulation
class startExecution(py_trees.behaviour.Behaviour):
    def __init__(self, name, simulator: IIWASimulator, rootOftree):
        super(startExecution, self).__init__(name)
        self.simulator = simulator
        self.tree = py_trees.trees.BehaviourTree(root = rootOftree)
        self.thread = None

    def update(self):
        print_tree(self.tree)
        self.simulator.startSimulation()
        return py_trees.common.Status.SUCCESS






# Model of the behaviour tree
class BT_model:
    def __init__(self, simulator: IIWASimulator):
        self.root = py_trees.composites.Sequence("root")
        self.simulator = simulator
        self.root.add_child(setup("Setup the simulator", self.simulator, self.root))

    ##add the trajectory
    def moveByTrajectory(self, segs: [Dot]):
        self.root.add_child(moveByTrajectory("Add the trajectory: " + self.trajectoryToStr(segs), segs, self.simulator, self.root))

    def trajectoryToStr(self, segs:[Dot]):
        s = ""
        for dot in segs:
            s += dot.toString() + " "
        return s

    def execute(self, numberOfTimes = py_trees.trees.CONTINUOUS_TICK_TOCK):
        py_trees.display.render_dot_tree(self.root)
        im = Image.open("root.png")
        fig = plt.imshow(im)
        plt.show()
        behaviour_tree = py_trees.trees.BehaviourTree(
            root=self.root
        )
        behaviour_tree.tick_tock(pre_tick_handler=print_tree(behaviour_tree), period_ms=500,
            number_of_iterations=numberOfTimes)


def print_tree(tree):
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))


def tryWithoutTree(sim: IIWASimulator):
    sim.setup()
    sim.startSimulation()
    sim.addTrajectory((Dot(0.1, 0.3), Dot(0.3, 0.3), Dot(0.3, 0.1), Dot(0.1, 0.1), Dot(0.1, 0.3)))
    sim.calculateSolution()
    for i in range(10):
        sim.executeTrajectory()


# Initializing the tree and executing it
if __name__ == '__main__':
    tree = BT_model(IIWASimulator())
    tree.moveByTrajectory([Dot(0.1, 0.3), Dot(0.3, 0.3), Dot(0.3, 0.1), Dot(0.1, 0.1), Dot(0.1, 0.3)])
    tree.moveByTrajectory([Dot(0.5, 0.5), Dot(0.1, 0.3)])
    tree.execute(1)
