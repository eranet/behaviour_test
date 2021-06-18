import py_trees
from SImulationTest import IIWASimulator, Dot


# Setting up the simulator
class setup(py_trees.behaviour.Behaviour):
    def __init__(self, name, simulator: IIWASimulator, rootOftree):
        super(setup, self).__init__(name)
        self.simulator = simulator
        self.tree = py_trees.trees.BehaviourTree(root = rootOftree)

    def update(self):
        print_tree(self.tree)
        self.simulator.setup()
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
        return py_trees.common.Status.SUCCESS


# Launching simulation
class startExecution(py_trees.behaviour.Behaviour):
    def __init__(self, name, simulator: IIWASimulator, rootOftree):
        super(startExecution, self).__init__(name)
        self.simulator = simulator
        self.tree = py_trees.trees.BehaviourTree(root = rootOftree)

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

    def moveByTrajectory(self, segs: [Dot]):
        self.root.add_child(moveByTrajectory("Add the trajectory", segs, self.simulator, self.root))

    def execute(self):
        self.root.add_child(startExecution("Start the simulation", self.simulator, self.root))
        behaviour_tree = py_trees.trees.BehaviourTree(
            root=self.root
        )
        notfinished = True
        while notfinished:
            self.root.tick_once()
            notfinished = self.root.status == py_trees.common.Status.RUNNING
            print_tree(behaviour_tree)


def print_tree(tree):
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))


# Initializing the tree and executing it
if __name__ == '__main__':
    tree = BT_model(IIWASimulator())
    tree.moveByTrajectory([Dot(0.1, 0.3), Dot(0.3, 0.3), Dot(0.3, 0.1), Dot(0.1, 0.1), Dot(0.1, 0.3)])
    tree.execute()
