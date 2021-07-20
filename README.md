# behaviour_test

The program testing the usage of py_trees with SrdPy

## To run and use:
 - clone the project
 - install all packages needed for SrdPy according to the instruction in the github - https://github.com/RumblingTurtle/SrdPy
 - install py_trees - https://github.com/splintered-reality/py_trees
 - for graphical representations of behaviour trees graphviz is needed

#### BT_model
main.py is the program with a class BT_model which allows to build and execute the behaviour tree, which would describe the behaviour of iiwa manipulator.

BT_model is working with IIWASimulator, a wrap-up class of tableBasedSimulation for SrdPy's iiwa stored in SimulationTest.py.
For now it has only 3 functions:
- setup(self): initialize and setup the model and it's description
- addTrajectory(self, coords: [Dot]): add the trajectory to the trajectory list in the form of coordinates
- startSimulation(self): calculates the tasks for the specified trajectory and starts the simulation in MeshCat

BT_model builds a behaviour tree for IIWASimulator, which has a sequence at its root, which always begins with setup action leaf - it triggers the setup for IIWASimulator - and 
always ends with startExecution action leaf - it triggers IIWASimulator's startSimulation
The middle contains the written behaviour. Since IIWASimulator only has the option of adding the trajectory, the only action leaves are add-the-trajectory leaves. They can be added to the tree by calling moveByTrajectory(self, segs: [Dot]) function. Other options might be added soon

After the tree is constructed, in can be executed by calling execute(self, numberOfTimes = py_trees.trees.CONTINUOUS_TICK_TOCK). The function shows the visualization of the tree and ticks it as much times as specified in numberOfTimes. By default it will tick continiously(re-launching endless models of same manipulators :D)
