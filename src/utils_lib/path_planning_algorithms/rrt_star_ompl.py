from ompl import base as ob
from ompl import geometric as og

def RRTStarOMPL(start_p, goal_p, state_validity_checker, dominion, max_time=2.0):
    space = ob.RealVectorStateSpace(2)
    bound_low=    dominion[0]
    bound_high= dominion[1]
    space.setBounds(bound_low, bound_high)
    space.setLongestValidSegmentFraction(0.001)
    si = ob.SpaceInformation(space) 
    si.setStateValidityChecker(ob.StateValidityCheckerFn(state_validity_checker))
        # create a start state
    start = ob.State(space)
    start[0] = start_p[0]
    start[1] = start_p[1]
    
    # create a goal state
    goal = ob.State(space)
    goal[0] = goal_p[0]
    goal[1] = goal_p[1]

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
    optimizingPlanner = og.RRTstar(si)
    optimizingPlanner.setRange(10)  
    optimizingPlanner.setGoalBias(0.2)

    # Set the problem instance for our planner to solve and call setup
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(max_time)
    
    # Get planner data
    pd = ob.PlannerData(si)
    optimizingPlanner.getPlannerData(pd)
    
    if solved:
        # get the path and transform it to a list
        path = pdef.getSolutionPath()
        print("Found solution:\n%s" % path)
        ret = []
        for i in path.getStates():
            ret.append((i[0], i[1]))
    else:
        ret = []
        print("No solution found")
    # print ("path", path)
    return ret