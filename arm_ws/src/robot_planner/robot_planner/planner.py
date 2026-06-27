from unified_planning.shortcuts import * #type: ignore
from unified_planning.model import Problem, Fluent, Object, InstantaneousAction #type: ignore

# Planner uses Classical Unified planning

def build_up_problem(block_positions: dict, target_shape: dict, goal_mapping: dict = None):
    """
    Builds a Unified Planning Problem entirely in Python.
    """
    Block = UserType("Block")
    Location = UserType("Location")
    Arm  = UserType("Arm")

    # FLUENTS 
    block_at = Fluent("block_at", BoolType(), block=Block, loc=Location)
    arm_free = Fluent("arm_free", BoolType(), arm=Arm)
    arm_holding = Fluent("arm_holding", BoolType(), arm=Arm, block=Block)
    target_filled = Fluent("target_filled", BoolType(), loc=Location)

    # PICK action 
    pick = InstantaneousAction("pick", arm=Arm, block=Block, location=Location)
    a, b, l = pick.arm, pick.block, pick.location
    pick.add_precondition(arm_free(a))           
    pick.add_precondition(block_at(b, l))        
    pick.add_effect(arm_holding(a, b), True)     
    pick.add_effect(arm_free(a), False)    
    pick.add_effect(block_at(b, l), False)    

    # PLACE action 
    place = InstantaneousAction("place", arm=Arm, block=Block, target=Location)
    a2, b2, t = place.arm, place.block, place.target
    place.add_precondition(arm_holding(a2, b2))      
    place.add_precondition(Not(target_filled(t)))    
    place.add_effect(block_at(b2, t), True)      
    place.add_effect(arm_free(a2), True)      
    place.add_effect(arm_holding(a2, b2), False)     
    place.add_effect(target_filled(t), True)      

    # Problem 
    problem = Problem("pick_and_place")
    problem.add_fluent(block_at, default_initial_value=False)
    problem.add_fluent(arm_free, default_initial_value=False)
    problem.add_fluent(arm_holding, default_initial_value=False)
    problem.add_fluent(target_filled, default_initial_value=False)
    problem.add_action(pick)
    problem.add_action(place)

    blocks  = {name: Object(name, Block) for name in block_positions}
    sources = {f"src_{name}": Object(f"src_{name}", Location) for name in block_positions}
    targets = {name: Object(name, Location) for name in target_shape}
    arms    = {name: Object(name, Arm) for name in ["arm1", "arm2"]}

    all_objects = (
            list(blocks.values()) + 
            list(sources.values()) +
            list(targets.values()) + 
            list(arms.values())
        )
    for obj in all_objects:
        problem.add_object(obj)

    # Initial state 
    for arm_obj in arms.values():
        problem.set_initial_value(arm_free(arm_obj), True)

    for block_name, src_name in zip(blocks, sources):
        problem.set_initial_value(
            block_at(blocks[block_name], sources[src_name]), True
        )

    # Goal - EXPLICIT MAPPING INTEGRATED HERE
    if goal_mapping:
        for block_name, target_name in goal_mapping.items():
            if block_name in blocks and target_name in targets:
                problem.add_goal(block_at(blocks[block_name], targets[target_name]))
    else:
        for block_name, target_name in zip(blocks, targets):
            problem.add_goal(block_at(blocks[block_name], targets[target_name]))

    return problem, blocks, sources, targets, arms

def parse_up_plan(result_plan) -> list:
    action_strings = []
    actions_list = getattr(result_plan, 'actions', list(result_plan))

    for ai in actions_list:
        action_name = ai.action.name
        params = [str(p) for p in ai.actual_parameters]
        action_strings.append(f"{action_name} {' '.join(params)}")

    return action_strings

def get_plan(block_positions: dict, target_shape: dict, goal_mapping: dict = None) -> list:
    print("  Building Unified Planning problem...")
    problem, blocks, sources, targets, arms = build_up_problem(
        block_positions, target_shape, goal_mapping
    )

    print("  Calling UP solver (auto-selects best available)...")
    try:
        with OneshotPlanner(problem_kind=problem.kind) as planner:
            result = planner.solve(problem)

        status = result.status.name
        if status in ("SOLVED_SATISFICING", "SOLVED_OPTIMALLY"):
            print(f"  Solver status: {status}")
            return parse_up_plan(result.plan)
        else:
            print(f"  WARNING: Solver returned: {status}")
            return []
    except Exception as e:
        print(f"[Solver]Error {e}")
        return []

def fallback_plan(block_positions: dict, target_shape: dict, goal_mapping: dict = None) -> list:
    plan = []
    
    if goal_mapping:
        for i, (block, target) in enumerate(goal_mapping.items()):
            if block in block_positions and target in target_shape:
                arm = "arm1" if i % 2 == 0 else "arm2"
                plan.append(f"pick {arm} {block} src_{block}")
                plan.append(f"place {arm} {block} {target}")
    else:
        blocks = list(block_positions.keys())
        targets = list(target_shape.keys())
        for i, (block, target) in enumerate(zip(blocks, targets)):
            arm = "arm1" if i % 2 == 0 else "arm2"
            plan.append(f"pick {arm} {block} src_{block}")
            plan.append(f"place {arm} {block} {target}")

    return plan