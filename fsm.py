from graphviz import Digraph

# Create a directed graph for the comprehensive FSM diagram
fsm_full = Digraph(format='png', engine='dot')
fsm_full.attr(rankdir='TB', nodesep='0.8', ranksep='0.7') 

# Add Phases 0 to 4 and their sub-states
# Phase 0: Home Position
fsm_full.node('P0', 'Phase 0: Home Position', shape='ellipse')
fsm_full.node('P0_1', 'Move to Home Position', shape='box')
fsm_full.node('P0_2', 'Grab Object', shape='box')
fsm_full.edge('P0', 'P0_1', label='Start Phase 0')
fsm_full.edge('P0_1', 'P0_2', label='Reached Home Position')
fsm_full.edge('P0_2', 'P1', label='Object Grabbed')

# Phase 1: Decide Handover Moment
fsm_full.node('P1', 'Phase 1: Decide Handover Moment', shape='ellipse')
fsm_full.node('P1_1', 'Wait for Draining Start', shape='box')
fsm_full.node('P1_2', 'Wait for Request', shape='box')
fsm_full.edge('P1', 'P1_1', label='Action 1:\nImmediate Handover')
fsm_full.edge('P1', 'P1_2', label='Action 2:\nWait for Request')
fsm_full.edge('P1_1', 'P2', label='Draining Started')
fsm_full.edge('P1_2', 'P2', label='Request Received')

# Phase 2: Decide Handover Orientation
fsm_full.node('P2', 'Phase 2: Decide Handover Orientation', shape='ellipse')
fsm_full.node('P2_1', 'Move to Serve Orientation', shape='box')
fsm_full.node('P2_2', 'Move to Drop Orientation', shape='box')
fsm_full.edge('P2', 'P2_1', label='Action 3:\nServe Orientation')
fsm_full.edge('P2', 'P2_2', label='Action 4:\nDrop Orientation')
fsm_full.edge('P2_1', 'P3', label='Reached Hand Position')
fsm_full.edge('P2_2', 'P3', label='Reached Hand Position')

# Phase 3: Decide When to Open the Hand
fsm_full.node('P3', 'Phase 3: Decide When to Open Hand', shape='ellipse')
fsm_full.node('P3_1', 'Open Hand', shape='box')
fsm_full.node('P3_2', 'Partial Open', shape='box')
fsm_full.node('P3_3', 'Keep Closed', shape='box')

# Transitions from Phase 3 sub-states
fsm_full.edge('P3', 'P3_1', label='Action 5:\nOpen')
fsm_full.edge('P3', 'P3_2', label='Action 6:\nPartial Open')
fsm_full.edge('P3', 'P3_3', label='Action 7:\nClose')

# Successful transitions
fsm_full.edge('P3_1', 'P4', label='Hand open')
fsm_full.edge('P3_2', 'P4', label='Handover Successful')
fsm_full.edge('P3_3', 'P4', label='Handover Successful')

# Unsuccessful transitions
fsm_full.edge('P3_2', 'P3', label='Handover\nNot Successful')
fsm_full.edge('P3_3', 'P3', label='Handover\nNot Successful')

# Phase 4: Termination
fsm_full.node('P4', 'Phase 4: Termination', shape='ellipse')
fsm_full.edge('P4', 'P0', label='Episode <\nMax Episodes')
fsm_full.edge('P4', 'End', label='Episode = Max Episodes')

# Add start and end states
fsm_full.node('Start', 'Start', shape='plaintext')
fsm_full.node('End', 'End', shape='plaintext')
fsm_full.edge('Start', 'P0', label='Initialize FSM')

# Save and render the FSM diagram
final_diagram_path = 'fsm_diagram'
fsm_full.render(final_diagram_path)

print(f"FSM diagram saved at: {final_diagram_path}.png")