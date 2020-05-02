'''
Before you begin, make sure path_planning.py and PathPlanPrinter.py are accessible to your
script, by being in the same folder, or under your PATH environment variable.
    1. Import path_planning. You can use the reserved word as in order to create an alias.
    2. Copy and paste the auxiliary functions.
'''

import path_planning as pp
import re

def calculate_path(src, dst):
    step = 40
    shape = pp.npdata.shape
    step_x = shape[0]/step
    step_y = shape[1]/step
    # Add algo='algo name' and heur='heur name' to the parameters for run_path_planning
    # in order to modify the behavior of the path planning part of the execution.
    return pp.run_path_planning(step,
                                    start=(src[0], src[1]),
                                    finish=(dst[0], dst[1]), algo='A*', heur='naive',
                                    show_grid=True)

# Function in charge of parsing. Copy, paste, pass the path to the source text file
# and store the return value into a variable. You can then parse the result of move,
# stored in the list along with its destination, and run path planning. Bear in mind:
# you should keep a reference to the agent's location at any given time.
def generate_task_list(filepath):
    tasks = []
    for line in open(filepath):
        if re.match(r'^\(\w(\w|_)*(\ *(\w(\w|_)*)?)*\)', re.sub(r"(\d(\.|\d)*: ) *", "", line)):
            split_line = re.sub(r"(\d(\.|\d)*: ) *", "", line).split()
            tasks.append(split_line[0].replace('(', '').replace(')', ''))
            if tasks[-1].lower() == 'move':
                coords = split_line[3].replace('p', '').replace(')', '')
                tasks[-1] = (tasks[-1], (int(coords[:2]), int(coords[2:])))
    print("TASKS:")
    print(tasks)
    return tasks
    
'''
    3. Call your path_planning's output_image method.
'''

# In this example, we create a small function that will go over a whole
# task plan and print any movement that needs to take place in it. Keep
# in mind this function is a stub, and should probably expanded. The
# general idea is there, but you might need to adapt it to your own
# specific need.

def visualize_paths_from_pddl(task_plan, path_to_map):
    position = (10, 10) # change this to the starting position for your PDDL problem
    pp.load_image(path_to_map)
    log = open("../out/planning_execution.log", "w+")
    task_number = 0
    for task in task_plan: # Iterate the whole task list
        task_number += 1
        log.write(str(task)+'\n')
        if type(task) is tuple: # Movement must be a tuple, so we are only interested in this case
            if task[0] == 'move': # This is the action we were looking for
                new_pos = task[1] # Get the destination for this action
                print(position, '->', new_pos)
                path = calculate_path(position, new_pos) # Calculate the path using a path planning algorithm
                # Output the path to a file
                pp.output_image(path_to_map, path,
                                # This last argument is the path where we want to save the file.
                                # Note that there is a good deal of string processing in order to trim the file
                                # name from the path to the map and use it as a basis to contruct the output file,
                                # along with an indication of what task is being executed.
                                '../out/'+
                                path_to_map.split('/')[-1].split('\\')[-1].split(".")[-2].split('/')[-1]+
                                '_task_'+str(task_number)+'.png') 
                position = new_pos # Update where the agent would be
    log.close()
                
# Now we are outside the function. This is going to be ran regardless of everything else.
task_plan = generate_task_list('path_to_task_plan_file') # Must be a path to a text file
visualize_paths_from_pddl(task_plan, 'path_to_map_file') # path_to_map_file should be a path to an image