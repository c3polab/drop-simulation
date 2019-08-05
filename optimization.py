from controller import Robot, Supervisor
import numpy as np
from scipy.optimize import minimize, basinhopping, brute, differential_evolution, shgo, dual_annealing
import fileinput

TIME_STEP = 3

supervisor = Supervisor()


#initial guess for the knee and heel levers
#the y component of KNEE_INITIAL is the lever length
#the z component of HEEL_INITIAL is the lever length
KNEE_INITIAL = np.array([0.0, 0.325, -0.01])
HEEL_INITIAL = np.array([0.0, 0.0, 0.15])

#these are ideal values for a 80kg robot
#KNEE_TENDON_INITIAL = 143526
#HEEL_TENDON_INITIAL = 160106

#this is the initial guess for the tendon values
KNEE_TENDON_INITIAL = 25000
HEEL_TENDON_INITIAL = 25000

#get a handle to the root node and create the robot
root = supervisor.getRoot()
children = root.getField("children")
children.importMFNode(-1, "Robot.wbo")

#set the leg spring constants to the initial values
change_spring_constant(int(x[0]), int(x[1]))

#function that changes the spring constant of the robot
#edits the robot.wbo file
#changing the spring constant through the robot node causes unexpected behaviour
def change_spring_constant(knee, heel):
    flag = False
    a = 0
    for line in fileinput.FileInput("Robot.wbo", inplace = 1):
        if line.strip() == "SliderJoint {":
            flag = True
        if flag and line.strip().split(" ", 1)[0] == "springConstant":
            if a == 0:
                print(line.replace(line.split(" ")[-1], str(knee) + "\n")[:-1])
                a += 1
            else:
                print(line.replace(line.split(" ")[-1], str(heel) + "\n")[:-1])
            flag = False
        else:
            print(line[:-1])

#function that drops the robot and returns the jump height
#takes in x, which is an np.array containint
def jump_height(x):
    
    #get and set the variables that are being optimized for

    knee_joint = supervisor.getFromDef("knee_joint_parameters")
    knee_solid = supervisor.getFromDef("knee_solid")
    heel_joint = supervisor.getFromDef("heel_joint_parameters")
    heel_solid = supervisor.getFromDef("heel_solid")
    
    knee_tendon = supervisor.getFromDef("knee_tendon")
    heel_tendon = supervisor.getFromDef("heel_tendon")
    
    knee_spring_constant = knee_tendon.getField("springConstant")
    heel_spring_constant = heel_tendon.getField("springConstant")

    knee_anchor = knee_joint.getField("anchor")
    knee_solid_translation = knee_solid.getField("translation")
    heel_anchor = heel_joint.getField("anchor")
    heel_solid_translation = heel_solid.getField("translation")
    
    #uncomment the following 4 lines for the simulation to optimize for lever lengths as well
    
    #knee_anchor.setSFVec3f([KNEE_INITIAL[0], x[0], KNEE_INITIAL[2]])
    #knee_solid_translation.setSFVec3f([KNEE_INITIAL[0], x[0], KNEE_INITIAL[2]])
    #heel_anchor.setSFVec3f([HEEL_INITIAL[0], HEEL_INITIAL[1], x[1]])
    #heel_solid_translation.setSFVec3f([HEEL_INITIAL[0], HEEL_INITIAL[1], x[1]])
    
    #change the spring constant to the new value
    change_spring_constant(int(x[0]), int(x[1]))
    
    
    #record the best height
    best_height = 0
    highest_velocity = 0
    
    #run the simulation for 1.5 seconds and get the max jump height
    for i in range(0, 1500, TIME_STEP):
        if supervisor.step(TIME_STEP) != -1:
            robot_node = supervisor.getFromDef("robot")
            velocity = robot_node.getVelocity()
            if velocity[1] > 0:
                if robot_node.getPosition()[1] > best_height:
                    best_height = robot_node.getPosition()[1]
            elif abs(velocity[1]) > 10:
                best_height = 0
                break
                    
    #reset the robot position
    robot_node.remove()
    children.importMFNode(-1, "Robot.wbo")
    
    #return the negative max jump height
    #this is because the scipy minimizes a function
    return -best_height


#uncomment this line and the four lines in the jump_height funtion to optimize for lever length as well
#x0 = np.array([KNEE_INITIAL[1], HEEL_INITIAL[2], KNEE_TENDON_INITIAL, HEEL_TENDON_INITIAL])


x0 = np.array([KNEE_TENDON_INITIAL, HEEL_TENDON_INITIAL])

#use a global optimization algorithm to find the optimal value
#the bounds for the variables being optimized for need to be specified
#the iters parameter will give more precise results for a higher value, but will take more time
minimizer_kwargs = dict(method = "Powell")

#to simply test the leg with the initial values provided, just comment this line.
optimized = shgo(jump_height, ((0, 30000), (0, 30000)), iters = 7, minimizer_kwargs = minimizer_kwargs)
print(optimized.x)
print(optimized.fun)