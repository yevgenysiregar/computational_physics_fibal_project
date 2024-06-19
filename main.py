import numpy as np
#numpy is used as a numeric computation and it supports many physics function
import matplotlib.pyplot as plt
#matplotlib.pyplot is used for making static graph

import matplotlib.animation as anim
#matplotlib.animation is used to make the animation for the graph
 
# Define constants and parameters. 
g = 9.8 #gravity in kg/m
length1 = float(input("Insert length 1: ")) #length1 (assume in m)
length2 = float(input("Insert length 2: ")) #length2 (assume in m)
mass1 = 8 #mass pendulum 1 (kg)
mass2 = 2 #mass pendulum 2 (kg)
 
# Finding the maximum length of the pendulum system so the figure window can display the system with adequate spacing.
tl = length1 + length2 + 0.5
 
# Define starting conditions
th1 = 1/2 * np.pi #theta1 - initial angle for pendulum 1 (in radian)
th2 = np.pi/2 #theta2 - intial angle pendulum 2 (dalam radian)
w1 = 1 #initial angular velocity pendulum 1 (radian/sec)
w2 = 1 #initial angular velocity pendulum 2 (radian/sec)
 
# Store the initial conditions in an array.
r = [th1,th2,w1,w2]
 
# Define a function to find the derivative of each component of r.

#dr = function that counts the derivatives of every component r
#v = empty array to store the derivatives values
#d = resistance value
#c = cosinus value
#s = sinus value

def dr(r):
    v = np.zeros_like(r)
    d = 1
    d = -1 * d
    v[0] = r[2]
    v[1] = r[3]
 
    # Calculate damping torque
    damping_torque_1 = -d * r[2]
    damping_torque_2 = -d * r[3]
 
    #simplification to make it easy to write the equation
    c = np.cos(r[1] - r[0])
    s = np.sin(r[1] - r[0])
 
    # Equations of motion with damping torque
    v[2] = (
            (mass2 * length1 * r[2]**2 * s * c) + 
            (mass2 * g * np.sin(r[1]) * c) + 
            (mass2 * length2 * r[3]**2 * s) - 
            ((mass1 + mass2) * g * np.sin(r[0])) - 
            damping_torque_1) / (((mass1 + mass2) * length1) - (mass2 * length1 * c**2))

    v[3] = (
            -mass2 * length2 * r[3]**2 * s * c + 
            (mass1 + mass2) * g * np.sin(r[0]) * c - 
            (mass1 + mass2) * length1 * r[2]**2 * s - 
            (mass1 + mass2) * g * np.sin(r[1]) - 
            damping_torque_2) / (((length2 / length1) * (mass1 + mass2) * length1) - (mass2 * length1 * c**2))

    
    return v
 
#rk4 is a function to implement the Runge-Kutta method (4th order) to solve differential equations
#k1 k2 k3 k4 are coefficients calculated in RK4
#R is the new value of r (after 1 integration)
def rk4(r):
    k1 = h*dr(r)
    k2 = h*dr(r + 0.5*k1)
    k3 = h*dr(r + 0.5*k2)
    k4 = h*dr(r + k3)
    R = r + (k1 + 2*k2 + 2*k3 + k4)/6
    return R
 
#The function to initialize animation
def init():
    line.set_data([],[]) #Setting the initial variables of the animation line (as empty)
    t.set_text('') #Setting the text as empty
    return line, t #return value
 
def animate(i): #The function to build the animation
    global r
    r = rk4(r) #Obtaining the new value of r using the Runge-Kutta (4th order) method
 
    # Draw lines to each mass.
    xpos = np.cumsum([0,length1*np.sin(r[0]),length2*np.sin(r[1])])
    ypos = np.cumsum([0,-length1*np.cos(r[0]),-length2*np.cos(r[1])])
    line.set_data([xpos],[ypos])
    return line, t


 #Creating objects/figures (fig) and axes (ax) for drawing and axes
fig = plt.figure() 
ax = plt.axes(xlim=(-tl , tl) , ylim=(-tl,tl))
 
#Defining the style of the line to be drawn
line, = ax.plot([], [], 'k-o')
#initialise variable text
t = ax.text(0,0,0)
 
#Determining the update rate (step size) per second
h = 1/60
 
#Creating animation using the animate and init functions
#anim.FuncAnimation is used to create animation by calling animate and init at each interval of 1000*h
anim = anim.FuncAnimation(fig,animate,init_func=init,interval=1000*h,blit=True)
 
#To display
plt.show()