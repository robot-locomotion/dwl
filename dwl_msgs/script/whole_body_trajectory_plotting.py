#!/usr/bin/python
import rosbag
import argparse
from dwl_msgs.msg import WholeBodyState
import numpy as np

from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from matplotlib import rc
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def extract(bagfile, whole_body_state_topic):
    n = 0
    state_list = list()
    initial_time = 0
    with rosbag.Bag(bagfile, 'r') as bag:
        initial_time = 4.9
        duration = 1.
        starting_time = bag.get_start_time() + initial_time
        if (duration == 0.0):
            ending_time = bag.get_end_time()
            duration = ending_time - starting_time
        else:
            ending_time = starting_time + duration
        
        # Recording the whole-body state trajectory
        time_list = list()
        base_pos_list = list()
        base_vel_list = list()
        base_acc_list = list()
        joint_pos_list = list()
        joint_vel_list = list()
        joint_acc_list = list()
        contact_list = list()
        for (topic, msg, ts) in bag.read_messages(topics=str(whole_body_state_topic)):
            # Recording the data in the desired time range
            if (ts.to_sec() >= starting_time and ts.to_sec() <= ending_time):
                # Setting the floating-base system DoF
                num_base_joints = len(msg.actual.base)
                num_joints = len(msg.actual.joints)
                num_contacts = len(msg.actual.contacts)
            
                # Getting the current time
                if (n == 0):
                    initial_time = ts.to_sec()
                time = ts.to_sec() - initial_time
            
                # Defining the state lists
                base_pos = list()
                base_vel = list()
                base_acc = list()
                joint_pos = list()
                joint_vel = list()
                joint_acc = list()
                contact_state = list()

                # Getting the base states
                for i in range(num_base_joints):
                    base_pos.append(msg.actual.base[i].position)
                    base_pos.append(msg.desired.base[i].position)
                    base_vel.append(msg.actual.base[i].velocity)
                    base_vel.append(msg.desired.base[i].velocity)
                    base_acc.append(msg.actual.base[i].acceleration)
                    base_acc.append(msg.desired.base[i].acceleration)
                
                # Getting the joint states
                for i in range(num_joints):
                    joint_pos.append(msg.actual.joints[i].position)
                    joint_pos.append(msg.desired.joints[i].position)
                    joint_vel.append(msg.actual.joints[i].velocity)
                    joint_vel.append(msg.desired.joints[i].velocity)
                    joint_acc.append(msg.actual.joints[i].acceleration)
                    joint_acc.append(msg.desired.joints[i].acceleration)
                                        
                # Getting the contact states
                for i in range(num_contacts):
                    contact_state.append(msg.actual.contacts[i].position.x)
                    contact_state.append(msg.actual.contacts[i].position.y)
                    contact_state.append(msg.actual.contacts[i].position.z)
                    contact_state.append(msg.desired.contacts[i].position.x)
                    contact_state.append(msg.desired.contacts[i].position.y)
                    contact_state.append(msg.desired.contacts[i].position.z)

                base_pos_list.append(base_pos)
                base_vel_list.append(base_vel)
                base_acc_list.append(base_acc)
                joint_pos_list.append(joint_pos)
                joint_vel_list.append(joint_vel)
                joint_acc_list.append(joint_acc)
                contact_list.append(contact_state)

                n += 1
    print('Loaded ' + str(n) + ' whole body states from bagfile.')
    
    # Getting the time info
    for i in range(n):
        time_list.append(i * duration/n)
    
    
    time_vec = np.array(time_list)
    base_pos_vec = np.array(base_pos_list)
    base_vel_vec = np.array(base_vel_list)
    base_acc_vec = np.array(base_acc_list)
    joint_pos_vec = np.array(joint_pos_list)
    joint_vel_vec = np.array(joint_vel_list)
    joint_acc_vec = np.array(joint_acc_list)
    contact_vec = np.array(contact_list)

    return time_vec, base_pos_vec, base_vel_vec, base_acc_vec, joint_pos_vec, joint_vel_vec, joint_acc_vec, contact_vec
          
# def record_data(x, y):
    #write date in txt file
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts WholeBodyControllerState messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    args = parser.parse_args()
        
    print('Extract pose from bag '+args.bag+' in topic ' + args.topic)
    time, base_pos, base_vel, base_acc, joint_pos, joint_vel, joint_acc, contact_pos = extract(args.bag, args.topic)

    # Plotting the base states
    num_base_joints = 1#len(joint_state[0])
    for i in range(num_base_joints):
        # Plotting the base position
        fig = plt.figure(i)
        ax = fig.add_subplot(111)
        ax.plot(time,base_pos[:,2*i+1], 'k', linewidth=2.5)
        ax.plot(time,base_pos[:,2*i]+0.2283, 'r', linewidth=2.5)
        plt.title('Base Position', fontsize=18, fontweight='bold')
        plt.ylabel('$z$ $[m]$', {'color':'k', 'fontsize':18})
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':18})
#         plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('base_pos_' + str(i) + '.pdf')
        
        # Plotting the base velocity
        fig = plt.figure(num_base_joints*(i+1))
        ax = fig.add_subplot(111)
        ax.plot(time,base_vel[:,2*i+1], 'k', linewidth=2.5)
        ax.plot(time,base_vel[:,2*i], 'r', linewidth=2.5)
        plt.ylabel('Base Velocity')
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
        plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('base_vel_' + str(i) + '.pdf')
        
        # Plotting the base acceleration
        fig = plt.figure(num_base_joints*(i+2))
        ax = fig.add_subplot(111)
        ax.plot(time,base_acc[:,2*i+1], 'k', linewidth=2.5)
        ax.plot(time,base_acc[:,2*i], 'r', linewidth=2.5)
        plt.ylabel('Base Acceleration')
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
        plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('base_acc_' + str(i) + '.pdf')
        
    # Plotting the joint states
    num_joints = 2#len(joint_state[0])
    for i in range(num_joints):
        fig = plt.figure(i+(num_base_joints*(num_base_joints+2)))
        ax = fig.add_subplot(111)
        ax.plot(time,joint_pos[:,num_joints*i+1], 'k', linewidth=2.5)
        ax.plot(time,joint_pos[:,num_joints*i], 'r', linewidth=2.5)
        plt.ylabel('Joint position')
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
        plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('joint_pos_' + str(i) + '.pdf')
        
        # Plotting the joint velocity
        fig = plt.figure(num_joints*(i+1)+(num_base_joints*(num_base_joints+2)))
        ax = fig.add_subplot(111)
        ax.plot(time,joint_vel[:,num_joints*i+1], 'k', linewidth=2.5)
        ax.plot(time,joint_vel[:,num_joints*i], 'r', linewidth=2.5)
        plt.ylabel('Joint Velocity')
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
        plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('joint_vel_' + str(i) + '.pdf')
        
        # Plotting the joint acceleration
        fig = plt.figure(num_joints*(i+2)+(num_base_joints*(num_base_joints+2)))
        ax = fig.add_subplot(111)
        ax.plot(time,joint_acc[:,num_joints*i+1], 'k', linewidth=2.5)
        ax.plot(time,joint_acc[:,num_joints*i], 'r', linewidth=2.5)
        plt.ylabel('Joint Acceleration')
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
        plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('joint_acc_' + str(i) + '.pdf')
        
    
    # Plotting the contact states
    num_contacts = 1#len(joint_state[0])
    for i in range(num_contacts):
        fig = plt.figure(i+(num_base_joints*(num_base_joints+2))+num_joints*(num_joints+2))
        ax = fig.add_subplot(111)
        actual_x = 3*num_contacts*i
        actual_y = 3*num_contacts*i+1
        actual_z = 3*num_contacts*i+2
        desired_x = 3*(num_contacts*i+1)
        desired_y = 3*(num_contacts*i+1)+1
        desired_z = 3*(num_contacts*i+1)+2
        ax.plot(contact_pos[:,desired_x],contact_pos[:,desired_z]+base_pos[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,contact_pos[:,desired_z]+base_pos[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,contact_pos[:,actual_z]+base_pos[:,2*i]+0.2283, 'r', linewidth=2.5)

        ax.plot(contact_pos[:,desired_x],contact_pos[:,desired_z]+base_pos[:,2*i+1], 'k', linewidth=2.5)
        ax.plot(contact_pos[:,actual_x],contact_pos[:,actual_z]+base_pos[:,2*i]+0.2283, 'r', linewidth=2.5)


        plt.ylabel('$z$ $[m]$', {'color':'k', 'fontsize':16})
        plt.xlabel('$x$ $[m]$', {'color':'k', 'fontsize':16})
        plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
        plt.grid(True)
        plt.gca().invert_xaxis()
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
#         someX, someY = 2, 3
#         currentAxis = plt.gca()
#         plt.gca().add_patch(Rectangle((someX - .05, someY - .05), 1, 1, facecolor="grey"))
        ax.broken_barh([(0.11, 0.185)] , (-0.5827, 0.15), facecolors='#FFE4B5')
#        ax.set_ylim(-0.5827,-0.3)
        fig.savefig('contact_pos_' + str(i) + '.pdf')
        
#         # Plotting the joint velocity
#         fig = plt.figure(num_contacts*(i+1)+(num_base_joints*(num_base_joints+2))+num_joints*(num_joints+2))
#         plt.plot(time,joint_vel[:,2*i+1], '--k', linewidth=4)
#         plt.plot(time,joint_vel[:,2*i], 'r', linewidth=2.5)
#         plt.ylabel('Joint velocity')
#         plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#         plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#         fig.tight_layout()
#         fig.savefig('joint_vel_' + str(i) + '.png')
#         
#         # Plotting the joint acceleration
#         fig = plt.figure(num_contacts*(i+2)+(num_base_joints*(num_base_joints+2))+num_joints*(num_joints+2))
#         plt.plot(time,joint_acc[:,2*i+1], '--k', linewidth=4)
#         plt.plot(time,joint_acc[:,2*i], 'r', linewidth=2.5)
#         plt.ylabel('Joint acceleration')
#         plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#         plt.legend((r'$Planned$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#         fig.tight_layout()
#         fig.savefig('joint_acc_' + str(i) + '.png')
