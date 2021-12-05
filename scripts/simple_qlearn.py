# [ROS tutorial] OpenAI Gym for ROS based Robots 101. Gazebo Simulator
import gym
import gym_gazebo
import time
import random
import time
import matplotlib
import matplotlib.pyplot as plt
from gym import wrappers
from liveplot import LivePlot

def render():
    render_skip = 0 #Skip first X episodes
    render_interval = 50 #Show render Every Y episodes
    render_episodes = 30 #SHow Z episodes every rendering

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

if __name__ == '__main__':
    env = gym.make("GazeboCircuitTurtlebotLidar-v0")  # select gym environment.
    print "Gym Make done"
    outdir = '/tmp/gazeobo_gym_experiments' #where to generate file
    # record the results and learning results data in order to evaluate if it's really learning or not.
    # GYm gives a wrapper that will record the state of the system data, called observations. 
    # THe observations are what we consider relevant data for learning the decisions of what actions to perfrom next.
    env = wrappers.Monitor(env, outdir, force=True)  # records reward/evolution of learning
    print "Monitor Wrapper started"
    last_time_steps = numpy.ndarray(0)

    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
    alpha=0.1, gamma=0.8, epsilon=0.9)

    inital_epsilon = qlearn.epsilon

    epsilon_discount = 0.999 # 1098 eps to reach 0.1
    start_time = time.time()
    total_episodes = 10
    highest_reward = 0

    for x in range(total_episodes):
        done = False

        cumulated_reward = 0 # should going forward give more reward then L/R?
        print ("Episode = "+str(x))
        observation = env.reset()
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        env.render()

        state = ''.join(map(str, observation))

        for i in range(500):  # how many steps before going to the next episode. Bare this number in mind because in 
                                # the environment setup you will have to put the same number or higher to avoid gym module related errors.

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            # Print ("Action Chosen" +str(action))
            # Execute the action and get feedback
            # observation: The observation is the state of the environment.
            # It will return different kind of data depending on how the environment settingup file is difined. 
            # In this case, it returns a discrete version of the laser readings of the robot. 
            # In your personal case has to be data needed to take AI decisions.
            # It could be altitude, image data, sonar, pointclouds, tactile data...
            # ANything that your AI algorithm need decide whats the next action. 
            # reward: it's the reward for the current step taken. THe higher the reward, the better the robot if performing based on the conditions you stated. 
            # done: it states if the episode is done or not. IN this case it will be done= True if the robot gone too close to a wall.
            # info: extra information, in this case its empty.
            observation, reward, done, info = env.step(action)  
            cumulated_reward += reward
            #print ("Reward="+str(reward))
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            qlearn.learn(state, action, reward, nextState)

            #env.monitor.flush(force=True)

            if not(done):
                #print "NOT done"
                state = nextState
            else:
                print "DONE"
                last_time_steps = numpy.append(last_time_steps, [init(i+1)])
                break

            m, s = divmod(int(time.time() - start_time), 60)
            h, m = divmod(m, 60)
            print ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha, 2))+" - gamma: "+str(round(qlearn.gamma, 2)))

            #Github table content
            print ("\n|"+str(total_episodes)+"|"+str(qlearn.alpha)="|"+str(qlearn.gamma)+"|"+str(inital_epsilon))

            l = last_time_steps.tolist()
            l.sort()

            #print ("parameters: a = "+str)
            print("overall score: {:0.2f}".format(last_time_steps.mean()))
            print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x+y, l[-100:]) / len(l[-100:])))



            ## if perform properly you could easily need 2000 episodes. 
            # recommend run a 10 or so to see how it performs.

            # Implement a new script using a different learning strategy, like sarsa. 
            # implement my own algorithm based on the data returned by the env.step()


            ## Environment file config
            # THe bassis of OPenAI-gym is the creating of a file called environment file.
            # THis environment file will define how the learning file communicates with the OpenAI-GYm API.
            # It has to be clear that you are going to use the gym_gazebo module, not the original gym module. 
            # THis is because in order to use Gazebo, this addon had to be created.
            # First lets see where you can find these files.
            # Theses files are part of the python installation of gym_gazebo module. 
            # SO the most desirable location to put it is in the dist-pakages dir. 
            # IN this case you will find it in "/usr/local/lib/pypthon2.7/dist-packages",inside the folder gym_gazebo.

            # __init__.py: this file registers the different environments available in the gym infrastrcture.
            # FOr the turtlebot simulation we set up different parameters being the most importatnt ones the id and the timestep_limit. 
            # THe time step limit allows you to make the learning episodes shorter or longer, depending on your needs. 
            # For example, if the action to perform is really short, lower the time. 
            # If for the contrary its an action that to perform it correctly needs some time, increase the time
            # to allow the simulation to have time to perform the entire action.

            