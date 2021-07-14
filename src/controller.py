#!/usr/bin/env python
#license removed for brevity

''' Hi, I'm Massimo Romano, a Cybernetic Engineer! I created a Robot Manipulator URDF file (ROS/Gazebo enviroment) in collaboration with Adeept (https://www.adeept.com/) and I wrote this simple Genetic Algorithm for my thesis work at the University of Palermo. This algorithm is based on the operation of any simple genetic algorithm. For those who do not know, it is possible to read up in 10 minutes (really 10 minutes haha) through the following very simple tutorial: https://www.tutorialspoint.com/genetic_algorithms/genetic_algorithms_survivor_selection.htm. Before reading my code it is strongly recommended that you read the tutorial of ROS in ROS Wiki site: http://wiki.ros.org/it. In this algorithm there are a few lines that assume you know about ROS Concepts (http://wiki.ros.org/ROS/Concepts), how to write a ROS Publisher (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) and how to write a TF Listener (http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29). This algorithm was developed and tested on the ROS Noetic platform version 1.15.11. However each line will be commented, so if you still have doubts you can write me to the email: romano.massimo.uni@gmail.com'''

''' The goal of the algorithm is to place the end effector of the manipulator robot in a point called point_task, by calculating the Euclidean distance between the point reached and the point_task at each attempt (population chromosome) '''

import rospy
import math
import tf
import random
import numpy as np
from std_msgs.msg import Float64
import csv


#Population (of 10 Chromosomes) initialization function 
def init_pop ():
	population = [] #Empty list that identifies the population

	for i in range(10):
		#Initialization of an empty vector that identifies a chromosome.
		#Each chromosome has three coordinates (three angles of Joint Robot)
		chromosome = np.zeros(3) 
		chromosome[0]= random.uniform(0, 3.14) #Random angle between [0,180] that goes to 0 position of chromosome
		
		for j in range(1,3):
			
			#Random angle between [-90, 90] that goes to 1 and 2 position of chromosome
			chromosome[j] = random.uniform(-1.57,1.57) 
			 
		print("The chromosome n°", i+1," is ", chromosome)
		print("")
		population.append(chromosome) #Chromosome goes to i position of population list
		
	
	print("The population generated is: ")
	print(population[0])
	print(population[1])
	print(population[2])
	print(population[3])
	print(population[4])
	print(population[5])
	print(population[6])
	print(population[7])
	print(population[8])
	print(population[9])
	print("")
	return population #The function return of the population list
	
#Fitness Function
def fitness_calc(trans):
	
	point_task = [-0.045, 0, 0.193] #This is the point task for the Robot Manipulator
	
	#The fitness function for this task is the Euclidean distance between the point reached and the point_task!
	d_x = trans[0] - point_task[0]
	d_y = trans[1] - point_task[1]
	d_z = trans[2] - point_task[2]
	d = math.sqrt((d_x*d_x) + (d_y*d_y) + (d_z*d_z))
	
	return d #The function return distance
	
#Function of tournament selection 
def selection_parent (pop, fit):
	
	#Empty lists that identifies the tournament and the fitness of candidate chromosomes
	tournament = []
	k_fit = []
	
	#I choose three candidates by generating a random index and extracting both the chromosome and its fitness for
	#that position 
	for i in range(3):
		rand_ind = random.randint(0,9) 
		tournament.append(pop[rand_ind])
		k_fit.append(fit[rand_ind])
	
	return tournament, k_fit #The function return three candidates and their fitness
	
#Function of tournament	
def tournament(tour, fit):
	
	#This function implements parent's selection with K-Way Tournament mode
	#This function checks the three candidates to extract the one with the least (best) fitness.
	#I assume the chromosome (and the corresponding fitness) in position 0 as the best and make a comparison with the other two!
	
	min_fit = fit[0]
	parent = tour[0]
	
	for i in range(3): 
		if(fit[i] < min_fit):
			min_fit = fit[i]
			parent = tour[i]
			
	return parent, min_fit #The function return one parent and its fitness (the best!)
	
#Function to calculate least fitness	
def calc_min_fit(fit):
	
	#I assume the least fitness in position 0 as the best and make a comparison with the other fitness of vector fit!
	min_i = 0
	min_fit = fit[0]
	for i in range(10):
		if(fit[i] < min_fit):
			min_fit = fit[i]
			min_i = i
			
	return min_i, min_fit #The function return one parent and its fitness (the best!)
	
	
#Function to crossover		
def crossover (father, mother, fath_fit, moth_fit):
	
	#This function implements crossover with Whole Arithmetic Recombination mode
	#The weighted average is carried out gene by gene of the father and mother chromosomes
	#Average weight alpha gives more importance to the parent with the lowest fitness!
	
	child = [] #Empty list that identifies the population
	alfa = (moth_fit/(fath_fit+moth_fit)) #Weighted average coefficient
	
	for i in range (3):
		
		media_i = (alfa*father[i]) + ((1-alfa)*mother[i]) #Weighted average 
		child.append(media_i) #New gene of child
		
	return child #The function return child!
	
#Function Survivor Selection
def survivor_sel (pop, pop_fit, child, child_fit):
	
	#This function implements Fitness-based survivor selection mechanism
	#Control first of all the higher (worse) fitness in the population
	#If the child's fitness is less (better) than the highest (worst) fitness of the population 
	#the child takes the place of the worst individual!

        #I assume the highest fitness in position 0 and make a comparison with the other fitness of population!
	max_fit = pop_fit[0]
	n = 0 #Coefficient to replace the child only once in the population
	high_fit_index = 0
	
	for i in range(10):
		if(pop_fit[i] > max_fit):
			max_fit = pop_fit[i]
			high_fit_index = i
			
	if(child_fit < max_fit):
		if n==0:
			pop_fit[high_fit_index] = child_fit
			pop[high_fit_index] = child
			n=1
				

#Function Main
def main():
	
	#This is the main of the algorithm where it is implemented a state machine 
	#In the first section there is the initialization of the ROS node "adeeptRobot_controller" 
	#There is also the initialization of the three publishers for the three joints of the Robot
	#In the second section there is an infinite while (until the ROS node that executes this code is closed)
	#There are various states in which we implement the mechanism of the GA
	
	base = rospy.Publisher('/adeeptrobot_base_controller/command', Float64, queue_size=10)
	joint1 = rospy.Publisher('/adeeptrobot_joint1_controller/command', Float64, queue_size=10)
	joint2 = rospy.Publisher('/adeeptrobot_joint2_controller/command', Float64, queue_size=10)
	rospy.init_node('adeeptRobot_controller', anonymous=True)
	rate = rospy.Rate(0.3) #0.3 hz -> Delay_Time = 3.3 s  
	
	listener = tf.TransformListener()
	
	population = init_pop() #initialization of population
	fitness = [] #Empty List of fitness
	min_fit_vet = [] #Empty List of least (best) fitness
	end_index=0 #Index that count the cycles of algorithm
	
	state = "START" #First State
	
	while not rospy.is_shutdown():
    	
    		if state == "START":
    			
    			print("GO! ;)")
    			rate.sleep() #Pause of 3.3 s
    			
	    		for i in range(10):
	    			
	    			#Test 10 chromosomes on the Robot and measure the fitness for each of them
	    			#Publish the "Base", "Joint1" and "Joint2" position
	    			
	    			chromosome = population[i]
	    		
	    			base_position = chromosome[0]
	    			joint1_position = chromosome[1]
	    			joint2_position = chromosome[2]
	    		
	    			print("Chromosome n°", i+1," :")
	    			print("Base: ", base_position)
	    			print("Joint 1: ", joint1_position)
	    			print("Joint 2: ", joint2_position)
	    			
	    			rate.sleep()
	    			
	    			base.publish(base_position) #Publish of base's value
	    			joint1.publish(joint1_position) #Publish of joint1's value
	    			joint2.publish(joint2_position) #Publish of joint2's value
	    		
	    			try:
		    			rate.sleep()
		    			
		    			#Use TF library of ROS to store the translation vector and the Rotation matrix 
		    			#of the end effector referred to the base_link of the Robot

		    			(trans,rot) = listener.lookupTransform('/base_link', '/endEffector', rospy.Time(0))
		    			score = fitness_calc(trans) #Calculate the Euclidean Distance with Translation Vector of End Eff.
		    			fitness.append(score) #Append the calculated fitness to the fitness list
		    			print("[x_p, y_p, z_p]= ",trans)
		    			print("Fitness: ",score)
		    			print("")
		    			
		    			#Save calculated fitness of pupolation in results.csv
		    			with open('results.csv', mode='a') as csv_file:
			    			nomicolonne = ['Chromosome', 'Fitness']
			    			writer = csv.DictWriter(csv_file, fieldnames=nomicolonne)
			    			writer.writerow({'Chromosome': i , 'Fitness' : score})
		    		
	    			
	    			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    				continue
	    				
	    		state = "SELECTION"
	    		
	    	elif state == "SELECTION":
	    	
	    		#In this state we implement the tournament selection by calling the "selection_parent" and "tournament" functions. 
	    	        
	    	        end_index += 1 #End index increase (useful for END status)
	    	        k_tour1, k_fit1 = selection_parent(population,fitness)
	    	        rate.sleep()
	    	        print("The following chromosomes were selected for the first tournament: ")
	    	        print("Competitor 1: ", k_tour1[0], "Fitness: ", k_fit1[0])
	    	        print("Competitor 2: ", k_tour1[1], "Fitness: ", k_fit1[1])
	    	        print("Competitor 3: ", k_tour1[2], "Fitness: ", k_fit1[2])
	    	        father, father_fitness = tournament(k_tour1,k_fit1)
	    	        print("The father is: ", father, " with Fitness: ", father_fitness)
	    	        
	    	        print("")
	    	        
	    	        k_tour2, k_fit2 = selection_parent(population,fitness)
	    	        print("The following chromosomes were selected for the second tournament: ")
	    	        print("Competitor 1: ", k_tour2[0], "Fitness: ", k_fit2[0])
	    	        print("Competitor 2: ", k_tour2[1], "Fitness: ", k_fit2[1])
	    	        print("Competitor 3: ", k_tour2[2], "Fitness: ", k_fit2[2])
	    	        mother, mother_fitness = tournament(k_tour2,k_fit2)
	    	        print("The mother is: ", mother, " with Fitness: ", mother_fitness)
	    	        print("")
	    	        rate.sleep()
	    	        
	    	        state = "CROSSOVER"
	    		
	    	elif state == "CROSSOVER":
	    		
	    		#In this state we implement the crossover mechanism by calling the "crossover" functions.
	    		
	    		child = crossover(father, mother, father_fitness, mother_fitness)
	    		print("Child is: ", child)
	    		print("")
	    		
	    		state = "CHILD_FITNESS"
	    		
	    	elif state == "CHILD_FITNESS":
	    		
	    		#In this state we implement the calculation of child's fitness.
	    		#Publish the "Base", "Joint1" and "Joint2" position

	    		base_position = child[0]
	    		joint1_position = child[1]
	    		joint2_position = child[2]
	    		
	    		rate.sleep()
	    		
	    		print("Child valutation: ")
	    		print("")
	    		print("Base: ", base_position)
	    		print("Joint 1: ", joint1_position)
	    		print("Joint 2: ", joint2_position)
	    		
	    		rate.sleep()
	    		
	    		base.publish(base_position)
	    		joint1.publish(joint1_position)
	    		joint2.publish(joint2_position)
	    		
	    		try:
	    			rate.sleep()
	    			
	    			#Use TF library of ROS to store the translation vector and the Rotation matrix 
		    		#of the end effector referred to the base_link of the Robot
		    		
	    			(trans,rot) = listener.lookupTransform('/base_link', '/endEffector', rospy.Time(0))
	    			score_child = fitness_calc(trans) #Calculate the Euclidean Distance with Translation Vector of child
	    			print("[x_p, y_p, z_p]= ",trans)
	    			print("Fitness: ",score_child)
	    			print("")
	    			
	    		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    			continue
	    			
	    		state = "SURVIVOR_SELECTION"
	    		
	    	elif state == "SURVIVOR_SELECTION":
	    		
	    		#In this state we implement the survivor selection by calling the "survivor_sel" function.
	    			    		
	    		survivor_sel(population, fitness, child, score_child)
	    		fit_i, min_fit = calc_min_fit(fitness) #Calculate the lowest (best) fitness and its index
	    		min_fit_vet.append(min_fit) #Append the best fitness in min_fit_vet list
	    		rate.sleep()
	    		print("New population is: ")
	    		print("")
	    		print(population[0])
	    		print(population[1])
	    		print(population[2])
	    		print(population[3])
	    		print(population[4])
	    		print(population[5])
	    		print(population[6])
	    		print(population[7])
	    		print(population[8])
	    		print(population[9])
	    		print("")
	    		print("Best fitness is n°: ", fit_i+1, " Value: ", min_fit)
	    		print("")
	    		state = "SAVE_DATA"
	    		
	    	elif state == "SAVE_DATA":
	    		
	    		#In this state we save the datas of the best Chromosome and its fitness in results.csv
	    		
	    		with open('results.csv', mode='a') as csv_file:
	    			nomicolonne = ['Chromosome', 'Fitness']
	    			writer = csv.DictWriter(csv_file, fieldnames=nomicolonne)
	    			writer.writerow({'Chromosome': fit_i , 'Fitness' : min_fit})
				
			#We perform a final check to eventually terminate the algorithm.
			#The algorithm end if the relative variation between the last calculated best fitness and the previous one 
			#is less than 1x10^-3
			#Otherwise we go back to the "SELECTION" state
	    		if(end_index>=10):
	    			if ((min_fit_vet[end_index-1] - min_fit_vet[end_index-2])/min_fit_vet[end_index-1] < 1e-3):
	    				print ("After ", end_index, " iterations the best chromosome found is: ", population[fit_i])
	    				print("Its Fitness: ", min_fit)
	    				state= "END"
	    				
	    		else:
	    			state="SELECTION"
	    		
	    	elif state == "END":
	    		pass
	    	
if __name__ == '__main__':
   try:
        main()
   except rospy.ROSInterruptException:
        pass
