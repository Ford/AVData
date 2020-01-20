'''
This script saves each topic in a bagfile as a csv.

Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory
www.speal.ca

Supervised by Professor Inna Sharf, Professor Meyer Nahon

Used and modified by Ford Motor Company only for the purpose of Ford AV data release project
'''

import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file
import yaml

#verify correct input arguments: 1 or 2
if (len(sys.argv) > 3):
	print "invalid number of arguments:   " + str(len(sys.argv))
	print "should be 3: 'bag_to_csv.py', 'bagName' and 'config file'"
	print "or just 2  : 'bag_to_csv.py' and 'config file'"
	sys.exit(1)
elif (len(sys.argv) == 3):
	listOfBagFiles = [sys.argv[1]]
	configFile = [sys.argv[2]][0]
	numberOfFiles = "1"
	print "reading only 1 bagfile: " + str(listOfBagFiles[0])
elif (len(sys.argv) == 2):
	listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	#get list of only bag files in current dir.
	configFile = [sys.argv[1]][0]
	numberOfFiles = str(len(listOfBagFiles))
	print "reading all " + numberOfFiles + " bagfiles in current directory: \n"
	for f in listOfBagFiles:
		print f
	print "\n press ctrl+c in the next 10 seconds to cancel \n"
	time.sleep(10)
else:
	print "bad argument(s): " + str(sys.argv)	#shouldnt really come up
	sys.exit(1)

#check if config path exists
if not os.path.exists(str(configFile)):
	print configFile + " config file does not exist. Exiting"
	sys.exit(1)
with open(str(configFile)) as f:
	data = yaml.load(f)
	listOfTopicsToBeAdded = []
	listOfTopicsToBeExcluded = []
	for topicName in data.values()[0]:
		listOfTopicsToBeAdded.append(topicName)
	for topicName in data.values()[1]:
		listOfTopicsToBeExcluded.append(topicName)

count = 0
for bagFile in listOfBagFiles:
	count += 1
	print "reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile

	#check if bag file path exists
	if not os.path.exists(bagFile):
		print bagFile + " bag file does not exist. Exiting"
		sys.exit(1)

	#access bag
	bag = rosbag.Bag(bagFile)
	bagContents = bag.read_messages()
	bagName = bag.filename



	#create a new directory
	folder = string.rstrip(bagName, ".bag")
	try:	#else already exists
		os.makedirs(folder)
	except:
		pass


	#get list of topics from the bag
	listOfTopics = []
	listOfTopicsNot = []
	for topic, msg, t in bagContents:
		if topic in listOfTopicsToBeExcluded:
			if topic in listOfTopicsNot:
				continue
			else:
				listOfTopicsNot.append(topic)
				print "Excluding topic " + topic
				continue
		elif topic in listOfTopicsToBeAdded:
			if topic not in listOfTopics:
				listOfTopics.append(topic)
				print topic + " added to list of topics to be extracted"

	for topicName in listOfTopics:
		#Create a new CSV file for each topic
		print "Extracting the topic - " + topicName
		filename = folder + '/' + topicName + '.csv'
		with open(filename, 'w+') as csvfile:
			filewriter = csv.writer(csvfile, delimiter = ',')
			firstIteration = True	#allows header row
			for subtopic, msg, t in bag.read_messages(topicName):	# for each instant in time that has data for topicName
				#parse data from this instant, which is of the form of multiple lines of "Name: value\n"
				#	- put it in the form of a list of 2-element lists
				msgString = str(msg)
				msgList = string.split(msgString, '\n')
				instantaneousListOfData = []
				for nameValuePair in msgList:
					splitPair = string.split(nameValuePair, ':')
					for i in range(len(splitPair)):	#should be 0 to 1
						splitPair[i] = string.strip(splitPair[i])
					instantaneousListOfData.append(splitPair)
				#write the first row from the first element of each pair
				if firstIteration:	# header
					headers = ["rosbagTimestamp"]	#first column header
					for pair in instantaneousListOfData:
						headers.append(pair[0])
					filewriter.writerow(headers)
					firstIteration = False
				# write the value from each pair to the file
				values = [str(t)]	#first column will have rosbag timestamp
				for pair in instantaneousListOfData:
					if len(pair) > 1:
						values.append(pair[1])
				filewriter.writerow(values)
	bag.close()
print "Done reading all " + numberOfFiles + " bag files."
