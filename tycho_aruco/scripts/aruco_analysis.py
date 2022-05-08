#!/usr/bin/env python3

import rospy
import rosbag
import os
import numpy as np
import matplotlib.pyplot as plt

def boxplots():
	"""Generate boxplots of marker detection rates from a set of rosbags."""
	# Scan directory for rosbags and compute mean detection rates for all markers
	#directory = "/home/trackingsystem/glegarda/rosbags/full_fleet/all"
	directory = "../rosbags"
	tmp_means = np.zeros([50, 1])
	for filename in os.scandir(directory):
		if ".bag" in filename.name:
			tmp_means = np.c_[tmp_means, detectionRates(filename)]
	means = np.transpose(tmp_means[:,1:])

	# Boxplot
	fig, axs = plt.subplots(2, 1, figsize=(18, 10), constrained_layout=True)
	axs[0].set_title("Unsorted")
	axs[0].set_ylim([0.0, 1.0])
	axs[0].set_ylabel("Detection rate")
	axs[0].set_xticks(range(50))
	axs[0].grid(axis='x')
	axs[0].set_xlabel("E-puck ID")
	axs[0].boxplot(means, notch=True, positions=range(50), patch_artist=True, boxprops=dict(facecolor='w'), flierprops=dict(marker='x'))

	# Sort in descending order and redo boxplot
	medians = np.median(means, axis=0)
	positions = np.zeros(50, dtype=int)
	labels = np.zeros(50, dtype=int)
	for i in range(50):
		index = np.argmax(medians)
		medians[index] = -1
		positions[index] = i
		labels[i] = index

	axs[1].set_title("Sorted by descending median")
	axs[1].set_ylim([0.0, 1.0])
	axs[1].set_ylabel("Detection rate")
	axs[1].grid(axis='x')
	axs[1].set_xlabel("E-puck ID")
	axs[1].boxplot(means, notch=True, positions=positions, patch_artist=True, boxprops=dict(facecolor='w'), flierprops=dict(marker='x'))
	axs[1].set_xticks(range(50))
	axs[1].set_xticklabels(labels)
	plt.savefig("../rosbags/epuck_detection_rates.pdf")



def detectionRates(filename):
	"""Determine which markers are detected most often."""
	bag = rosbag.Bag(filename.path)
	types_and_topics = bag.get_type_and_topic_info()
	n_epucks = 50

	# Get number of messages published by each camera
	n_cam0, n_cam1,  n_cam2 = 0, 0, 0
	for key, value in types_and_topics.topics.items():
		n_msg = value.message_count
		if key == "/camera_0/tracker/positions_stamped":
			n_cam0 = n_msg
		elif key == "/camera_1/tracker/positions_stamped":
			n_cam1 = n_msg
		elif key == "/camera_2/tracker/positions_stamped":
			n_cam2 = n_msg

	# Calculate rate of marker detection for each marker and camera
	m_cam0 = np.zeros([n_epucks, 1])
	m_cam1 = np.zeros([n_epucks, 1])
	m_cam2 = np.zeros([n_epucks, 1])
	for key, value in sorted(types_and_topics.topics.items()):
		if "epuck" in key:
			slash_pos = key.rfind("/")
			n_msg = value.message_count
			if "camera_0" in key:
				epuck = int(key[16:slash_pos])
				m_cam0[epuck] = n_msg / n_cam0
			elif "camera_1" in key:
				epuck = int(key[16:slash_pos])
				m_cam1[epuck] = n_msg / n_cam1
			elif "camera_2" in key:
				epuck = int(key[16:slash_pos])
				m_cam2[epuck] = n_msg / n_cam2

	# Mean detection rates
	m_cam_all = np.zeros([n_epucks, 1])
	for i in range(n_epucks):
		m_cam_all[i] = np.mean(np.array([m_cam0[i], m_cam1[i], m_cam2[i]]))

	# Recommended markers
	mean_rates = {}
	for i in range(n_epucks):
		mean_rates[i] = m_cam_all[i, 0]

	# print("Recommended markers:")
	# print("ID	Rate")
	# it = 0
	# for key, value in sorted(mean_rates.items(), key=lambda item: item[1], reverse=True):
	# 	print("{0:2}    {1}".format(key, value))
	# 	it = it + 1
	# 	if it == 20:
	# 		print("--------------------")

	# Plot results
	x_epuck = range(n_epucks)
	plt.figure(figsize=(15, 4.8))
	plt.scatter(x_epuck, m_cam0, marker='x', label="Camera 0")
	plt.scatter(x_epuck, m_cam1, marker='x', label="Camera 1")
	plt.scatter(x_epuck, m_cam2, marker='x', label="Camera 2")
	plt.plot(x_epuck, m_cam_all, 'ko--', label="Mean")
	plt.legend(loc='best')
	plt.ylim([0.0, 1.0])
	plt.ylabel("Detection rate")
	plt.xticks(range(n_epucks))
	plt.grid(axis='x')
	plt.xlabel("E-puck ID")
	plt.savefig(filename.path + ".pdf")

	bag.close()

	return m_cam_all

if __name__ == '__main__':
	try:
		boxplots()
	except rospy.ROSInterruptException:
		pass
