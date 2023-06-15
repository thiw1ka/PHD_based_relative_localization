#!/usr/bin/env python

import numpy as np
import pandas as pd
# import matplotlib
# matplotlib.use('Agg')
from matplotlib import pyplot as plt
import rospy
import os
from sensor_msgs.msg import PointCloud, PointCloud2
from gazebo_msgs.msg import ModelStates
import sensor_msgs.point_cloud2 as pc2
import datetime
import time

class subcriber_calls:
    def __init__(self, index, x = [], y = [], wy = [], pdx = [], pdy = [], psx = [], ps = []):
        self.index = index
        self.x      = x
        self.y      = y
        self.wy     = wy
        self.count  = 0
        self.pdy    = pdy  
        self.pdx    = pdx 
        self.ps     = ps 
        self.psx    = psx 
        # print("subcall created = ", self.index)

    def subcriber_call (self,msg):
        # print("inside subcriber ", self.index)
        self.count = self.count + 1
        sumOfWeights = 0
        for pt in msg.points:
            sumOfWeights = sumOfWeights + pt.z
        self.wy.append(sumOfWeights)
        self.y.append(len(msg.points))
        self.x.append(self.count)
        for pd in msg.channels[0].values:
            self.pdy.append(pd)
            self.pdx.append(self.count)
        for ps in msg.channels[1].values:
            self.ps.append(ps)
            self.psx.append(self.count)  

class catabot_plotter:
    def __init__(self, plot_in_background = False):
        self.plot_in_background = plot_in_background
        if plot_in_background:
            plt.switch_backend('Agg')
        self.counter_to_kill_node = 0
        rospy.init_node("catabot_plotter", anonymous = True)
        pdlist = rospy.get_param('~/pd_for_each_target',[0, 0, 0, 0])
        self.pdtitle = ""
        for pd in pdlist:
            self.pdtitle = self.pdtitle + "_" + str(pd)
        print("[plotter] pd list - ", self.pdtitle)
        print("[Plotter] catabot_plotter starting...")
        # self.count = 0
        self.count_2 = 0
        # self.x = []
        # self.y = []
        # self.wy = []
        self.x2 = []
        self.y2 = []

        topics_exists = False
        topic_colors = ["b", "r", "y", "g"]
        print("[Plotter] waiting for topics to get publish..")
        while topics_exists == False and not rospy.is_shutdown() :
            self.topics_to_plot = {}
            topics = rospy.get_published_topics()
            topic_count = 0
            for topic, msg_type in topics:
                if topic.find("/filter_results_for_plotter") != -1:
                    print(topic)
                    self.topics_to_plot[topic_count] = {}
                    self.topics_to_plot[topic_count]['name'] = topic
                    self.topics_to_plot[topic_count]['x'] = []
                    self.topics_to_plot[topic_count]['y'] = []
                    self.topics_to_plot[topic_count]['wy'] = [] #weight
                    self.topics_to_plot[topic_count]['pdx'] = []
                    self.topics_to_plot[topic_count]['pdy'] = []
                    self.topics_to_plot[topic_count]['psx'] = []
                    self.topics_to_plot[topic_count]['ps'] = []
                    self.topics_to_plot[topic_count]['color'] = topic_colors[topic_count]
                    self.topics_to_plot[topic_count]['func'] = subcriber_calls(topic_count,
                                                                                self.topics_to_plot[topic_count]['x'],
                                                                                self.topics_to_plot[topic_count]['y'],
                                                                                self.topics_to_plot[topic_count]['wy'],
                                                                                self.topics_to_plot[topic_count]['pdx'],
                                                                                self.topics_to_plot[topic_count]['pdy'],
                                                                                self.topics_to_plot[topic_count]['psx'],
                                                                                self.topics_to_plot[topic_count]['ps'] 
                                                                                ).subcriber_call
                    rospy.Subscriber(topic, PointCloud, self.topics_to_plot[topic_count]['func'])

                    topic_count = topic_count + 1
            if len(self.topics_to_plot) > 0:
                topics_exists = True
                continue
            else:
                rospy.sleep(rospy.Duration(1.0))

        measurement_topic_name = rospy.get_param("measurement_topic_name",'/centroid_label_obstacle')
        self.isGazeboSimtrue = rospy.get_param ("gazebo_sim_rosbag", False)
        print("[Plotter] measurement topic is " + measurement_topic_name)
        if self.isGazeboSimtrue:
             rospy.Subscriber(measurement_topic_name, PointCloud, self.callbackMeasurementPt)
        elif measurement_topic_name == '/centroid_label_obstacle':
            rospy.Subscriber(measurement_topic_name, PointCloud2, self.callbackMeasurementPt2)
        else:
            rospy.Subscriber(measurement_topic_name, PointCloud, self.callbackMeasurementPt)

        rospy.Timer(rospy.Duration(1.0), self.plot)

        try:
            rospy.spin()
            rospy.on_shutdown(self.savePlot())
            print("plotter is closing")
        except KeyboardInterrupt:
            # rospy.on_shutdown(self.savePlot())
            # self.savePlot()
            print("shutting down")

    def plot(self, event = None):
        try:
            plt.rcParams["figure.figsize"] = [40, 40]
            plt.rcParams["figure.autolayout"] = True

            plt.subplot(411)
            plt.title("Estimate count and Sum of weight")
            plt.cla
            plt.xlabel("Time Steps")
            plt.ylabel("Number of estimates / sum of weight")

            label_count_list = []
            label_names_list = []
            for idx in range(len(self.topics_to_plot)):
                globals()["label_es" + str(idx)]    = str(self.topics_to_plot[idx]['name']+" estimate count")
                globals()["scat" + str(idx)]        = plt.scatter(self.topics_to_plot[idx]['x'],self.topics_to_plot[idx]['y'],marker='+', color = self.topics_to_plot[idx]['color'], label = globals()["label_es" + str(idx)])
                label_count_list.append(globals()["scat" + str(idx)])
                label_names_list.append(globals()["label_es" + str(idx)])
                globals()["label_w" + str(idx)]     = self.topics_to_plot[idx]['name']+" sum of weights"
                globals()["line" + str(idx) +","]   = plt.plot(self.topics_to_plot[idx]['x'], self.topics_to_plot[idx]['wy'], self.topics_to_plot[idx]['color'], label = globals()["label_w" + str(idx)])
                label_count_list.append(globals()["line" + str(idx) +","])
                label_names_list.append(globals()["label_w" + str(idx)])
            plt.legend(label_count_list, label_names_list)

            plt.subplot(413)
            plt.title("Measurement Count")
            plt.xlabel("Time Steps")
            plt.ylabel("Number of measurements")
            raw_measurements = plt.scatter(self.x2, self.y2, color="g", label = 'Measurement count', marker='+')
            plt.legend([raw_measurements],["Measurement count"])
            # self.savePlot()

            plt.subplot(412)
            plt.title("Pd list - " + self.pdtitle)
            plt.xlabel("Time Steps")
            plt.ylabel("PD")
            label_count_list = []
            label_names_list = []
            for idx in range(len(self.topics_to_plot)):
                globals()["label_pd" + str(idx)] = str(self.topics_to_plot[idx]['name']+"- PD")
                globals()["pd_scat" + str(idx)]  = plt.scatter(self.topics_to_plot[idx]['pdx'],self.topics_to_plot[idx]['pdy'],marker='+', color = self.topics_to_plot[idx]['color'], label = globals()["label_pd" + str(idx)])
                label_count_list.append(globals()["pd_scat" + str(idx)])
                label_names_list.append(globals()["label_pd" + str(idx)])
            plt.legend(label_count_list, label_names_list)

            plt.subplot(414)
            plt.title("PS")
            plt.xlabel("Time Steps")
            plt.ylabel("PS")

            for idx in range(len(self.topics_to_plot)):
                globals()["label_ps" + str(idx)] = str(self.topics_to_plot[idx]['name'])
                globals()["ps_scat" + str(idx)]  = plt.scatter(self.topics_to_plot[idx]['psx'],self.topics_to_plot[idx]['ps'],marker='+', color = self.topics_to_plot[idx]['color'], label = globals()["label_ps" + str(idx)])
                label_count_list.append(globals()["ps_scat" + str(idx)])
                label_names_list.append(globals()["label_ps" + str(idx)])
            plt.legend(label_count_list, label_names_list)

            plt.pause(0.00001)
        except Exception as e:
            if self.plot_in_background == False:
                print("[Plotter] Error in plotter function: ",e)
                plt.cla
                self.counter_to_kill_node = self.counter_to_kill_node + 1
                if self.counter_to_kill_node > 4 :
                    rospy.signal_shutdown("killing the node")
            # exit(0)

        # self.savePlot()

    def callbackGazeboModelStates (self, msg):
        self.robot_list = {}
        for name in msg.name:
            if name.find("Robot") != -1:
                self.robot_list[name] = int(name[5:])
                print(self.robot_list[name])
        for robot in self.robot_list:
            pose = msg.pose[self.robot_list[robot]]
            print(self.robot_list[robot],pose)



    #for mingis measurement topic which has point cloud 2
    def callbackMeasurementPt2 (self, msg):
        # print(type(msg))
        self.count_2 = self.count_2 + 1
        self.measurments = pc2.read_points_list(msg,skip_nans=True)
        self.y2.append(len(self.measurments))
        self.x2.append(self.count_2)

    #for synthetic measurement which has point cloud 1
    def callbackMeasurementPt (self, msg):
        # print(type(msg))
        self.count_2 = self.count_2 + 1
        # self.measurments = pc.read_points_list(msg,skip_nans=True)
        self.y2.append(len(msg.points))
        self.x2.append(self.count_2)

    def lowPassFilterAverage (self):
        pass

    def movingAverage (self):
        timesteps = 50

    def savePlot (self):
        path = "~/catkin_ws/src/uri_soft_wip/catabot_project/catabot_phd_tracking/plots/"

        try:
            # plt.close()
            self.plot()
            # plt.switch_backend('Agg')
            #  use('Agg')
            tnow = datetime.datetime.now()
            figname = str(tnow.year)+ str(tnow.month) + str(tnow.day) + str(tnow.hour) + str(tnow.minute) #+ str(tnow.second)
            path_to_save_plot = os.path.expanduser(path)
            plt.savefig(path_to_save_plot + figname + '.png', format="png",dpi = 100)
            print("Plot is saved...")

        except OSError:
            print("[ERR Saving Plot] saving path is invalid...")

    def __del__(self):
        # self.savePlot()
        print("plotter destructor called...")

if __name__ == '__main__':
    try:
        plotter = catabot_plotter(plot_in_background = False)
    except Exception as e:
        print("[Error running catabot plotter] : ",e)