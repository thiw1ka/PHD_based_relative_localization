#!/usr/bin/env python

import numpy as np
import pandas as pd
# import matplotlib
# matplotlib.use('Agg')
import matplotlib
from matplotlib import pyplot as plt
import rospy
import os
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
import datetime
import time

class subcriber_calls:
    def __init__(self, index, x = [], y = [], wy = [], 
                 pdx = [], pdy = [], psx = [], ps = [],
                 posX = [], posY = [], mavg = [], variance = [],
                 error = [], meas_name = None, sumOfError = [],
                 pd_camx = [], pd_camy = [], pd_lidx = [],pd_lidy = []):
        self.index = index
        self.x      = x
        self.y      = y
        self.wy     = wy
        self.count  = 0
        self.pdy    = pdy  
        self.pdx    = pdx 
        self.ps     = ps 
        self.psx    = psx
        self.posX   = posX
        self.posY   = posY
        self.regularPD = None # PD used for regular PHD filter
        self.regularPS = None # PS used for regular PHD filter
        self.regularPSPDObtained = False
        self.mavg   = mavg
        self.mavg_value = 0.0
        self.mean   = 0.0
        self.meansq = 0.0
        self.variance = variance
        self.error  = error
        self.meas_name = meas_name
        self.err_value = 0.0
        self.WindErrSize = 0.5
        sumOfError.append(0.0)
        self.sumOfError = sumOfError
        self.pd_camy = pd_camy
        self.pd_camx = pd_camx
        self.pd_lidy = pd_lidy
        self.pd_lidx = pd_lidx

    def moving_average (self, sumOfWeights):
        self.wSize = 0.8
        self.mavg_value = self.mavg_value * self.wSize + (1 - self.wSize) * sumOfWeights
        self.mavg.append(self.mavg_value)
        #counting variance
        error = sumOfWeights - self.mean
        self.mean = self.mean + (error)/self.count
        self.meansq = self.meansq + error * error
        self.variance.append(self.meansq / self.count)

    def moving_average_error (self, sumOfError):
        self.WindErrSize = 0.8
        self.err_value = self.err_value * self.wSize + (1 - self.wSize) * sumOfError
        return self.err_value

    def subcriber_call (self,msg):
        # print("inside subcriber ", self.index)
        if not self.regularPSPDObtained: #one time thing
            # self.sumOfError = 0.0
            startindx = msg.channels[0].name.find('-')
            self.regularPD = (msg.channels[0].name[startindx+1:])
            startindx = msg.channels[1].name.find('-')
            self.regularPS = (msg.channels[0].name[startindx+1:])
            print("[plotter] regular pd - ",self.regularPD,", regular ps",self.regularPS)
            self.regularPSPDObtained = True
        self.count = self.count + 1
        sumOfWeights = 0
        for pt in msg.points:
            sumOfWeights = sumOfWeights + pt.z
            self.posX.append(pt.x)
            self.posY.append(pt.y)
        self.wy.append(sumOfWeights)
        self.moving_average (sumOfWeights) #moving average
        self.y.append(len(msg.points))
        self.x.append(self.count)
        for pd in msg.channels[0].values:
            self.pdy.append(pd)
            self.pdx.append(self.count)
        for ps in msg.channels[1].values:
            self.ps.append(ps)
            self.psx.append(self.count)

        if not msg.channels[2].name == "camera_pd":
            print("[plotter] subcriber Error. channel 2 not camera pd. chennel2 name:", msg.channels[2].name)
            raise Exception("Subcriber Error.Channel 2 not camera pd") 
        for pdCam in msg.channels[2].values: #pd camera
            self.pd_camy.append(pdCam)
            self.pd_camx.append(self.count)
        
        if not msg.channels[3].name == "lidar_pd":
            print("[plotter] subcriber Error. channel 3 not lidar pd. chennel3 name:", msg.channels[3].name)
            raise Exception("Subcriber Error.Channel 3 not camera pd") 
        for pdLidar in msg.channels[3].values: #pd camera
            self.pd_lidy.append(pdLidar)
            self.pd_lidx.append(self.count)
   
        if self.meas_name != '/centroid_label_obstacle' and self.meas_name != '/centroid_label_obstacle_republished':
            error = 0
            for e in msg.channels[4].values:
                error = error + e
            self.error.append(self.moving_average_error(error))
            self.sumOfError[0] = self.sumOfError[0] + error #total error

class catabot_plotter:
    def __init__(self, plot_in_background = False):
        # print(matplotlib.font_manager.get_font_names())
        self.path = "~/catkin_ws/src/uri_soft_wip/catabot_project/catabot_phd_tracking/plots/"
        self.plot_in_background = plot_in_background
        if plot_in_background:
            plt.switch_backend('Agg')
        self.counter_to_kill_node = 0
        rospy.init_node("catabot_plotter", anonymous = True)
        pdlist = rospy.get_param('~/pd_for_each_target', [0, 0, 0, 0])
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
        self.xsgt = []
        self.ysgt = []
        self.plotGTruth = False

        topics_exists = False
        topic_colors = ["g", "y", "b"]
        topic_markers = ["^","o","s"]
        print("[Plotter] waiting for topics to get publish..")
        measurement_topic_name = rospy.get_param("measurement_topic_name",'/centroid_label_obstacle')
        self.isGazeboSimtrue = rospy.get_param ("gazebo_sim_rosbag", False)
        print("[Plotter] measurement topic is " + measurement_topic_name)
        if measurement_topic_name == '/centroid_label_obstacle' or measurement_topic_name == '/centroid_label_obstacle_republished':
            rospy.Subscriber(measurement_topic_name, PointCloud2, self.callbackMeasurementPt2)
        elif self.isGazeboSimtrue :
            print("[plotter] plotter in gazebo simulation mode")
            self.plotGTruth = True
            rospy.Subscriber(measurement_topic_name, PointCloud, self.callbackMeasurementPt)
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.callbackGazeboModelStates)

            # rospy.Subscriber('/synthetic_measurements_vrpn', PointCloud, self.callbackSyntheticGroundTruth)

        else:
            self.plotGTruth = True
            rospy.Subscriber(measurement_topic_name, PointCloud, self.callbackMeasurementPt)
            rospy.Subscriber('/synthetic_measurements_vrpn', PointCloud, self.callbackSyntheticGroundTruth)
        while topics_exists == False and not rospy.is_shutdown() :
            self.topics_to_plot = {} # dictionary to keep plotter topics for each filter variance
            topics = rospy.get_published_topics()
            topic_count = 0
            for topic, msg_type in topics:
                # print(topic)
                if topic.find("/filter_results_for_plotter") != -1:
                    substring_end_idx = topic.find('/',1)
                    self.topics_to_plot[topic_count]                = {}
                    self.topics_to_plot[topic_count]['name']        = topic[1:substring_end_idx].capitalize()
                    self.topics_to_plot[topic_count]['x']           = []
                    self.topics_to_plot[topic_count]['y']           = []
                    self.topics_to_plot[topic_count]['wy']          = [] #weight
                    self.topics_to_plot[topic_count]['pdx']         = []
                    self.topics_to_plot[topic_count]['pdy']         = []
                    self.topics_to_plot[topic_count]['psx']         = []
                    self.topics_to_plot[topic_count]['ps']          = []
                    self.topics_to_plot[topic_count]['posX']        = []
                    self.topics_to_plot[topic_count]['posY']        = []
                    self.topics_to_plot[topic_count]['mavg']        = [] #moving average
                    self.topics_to_plot[topic_count]['variance']    = [] #variance
                    self.topics_to_plot[topic_count]['error']       = [] #error for each robot
                    self.topics_to_plot[topic_count]['meas_name']   = measurement_topic_name
                    self.topics_to_plot[topic_count]['sumOfError']  = []
                    self.topics_to_plot[topic_count]['pdCamy']      = []
                    self.topics_to_plot[topic_count]['pdCamx']      = []
                    self.topics_to_plot[topic_count]['pd_lidy']     = []
                    self.topics_to_plot[topic_count]['pd_lidx']     = []
                    if self.topics_to_plot[topic_count]['name']     == "Standard":
                        self.topics_to_plot[topic_count]['color']   = "r"
                        self.topics_to_plot[topic_count]['xsgt']        = self.xsgt
                        self.topics_to_plot[topic_count]['ysgt']        = self.ysgt
                    else :
                        self.topics_to_plot[topic_count]['color'] = topic_colors[topic_count]
                    self.topics_to_plot[topic_count]['marker'] = topic_markers[topic_count]
                    self.topics_to_plot[topic_count]['func'] = subcriber_calls(topic_count,
                                                                                self.topics_to_plot[topic_count]['x'],
                                                                                self.topics_to_plot[topic_count]['y'],
                                                                                self.topics_to_plot[topic_count]['wy'],
                                                                                self.topics_to_plot[topic_count]['pdx'],
                                                                                self.topics_to_plot[topic_count]['pdy'],
                                                                                self.topics_to_plot[topic_count]['psx'],
                                                                                self.topics_to_plot[topic_count]['ps'],
                                                                                self.topics_to_plot[topic_count]['posX'],
                                                                                self.topics_to_plot[topic_count]['posY'],
                                                                                self.topics_to_plot[topic_count]['mavg'],
                                                                                self.topics_to_plot[topic_count]['variance'],
                                                                                self.topics_to_plot[topic_count]['error'],                                                                       
                                                                                self.topics_to_plot[topic_count]['meas_name'],
                                                                                self.topics_to_plot[topic_count]['sumOfError'],
                                                                                self.topics_to_plot[topic_count]['pdCamx'],
                                                                                self.topics_to_plot[topic_count]['pdCamy'],
                                                                                self.topics_to_plot[topic_count]['pd_lidx'],
                                                                                self.topics_to_plot[topic_count]['pd_lidy']                                                             
                                                                                ).subcriber_call
                    rospy.Subscriber(topic, PointCloud, self.topics_to_plot[topic_count]['func'])
                    print("[Plotter] Topic added to the list - " + topic)
                    topic_count = topic_count + 1 #number of plotter topics
            if len(self.topics_to_plot) > 0:
                #After going through list of topics
                topics_exists = True

                continue
            else:
                rospy.sleep(rospy.Duration(1.0))
        try:
            rospy.spin()
            rospy.on_shutdown(self.seperatePlots())
            print("plotter is closing")
        except KeyboardInterrupt:
            # rospy.on_shutdown(self.savePlot())
            # self.savePlot()
            print("shutting down")

    def seperatePlots(self, event = None):
        try:
            tnow = datetime.datetime.now()
            figname = str(tnow.year)+ str(tnow.month) + str(tnow.day) + str(tnow.hour) + str(tnow.minute) #+ str(tnow.second)
            path_to_save_plot = os.path.expanduser(self.path)

            # plt.rcParams['pdf.fonttype'] = 42
            # plt.rcParams["ps.useafm"] = True

            plt.rcParams.update({
                "text.usetex": True,
                "font.family": "times"
            })

            #save all to excel
            for idx in range(len(self.topics_to_plot)):
                name = str(self.topics_to_plot[idx]['name'])
                globals()[str(self.topics_to_plot[idx]['name'])] = pd.DataFrame({key:pd.Series(val) for key,val in self.topics_to_plot[idx].items()})
                globals()[str(self.topics_to_plot[idx]['name'])].to_csv(path_to_save_plot + figname+ name+"_alldata" + '.csv')
            print("[plotter] excel saved")

            #measurement count plot
            self.figure_2 = plt.figure(2)
            plt.cla
            plt.title("Measurement Count")
            plt.xlabel("Time Steps")
            plt.ylabel("Number of measurements")
            raw_measurements = plt.scatter(self.x2, self.y2, color="g", label = 'Measurement count', marker='+')
            plt.legend([raw_measurements],["Measurement count"])
            self.figure_2.savefig(path_to_save_plot + figname+ "_MC" + '.png', format="png",dpi = 100)
            print("[plotter] plot saved - Measurement Count")

            #MOVING AVERAGE PLOT
            self.figure_6 = plt.figure(6)
            # plt.title("Estimate count and Sum of weight")
            plt.cla
            plt.xlabel("Time Steps (k)")
            plt.ylabel("Sum of Weights - Moving Average")
            plt.tight_layout()
            # plt.xlim(0,160)
            # plt.ylim(0,9)
            if self.plotGTruth:
                pass
                # plt.xlim(5,1000)
                # plt.ylim(0,12)
            label_count_list = []
            label_names_list = []
            for idx in range(len(self.topics_to_plot)):
                globals()["label_w" + str(idx)]     = self.topics_to_plot[idx]['name']
                globals()["line" + str(idx) +","]   = plt.plot( self.topics_to_plot[idx]['x'], 
                                                                self.topics_to_plot[idx]['mavg'], 
                                                                self.topics_to_plot[idx]['color'], 
                                                                label = globals()["label_w" + str(idx)])
                globals()["label_w" + str(idx)]     = self.topics_to_plot[idx]['name']+" Variance"
                globals()["line" + str(idx) +","]   = plt.plot( self.topics_to_plot[idx]['x'], 
                                                                self.topics_to_plot[idx]['variance'], 
                                                                self.topics_to_plot[idx]['color'],
                                                                marker = "^", 
                                                                label = globals()["label_w" + str(idx)])
                # label_count_list.append(globals()["line" + str(idx) +","])
                # label_names_list.append(globals()["label_w" + str(idx)])
            # self.figure_1.legend(label_count_list, label_names_list)
            # self.figure_6.legend(loc='lower left', borderaxespad = 5)
            # plt.savefig(path_to_save_plot + figname+ "_weights" + '.png', format="png",dpi = 100)
            self.figure_6.savefig(path_to_save_plot + figname+ "_mvg" + '.png', format="png",dpi = 100)
            print("[plotter] plot saved - Moving Avg")

            #PD plots
            self.figure_3 = plt.figure(3)
            plt.cla
            title = 'Pd ' + self.pdtitle
            # print(self.topics_to_plot[0]['func'].regularPD)
            # plt.title(title)
            plt.xlabel("Time Steps (k)")
            plt.ylabel("Probability of Detection ($P_D^i$)")
            plt.ylim(0,1.0)
            plt.tight_layout()
            label_count_list = []
            label_names_list = []
            print(1)
            for idx in range(len(self.topics_to_plot)):
                print(2)
                globals()["label_pd" + str(idx)] = str(self.topics_to_plot[idx]['name'])
                print(3)
                globals()["pd_scat" + str(idx)]  = plt.scatter( self.topics_to_plot[idx]['pdx'],
                                                                self.topics_to_plot[idx]['pdy'],
                                                                marker=self.topics_to_plot[idx]['marker'], 
                                                                color = self.topics_to_plot[idx]['color'], 
                                                                label = globals()["label_pd" + str(idx)])
                print(4)
                label_count_list.append(globals()["pd_scat" + str(idx)])
                print(5)
                label_names_list.append(globals()["label_pd" + str(idx)])
            # plt.legend(label_count_list, label_names_list)
            print(6)
            # self.figure_3.legend(loc='lower left', borderaxespad = 5)
            print(7)
            self.figure_3.savefig(path_to_save_plot + figname+ "_pd" + '.png', format="png")
            print("[plotter] plot saved - Probability of Detection Total")

            #PS plot
            self.figure_4 = plt.figure(4)
            plt.cla
            # plt.title("PS")
            plt.xlabel("Time Steps (k)")
            plt.ylabel("Probability of Survival ($P_S^i$)")
            plt.ylim (0, 1)
            plt.tight_layout()
            label_count_list = []
            label_names_list = []
            for idx in range(len(self.topics_to_plot)):
                globals()["label_ps" + str(idx)] = str(self.topics_to_plot[idx]['name'])
                globals()["ps_scat" + str(idx)]  = plt.scatter( self.topics_to_plot[idx]['psx'],
                                                                self.topics_to_plot[idx]['ps'],
                                                                marker='+', 
                                                                color = self.topics_to_plot[idx]['color'], 
                                                                label = globals()["label_ps" + str(idx)])
                label_count_list.append(globals()["ps_scat" + str(idx)])
                label_names_list.append(globals()["label_ps" + str(idx)])
            # plt.legend(label_count_list, label_names_list)
            self.figure_4.legend(loc='lower left', borderaxespad = 5)
            self.figure_4.savefig(path_to_save_plot + figname+ "_ps" + '.png', format="png",dpi = 100)
            print("[plotter] plot saved - Probability of Survival")
            

            #Ground Truth plot
            print("[plotter] Ground Truth plot bool check :",self.plotGTruth)
            if self.plotGTruth:
                self.figure_5 = plt.figure(5)
                plt.cla
                # plt.title("PS")
                plt.xlabel("X Coordinates (m)")
                plt.ylabel("Y Coordinates (m)")
                # plt.ylim (0, 1)
                plt.tight_layout()
                label_count_list = []
                label_names_list = []
                xy_scatter_groundtruth = plt.scatter( self.xsgt, self.ysgt, marker='.', color = 'y', label = "ground truth", alpha = 0.3, s= 300)
                op = [0.7, 0.2, 0.3]#opacity for each scatter plot
                print("1")
                for idx in range(len(self.topics_to_plot)):
                    print("2")
                    globals()["label_xy" + str(idx)] = str(self.topics_to_plot[idx]['name'])
                    print("3")
                    globals()["xy_scat" + str(idx)]  = plt.scatter( self.topics_to_plot[idx]['posX'],
                                                                    self.topics_to_plot[idx]['posY'],
                                                                    marker = self.topics_to_plot[idx]['marker'], 
                                                                    color = self.topics_to_plot[idx]['color'],
                                                                    alpha = op[idx],
                                                                    label = globals()["label_xy" + str(idx)])
                    print("4")
                    label_count_list.append(globals()["xy_scat" + str(idx)])
                    print("5")
                    label_names_list.append(globals()["label_xy" + str(idx)])
                    print("6")
                # plt.legend(label_count_list, label_names_list)
                print("7")
                self.figure_5.legend(loc='lower left', borderaxespad = 2)
                print("8")
                self.figure_5.savefig(path_to_save_plot + figname+ "_xy" + '.png', format="png",dpi = 100)
                print("[plotter] plot saved - XY")

                #SUM OF WEIGHT PLOT
                self.figure_7 = plt.figure(7)
                # plt.title("Estimate count and Sum of weight")
                plt.cla
                plt.xlabel("Time Steps (k)")
                plt.ylabel("Sum of Errors")
                # plt.tight_layout()
                # plt.xlim(0,1000)
                # plt.ylim(0,3)
                for idx in range(len(self.topics_to_plot)):
                    globals()["label_w" + str(idx)]     = self.topics_to_plot[idx]['name']#+" Error sum"
                    globals()["line" + str(idx)]   = plt.scatter( self.topics_to_plot[idx]['x'], 
                                                                    self.topics_to_plot[idx]['error'],
                                                                    marker = self.topics_to_plot[idx]['marker'],
                                                                    alpha = op[idx],  
                                                                    color = self.topics_to_plot[idx]['color'], 
                                                                    label = globals()["label_w" + str(idx)])
                self.figure_7.legend(loc='lower left', borderaxespad = 2)
                self.figure_7.savefig(path_to_save_plot + figname+ "_error" + '.png', format="png",dpi = 100)
                print("[plotter] plot saved - Sums of Error")

                #TOTAL ERROR TABLE
                self.figure_8 = plt.figure(8)
                # plt.title("Estimate count and Sum of weight")
                plt.cla
                plt.xlabel("Time Steps (k)")
                plt.ylabel("Total Error")
                # plt.tight_layout()
                # plt.xlim(0,1000)
                # plt.ylim(0,3)
                col_label = []
                row_label = []
                row_label.append("Values")
                cellvalue = []
                table_data = []
                for idx in range(len(self.topics_to_plot)):
                    cellvalue.append(str(self.topics_to_plot[idx]['sumOfError'][0]))
                    col_label.append(str(self.topics_to_plot[idx]['name']))
                    # row_label.append(str(self.topics_to_plot[idx]['name']))
                    table_data.append([ str(self.topics_to_plot[idx]['name']), str(self.topics_to_plot[idx]['sumOfError'][0]) ])
                df = pd.DataFrame(cellvalue, index=col_label)
                df.to_csv(path_to_save_plot + figname+ "_table" + '.csv')
                df.to_latex(path_to_save_plot + figname+ "_latex" )
                print(df)
                print("[plotter] data file saved - Total error")


            

            #Camera PD plot
            self.figure_8 = plt.figure(8)
            plt.cla
            plt.xlabel("Time Steps (k)")
            plt.ylabel("Probability of Detection ($P_D^i$) - Camera")
            plt.ylim(0,1.0)
            plt.tight_layout()
            plt.tight_layout()
            # plt.xlim(0,160)
            # plt.ylim(0,9)
            for idx in range(len(self.topics_to_plot)):
                globals()["label_pd" + str(idx)] = str(self.topics_to_plot[idx]['name'])
                globals()["pd_scat" + str(idx)]  = plt.scatter( self.topics_to_plot[idx]['pdCamx'],
                                                                self.topics_to_plot[idx]['pdCamy'],
                                                                marker='+', 
                                                                color = self.topics_to_plot[idx]['color'], 
                                                                label = globals()["label_pd" + str(idx)])
            self.figure_8.legend(loc='lower left', borderaxespad = 5)
            self.figure_8.savefig(path_to_save_plot + figname+ "_pdCam" + '.png', format="png",dpi = 100)
            print("[plotter] plot saved - PD camera")

            #Lidar PD plot
            self.figure_9 = plt.figure(9)
            plt.cla
            plt.xlabel("Time Steps (k)")
            plt.ylabel("Probability of Detection ($P_D^i$) - Lidar")
            plt.ylim(0,1.0)
            plt.tight_layout()
            plt.tight_layout()
            print("1")
            for idx in range(len(self.topics_to_plot)):
                print("2")
                globals()["label_pd" + str(idx)] = str(self.topics_to_plot[idx]['name'])
                print("3")
                # print(self.topics_to_plot[idx]['pd_lidx'])
                # print(self.topics_to_plot[idx]['pd_lidy'])
                globals()["pd_scat" + str(idx)]  = plt.scatter( self.topics_to_plot[idx]['pd_lidx'],
                                                                self.topics_to_plot[idx]['pd_lidy'],
                                                                marker='+', 
                                                                color = self.topics_to_plot[idx]['color'], 
                                                                label = globals()["label_pd" + str(idx)])
                print("4")
            print("5")
            self.figure_9.legend(loc='lower left', borderaxespad = 5)
            print("6")
            self.figure_9.savefig(path_to_save_plot + figname+ "_pdLidar" + '.png', format="png", dpi = 100)
            print("[plotter] plot saved - PD Lidar")
            plt.pause(0.00001)

            
            plt.pause(0.00001)
            rospy.signal_shutdown("killing the node")
        except Exception as e:
            if self.plot_in_background == False:
                print("[Plotter] Error in plotter function: ",e)
                plt.cla
                self.counter_to_kill_node = self.counter_to_kill_node + 1
                if self.counter_to_kill_node > 4 :
                    rospy.signal_shutdown("killing the node")

    def callbackGazeboModelStates (self, msg):
        self.robot_list = {}
        for name in msg.name:
            if name.find("Robot") != -1:
                self.robot_list[name] = int(name[5:])
                # print(self.robot_list[name])
        for robot in self.robot_list:
            pose = msg.pose[self.robot_list[robot]]
            # print(self.robot_list[robot],pose)
            self.xsgt.append(pose.position.x) #xsgt - X synthetic ground truth
            self.ysgt.append(pose.position.y) #ysgt - Y synthetic ground truth

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

    def callbackSyntheticGroundTruth(self, msg):
        for pt in msg.points:
            self.xsgt.append(pt.x) #xsgt - X synthetic ground truth
            self.ysgt.append(pt.y) #ysgt - Y synthetic ground truth

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

    def saveSeperatePlots(self):
        try:
            self.seperatePlots()

        except OSError:
            print("[ERR Saving Seperate Plots] saving path is invalid...")

    def __del__(self):
        # self.savePlot()
        print("plotter destructor called...")

if __name__ == '__main__':
    try:
        plotter = catabot_plotter(plot_in_background = True)
    except Exception as e:
        print("[Error running catabot plotter] : ",e)