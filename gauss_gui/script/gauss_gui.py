#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
 Software License Agreement (BSD License)

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
"""

from __future__ import division
import rospy
import math
import tf
import moveit_commander
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from gauss_gui_msgs.srv import SetString, SetStringRequest, SetStringResponse
from gauss_gui_msgs.srv import SetInt16, SetInt16Request, SetInt16Response
import wx
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
import dynamic_reconfigure.client
import rosgraph
import time
import socket
import Queue

g_joint_queue = Queue.Queue()
g_stop_joint_move = False
g_pose_queue = Queue.Queue()
g_stop_pose_move = False

class MyFrame(wx.Frame):  
  
    def __init__(self,parent,id):  
        the_size=(700, 450)
        wx.Frame.__init__(self,parent,id,'Gauss Control Panel',pos=(250,100)) 
        self.panel=wx.Panel(self)
        font=self.panel.GetFont()
        font.SetPixelSize((12, 24))
        self.panel.SetFont(font)
        
        self.listener = tf.TransformListener()
        
        self.robot=moveit_commander.RobotCommander(ns="gauss")
        self.group=moveit_commander.MoveGroupCommander('gauss_arm')

        self.joint_queue = g_joint_queue
        self.pose_queue = g_pose_queue
        self.joint_th = JointThread(self.joint_queue, self.group)
        self.joint_th.start()

        self.pose_th = PoseThread(self.pose_queue, self.group)
        self.pose_th.start()
        
        self.joint_names= ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6' ]
        
        # self.set_pose_reference_frame = "base_link"
        # self.ref_link_name=self.group.get_planning_frame()
        self.ref_link_name="base_link"

        self.end_link_name=self.group.get_end_effector_link()
        print "ref_link_name is: ", self.ref_link_name
        print "end_link_name is: ", self.end_link_name
        
        self.ref_link_lock=threading.Lock()
        self.end_link_lock=threading.Lock()

        self.js_display=[0]*6 # joint_states
        self.jm_button=[0]*6 # joints_minus
        self.jp_button=[0]*6 # joints_plus
        self.js_label=[0]*6 # joint_states
                      
        self.ps_display=[0]*6 # pcs_states
        self.pm_button=[0]*6 # pcs_minus
        self.pp_button=[0]*6 # pcs_plus
        self.ps_label=[0]*6 # pcs_states
                      
        self.display_init()
        self.key=[]  
            
        # self.action_client=SimpleActionClient(self.controller_ns+'follow_joint_trajectory',
        #                                       FollowJointTrajectoryAction)
        # self.action_goal=FollowJointTrajectoryGoal()
        # self.action_goal.trajectory.joint_names=self.joint_names
        btn_height=220
        btn_lengths=[]

        self.home_btn=wx.Button(self.panel, label='Home', name='home_btn', pos=(20, btn_height))
        btn_lengths.append(self.home_btn.GetSize()[0])
        btn_total_length=btn_lengths[0]

        self.home_btn.Bind(wx.EVT_LEFT_DOWN, 
                           lambda evt: self.move_home(evt))
        # self.home_btn.Bind(wx.EVT_LEFT_UP,
        #                    lambda evt, mark=100:
        #                    self.release_button(evt, mark) )

        self.stop_btn=wx.Button(self.panel, label='Stop', name='Stop')
        btn_lengths.append(self.stop_btn.GetSize()[0])
        btn_total_length+=btn_lengths[1]

        self.home_btn.SetPosition((220, 320))
        self.stop_btn.SetPosition((350, 320))

        self.reply_show_label=wx.StaticText(self.panel, label='Result:',
                                           pos=(20, btn_height+140))
        self.reply_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', size=(670, 30), pos=(20, btn_height+160))
        
        self.SetMinSize(the_size)
        self.SetMaxSize(the_size)
    
    def display_init(self):
        js_pos=[20, 20]
        js_btn_length=[70, 70, 61, 80]
        js_distances=[10, 20, 10, 26]
        dis_h=50
        ################### joint +
        for i in xrange(len(self.js_display)):
            self.jp_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' +', 
                                        pos=(js_pos[0],
                                             js_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp=js_btn_length[0]+js_distances[0]                                        
            self.jp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.joints_press(evt, mark) )
            self.jp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.joints_release(evt, mark) )

        ################### joint -            
            self.jm_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' -', 
                                        pos=(js_pos[0]+dis_tmp,
                                             js_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=js_btn_length[1]+js_distances[1]                                       
            self.jm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.joints_press(evt, mark) )
            self.jm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.joints_release(evt, mark) )
            
        #####   joint display
            
            pos_js_label=(js_pos[0]+dis_tmp, js_pos[1]+(5-i)*dis_h)
            self.js_label[i]=wx.StaticText(self.panel,
                                           label='J'+str(i+1)+'/deg:',
                                           pos=pos_js_label)
            self.js_label[i].SetPosition((pos_js_label[0], pos_js_label[1]+abs(40-self.js_label[i].GetSize()[1])/2))
            dis_tmp+=js_btn_length[2]+js_distances[2]

            pos_js_display=(js_pos[0]+dis_tmp, js_pos[1]+(5-i)*dis_h)
            self.js_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value=' 0000.00 ', 
                                           pos=pos_js_display)
            self.js_display[i].SetPosition((pos_js_display[0], pos_js_display[1]+abs(40-self.js_display[i].GetSize()[1])/2))
            dis_tmp+=js_btn_length[3]+js_distances[3]

        ps_pos=[js_pos[0]+dis_tmp, 20]
        ps_btn_length=[70, 70, 53, 80]
        ps_distances=[10, 20, 10, 20]
        pcs_btn_label=['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        pcs_label=['X', 'Y', 'Z', 'R', 'P', 'Y']
        unit_label=['/mm:', '/mm:', '/mm:', '/deg:', '/deg:', '/deg:']
        for i in xrange(len(self.ps_display)):
            self.pp_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' +', 
                                        pos=(ps_pos[0],
                                             ps_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp=ps_btn_length[0]+ps_distances[0]                                     

            self.pp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.pose_press(evt, mark) )
            self.pp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.pose_release(evt, mark) )
            ######
            self.pm_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' -', 
                                        pos=(ps_pos[0]+dis_tmp,
                                             ps_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=ps_btn_length[1]+ps_distances[1]                                        
            self.pm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.pose_press(evt, mark) )
            self.pm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.pose_release(evt, mark) )
            
            pos_ps_label=(ps_pos[0]+dis_tmp, ps_pos[1]+(5-i)*dis_h)
            self.ps_label[i]=wx.StaticText(self.panel, 
                                           label=pcs_label[i]+unit_label[i],
                                           pos=pos_ps_label)
            self.ps_label[i].SetPosition((pos_ps_label[0], pos_ps_label[1]+abs(40-self.ps_label[i].GetSize()[1])/2))
            dis_tmp+=ps_btn_length[2]+ps_distances[2]
            
            pos_ps_display=(ps_pos[0]+dis_tmp, ps_pos[1]+(5-i)*dis_h)
            self.ps_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=pos_ps_display)
            self.ps_display[i].SetPosition((pos_ps_display[0], pos_ps_display[1]+abs(40-self.ps_display[i].GetSize()[1])/2))
            dis_tmp+=ps_btn_length[3]+ps_distances[3]          

    def move_home(self,event):
        values = [0,0,0,0,0,0]
        try:
            self.group.set_joint_value_target(values)
        except Exception as err:
            print(err)
            return
        finally:
            print "move_home ok"
        self.group.go()
        resp = SetInt16Response()
        resp.success = True
        resp.message = "got move_home command success"
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()

    def joints_press(self,event,mark): 
        global g_stop_joint_move
        g_stop_joint_move = False
        print "mark is:", mark
        direction = -1 if mark<0 else 1
        self.joint_queue.put({ "joint_id": str(abs(mark)), "direction": direction})
        resp = SetInt16Response()
        resp.success = True
        resp.message = "got joint command success"
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def joints_release(self, event, mark):
        global g_stop_joint_move
        g_stop_joint_move = True
        # print "g_stop_joint_move is True"
        # self.joint_queue.queue.clear()
        resp = SetInt16Response()
        resp.success = True
        resp.message = "execute joint command success"
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()

    def pose_press(self,event,mark): 
        global g_stop_pose_move
        g_stop_pose_move = False
        print "mark is:", mark
        direction = -1 if mark<0 else 1
        self.pose_queue.put({ "axis_id": str(abs(mark)), "direction": direction})
        resp = SetInt16Response()
        resp.success = True
        resp.message = "got joint command success"
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def pose_release(self, event, mark):
        global g_stop_pose_move
        g_stop_pose_move = True
        print "g_stop_pose_move is True"
        # self.joint_queue.queue.clear()
        resp = SetInt16Response()
        resp.success = True
        resp.message = "execute pose command success"
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()

    def update_reply_show(self,msg):
        if msg.success:
            self.reply_show.SetBackgroundColour(wx.Colour(200, 225, 200))
        else:
            self.reply_show.SetBackgroundColour(wx.Colour(225, 200, 200))
        self.reply_show.SetValue(msg.message)
        
    def closewindow(self,event):
        pass    

    def updateDisplay(self, msg):      
        # print "updateDisplay"
        for i in xrange(len(self.js_display)):
            self.js_display[i].SetValue(msg[i])
        
        for i in xrange(len(self.ps_display)):
            self.ps_display[i].SetValue(msg[i+6]) 
    
    def monitor_status(self, evt):
        self.key=[]
        
        current_joint_values=self.group.get_current_joint_values()
        for i in xrange(len(current_joint_values)):
            self.key.append(str(round(current_joint_values[i]*180/math.pi, 2)))
        
        if self.ref_link_lock.acquire():
            ref_link=self.ref_link_name
            self.ref_link_lock.release()
        
        if self.end_link_lock.acquire():
            end_link=self.end_link_name
            self.end_link_lock.release()
        
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(ref_link, end_link, rospy.Time(0), rospy.Duration(100))
                (xyz,qua) = self.listener.lookupTransform(ref_link, end_link, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "error"
                continue
            
        rpy=tf.transformations.euler_from_quaternion(qua)
            
        self.key.append(str(round(xyz[0]*1000, 2)))
        self.key.append(str(round(xyz[1]*1000, 2)))
        self.key.append(str(round(xyz[2]*1000, 2)))
        
        self.key.append(str(round(rpy[0]*180/math.pi, 2)))
        self.key.append(str(round(rpy[1]*180/math.pi, 2)))
        self.key.append(str(round(rpy[2]*180/math.pi, 2)))
        
        wx.CallAfter(self.updateDisplay, self.key)
        
    def listen(self):        
        rospy.Timer(rospy.Duration(secs=0.1), self.monitor_status)

class JointThread(threading.Thread):
    def __init__(self, q, move_group):
        super(JointThread, self).__init__()
        self.queue = q
        self.group = move_group
 
    def run(self):
        while True:
            time.sleep(0.5)
            if not self.queue.empty():
                command = self.queue.get()
                current_values = self.group.get_current_joint_values()
                values = current_values
                joint_id = int(command["joint_id"])
                direction = int(command["direction"])
                if direction >0:
                    print "direction > 0"
                    values[joint_id-1] += 2/180*3.1415926
                else:
                    print "direction < 0"
                    values[joint_id-1] -= 2/180*3.1415926
                # print "values: ", values

                try:
                    self.group.set_joint_value_target(values)
                except Exception as err:
                    print(err)
                    continue
                finally:
                    print "set_joint_value_target ok"

                self.group.go()
                if not g_stop_joint_move:
                    print "g_stop_joint_move is: ",g_stop_joint_move
                    while True:
                        if direction >0:
                            print "direction > 0"
                            values[joint_id-1] += 2/180*3.1415926
                        else:
                            print "direction < 0"
                            values[joint_id-1] -= 2/180*3.1415926
                        # print "values: ", values
                        self.group.set_joint_value_target(values)
                        self.group.go()
                        time.sleep(0.1)
                        if g_stop_joint_move:
                            break

class PoseThread(threading.Thread):
    def __init__(self, q, move_group):
        super(PoseThread, self).__init__()
        self.queue = q
        self.group = move_group
 
    def run(self):
        while True:
            time.sleep(0.5)
            if not self.queue.empty():
                command = self.queue.get()
                current_pose = self.group.get_current_pose()
                pose_value = current_pose
                x = current_pose.pose.position.x
                y = current_pose.pose.position.y
                z = current_pose.pose.position.z

                qua = current_pose.pose.orientation

                rpy = tf.transformations.euler_from_quaternion([qua.x, qua.y, qua.z, qua.w])               
                roll = rpy[0]
                pitch = rpy[1]
                yaw = rpy[2]

                axis_id = int(command["axis_id"])
                direction = int(command["direction"])
                if direction >0:
                    print "direction > 0"
                    if axis_id == 6:
                        yaw += 2/180 * 3.1415926
                    elif axis_id == 5:
                            pitch += 2/180 * 3.1415926 
                    elif axis_id == 4:
                            roll += 2/180 * 3.1415926
                    elif axis_id == 3:
                            z += 2/1000 
                    elif axis_id == 2:
                            y += 2/1000 
                    elif axis_id == 1:
                            x += 2/1000 
                else:
                    print "direction < 0"
                    if axis_id == 6:
                        yaw -= 2/180 * 3.1415926
                    elif axis_id == 5:
                        pitch -= 2/180 * 3.1415926 
                    elif axis_id == 4:
                        roll -= 2/180 * 3.1415926
                    elif axis_id == 3:
                        z -= 2/1000 
                    elif axis_id == 2:
                        y -= 2/1000 
                    elif axis_id == 1:
                        x -= 2/1000   

                qua = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                pose_value.pose.position.x = x
                pose_value.pose.position.y = y
                pose_value.pose.position.z = z - 0.77
                pose_value.pose.orientation.x = qua[0]
                pose_value.pose.orientation.y = qua[1]
                pose_value.pose.orientation.z = qua[2]
                pose_value.pose.orientation.w = qua[3]

                try:
                    self.group.set_pose_target(pose_value)
                except Exception as err:
                    print(err)
                    continue
                finally:
                    print "set_pose_target ok"
                print x, " ", y, " ", z," ", roll/3.14*180," " ,pitch/3.14*180, " ",yaw/3.14*180
                plan = self.group.plan()
                self.group.go()

                if not g_stop_joint_move:
                    print "g_stop_joint_move is: ",g_stop_joint_move
                    while True:
                        if direction >0:
                            print "direction > 0"
                            if axis_id == 6:
                                yaw += 2/180 * 3.1415926
                            elif axis_id == 5:
                                    pitch += 2/180 * 3.1415926 
                            elif axis_id == 4:
                                    roll += 2/180 * 3.1415926
                            elif axis_id == 3:
                                    z += 2/1000 
                            elif axis_id == 2:
                                    y += 2/1000 
                            elif axis_id == 1:
                                    x += 2/1000 
                        else:
                            print "direction < 0"
                            if axis_id == 6:
                                yaw -= 2/180 * 3.1415926
                            elif axis_id == 5:
                                pitch -= 2/180 * 3.1415926 
                            elif axis_id == 4:
                                roll -= 2/180 * 3.1415926
                            elif axis_id == 3:
                                z -= 2/1000 
                            elif axis_id == 2:
                                y -= 2/1000 
                            elif axis_id == 1:
                                x -= 2/1000   

                        qua = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                        pose_value.pose.position.x = x
                        pose_value.pose.position.y = y
                        pose_value.pose.position.z = z - 0.77
                        pose_value.pose.orientation.x = qua[0]
                        pose_value.pose.orientation.y = qua[1]
                        pose_value.pose.orientation.z = qua[2]
                        pose_value.pose.orientation.w = qua[3]
                        print x, " ", y, " ", z," ", roll/3.14*180," " ,pitch/3.14*180, " ",yaw/3.14*180

                        try:
                            self.group.set_pose_target(pose_value)
                        except Exception as err:
                            print(err)
                            continue
                        finally:
                            print "set_joint_value_target ok"
                        plan = self.group.plan()
                        self.group.go()
                        
                        time.sleep(0.1)
                        if g_stop_pose_move:
                            break
  
def get_node_names():
    """
    @type  namespace: str
    @return: list of node caller IDs
    @rtype: [str]
    @raise ROSNodeIOException: if unable to communicate with master
    """
    ID = '/rosnode'
    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        return None
    nodes = []
   
    for s in state:
        for t, l in s:
            nodes.extend(l)
    return list(set(nodes))

if __name__=='__main__': 
    nodes = get_node_names()
    flag = True
    while nodes is None:
        time.sleep(2)
        mpdes = get_node_names() 
        # print "master is not started"     
    while "/move_group" not in nodes:
        time.sleep(2)
        mpdes = get_node_names() 
        # print "/move_group is not started"   
    print "move_group is ok"     

    rospy.init_node('elfin_gui')
    app=wx.App(False)  
    myframe=MyFrame(parent=None,id=-1)  
    myframe.Show(True)

    myframe.listen()

    app.MainLoop()
