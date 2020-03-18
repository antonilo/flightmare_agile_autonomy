#!/usr/bin/env python3
import os
import rospy
import rospkg

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import quadrotor_msgs.msg as quadrotor_msgs
import std_msgs.msg as std_msgs

import math
import numpy as np

# tensorflow 2.0
import tensorflow as tf
from tensorflow.keras.layers import Dense, Input, concatenate
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam

# rpg baselines
from rpg_baselines.common import logger
from rpg_baselines.common import util as U
from rpg_baselines.algos.sac.sac import Actor

#
U.tf_config_gpu_memory()

class StateBuffer:
    def __init__(self, obs_dim, max_size=100):
        self._obs_buf = []
        #
        self._index = 0
        self._max_size = max_size
        self._size = 0

    def push(self, obs):
        # append state at the end of the list
        self._obs_buf.append(obs)
        #
        if (len(self._obs_buf) > self._max_size):
            # when the buffer is full, delete the oldest element
            self._obs_buf.pop(0) 

    def pop(self,):
        if len(self._obs_buf) > 0:
            # return the lastest state
            return self._obs_buf.pop()
        else:
            False

class RunMlPController:
    def __init__(self, mlp_actor):
        self.actor = mlp_actor

        # set variables
        self._armed = False
        self._quad_namespace = None
        self._connected = False

        self._control_rate = rospy.Rate(100)

        self._arm_bridge_pub = None
        self._start_pub = None
        self._land_pub = None
        self._off_pub = None
        self._force_hover_pub = None
        self._go_to_pose_pub = None

        self._control_command = None

        self._autopilot_feedback_sub = None
        self._autopilot_feedback = quadrotor_msgs.AutopilotFeedback()
        self._autopilot_feedback_stamp = rospy.Time.now()

        self._act_prev = np.array([0, 0, 0, 0])
        self._act_error = np.array([0, 0, 0, 0]) 
        self._count = 0

        self._int_vxvy = np.array([0, 0, 0])
        self._err_vxvy = np.array([0, 0, 0])

        self._obs = [ 0,
            0., 0., 0., 
            0.0, 0., 0., 1.,
            0., 0., 0.]
        self._obs_buf = StateBuffer(obs_dim=13)

    def connect(self, quad_namespace):
        self._quad_namespace = quad_namespace

        self._arm_bridge_pub = rospy.Publisher(
            quad_namespace+'/bridge/arm', std_msgs.Bool, queue_size=1)
        self._start_pub = rospy.Publisher(
            quad_namespace+'/autopilot/start', std_msgs.Empty, queue_size=1)
        self._land_pub = rospy.Publisher(
            quad_namespace+'/autopilot/land', std_msgs.Empty, queue_size=1)
        self._off_pub = rospy.Publisher(
            quad_namespace+'/autopilot/off', std_msgs.Empty, queue_size=1)
        self._force_hover_pub = rospy.Publisher(
            quad_namespace+'/autopilot/force_hover', std_msgs.Empty,
            queue_size=1)
        self._go_to_pose_pub = rospy.Publisher(
            quad_namespace+'/autopilot/pose_command', geometry_msgs.PoseStamped,
            queue_size=1)
        self._control_cmd_pub = rospy.Publisher(
            quad_namespace+"/control_command", quadrotor_msgs.ControlCommand,
            queue_size=1)
        self._autopilot_feedback_sub = rospy.Subscriber(
            quad_namespace+'/autopilot/feedback',
            quadrotor_msgs.AutopilotFeedback, self.autopilot_feedback_cb)

        self._odometry_sub = rospy.Subscriber(
            quad_namespace+'/ground_truth/odometry', nav_msgs.Odometry,
            self.odometry_cb
        )

        self._connected = True

        # the main loop timer will be called for every 0.01 second (100 Hz)
        rospy.Timer( rospy.Duration(0.03), self.main_loop_callback)

    def disconnect_pub_sub(self, pub):
        if pub is not None:
            pub.unregister()
            pub = None

    def disconnect(self):
        self.disconnect_pub_sub(self._autopilot_feedback_sub)
        self.disconnect_pub_sub(self._arm_bridge_pub)
        self.disconnect_pub_sub(self._start_pub)
        self.disconnect_pub_sub(self._land_pub)
        self.disconnect_pub_sub(self._off_pub)
        self.disconnect_pub_sub(self._force_hover_pub)
        self.disconnect_pub_sub(self._go_to_pose_pub)
        self._connected = False
    
    def arm_bridge(self,):
        arm_message = std_msgs.Bool(True)
        self._arm_bridge_pub.publish(arm_message)

    def main_loop_callback(self, event):
        self._count += 1
        obs = self._obs_buf.pop()
        if obs:
            self._obs = obs
        else:
            self._obs = self._obs
        self._err_vxvy = np.array([self._obs[0] - 3, self._obs[1], self._obs[2]])

        self._int_vxvy = self._int_vxvy * 0.99 + self._err_vxvy

        obs_t = self._obs + list(self._act_prev)
        obs_t = np.reshape(obs_t, (1, -1))

        action = self.actor.step(obs_t, stochastic=False).numpy()[0]
        if self._count > 1:
            self._act_error = np.abs(action - self._act_prev)
        self._act_prev = action
        # publishing 
        control_command = quadrotor_msgs.ControlCommand()
        control_command.armed = True
        control_command.control_mode = 2 # Body rates
        control_command.collective_thrust = action[0]
        control_command.bodyrates.x = action[1]
        control_command.bodyrates.y = action[2]
        control_command.bodyrates.z = action[3]
        self._control_cmd_pub.publish(control_command)

        if self._count % 100 == 0:
            print(obs_t)
    
    def odometry_cb(self, msg):
        # call_time = rospy.Time.now()
        # header = msg.header
        # child_frame_id = msg.child_frame_id
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        linear_vel = msg.twist.twist.linear
        angular_vel = msg.twist.twist.angular
        if self._count % 100 == 0:
            print(position)

        obs = [position.z,
               linear_vel.x, linear_vel.y, linear_vel.z,
               quaternion.x, quaternion.y, quaternion.z, quaternion.w,
               angular_vel.x, angular_vel.y, angular_vel.z]

        self._obs_buf.push(obs)
        
    def autopilot_feedback_cb(self, msg):
        if not self._armed:
            self.arm_bridge()
        self._autopilot_feedback = msg
        self._autopilot_feedback_stamp = rospy.Time.now()

def main():
    rospy.init_node('run_mlp_controller', anonymous=True)

    # hacks
    obs_dim = 15
    act_dim = 4
    max_action = np.array([20.0, 6.0, 6.0, 6.0])
    min_action = np.array([1, -6.0, -6.0, -6.0])
    #
    actor_params = dict(
        learning_rate=3e-4,
        hidden_units=[256, 256],
        activation='relu'
    )

    load_dir = "/home/sysadmin/RPG_Flightmare/catkin_rpgq/src/rpg_gym/rpg_baselines/examples/run_quad/save_model/Quad_v1/sac-12-13-00-18-43/actor_net/weights_2000.h5"
    
    # create mlp actor and load trained weights
    actor = Actor(obs_dim, act_dim, max_action, min_action, **actor_params)
    actor.load_weights(load_dir)

    # create mlp_runner
    mlp_runner = RunMlPController(actor)

    mlp_runner.connect(quad_namespace="/hummingbird")
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    main()