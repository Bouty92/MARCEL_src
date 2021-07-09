#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from rover_ctrl.msg import Rov_ctrl, Joints_info
from std_msgs.msg import String
from robotiq_ft_sensor.srv import sensor_accessor
from geometry_msgs.msg import WrenchStamped
import numpy as np


angle_amplitude = 35 #°
angle_margin = 0.5 #°
angle_rate = 10 #°/s
angle_rate_up_ramp_duration = 1 #s
angle_rate_down_ramp_duration = 1 #s

torque_start  = 15 #N.m
torque_middle = 25 #N.m
torque_end    = 15 #N.m
torque_offset = 10 #°
torque_margin = 1 #N.m

FT_SERIAL_NUMBER_REAR = "  F-31952"



# Look for the ID of the rear FT sensor:
rear_ft_sensor_id = -1
for i in [ 0, 1 ] :
	sensor_svr = rospy.ServiceProxy( 'ft_request_%d' % i, sensor_accessor )
	#response = sensor_svr( sensor_accessor.COMMAND_GET_SERIAL_NUMBER, '' )
	response = sensor_svr( 1, '' )
	serial_number = response.res
	if serial_number == FT_SERIAL_NUMBER_REAR :
		rear_ft_sensor_id = i
if rear_ft_sensor_id < 0 :
	#rospy.logfatal( 'Failed to identify the rear FT sensor' )
	print( 'Failed to identify the rear FT sensor' )
	exit( 0 )


cmd = Rov_ctrl()
cmd.engaged = True
cmd.crawling_mode = True
cmd.rate_mode = True
cmd.speed = 0.0
cmd.torque = 0.0


actual_angle = 0.0
deduced_torque = 0.0
actual_torque = 0.0
torque_offset = 0.0

def callback_joints( msg ):
	global actual_angle, deduced_torque

	actual_angle = msg.cj_angle
	deduced_torque = msg.sea_torque


def callback_string( msg ):
	print( msg.data )


def callback_torque( msg ):
	global actual_torque, torque_offset

	actual_torque = -msg.wrench.torque.y - torque_offset


pub = rospy.Publisher( 'nav_ctrl', Rov_ctrl, queue_size=10 )
rospy.init_node( 'crawling_gait', disable_signals=True )
joints_info = rospy.Subscriber( 'joints_info', Joints_info, callback_joints )
string_info = rospy.Subscriber( 'nav_info_string', String, callback_string )
ft_sub = rospy.Subscriber( 'ft_wrench_%d' % rear_ft_sensor_id, WrenchStamped, callback_torque )

freq = rospy.get_param( '~freq', 10 )
period = rospy.Rate( freq )


# Calibrate torque measurements:
while actual_torque == 0 :
	period.sleep()
torque_offset = actual_torque
period.sleep()



def publish_cmd() :
	cmd.header.stamp = rospy.Time.now()
	pub.publish( cmd )
	print rospy.get_time(), cmd.steer, cmd.torque, actual_angle, deduced_torque, actual_torque


def get_desired_torque( rate_sign ) :

	if rate_sign*actual_angle < -torque_offset :
		torque_edge = torque_start
		offset_sign = 1
	else :
		torque_edge = torque_end
		offset_sign = -1

	desired_torque = torque_middle - ( torque_middle - torque_edge )*abs( actual_angle + rate_sign*torque_offset )/( angle_amplitude - offset_sign*torque_offset )

	return -rate_sign*desired_torque


def steer( target_angle ) :

	if ( -1 if target_angle < 0 else 1 )*actual_angle + angle_margin >= abs( target_angle ) :
		return

	rate_sign = 1 if target_angle > actual_angle else -1


	# Get close to the desired torque before starting to move:
	cmd.torque = get_desired_torque( rate_sign )

	while ( -1 if cmd.torque < 0 else 1 )*deduced_torque + torque_margin < abs( cmd.torque ) :

		publish_cmd()
		period.sleep()


	# Gradually increase the steering rate:
	for rate in np.linspace( 0, rate_sign*angle_rate, max( 2, int( angle_rate_up_ramp_duration*freq ) ) )[1:] :

		if ( -1 if target_angle < 0 else 1 )*actual_angle + angle_margin >= abs( target_angle ) :
			break

		cmd.steer = rate
		cmd.torque = get_desired_torque( rate_sign )

		publish_cmd()
		period.sleep()


	# Move at the maximum steering rate until the target angle is reached:
	while ( -1 if target_angle < 0 else 1 )*actual_angle + angle_margin < abs( target_angle ) :

		cmd.torque = get_desired_torque( rate_sign )

		publish_cmd()
		period.sleep()


	# Gradually decrease the steering rate:
	for rate in np.linspace( rate_sign*angle_rate, 0, max( 2, int( angle_rate_down_ramp_duration*freq ) ) )[1:] :

		cmd.steer = rate
		cmd.torque = get_desired_torque( rate_sign )

		publish_cmd()
		period.sleep()


try :

	steer( angle_amplitude )

	while not rospy.is_shutdown() :

		steer( -angle_amplitude )

		steer( angle_amplitude )


except KeyboardInterrupt :
	cmd.engaged = False

	publish_cmd()

	rospy.signal_shutdown( 'Received SIGINT' )
