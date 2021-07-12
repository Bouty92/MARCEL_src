#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
from robotiq_ft_sensor.srv import sensor_accessor
from geometry_msgs.msg import WrenchStamped
import sys

FT_SERIAL_NUMBER_FRONT = "  F-31951"
FT_SERIAL_NUMBER_REAR  = "  F-31952"



# Look for the ID of both FT sensor:
ft_sensor_ids = []
for i in [ 0, 1 ] :
	sensor_svr = rospy.ServiceProxy( 'ft_request_%d' % i, sensor_accessor )
	#response = sensor_svr( sensor_accessor.COMMAND_GET_SERIAL_NUMBER, '' )
	response = sensor_svr( 1, '' )
	serial_number = response.res
	if serial_number == FT_SERIAL_NUMBER_FRONT :
		ft_sensor_ids.insert( 0, i )
	elif serial_number == FT_SERIAL_NUMBER_REAR :
		ft_sensor_ids.append( i )
if len( ft_sensor_ids ) < 2 :
	#rospy.logfatal( 'Failed to identify both FT sensor' )
	print( 'Failed to identify both FT sensor' )
	exit( 0 )



fz = [ 0.0, 0.0 ]
fz_offset = [ 0.0, 0.0 ]


def ft_callback( msg, i ) :
	global fz, fz_offset

	fz[i] = -msg.wrench.force.z - fz_offset[i]


rospy.init_node( 'force_feedback', disable_signals=True )
for i in [ 0, 1 ] :
	ft_sub = rospy.Subscriber( 'ft_wrench_%d' % ft_sensor_ids[i], WrenchStamped, ft_callback, i )

freq = rospy.get_param( '~freq', 10 )
period = rospy.Rate( freq )


# Calibrate torque measurements:
while fz[0] == 0 and fz[1] == 0 :
	period.sleep()
for i in [ 0, 1 ] :
	fz_offset[i] = fz[i]
period.sleep()




try :

	while not rospy.is_shutdown() :

		print rospy.get_time(), fz[0], fz[1]
		sys.stdout.flush()

		period.sleep()


except KeyboardInterrupt :
	rospy.signal_shutdown( 'Received SIGINT' )
