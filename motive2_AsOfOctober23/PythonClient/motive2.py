# Getting rigid body data from motive 2.0
# Code modified based on NatNetClient 
# Yixiao Liu 07/06/2023

import socket
import struct
from utils import *
import sys
import MoCapData
import math

# address for the robots 
address_id_dict = {1:"192.168.1.71",
                   2:"192.168.1.72", 
                   3:"192.168.1.73", 
                   4:"192.168.1.74", 
                   5:"192.168.1.75",
                   6:"192.168.1.76", 
                   7:"192.168.1.77", 
                   8:"192.168.1.78", 
                   9:"192.168.1.70",
                   10:"192.168.1.69",
                   11:"192.168.1.68"}

# Data Port Set in Optitrack Streaming Properties
opti_port = 1511

# port for the robot
udp_port = 3500

# motive ip
multicast_address = "239.255.42.99"
# opti track computer ip
local_ip_address = "192.168.0.99"


##################################################################
##################################################################
# DO NOT EDIT ANYTHING BENEATH THIS LINE!!!!
##################################################################
##################################################################

# Create structs for reading various object types to speed up parsing.
Vector3 = struct.Struct( '<fff' )
Quaternion_ = struct.Struct( '<ffff' ) # modify to Quaternion_ to avoid comfusion with the Quaternion class 
FloatValue = struct.Struct( '<f' )

# helper functions from the natnet client to unpack the data
def __unpack_frame_prefix_data(data):
        offset = 0
        # Frame number (4 bytes)
        frame_number = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        return offset, frame_number

def __unpack_marker_set_data( data, packet_size):
        offset = 0
        # Marker set count (4 bytes)
        marker_set_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4

        for i in range( 0, marker_set_count ):

            # Marker count (4 bytes)
            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4

            for j in range( 0, marker_count ):
                offset += 12

        # Unlabeled markers count (4 bytes)
        unlabeled_markers_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4

        for i in range( 0, unlabeled_markers_count ):
            offset += 12
        return offset

def __unpack_rigid_body_data( data, packet_size, major, minor):
    rigid_body_data = []
    offset = 0
    # Rigid body count (4 bytes)
    rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
    offset += 4

    for i in range( 0, rigid_body_count ):
        offset_tmp, rigid_body = __unpack_rigid_body( data[offset:], major, minor, i )
        offset += offset_tmp
        # only add to the list when the tracking is valid
        if (rigid_body.tracking_valid):
            rigid_body_data.append(rigid_body)

    return offset, rigid_body_data

def __unpack_rigid_body(data, major, minor, rb_num):
    offset = 0

    # ID (4 bytes)
    new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
    offset += 4

    # Position and orientation
    pos = Vector3.unpack( data[offset:offset+12] )
    offset += 12

    rot = Quaternion_.unpack( data[offset:offset+16] )
    offset += 16
    rigid_body = MoCapData.RigidBody(new_id, pos, rot)
    quat = Quaternion()
    quat.x, quat.y, quat.z,quat.w, = rot
    # if(new_id == 3):
        # print(quat.w, quat.x, quat.y, quat.z)
    angles = optiquat2euler(quat.vec())
    rigid_body.theta = angles.theta

    # RB Marker Data ( Before version 3.0.  After Version 3.0 Marker data is in description )
    if( major < 3  and major != 0) :
        # Marker count (4 bytes)
        marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        marker_count_range = range( 0, marker_count )

        rb_marker_list=[]
        for i in marker_count_range:
            rb_marker_list.append(MoCapData.RigidBodyMarker())

        # Marker positions
        for i in marker_count_range:
            pos = Vector3.unpack( data[offset:offset+12] )
            offset += 12
            rb_marker_list[i].pos=pos


        if major >= 2:
            # Marker ID's
            for i in marker_count_range:
                new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
                offset += 4
                rb_marker_list[i].id=new_id

            # Marker sizes
            for i in marker_count_range:
                size = FloatValue.unpack( data[offset:offset+4] )
                offset += 4
                rb_marker_list[i].size=size

        for i in marker_count_range:
            rigid_body.add_rigid_body_marker(rb_marker_list[i])
    if major >= 2 :
        marker_error, = FloatValue.unpack( data[offset:offset+4] )
        offset += 4
        rigid_body.error = marker_error

    # Version 2.6 and later
    if ( ( major == 2 ) and ( minor >= 6 ) ) or major > 2 :
        param, = struct.unpack( 'h', data[offset:offset+2] )
        tracking_valid = ( param & 0x01 ) != 0
        offset += 2
        is_valid_str='False'
        if tracking_valid:
            is_valid_str = 'True'
        if tracking_valid:
            rigid_body.tracking_valid = True
        else:
            rigid_body.tracking_valid = False

    return offset, rigid_body


if __name__ == '__main__':
    # Create multicast socket for getting optitrack data
    s = socket.socket( socket.AF_INET,     # Internet
                                  socket.SOCK_DGRAM,
                                  0)    # UDP
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(multicast_address) + socket.inet_aton(local_ip_address))
    # bind multicast
    try:
        s.bind( (local_ip_address, opti_port) )
        print("successfully bind multicast")
    except socket.error as msg:
        print("ERROR: data socket error occurred:\n%s" %msg)
        print("  Check Motive/Server mode requested mode agreement.  You requested Multicast ")
        s = None
    if s is None:
        sys.exit(1)
    
    # Initialize UDP Socket for sending to robot
    udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 64k buffer size
    recv_buffer_size=64*1024
    data=bytearray(0)

    # this is determined by running the original natnet client and print out major and minor version
    major = 3
    minor = 1
    
    while True:
        try:
            data, addr = s.recvfrom( recv_buffer_size )
        except socket.error as msg:
            print("shutting down")
        if len(data) > 0:
            message_id = int.from_bytes( data[0:2], byteorder='little' )
            packet_size = int.from_bytes( data[2:4], byteorder='little' )
            #skip the 4 bytes for message ID and packet_size
            offset = 4
            if message_id == 7 : #NAT_FRAMEOFDATA
                data = memoryview( data )
                rel_offset = 0

                #Frame Prefix Data, only care about the offset here
                rel_offset, frame_number = __unpack_frame_prefix_data(data[offset:])
                offset += rel_offset

                # Marker Set Data, only care about the offset here
                rel_offset =__unpack_marker_set_data(data[offset:], (packet_size - offset))
                offset += rel_offset

                # Rigid Body Data
                rel_offset, rigid_body_data = __unpack_rigid_body_data(data[offset:], (packet_size - offset),major, minor)
                offset += rel_offset
                rigid_body_count =len(rigid_body_data)
                if rigid_body_count > 0:
                    if (frame_number % 25 == 0):
                            print("\n")
                            print("id        frame number       x          z          theta      ")
                for rigid_body in rigid_body_data:
                    x = rigid_body.pos[0]
                    z = rigid_body.pos[2]
                    theta = rigid_body.theta *180 / math.pi # need to convert to degrees for it to be conpatible with the C code
                    id_rb = rigid_body.id_num
                    udp_message = struct.pack('fffff', x,z,theta,id_rb,frame_number) #pack udp message
                    # only send the data when the id is in the dictionary to avoid key error
                    try:
                        udpsock.sendto(udp_message, (address_id_dict[id_rb], udp_port)) #send udp message
                    except:
                        pass
                    if (frame_number % 25 == 0):
                        print("%.0f         %.0f          %.3f       %.3f       %.3f" % (id_rb, frame_number,x, z, theta))
