#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, EJ Kreinar
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('novatel_gps_driver')
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import TwistStamped

import serial, string, math, time, calendar, struct


#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)

#Novatel Provided Function:
#CRC helping function
def CRC32Value(i):
    CRC = i
    for idx in range(8):
        if (CRC & 1):
            CRC = (CRC >> 1) ^ 0xEDB88320
        else:
            CRC = CRC >> 1
    return CRC

#Novatel Provided Function:
#Calculate the CRC for supplied data (bytearray)
def CalculateBlockCRC32(data):
    CRC = 0;
    for idx in range(len(data)):
        tmp1 = (CRC >> 8) & 0x00FFFFFFFF
        tmp2 = CRC32Value( (CRC^data[idx]) & 0xFF)
        CRC  = tmp1 ^ tmp2
    return CRC


class NovatelParser:
    """
        Parses a binary Novatel Message
    """
    def __init__(self):
        """ Initialize the NovatelParser """
        self.hdr_len     = 0;
        self.hdr_msgID   = 0;
        self.hdr_msgLen  = 0;
        self.hdr_week    = 0;
        self.hdr_msow    = 0;
        self.hdr_rcvStat = 0;

        self.BESTPOS = 42;
        self.BESTVEL = 99;

    def VerifyChecksum(self, data, CRC):
        """ Verify the Checksum, return bool """
        chk = struct.unpack('<L', CRC)
        checksum = CalculateBlockCRC32(bytearray(data))
        return (chk[0] == checksum)
    
    def ParseHeader(self, data):
        """ Read the Header, return bool """
        header = struct.unpack('<BHbBHHBBHlLHH',data);
        self.hdr_len    = header[0]
        self.hdr_msgID  = header[1]
        self.hdr_msgLen = header[4]
        self.hdr_week   = header[8]
        self.hdr_msow   = header[9]
        self.hdr_rcvStat= header[10]
        return (len(data) == self.hdr_len-3)

    def ParseTimeRef(self, timeMsg):
        """ Populate timeMsg from the header info
            timeMsg: ROS message TimeReference
        """
        timeMsg.time_ref = 0;
        return True
    
    def ParseBestPos(self, data, navMsg):
        """ Parse a BestPos message, populate navMsg
            data: string
            navMsg: ROS message NavSatFix
        """
        bestPos = struct.unpack('<lldddflffffffBBBBBBBB',msg)
        solStat, posType, lat    , lon   , hgt    , und   , datum , \
        latStd , lonStd , hgtStd , stdId , diffAge, solAge, SVs   , \
        slnSVs , obs    , mult   , rsv1  , extStat, rsv2  , sigMsk  = bestPos
        rospy.loginfo(bestPos)

        # Populate the NavSatFix message
        navMsg.latitude  = lat
        navMsg.longitude = lon
        navMsg.altitude  = hgt
        #TODO: Replace the covariance
        navMsg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # Populate the Status 
        #TODO: Send a string that represents the status
        navData.status.service = NavSatStatus.SERVICE_GPS
        if solStat == 0 && posType == 50:   # NARROW_INT
            navMsg.status.status = NavSatStatus.STATUS_FIX 
        elif solStat == 0 && posType == 18: # WAAS
            navMsg.status.status = NavSatStatus.STATUS_SBAS_FIX
        else: # catch the rest of the states
            navMsg.status.status = NavSatStatus.STATUS_NO_FIX

        return True

    def ParseBestVel(self, data, velMsg):
        """ Parse a BestPos message, populate velMsg
            data: string
            velMsg: ROS message TwistStamped
        """
        bestVel = struct.unpack('<llffdddf',msg)
        solStat, velType, lat, age, horSpd, trkAngle, vertSpeed, rsv1 = bestVel
        rospy.loginfo(bestVel)
        #print bestVel

        #TODO: Change the velocity representation if we're going to use it... I dont think this is what we want
        #       I would prefer to send the speed over ground, and then do the math somewhere else
        # Populate the gps's twist message
        velMsg.twist.linear.x = float(horSpd)*0.514444444444*math.sin(math.radians(float(trkAngle)))
        velMsg.twist.linear.y = float(horSpd)*0.514444444444*math.cos(math.radians(float(trkAngle)))

        return True



if __name__ == "__main__":
    #ROS init
    rospy.init_node('novatel_gps_driver')
    gpsPub = rospy.Publisher('gps_fix', NavSatFix)
    gpsVelPub = rospy.Publisher('gps_vel',TwistStamped)
    gpsTimePub = rospy.Publisher('time_reference', TimeReference)
    #Init GPS port
    GPSport = rospy.get_param('~port','/dev/ttyUSB0')
    GPSrate = rospy.get_param('~baud',57600)
    frame_id = rospy.get_param('~frame_id','gps')
    if frame_id[0] != "/":
       frame_id = addTFPrefix(frame_id)

    time_ref_source = rospy.get_param('~time_ref_source', frame_id)
    navData = NavSatFix()
    gpsVel = TwistStamped()
    gpstime = TimeReference()
    gpstime.source = time_ref_source
    navData.header.frame_id = frame_id
    gpsVel.header.frame_id = frame_id
    GPSLock = False

    parser = NovatelParser()
    
    try:
        GPS = serial.Serial(port=GPSport,baudrate=GPSrate,timeout=.01)
        #Read in GPS data
        sync0 = '\x00'; sync1 = '\x00'; sync2 = '\x00';
        while not rospy.is_shutdown():
            # READ UNTIL SYNC
            data  = GPS.read(1)
            sync2 = sync1; sync1 = sync0; sync0 = data;
            sync  = sync2+sync1+sync0;
            match = '\xAA\x44\x12'
            if sync != match:
                continue
            else:
                rospy.loginfo("Beginning new message")

            # READ HEADER
            header      = GPS.read(25)
            if (not parser.ParseHeader(header)):
                rospy.logwarn("Packet Failed: Unexpected header size")
                continue

            # READ MESSAGE
            msg = GPS.read(parser.hdr_msgLen)
            if (len(msg) != parser.hdr_msgLen):
                rospy.loginfo("Packet Failed: Message length unexpected")
                continue

            # READ CRC
            chk = GPS.read(4)
            if (not parser.VerifyChecksum(sync+header+msg,chk)):
                rospy.logwarn("Packet Failed: CRC Did not Match")
                continue

            # PARSE MESSAGE
            timeNow = rospy.get_rostime()
            gpstime.header.stamp = timeNow
            if parser.hdr_msgID == parser.BESTPOS:
                #BESTPOS message
                navData.header.stamp = timeNow
                parser.ParseBestPos(msg,navData)
                parser.ParseTimeRef(gpstime)
            
                # Publish navData and time reference
                gpsPub.publish(navData)
                gpsTimePub.publish(gpstime)

            elif parser.hdr_msgID == 99:
                #BESTVEL message
                gpsVel.header.stamp = timeNow
                parser.ParseBestVel(msg,gpsVel)

                # Publish gpsVel
                gpsVelPub.publish(gpsVel)

            else:
                rospy.logwarn("Novatel message ID not recognized. BESTPOS and BESTVEL supported only")

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial port
