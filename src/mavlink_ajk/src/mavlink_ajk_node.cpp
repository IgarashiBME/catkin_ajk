/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C++11
 * Robot Operating System Kinetic
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
**/

/* C++ standard library */
#include "stdio.h"
#include "errno.h"
#include "string.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "unistd.h"
#include "stdlib.h"
#include "fcntl.h"
#include "time.h"
#include "sys/time.h"
#include "arpa/inet.h"
#include "iostream"
#include "fstream"

/* ROS library */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"

/* ublox NavPVT custom ROS message */
#include "mavlink_ajk/NavPVT.h"
#include "mavlink_ajk/MAV_Mission.h"
#include "mavlink_ajk/MAV_Modes.h"
#include "look_ahead/Auto_Log.h"

/* mavlink library */
#include "mavlink.h"

//#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#define BUFFER_LENGTH 1025

using namespace std;

uint64_t microsSinceEpoch();

class Listener{
public:
    void gnss_callback(const mavlink_ajk::NavPVT::ConstPtr& msg);
    /* sanyokiki lat 34.500682  lon 133.558131
       yayoi     lat 35.716761  lon 139.761254  */
    int lat = 35.717736 * 10000000;  // latitude
    int lon = 139.759512 * 10000000;  // longitude
    //int lat = 34.500682 * 10000000;
    //int lon = 133.558131 * 10000000;
    int alt = 10000;  // altitude above elliposid
    int fix_type = 0;
    int satellites = 0; // number of satellites visible. If unknown, set to 255.

    void auto_log_callback(const look_ahead::Auto_Log::ConstPtr& msg);
    int current_seq = 0;
};

void Listener::gnss_callback(const mavlink_ajk::NavPVT::ConstPtr& msg){
    lat = msg->lat * 10000000;
    lon = msg->lon * 10000000;
    satellites = msg->numSV;

    switch(msg->fix_status){
        case 2:
            fix_type = GPS_FIX_TYPE_RTK_FIXED;
        case 1:
            fix_type = GPS_FIX_TYPE_RTK_FLOAT;
        case 0:
            fix_type = GPS_FIX_TYPE_NO_FIX;
    }
    //ROS_INFO("info [%f]", msg->lat);
}

void Listener::auto_log_callback(const look_ahead::Auto_Log::ConstPtr& msg){
    current_seq = msg->waypoint_seq;

    //ROS_INFO("info [%f]", msg->lat);
}

int main(int argc, char **argv){
    char target_ip[100];
	
    float position[6] = {};
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in gcAddr; 
    struct sockaddr_in locAddr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;
    int bytes_sent;
    mavlink_message_t mavmsg;
    uint16_t len;
    int i = 0;
    unsigned int temp = 0;
    unsigned int mission_total_seq = 0;
    unsigned int mission_seq = 0;
    int pre_mission_seq = -1;

    // mavlink mode
    uint16_t base_mode = 0; //MAV_MODE_GUIDED_DISARMED;
    uint16_t custom_mode = 0;

    // time interval
    uint64_t pre_heartbeat_time;
    int heartbeat_interval = 900000; //0.9 second

    uint64_t pre_request_time;
    int request_interval = 30000; // 0.05 second

    // Change the target ip if parameter was given
    strcpy(target_ip, "127.0.0.1");
    if (argc == 2){
        strcpy(target_ip, argv[1]);
    }

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14551);
	
    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
    if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    } 
	
    /* Attempt to make it non blocking */
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0){
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    gcAddr.sin_port = htons(14550);

    /* time setting for interval */
    pre_heartbeat_time = microsSinceEpoch();
    pre_request_time = microsSinceEpoch();

    /* ros intializer */
    ros::init(argc, argv, "mavlink_node");
    ROS_INFO("fake fcu start");
    ros::NodeHandle n;

    Listener listener;
    ros::Subscriber sub = n.subscribe("/navpvt", 10, &Listener::gnss_callback, &listener);
    ros::Subscriber auto_log = n.subscribe("/auto_log", 1, &Listener::auto_log_callback, &listener);

    ros::Publisher pub_mission = n.advertise<mavlink_ajk::MAV_Mission>("/mav/mission", 1000);
    mavlink_ajk::MAV_Mission mission_rosmsg;

    ros::Publisher pub_modes = n.advertise<mavlink_ajk::MAV_Modes>("/mav/modes", 1);
    mavlink_ajk::MAV_Modes modes_rosmsg;

    while (ros::ok()){
        /* time interval */        
        if (microsSinceEpoch() - pre_heartbeat_time > heartbeat_interval){
            /* print */
            //std::cout << microsSinceEpoch() - pre_time << std::endl;
            //std::cout << listener.current_seq << std::endl; 

            /*Send Heartbeat */
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, type, autopilot, 
            //                           base_mode, custom_mode, system_status);
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
            //                           MAV_MODE_GUIDED_ARMED, 3, MAV_STATE_STANDBY);
            mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                       base_mode, custom_mode, MAV_STATE_STANDBY);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            /* Send Status */
            mavlink_msg_sys_status_pack(1, 200, &mavmsg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));

            /* Send Local Position */
            mavlink_msg_local_position_ned_pack(1, 200, &mavmsg, microsSinceEpoch(), 
                                                position[0], position[1], position[2],
                                                position[3], position[4], position[5]);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send attitude */ 
            mavlink_msg_attitude_pack(1, 200, &mavmsg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send GPS */
            mavlink_msg_gps_raw_int_pack(1, 200, &mavmsg, 0, listener.fix_type, 
                                         listener.lat, listener.lon, listener.alt, 65535, 65535, 
                                         65535, 65535, listener.satellites);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            pre_heartbeat_time = microsSinceEpoch();

            /* publish ARM-DISARM ROS message */
            modes_rosmsg.base_mode = base_mode;
            modes_rosmsg.custom_mode = custom_mode;
            pub_modes.publish(modes_rosmsg);

            /* send mission_current */
            if (base_mode == MAV_MODE_GUIDED_ARMED){
                if (listener.current_seq+1 == mission_total_seq){
                    base_mode = MAV_MODE_GUIDED_DISARMED;
                }
                mavlink_msg_mission_current_pack(1, 200, &mavmsg, listener.current_seq);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
            }
        }

        /* Mission Request */
        if (mission_total_seq > 0 && microsSinceEpoch() - pre_request_time > request_interval){
            //printf("request\n");
            //std::cout << microsSinceEpoch() - pre_request_time << std::endl;
            mavlink_msg_mission_request_int_pack(1, 200, &mavmsg, 0, 0, mission_seq);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            pre_request_time = microsSinceEpoch();
        }

        /* receiver section */
        memset(buf, 0, BUFFER_LENGTH);
        recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

        if (recsize > 0){
            // Something received - print out all bytes and parse packet
            mavlink_message_t mavmsg;
            mavlink_status_t status;

            //printf("Bytes Received: %d\nDatagram: ", (int)recsize);

            for (i = 0; i < recsize; ++i){
                temp = buf[i];
                /* Packet received */
                //printf("%02x ", (unsigned char)temp);
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mavmsg, &status)){
                    // Packet decode
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                           mavmsg.sysid, mavmsg.compid, mavmsg.len, mavmsg.msgid);
                }
            }
            if (mavmsg.msgid == 44){
                mavlink_mission_count_t mavmc;
                printf("mission count was received\n");
                    
                mavlink_msg_mission_count_decode(&mavmsg, &mavmc);
                mission_total_seq = mavmc.count;
                mission_seq = 0;
                printf("%i\n", mission_total_seq);
            }

            /* mission receiver */
            if (mavmsg.msgid == 73){
                mavlink_mission_item_int_t mavmii;

                //printf("mission item was received\n");

                // decode MISSION_ITEM_INT message
                mavlink_msg_mission_item_int_decode(&mavmsg, &mavmii);
                double waypoint_x = mavmii.x/10000000.0;
                double waypoint_y = mavmii.y/10000000.0;
                    
                // print and output waypoint
                if (pre_mission_seq != mavmii.seq && mission_seq == mavmii.seq){
                    printf("%i, %i, %i, %.09f, %.09f\n", mavmii.seq, mission_total_seq, mavmii.command,
                           waypoint_x, waypoint_y);
                    fstream fs;
                    fs.open("/home/nouki/waypoint.csv", ios::out | ios::app);
                    fs << mission_seq << ",";
                    fs << fixed << setprecision(8) << waypoint_x << "," << waypoint_y << endl; 
                    fs.close();

                    // publish ROS message
                    mission_rosmsg.seq = mavmii.seq;
                    mission_rosmsg.total_seq = mission_total_seq;
                    mission_rosmsg.command = mavmii.command;
                    mission_rosmsg.latitude = waypoint_x;
                    mission_rosmsg.longitude = waypoint_y;
                    pub_mission.publish(mission_rosmsg);

                    pre_mission_seq = mavmii.seq;
                    mission_seq = mavmii.seq+1;
                }

                // if mission sequence is end, send mission ack
                if (mission_seq == mission_total_seq){
                    mavlink_msg_mission_ack_pack(1, 200, &mavmsg, 0, 0, MAV_MISSION_TYPE_MISSION);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

                    mission_total_seq = 0;
                }
            }
            /* SET_MODE decoder */
            if (mavmsg.msgid == 11){
                mavlink_set_mode_t mavsm;

                // decode SET_MODE message
                mavlink_msg_set_mode_decode(&mavmsg, &mavsm);
                printf("%i, %i, %i", mavsm.custom_mode, mavsm.target_system, mavsm.base_mode);
                custom_mode = mavsm.custom_mode;
                base_mode = mavsm.base_mode;
            }

            /* COMMAND_LONG decoder */
            if (mavmsg.msgid == 76){
                mavlink_command_long_t mavcl;

                // decode SET_MODE message
                mavlink_msg_command_long_decode(&mavmsg, &mavcl);
                printf("%i, %f", mavcl.command, mavcl.param1);

                /* Send COMMAND_ACK */ 
                if (mavcl.command == MAV_CMD_COMPONENT_ARM_DISARM && mavcl.param1 == 1.0){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, MAV_CMD_COMPONENT_ARM_DISARM,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    base_mode = MAV_MODE_GUIDED_ARMED;
                }
                if (mavcl.command == MAV_CMD_COMPONENT_ARM_DISARM && mavcl.param1 == 0.0){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, MAV_CMD_COMPONENT_ARM_DISARM,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    base_mode = MAV_MODE_GUIDED_DISARMED;
                }
                if (mavcl.command == MAV_CMD_REQUEST_PROTOCOL_VERSION){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                }
                if (mavcl.command == MAV_CMD_MISSION_START){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                }
            }                
            //printf("\n");
        }
        memset(buf, 0, BUFFER_LENGTH);
        usleep(10000); // Sleep 10 msec
        ros::spinOnce();
    }
}

uint64_t microsSinceEpoch(){
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}
