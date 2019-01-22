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

/* ublox NavPVT custom ROS message */
#include "mavlink_ajk/NavPVT.h"

/* mavlink library */
#include "mavlink.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

using namespace std;

uint64_t microsSinceEpoch();
int CheckReceivable(int fd);

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
    int satellites = 12; // number of satellites visible. If unknown, set to 255.
};

void Listener::gnss_callback(const mavlink_ajk::NavPVT::ConstPtr& msg){
    lat = msg->lat * 10000000; //
    lon = msg->lon * 10000000;
    satellites = msg->numSV;
    
    //ROS_INFO("info [%f]", msg->lat);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mavlink_node");
    ROS_INFO("fake fcu start");
    ros::NodeHandle n;

    Listener listener;
    ros::Subscriber sub = n.subscribe("/navpvt", 10, &Listener::gnss_callback, &listener);
    
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
    uint64_t pre_time = 0;
    int time_interval = 900000; //0.5 second
    int recv_check;

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

    pre_time = microsSinceEpoch();
    while (ros::ok()){
        ros::spinOnce();

        /* time interval */        
        if (microsSinceEpoch() - pre_time > time_interval){
        //if (recv_check == 1){
            /* time print */
            //std::cout << microsSinceEpoch() - pre_time << std::endl;

            /*Send Heartbeat */
            // https://github.com/mavlink/c_library_v1/blob/3da9db30f3ea7fe8fa8241a74ab343b9971e7e9a/common/common.h#L166
            // mavlink_msg_heartbeat_pack(1, 1, &mavmsg, type, autopilot, 
            //                           base_mode, custom_mode, system_status);
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
            //                           MAV_MODE_GUIDED_DISARMED, 3, MAV_STATE_STANDBY);
            mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
                                       MAV_MODE_GUIDED_ARMED, 3, MAV_STATE_STANDBY);
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
            mavlink_msg_gps_raw_int_pack(1, 200, &mavmsg, 0, GPS_FIX_TYPE_RTK_FIXED, 
                                         listener.lat, listener.lon, listener.alt, 65535, 65535, 
                                         65535, 65535, listener.satellites);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            pre_time = microsSinceEpoch();
        }

        // Mission Request
        if (mission_total_seq > 0 && pre_mission_seq != mission_seq){
            mavlink_msg_mission_request_int_pack(1, 200, &mavmsg, 0, 0, mission_seq);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
            pre_mission_seq = mission_seq;
        }

        /* receiver section */
        memset(buf, 0, BUFFER_LENGTH);
        //recv_check = CheckReceivable(sock);
        //recv_check == 1;
        //std::cout << recv_check << std::endl; //receive check print

        //if (recv_check == 1){
            recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

            if (recsize > 0){
                // Something received - print out all bytes and parse packet
                mavlink_message_t mavmsg;
                mavlink_status_t status;

                printf("Bytes Received: %d\nDatagram: ", (int)recsize);

                for (i = 0; i < recsize; ++i){
                    temp = buf[i];
                    /* Packet received */
                    printf("%02x ", (unsigned char)temp);
                    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mavmsg, &status)){
                        // Packet decode
                        //printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                        //       mavmsg.sysid, mavmsg.compid, mavmsg.len, mavmsg.msgid);
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

                /* MISSION_ITEM_INT decoder */
                if (mavmsg.msgid == 73){
                    mavlink_mission_item_int_t mavmii;

                    //printf("mission item was received\n");

                    // decode MISSION_ITEM_INT message
                    mavlink_msg_mission_item_int_decode(&mavmsg, &mavmii);
                    float waypoint_x = mavmii.x/10000000.0;
                    float waypoint_y = mavmii.y/10000000.0;
                    printf("%i, %i, %i, %f, %f\n", mavmii.seq, mission_total_seq, mavmii.command, waypoint_x, waypoint_y);
                    
                    // output waypoint
                    if (mavmii.command == 16){
                        fstream fs;
                        fs.open("/home/nouki/waypoint.csv", ios::out | ios::app);
                        fs << mission_seq << ",";
                        fs << fixed << setprecision(6) << waypoint_x << "," << waypoint_y << endl; 
                        fs.close();
                    }

                    // next waypoint
                    mission_seq = mavmii.seq+1;

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
                }

                /* COMMAND_LONG decoder */
                if (mavmsg.msgid == 76){
                    mavlink_command_long_t mavcl;

                    // decode SET_MODE message
                    mavlink_msg_command_long_decode(&mavmsg, &mavcl);
                    //printf("%i", mavcl.command);
                }
                
                printf("\n");
            }

        //}
        memset(buf, 0, BUFFER_LENGTH);

        usleep(10000); // Sleep 10 msec
    }
}

uint64_t microsSinceEpoch(){
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}

int CheckReceivable(int fd){
    fd_set fdset;
    int re;
    struct timeval timeout;
     
    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

    /* timeout */
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;

    /* check receivable */
    re = select(fd+1, &fdset, NULL, NULL, &timeout);
    return re;
}
