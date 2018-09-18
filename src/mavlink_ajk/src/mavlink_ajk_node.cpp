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

/* ROS library */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/* ublox NavPVT custom ROS message */
#include "mavlink_ajk/NavPVT.h"

/* mavlink library */
#include "mavlink.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();

class Listener{
public:
    void gnss_callback(const mavlink_ajk::NavPVT::ConstPtr& msg);
    /* sanyokiki lat 34.85586  lon 133.1018045
       yayoi     lat 35.716761 lon 139.761254  */
    int lat = 35.716761 * 10000000;
    int lon = 139.761254 * 10000000;
    int alt = 10000;
    int fix_type = 0;
    int satellites = 8; // number of satellites visible. If unknown, set to 255.
};

void Listener::gnss_callback(const mavlink_ajk::NavPVT::ConstPtr& msg){
    lat = msg->lat;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mavlink_node");
    ROS_INFO("fake fcu start");
    ros::NodeHandle n;

    Listener listener;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, &Listener::gnss_callback, &listener);
    
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

    while (ros::ok()){
        ros::spinOnce();
        /*Send Heartbeat */
        // https://github.com/mavlink/c_library_v1/blob/3da9db30f3ea7fe8fa8241a74ab343b9971e7e9a/common/common.h#L166
        // Q: Change MANUAL_ARMED Mode
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
		
        memset(buf, 0, BUFFER_LENGTH);

        /* Send GPS */
        mavlink_msg_gps_raw_int_pack(1, 200, &mavmsg, 0, GPS_FIX_TYPE_RTK_FIXED, 
                                     listener.lat, listener.lon, listener.alt, 65535, 65535, 
                                     65535, 65535, listener.satellites);
        len = mavlink_msg_to_send_buffer(buf, &mavmsg);
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
 
        /* receiver section */
        recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

        if (recsize > 0){
            // Something received - print out all bytes and parse packet
            mavlink_message_t mavmsg;
            mavlink_status_t status;

            printf("Bytes Received: %d\nDatagram: ", (int)recsize);

            for (i = 0; i < recsize; ++i){
                temp = buf[i];
                printf("%02x ", (unsigned char)temp);
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mavmsg, &status)){
                    // Packet received
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                           mavmsg.sysid, mavmsg.compid, mavmsg.len, mavmsg.msgid);
                }
            }
                if (mavmsg.msgid == 44){
                    printf("mission count was received\n");
                    printf("%x \n", (unsigned char)buf[6]);
                    mavlink_msg_mission_request_int_pack(1, 1, &mavmsg, 255, 0, 0);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                        sizeof (struct sockaddr_in));	
                }

                printf("\n");
        }
        memset(buf, 0, BUFFER_LENGTH);
        ROS_INFO("info [%f]", listener.lat);
        sleep(1); // Sleep one second
    }
}

uint64_t microsSinceEpoch(){
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
} 
