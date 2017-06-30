/*******************************************************************************
	*    @ file       APM_MAVLink.c 
	*    @ author     Yu-Chen,Ke
	*    @ version     
	*    @ date       12/08/2016
	*    @ brief       
*******************************************************************************/

// compiler header
	#include <string.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <errno.h>
	#include <termios.h>
	#include <stdio.h>
	#include <sched.h>
	#include <stdlib.h>
	#include <math.h>
	#include <pthread.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/poll.h>
	#include <sys/wait.h>
	#include <signal.h>
	#include <sys/resource.h>
	#include <time.h>
	#include <sys/ioctl.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include "mavlink/v1.0/common/mavlink.h" 	// for MAVlink

// define print text color
	#define NONE                 "\e[0m"
	#define BLACK                "\e[0;30m"
	#define L_BLACK              "\e[1;30m"
	#define RED                  "\e[0;31m"
	#define L_RED                "\e[1;31m"
	#define GREEN                "\e[0;32m"
	#define L_GREEN              "\e[1;32m"
	#define BROWN                "\e[0;33m"
	#define YELLOW               "\e[1;33m"
	#define BLUE                 "\e[0;34m"
	#define L_BLUE               "\e[1;34m"
	#define PURPLE               "\e[0;35m"
	#define L_PURPLE             "\e[1;35m"
	#define CYAN                 "\e[0;36m"
	#define L_CYAN               "\e[1;36m"
	#define GRAY                 "\e[0;37m"
	#define WHITE                "\e[1;37m"
	#define BOLD                 "\e[1m"
	#define UNDERLINE            "\e[4m"
	#define BLINK                "\e[5m"
	#define REVERSE              "\e[7m"
	#define HIDE                 "\e[8m"
	#define CLEAR                "\e[2J"
	#define CLRLINE              "\r\e[K"

// define programme value
	#define printDataText "all"
	#define PrintFileName "PrintText.txt"
	#define RS_DEVICE_APM "/dev/ttyUSB0"
	#define BAUDRATE_APM  57600
	#define NET_HOST	  "203.66.168.239"
	#define NET_PORT	  9876
	#define false         0
	#define true          1
	#define debug		  false
	#define RDD			  false
	#define r2d 		  57.295779513082320876798154814105
	#define d2r 		  0.01745329251994329576923690768489
	#define ft2m 		  0.3047999995367040007042099189296

// Declared programme value
    FILE *printFile;
    int APM_Serial;
    int ExitRun = true;
	int shootout = false;
    pthread_mutex_t  mutex_APM;
    pthread_mutex_t  mutex_UDP;
    pthread_mutex_t  mutex_FILE;
	pthread_cond_t   trigger_APM;
	pthread_cond_t   trigger_FILE;

// Declared MAVLINK value
    unsigned char MAVLINK_MESSAGE_CRCS_ARRAY[256] = MAVLINK_MESSAGE_CRCS;
    unsigned char MAV_sysid = 0;
    unsigned char MAV_compid = 0;
    unsigned char buf[MAVLINK_MAX_PACKET_LEN];
	unsigned char custom_mode = 0;
	unsigned char system_status = 0;
	unsigned char battery_remaining = 0;
	unsigned char fix_type = 0;

    int MAVLINK_MESSAGE_LENGTHS_ARRAY[256] = MAVLINK_MESSAGE_LENGTHS;
	int HeartBeat = 0;
	int time_boot_ms = 0; 

	double accX = 0, accY = 0, accZ = 0;
	double gyroX = 0, gyroY = 0, gyroZ = 0;
	double magX = 0, magY = 0, magZ = 0;
    double roll = 0, pitch = 0, yaw = 0;
	double lat = 0, lon = 0, alt = 0;

	mavlink_message_t msg;
	mavlink_heartbeat_t             sys_heartbeat;          // #00
	mavlink_sys_status_t            sys_status;             // #01
	mavlink_system_time_t 			system_time;			// #02
	mavlink_gps_raw_int_t           gps_raw_int;			// #24
	mavlink_scaled_imu_t            scaled_imu;     		// #26
	mavlink_raw_imu_t               raw_imu;  				// #27
	mavlink_attitude_t              attitude;    			// #30
	mavlink_global_position_int_t   global_position_int;    // #33
	mavlink_rc_channels_raw_t 		rc_channels_raw;		// #35
	mavlink_servo_output_raw_t 		servo_output_raw; 		// #36
	mavlink_mission_item_t          mission_item;           // #39
	mavlink_mission_request_t       mission_request;        // #40
	mavlink_mission_set_current_t	mission_set_current;	// #41
	mavlink_mission_current_t       mission_current;        // #42
	mavlink_mission_request_list_t	mission_request_list;	// #43
	mavlink_mission_count_t         mission_count;          // #44
	mavlink_mission_clear_all_t		mission_clear_all;		// #45
	mavlink_mission_item_reached_t 	mission_item_reached;	// #46
	mavlink_mission_ack_t           mission_ack;		    // #47
	mavlink_nav_controller_output_t nav_controller_output;  // #62
	mavlink_vfr_hud_t               vfr_hud;                // #74
	mavlink_highres_imu_t           highres_imu;			// #105
	mavlink_radio_status_t          radio_status;			// #109	

// programme's library 
	int setSerialport(int serial, int baudrate)
	{
	    struct termios tio;
	    bzero(&tio,sizeof(tio));
	    tio.c_cflag = baudrate|CS8|CLOCAL|CREAD;
	    tio.c_iflag = IGNBRK|IGNPAR;
	    tio.c_oflag = 0;
	    tio.c_lflag = 0;
	    tio.c_cc[VMIN] = 1;
	    tcflush(serial,TCIOFLUSH);
	    
	    if (tcsetattr(serial,TCSANOW,&tio) == 0)
	    {
	        return (1);
	    }
	    return (0);
	}

	void setupThread(int schedPriority, pthread_t *threads, void *function, void *value)
	{
	    pthread_attr_t      attr;
	    struct sched_param  param;

	    //initialize thread detached attribute
	    pthread_attr_init(&attr);
	    //setup thread detached attribute
	    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	    /*setup scheduling policy*/
	    pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
	    pthread_attr_setschedpolicy(&attr, SCHED_RR);

	    param.sched_priority = schedPriority;
	    pthread_attr_setschedparam(&attr, &param);

	    if( pthread_create(&(*threads), &attr, function, value ) )
	    {
	        printf("ERROR: return code from pthread_create() is %s\n", (char *) function);
	        _exit(-1);
	    }
	}

	void setupTimer(int timeout_s, int inter_s, void (*handle)(int s), sigset_t signalName)
	{
	    struct timespec     timeout;
	    struct itimerval    it;
	    struct sigaction    sa;
	    /*setup timer*/
	    timeout.tv_sec = timeout_s;
	    timeout.tv_nsec = 0;
	    it.it_interval.tv_sec = inter_s;
	    it.it_interval.tv_usec = 0;
	    it.it_value = it.it_interval;
	    /*setup sigalrm*/
	    sigemptyset(&sa.sa_mask);
	    sa.sa_flags = 0;
	    sa.sa_handler = handle;
	    sigaction(SIGALRM, &sa, NULL);
	    setitimer(ITIMER_REAL, &it, NULL);
	    sigemptyset( &signalName );
	}

    void MavlinkRequestData(MAV_DATA_STREAM ID, ushort hzrate, ushort active)
	{
		uint16_t send_len;
		msg.seq ++;
		mavlink_msg_request_data_stream_pack(127, 0, &msg, MAV_sysid, MAV_compid, ID, hzrate, active);
		send_len = mavlink_msg_to_send_buffer(buf, &msg);
		if( write(APM_Serial, buf, send_len) < 0 )
	    {
	        printf(RED "request data stream updata error.\n");
	    }
        tcflush(APM_Serial, TCOFLUSH);
        memset(&buf, 0, sizeof(buf));
	}

	void MavlinkSendMessage()
	{
		uint16_t send_len;
		send_len = mavlink_msg_to_send_buffer(buf, &msg);
		if( write(APM_Serial, buf, send_len) < 0 )
	    {
	        printf(RED "data stream updata error.\n");
	    }
        tcflush(APM_Serial, TCOFLUSH);
        memset(&buf, 0, sizeof(buf));
	}

	char* mav_Mode(uint8_t mode)
 	{
		switch(mode)
		{
			case 0:
				return "STABILIZE";
			break;

			case 1:
				return "Acro";
			break;

			case 2:
				return "AltHold";
			break;

			case 3:
				return "AUTO";
			break;

			case 4:
				return "Guided";
			break;

			case 5:
				return "Loiter";
			break;

			case 6:
				return "RTL";
			break;

			case 7:
				return "Circle";
			break;

			case 9:
				return "Land";
			break;

			case 10:
				return "OF_Loiter";
			break;

			case 11:
				return "Drift";
			break;

			case 13:
				return "Sport";
			break;
		}
	}

	char* state_type(uint8_t type)
	{
		switch(type)
		{
			case 0:
				return "Uninitialized system, state is unknown.";
			break;

			case 1:
				return "System is booting up.";
			break;

			case 2:
				return "System is calibrating and not flight-ready.";
			break;

			case 3:
				return "System is grounded and on standby. It can be launched any time.";
			break;

			case 4:
				return "System is active and might be already airborne. Motors are engaged.";
			break;

			case 5:
				return "System is in a non-normal flight mode. It can however still navigate.";
			break;

			case 6:
				return "It is in mayday and going down.";
			break;

			case 7:
				return "System will be shut down now.";
			break;
		}
	}

	char* gps_status(uint8_t type)
	{
		switch(type)
		{
			case 0:
			case 1:
				return "no fix";
			break;

			case 2:
				return "2D fix";
			break;

			case 3:
				return "3D fix";
			break;
		}
	}

	void print_info()
	{
		printf(GREEN "\n Command List:\n");
		printf("\t 0 : quit the program\n");
		printf("\t 1 : mavlink request data\n");
		printf("\t 2 : mavlink set mode\n");
		printf("\t 3 : mavlink mission clear all\n");
		printf("\t 4 : mavlink waypoint home upload\n");
		printf("\t 5 : mavlink mission waypoint upload\n");
		printf("\t 6 : mavlink mission waypoint download\n");
		printf("\t 7 : mavlink arms/disarms the component\n");
		printf("\t 8 : mavlink set a servo to a desired PWM value\n");
		printf("\t 9 : mavlink take-off from ground\n");
		printf("\t 10 : mavlink start running the current mission\n\n");
		printf(WHITE "Command : ");
	}

	double get_time()
	{
	    static struct timespec tset;
	    struct timespec t;
	    double tnow;
	    static int init = 0;
	    
	    if(init == 0)
	    {
	        init = 1;
	        clock_gettime(CLOCK_REALTIME,&tset);
	        return 0.0;
	    }
	    clock_gettime(CLOCK_REALTIME, &t);
	    tnow = (t.tv_sec-tset.tv_sec) + 1.0e-9*(double)(t.tv_nsec - tset.tv_nsec);
	    return tnow;
	}

	void timer_inter(int sig)
	{
		pthread_cond_signal(&trigger_APM);
    	return;
	}

	int openfile2read(mavlink_mission_item_t *mWP)
	{
		char buffer[256];
		int status = false, i = 0;
		FILE *file;
		if((file = fopen("WP.txt", "r+")) == NULL)
		{
			printf(RED "can't open file to read!\n");
			return 0;
		}
		while(!feof(file))
		{	
			if(fgets(buffer, sizeof(buffer), file) != NULL)
			{				
				if (!status & strcspn(buffer, "1234567890") == 8 & buffer[0] == 'Q')
				{
					status = true;
				}
				else
				{
					sscanf(buffer, "%hu %s %s %hu %f %f %f %f %f %f %f %s", 
						&mWP[i].seq, &mWP[i].current, &mWP[i].frame, &mWP[i].command, &mWP[i].param1, &mWP[i].param2, 
						&mWP[i].param3, &mWP[i].param4, &mWP[i].x, &mWP[i].y, &mWP[i].z, &mWP[i].autocontinue);
					i++;
				}
			}
		}
		fclose(file);
		return (i);
	}

// programme's main function
	//void *VideoDetector(void *thread_id);

	void *serial_read(void *thread_id)
	{
		struct  pollfd fdAPM[1];
		int readCount = 0;
		int APMRec = 0, APMRec_tmp = 0;
		unsigned char RecBuffer[270]={0,};
		printf("Start serial read threads.\n");
	    if( (printFile = fopen(PrintFileName,"w+b")) == NULL )
	    {
	        printf(L_RED "print text file is open error.\n");
	        exit(1);
	    }
	    if ( (APM_Serial = open(RS_DEVICE_APM, O_RDWR|O_NOCTTY)) < 0 )
	    {
	        printf(L_RED "APM comport open error.\n");
	        exit(1);
	    }
	    printf("open APM comport successful.\n");
	    if ( setSerialport(APM_Serial, BAUDRATE_APM) )
	    {
	    	printf("read APM I/O and get data.\n");
			printf("=============================================\n");
	    }
		fdAPM[0].fd = APM_Serial;
		fdAPM[0].events = POLLIN;
		while(ExitRun == 1)
		{
			int APMPoll = poll(fdAPM, 1, 1000);
			if ( fdAPM[0].revents & POLLIN )
			{ 
				if((APMRec_tmp = read(APM_Serial, RecBuffer + 0, 1)) > 0)
				{
					APMRec = APMRec_tmp;
				}	
				
				if ( RecBuffer[0] == MAVLINK_STX )
				{
					//read header
					while(APMRec < 6)
					{
						if((APMRec_tmp = read(APM_Serial, RecBuffer + APMRec, 6 - APMRec)) > 0)
						{
							APMRec += APMRec_tmp;
						}
					}
					#if debug
						printf(L_RED "0x%X, %d, %d, 0x%X, 0x%X, 0x%X\n", RecBuffer[0], RecBuffer[1], RecBuffer[2], RecBuffer[3], RecBuffer[4], RecBuffer[5]);
					#endif
        			// check message length vs table
					if ( (int)RecBuffer[1] == MAVLINK_MESSAGE_LENGTHS_ARRAY[(int)RecBuffer[5]] )
            		{
            			// packet length
            			int lengthtoread = (int)RecBuffer[1] + 8; // data + header + checksum - STX - length
            			while( APMRec < lengthtoread)
	            		{
							if((APMRec_tmp = read(APM_Serial, RecBuffer + APMRec, lengthtoread - APMRec)) > 0)
							{
								APMRec += APMRec_tmp;
							}
	            		}
	            		// calc crc
            			ushort crc = crc_calculate(&RecBuffer[1], (ushort)(lengthtoread - 3));
	            		// calc extra bit of crc for mavlink 1.0	
            			crc_accumulate(MAVLINK_MESSAGE_CRCS_ARRAY[(int)RecBuffer[5]], &crc);
            			// check crc
            			if ((uint8_t)(crc & 0xFF) == RecBuffer[lengthtoread - 2] && (uint8_t)(crc >> 8) == RecBuffer[lengthtoread - 1])
            			{
							switch(RecBuffer[5])
							{
								case MAVLINK_MSG_ID_HEARTBEAT:		//#00
									pthread_mutex_lock(&mutex_APM);
									sys_heartbeat = *(struct __mavlink_heartbeat_t *)&RecBuffer[6];
									//mavlink_msg_heartbeat_decode((mavlink_message_t *)&RecBuffer[6] ,&sys_heartbeat);
									HeartBeat++;
									if (HeartBeat == 10)
				                    {
				                        MAV_sysid = RecBuffer[3];
				                        MAV_compid = RecBuffer[4];
				                        MavlinkRequestData(MAV_DATA_STREAM_ALL, 1 , true);
				                    }
				                    custom_mode = sys_heartbeat.custom_mode;
				                    system_status = sys_heartbeat.system_status;
				                    #if RDD
										printf(WHITE "get the %d times HEARTBEAT data!\n", HeartBeat);
										printf(GREEN "flight mode : %s\n", mav_Mode(sys_heartbeat.custom_mode));
										printf(GREEN "status : %s\n", state_type(sys_heartbeat.system_status));
									#endif
									pthread_mutex_unlock(&mutex_APM);
								break;

								case MAVLINK_MSG_ID_SYS_STATUS:		//#01
									pthread_mutex_lock(&mutex_APM);
                					sys_status = *(struct __mavlink_sys_status_t *)&RecBuffer[6];
                					#if RDD
                						printf(GREEN "system remaining battery energy is %d %% \n", sys_status.battery_remaining);
                						printf(GREEN "system communication drops in percent %d %%\n", sys_status.drop_rate_comm);
                					#endif
                					pthread_mutex_unlock(&mutex_APM);	
                				break;

                				case MAVLINK_MSG_ID_SYSTEM_TIME:	//#02
                					pthread_mutex_lock(&mutex_APM);
                					system_time = *(struct __mavlink_system_time_t *)&RecBuffer[6];
                					time_boot_ms = system_time.time_boot_ms;
                					#if RDD
										printf(L_PURPLE "system times is %d ms\n", system_time.time_boot_ms);
									#endif
									pthread_mutex_unlock(&mutex_APM);
								break;

            					case MAVLINK_MSG_ID_GPS_RAW_INT:	//#24
                					pthread_mutex_lock(&mutex_APM);
                					gps_raw_int = *(struct __mavlink_gps_raw_int_t *)&RecBuffer[6];
                					fix_type = gps_raw_int.fix_type;
                					#if RDD
										printf(YELLOW "gpa status is %s\n", gpa_status(gps_raw_int.fix_type));
									#endif
									pthread_mutex_unlock(&mutex_APM);
                				break;

            					case MAVLINK_MSG_ID_SCALED_IMU:		//#26
                					pthread_mutex_lock(&mutex_APM);
                					scaled_imu = *(struct __mavlink_scaled_imu_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
                				break;
            
            					case MAVLINK_MSG_ID_RAW_IMU:		//#27
                					pthread_mutex_lock(&mutex_APM);
                					raw_imu = *(struct __mavlink_raw_imu_t *)&RecBuffer[6];
                					accX = raw_imu.xacc;
                					accY = raw_imu.yacc;
                					accZ = raw_imu.zacc;
                					gyroX = raw_imu.xgyro;
                					gyroY = raw_imu.ygyro;
                					gyroZ = raw_imu.zgyro;
                					magX = raw_imu.xmag;
                					magY = raw_imu.ymag;
                					magZ = raw_imu.zmag;

                					#if RDD
                    					printf(L_BLUE "accX : %.4f, accY : %.4f, accZ : %4f\n", raw_imu.xacc, raw_imu.yacc, raw_imu.zacc);
										printf(L_BLUE "gyroX : %.4f, gyroY : %.4f, gyroZ : %4f\n", raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro);
										printf(L_BLUE "magX : %.4f, magY : %.4f, magZ : %.4f\n", raw_imu.xmag, raw_imu.ymag, raw_imu.zmag);
									#endif
									pthread_mutex_unlock(&mutex_APM);
                				break;
            
            					case MAVLINK_MSG_ID_ATTITUDE:		//#30
                					pthread_mutex_lock(&mutex_APM);
                					attitude = *(struct __mavlink_attitude_t *)&RecBuffer[6];
                					roll = r2d*attitude.roll;
                					pitch = r2d*attitude.pitch;
                					yaw = r2d*attitude.yaw;

                					#if RDD
										printf(BROWN "roll: %.4f, pitch: %.4f, yaw: %.4f\n", r2d*attitude.roll, r2d*attitude.pitch, r2d*attitude.yaw);
			    					#endif
									pthread_mutex_unlock(&mutex_APM);
                				break;

            					case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:	//#33
                					pthread_mutex_lock(&mutex_APM);
                					global_position_int = *(struct __mavlink_global_position_int_t *)&RecBuffer[6];
                					lat = global_position_int.lat/1E7;
                					lon = global_position_int.lon/1E7;
                					alt = global_position_int.alt/1000;

                					#if RDD
										printf(L_CYAN "lat : %.6f, lon : %.6f, alt : %.4f \n", global_position_int.lat/1E7, global_position_int.lon/1E7, global_position_int.alt/1000);
			                    	#endif
									pthread_mutex_unlock(&mutex_APM);
			                    break;

			                    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:	//#35
                					pthread_mutex_lock(&mutex_APM);
									rc_channels_raw = *(struct __mavlink_rc_channels_raw_t *)&RecBuffer[6];
									#if RDD
										//Label58.Caption = rc_channels_raw.chan1_raw;
										//Label59.Caption = rc_channels_raw.chan2_raw;
										//Label60.Caption = rc_channels_raw.chan3_raw;
										//Label61.Caption = rc_channels_raw.chan4_raw;
										//Label62.Caption = rc_channels_raw.chan5_raw;
										//Label63.Caption = rc_channels_raw.chan6_raw;
										//Label64.Caption = rc_channels_raw.chan7_raw;
										//Label65.Caption = rc_channels_raw.chan8_raw;
									#endif
									pthread_mutex_unlock(&mutex_APM);
								break;

								case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:	//#36
                					pthread_mutex_lock(&mutex_APM);
									servo_output_raw = *(struct __mavlink_servo_output_raw_t *)&RecBuffer[6];
									#if RDD
										//Label74.Caption = servo_output_raw.servo1_raw;
										//Label75.Caption = servo_output_raw.servo2_raw;
										//Label76.Caption = servo_output_raw.servo3_raw;
										//Label77.Caption = servo_output_raw.servo4_raw;
										//Label78.Caption = servo_output_raw.servo5_raw;
										//Label79.Caption = servo_output_raw.servo6_raw;
										//Label80.Caption = servo_output_raw.servo7_raw;
										//Label81.Caption = servo_output_raw.servo8_raw;
									#endif
									pthread_mutex_unlock(&mutex_APM);
								break;

        					    case MAVLINK_MSG_ID_MISSION_ITEM:	//#39
                					pthread_mutex_lock(&mutex_APM);
                					mission_item = *(struct __mavlink_mission_item_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
			                    break;

            					case MAVLINK_MSG_ID_MISSION_REQUEST:	//#40
                					pthread_mutex_lock(&mutex_APM);
                					mission_request = *(struct __mavlink_mission_request_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
									//printf("msg #40 %d\n", mission_request.seq);
			                    break;

            					case MAVLINK_MSG_ID_MISSION_CURRENT:	//#42
                					pthread_mutex_lock(&mutex_APM);
                					mission_current = *(struct __mavlink_mission_current_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
			                    break;

            					case MAVLINK_MSG_ID_MISSION_COUNT:		//#44
                					pthread_mutex_lock(&mutex_APM);
                					mission_count = *(struct __mavlink_mission_count_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
			                    break;

            					case MAVLINK_MSG_ID_MISSION_ACK:		//#47
                					pthread_mutex_lock(&mutex_APM);
                					mission_ack = *(struct __mavlink_mission_ack_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
									//printf("msg #47 %d\n", mission_ack.type);
			                    break;

            					case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:	//#62
                					pthread_mutex_lock(&mutex_APM);
                					nav_controller_output = *(struct __mavlink_nav_controller_output_t *)&RecBuffer[6];
									#if RDD
    									//StringGrid1.Cells[5][14] = FormatFloat("0.0000",nav_controller_output.nav_roll);
										//StringGrid1.Cells[5][15] = FormatFloat("0.0000",nav_controller_output.nav_pitch);
										//StringGrid1.Cells[5][16] = FormatFloat("0.0000",nav_controller_output.nav_bearing);
										//StringGrid1.Cells[5][17] = FormatFloat("0.0000",nav_controller_output.target_bearing);
										//StringGrid1.Cells[5][18] = FormatFloat("0.0000",nav_controller_output.wp_dist);
										//StringGrid1.Cells[5][19] = FormatFloat("0.0000",nav_controller_output.alt_error);
										//StringGrid1.Cells[7][1] = FormatFloat("0.0000",nav_controller_output.aspd_error);
										//StringGrid1.Cells[7][2] = FormatFloat("0.0000",nav_controller_output.xtrack_error);
			                    	#endif
									pthread_mutex_unlock(&mutex_APM);
			                    break;

            					case MAVLINK_MSG_ID_VFR_HUD:	//#74
                					pthread_mutex_lock(&mutex_APM);
                					vfr_hud = *(struct __mavlink_vfr_hud_t *)&RecBuffer[6];
									#if RDD
    									//StringGrid1.Cells[1][14] = FormatFloat("0.0000",vfr_hud.airspeed);
										//StringGrid1.Cells[1][18] = FormatFloat("0.0000",vfr_hud.groundspeed);
										//StringGrid1.Cells[7][5] = FormatFloat("0.0000",vfr_hud.climb);
			                    	#endif
									pthread_mutex_unlock(&mutex_APM);
			                    break;

            					case MAVLINK_MSG_ID_HIGHRES_IMU:	//#105
                					pthread_mutex_lock(&mutex_APM);
                					highres_imu = *(struct __mavlink_highres_imu_t *)&RecBuffer[6];
									pthread_mutex_unlock(&mutex_APM);
			                    break;

            					case MAVLINK_MSG_ID_RADIO_STATUS:	//#109
                					pthread_mutex_lock(&mutex_APM);
                					radio_status = *(struct __mavlink_radio_status_t *)&RecBuffer[6];
									#if RDD
										//StringGrid1.Cells[11][9] = radio_status.rssi;
										//StringGrid1.Cells[11][10] = radio_status.remrssi;
										//StringGrid1.Cells[11][11] = radio_status.txbuf;
										//StringGrid1.Cells[11][12] = radio_status.noise;
										//StringGrid1.Cells[11][13] = radio_status.remnoise;
										//StringGrid1.Cells[11][14] = radio_status.rxerrors;
										//StringGrid1.Cells[11][15] = radio_status.fixed;
			                    	#endif
									pthread_mutex_unlock(&mutex_APM);
			                    break;

								default:
									//Do nothing
								break;
							}
            			}
            		}	
				}
				memset(&RecBuffer,0,sizeof(RecBuffer));
			}
		}
		fclose(printFile);
	    close(APM_Serial);
	    printf("close serial thread\n");
	    pthread_exit(NULL);
	}

	void *user_command(void *thread_id)
	{
		int sleep_break, i;
		printf("Start user command threads.\n");
		sleep(20);
		print_info();
		while(ExitRun == 1)
		{
			if( pthread_cond_wait(&trigger_FILE, &mutex_FILE) == 0 )
			{
				char choise[5] = {}, character[128] = {};
				int server_action[3] = {5, 7, 10};		
				for(i = 0; i < 3; i++)
				//if (fgets(choise, sizeof(choise), stdin) != NULL)
				{
					switch(server_action[i])
					//switch(atoi(choise))
				    {
				        case 0:
				        	ExitRun = false;
				        	printf(WHITE "\nend programme now\n");
				        break;

				        case 1:
				        	printf(WHITE "\n[request_id,request_rate,active]\tex:[0,1,1]\n");
				        	mavlink_request_data_stream_t p1;
				        	while(fgets(character, sizeof(character), stdin) == NULL);
				        	sscanf(character,"[%s,%hu,%s]",&p1.req_stream_id, &p1.req_message_rate, &p1.start_stop);
				        	msg.seq ++;
							mavlink_msg_request_data_stream_pack(127, 0, &msg, MAV_sysid, MAV_compid, p1.req_stream_id, p1.req_message_rate, p1.start_stop);
							MavlinkSendMessage();
						break;

						case 2:
							printf(WHITE "[set_mode 0-13]\t ex:[0]\n");
							mavlink_set_mode_t p2;
							while(fgets(character, sizeof(character), stdin) == NULL);
				        	sscanf(character,"[%d]", &p2.custom_mode);
							//p2.custom_mode = 3;
							msg.seq ++;
							mavlink_msg_set_mode_pack(127, 0, &msg, MAV_sysid, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2.custom_mode) ;
							MavlinkSendMessage();
						break;

						case 3:
							msg.seq++;
							mavlink_msg_mission_clear_all_pack(127, 0, &msg, MAV_sysid, MAV_compid);
							MavlinkSendMessage();
							sleep_break = 200000; //100ms
							while(sleep_break > 0)
							{
								sleep_break = usleep(sleep_break);
							}
							if(mission_ack.type == 0)
							{
								printf(WHITE "mav mission is cleared!\n");
							}
						break;

						case 4:
							printf(WHITE "[lat,lon,alt]\t ex:[24.123456,120.123456,0]\n");
							mavlink_mission_item_t p4;
							while(fgets(character, sizeof(character), stdin) == NULL);
				        	sscanf(character,"[%f,%f,%f]", &p4.x, &p4.y, &p4.z);
				        	msg.seq++;
				        	mavlink_msg_mission_count_pack(127, 0, &msg, MAV_sysid, MAV_compid, 1);
							MavlinkSendMessage();
							sleep_break = 200000; //100ms
							while(sleep_break > 0)
							{
								sleep_break = usleep(sleep_break);
							}
							msg.seq++;
							mavlink_msg_mission_item_pack(127, 0, &msg, MAV_sysid, MAV_compid, 0, 0, 179, 1, 1, 1, 0, 0, 0, p4.x, p4.y, p4.z);
							MavlinkSendMessage();
							sleep_break = 200000; //100ms
							while(sleep_break > 0)
							{
								sleep_break = usleep(sleep_break);
							}
							if(mission_ack.type == 1)
							{
								printf(RED "home upload error !!\n");
							}
							else
							{
								printf(WHITE "home upload succeeded!!\n");
							}
						break;

						case 5:
							msg.seq++;
							mavlink_msg_mission_clear_all_pack(127, 0, &msg, MAV_sysid, MAV_compid);
							MavlinkSendMessage();
							sleep_break = 300000; //100ms
							while(sleep_break > 0)
							{
								sleep_break = usleep(sleep_break);
							}
							mavlink_mission_item_t p5[256];
							int wpNum = openfile2read(p5);
							printf(WHITE "upload [%d] waypoints in [MissionPlanner_WP.txt] file\n", wpNum);
							if (mission_ack.type == 0) 
							{
								msg.seq++;
								mavlink_msg_mission_count_pack(127, 0, &msg, MAV_sysid, MAV_compid, wpNum);
								MavlinkSendMessage();
							}
							//int wpNum_check = wpNum;
							int wp_seq = 0;
							while( wpNum > wp_seq & mission_ack.type != 1)// & wpNum_check >= mission_request.seq+1)
							{
								sleep_break = 300000; //100ms
								while(sleep_break > 0)
								{
									sleep_break = usleep(sleep_break);
								}
								//int wp_seq = mission_request.seq;
								if (wp_seq == 0)
								{
									msg.seq++;
									mavlink_msg_mission_item_pack(127, 0, &msg, MAV_sysid, MAV_compid, 0, 0, 179, 1, 1, 1, 0, 0, 0, p5[0].x, p5[0].y, p5[0].z);
									MavlinkSendMessage();
								}
								else
								{
									msg.seq++;
									mavlink_msg_mission_item_pack(127, 0, &msg, MAV_sysid, MAV_compid, wp_seq/*p5[wp_seq].seq*/, 
												0/*p5[wp_seq].frame*/, p5[wp_seq].command, p5[wp_seq].current, p5[wp_seq].autocontinue,
												p5[wp_seq].param1, p5[wp_seq].param2, p5[wp_seq].param3, p5[wp_seq].param4,
										  		p5[wp_seq].x, p5[wp_seq].y, p5[wp_seq].z);
									MavlinkSendMessage();
								}
								//printf("mission_ack.type = %d, mission_request.seq  = %d\n", mission_ack.type, mission_request.seq);
								printf(L_BLUE "Upload waypoint[%d] : [%.6f,%.6f,%.6f] \n", wp_seq, p5[wp_seq].x, p5[wp_seq].y, p5[wp_seq].z);
								//wpNum--;
								wp_seq++;
								if (mission_ack.type == 13)
								{
									//wpNum++;
									wp_seq--;
									mission_ack.type = 0;
									sleep_break = 600000; //200ms
									while(sleep_break > 0)
									{
										sleep_break = usleep(sleep_break);
									}
								}
							}
							if(mission_ack.type == 1)
							{
								printf(RED "mission upload accepted error !!\n");
							}
							else
							{
								printf(WHITE "mission upload accepted OK !!\n");
							}
						break;

						case 6:
							msg.seq++;
							mavlink_msg_mission_request_list_pack(127, 0, &msg, MAV_sysid, MAV_compid);
							MavlinkSendMessage();
							sleep_break = 1;
							while(sleep_break > 0)
							{
								sleep_break = sleep(sleep_break);
							}				
							printf(WHITE "waypoint count = %d\n", mission_count.count);
							int WP_rec_indx = 0;
							while(WP_rec_indx < mission_count.count)
							{
								msg.seq++;
								mavlink_msg_mission_request_pack(127, 0, &msg, MAV_sysid, MAV_compid, WP_rec_indx);
								MavlinkSendMessage();
								sleep_break = 500000; //100ms
								while(sleep_break > 0)
								{
									sleep_break = usleep(sleep_break);
								}
								if(mission_ack.type == 1)
									break;		
								if(mission_item.seq != WP_rec_indx)		
									continue;
								printf(L_BLUE "%d, %d, %.6f, %.6f, %.4f, %.2f, %.2f\n", WP_rec_indx, mission_item.command, mission_item.x, mission_item.y, mission_item.z, mission_item.param1, mission_item.param2);
								//int sleep_break = 400000; //100ms
								//while(sleep_break > 0)
								//{
								//	sleep_break = usleep(sleep_break);
								//}	
								
								WP_rec_indx++;
							}
							if(mission_ack.type != 1)
							{
								msg.seq++;
								mavlink_msg_mission_ack_pack(127, 0, &msg, MAV_sysid, MAV_compid, 1);
								MavlinkSendMessage();
								printf(WHITE "Mission Download accepted OK !!\n" );
							}
							else
							{
								printf(RED "Mission Download accepted error !!\n");
								mission_ack.type = 0;
							}

						break;

						case 7:
							printf(WHITE "[arm_disarm] ex:[0] 0:disarm 1:arm\n");
							mavlink_command_long_t p7;
							//while(fgets(character, sizeof(character), stdin) == NULL);
				        	//sscanf(character,"[%f]", &p7.param1);
				        	p7.param1 = 1;
							while(1)
							{
								if(p7.param1 == 1)
								{
									if(shootout == 0)
									{
										msg.seq++;
										mavlink_msg_command_long_pack(127, 0, &msg, MAV_sysid, MAV_compid, 400, 0, p7.param1, 0, 0, 0, 0, 0, 0);
										MavlinkSendMessage();
										printf(WHITE "apm arm status is been changed.\n");
										break;
									}
									else
									{
										printf(RED "I catch %d people in front of me!!\n", shootout);
										printf(RED "I will wait for him to leave at 10 sec...\n");
										sleep(10);
										continue;
									}
								}
								else
								{
									msg.seq++;
									mavlink_msg_command_long_pack(127, 0, &msg, MAV_sysid, MAV_compid, 400, 0, p7.param1, 0, 0, 0, 0, 0, 0);
									MavlinkSendMessage();
									printf(WHITE "apm arm status is been changed.\n");
									break;
								}
							}
						break;

						case 8:
							printf(WHITE "[Servo_num 1-8, PWM 1000-2000] ex:[8,1700]\n");
							mavlink_command_long_t p8;
							while(fgets(character, sizeof(character), stdin) == NULL);
				        	sscanf(character,"[%f,%f]", &p8.param1, &p8.param2);
				        	msg.seq++;
				        	mavlink_msg_command_long_pack(127, 0, &msg, MAV_sysid, MAV_compid, 184, 0, p8.param1, p8.param2, 0, 0, 0, 0, 0);
							MavlinkSendMessage();
							printf(WHITE "finish updata servo[%d]'s PWM.\n", (int)p8.param1);
						break;

						case 9:
							printf(WHITE "[climb_alt] ex:[15]\n");
							mavlink_command_long_t p9;
							while(fgets(character, sizeof(character), stdin) == NULL);
				        	sscanf(character,"[%f]", &p9.param7);
				        	//p9.param7 = 10;
				        	msg.seq++;
				        	mavlink_msg_command_long_pack(127, 0, &msg, MAV_sysid, MAV_compid, 22, 0, 0, 0, 0, 0, 0, 0, p9.param7);
							MavlinkSendMessage();
							printf(WHITE "climb now.\n");
						break;

						case 10:
							msg.seq++;
				        	mavlink_msg_command_long_pack(127, 0, &msg, MAV_sysid, MAV_compid, 300, 0, 0, 0, 0, 0, 0, 0, 0);
							MavlinkSendMessage();
							printf("becareful the amp start mission now\n");
						break;

				        default:
				            printf(WHITE "I don't havs you choice!\nTry again\n\n");
				        break;  
				    }

				    if (atoi(choise) != 0)
			        {
						printf(WHITE "Command : ");
			        } 
			        sleep(1);   
				}
			}
		}
		printf("close user command thread\n");
		pthread_exit(NULL);
	}

	void *udp_send(void *thread_id)
	{
		printf("Start udp send threads.\n");
		struct sockaddr_in s_addr;
		int sock;
		char buff[1024];
		if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
		{
			printf("socket open error.\n");
			exit(1);
		}
		printf("create socket successful.\n");
		s_addr.sin_family = AF_INET;
		s_addr.sin_port = htons(NET_PORT);
		s_addr.sin_addr.s_addr = inet_addr(NET_HOST);
		//bind(sock, (struct sockaddr *) &s_addr, sizeof(s_addr));
		while(ExitRun == 1)
		{
	        if ( pthread_cond_wait(&trigger_APM, &mutex_UDP) == 0 )
	        {
				#if 0 //debug
					sprintf(buff, WHITE"get the %d times HEARTBEAT data!\n\r""system times is %d\n\r"
										"======================================================\n\r"
								  L_PURPLE"flight mode : %s\n\r""status : %s\n\r""gpa status is %s\n\r"
										"======================================================\n\r"
								  L_BLUE"accX : %.4f, accY : %.4f, accZ : %4f\n\r"
										"gyroX : %.4f, gyroY : %.4f, gyroZ : %4f\n\r"
										"magX : %.4f, magY : %.4f, magZ : %.4f\n\r"
										"roll: %.4f, pitch: %.4f, yaw: %.4f\n\r"
										"======================================================\n\r"
								  L_CYAN "lat : %.6f, lon : %.6f, alt : %.4f\n\r"
										"======================================================\n\r"
							, HeartBeat, time_boot_ms, mav_Mode(custom_mode), state_type(system_status)
							, gps_status(fix_type), accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ
							, roll, pitch, yaw, lat/1E7, lon/1E7, alt/1000);
					sendto(sock, &buff[0], strlen(buff) , 0, (struct sockaddr *)&s_addr, sizeof(s_addr));
					usleep(10);
				#else
		        	sprintf(buff,"16111002,254,%s,1,%d,%d,%d,%.6f,%.6f,%.4f,%.4f,%d,%d,%d", mav_Mode(custom_mode), (HeartBeat>10?1:0), sys_status.battery_remaining, mission_current.seq, lon, lat, alt, (double)gps_raw_int.vel*100, fix_type, gps_raw_int.satellites_visible, latitude);
		        	//IDEAS_ID, Air_ID, Mode, flight_C, Sys_Status, battery, current_WP, lon, lat, alt, speed, gps_fix, gps_sv
					sendto(sock, &buff[0], strlen(buff) , 0, (struct sockaddr *)&s_addr, sizeof(s_addr));
					usleep(10);
				#endif
	        }
		}
		close(sock);
		printf("close udp thread\n");
		pthread_exit(NULL);
	}


	void main()
	{
		pthread_t       threads[4];
		sigset_t        allsigs, waitMask;
		int 			sigTimes = 0;
		int 			sys_command = 0;
		/*initialize multi-thread operation and thread priority*/
		pthread_mutex_init(&mutex_APM,NULL);
		pthread_mutex_init(&mutex_UDP,NULL);
		pthread_mutex_init(&mutex_FILE,NULL);
		pthread_cond_init(&trigger_APM,NULL);
		pthread_cond_init(&trigger_FILE,NULL);
		setupThread(2, &threads[0], serial_read, (void *)0);
		setupThread(2, &threads[1], user_command, (void *)1);
		setupThread(2, &threads[2], udp_send, (void *)2);
		//setupThread(2, &threads[3], VideoDetector, (void *)3);
		/*setup timer*/
		setupTimer(10, 1, timer_inter, allsigs);
		printf("Finish setup all config and run programme now\n");
		printf("=============================================\n");
		//sleep(1);
		//sys_command = system("mate-terminal -e 'sudo screen /dev/ttyUSB0 57600'");
		while(ExitRun == 1)
		{
			//sigemptyset(&waitMask);
			sigsuspend(&allsigs);
			if (sigTimes == 3)
			{
				FILE *file;
				if((file = fopen("System_Command.txt", "r+")) != NULL)
				{
					pthread_cond_signal(&trigger_FILE);
					fclose(file);
					remove("System_Command.txt");
					printf("I have file!\n");
				}
				sigTimes = 0;
			}
			sigTimes++;
		}
		
		printf(UNDERLINE YELLOW "cost 5 sec to wait programme shotdown...\n" NONE);	
		//sys_command = system("screen -d");	
		sleep(5);
		pthread_mutex_destroy(&mutex_APM);
		pthread_cond_destroy(&trigger_APM);
		pthread_cond_destroy(&trigger_FILE);
		exit(0);
	}




