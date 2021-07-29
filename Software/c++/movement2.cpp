#if defined(__linux__) || defined(__APPLE__)
    #include <fcntl.h>
    #include <termios.h>
    #define STDIN_FILENO 0
    #elif defined(_WIN32) || defined(_WIN64)
    #include <conio.h>
#endif
#include <wiringPi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          	562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          	596
#define ADDR_PRO_PRESENT_POSITION       611

#define ADDR_MX_TORQUE_ENABLE          	64                 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION          	116
#define ADDR_MX_PRESENT_POSITION         132
// Data Byte Length
#define LEN_MX_GOAL_POSITION           	4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        	1000000			//57600
#define DEVICENAME                      	"/dev/ttyUSB0"      // Check which port is being used on your controller // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
                                                            
#define PORT 					8080
#define TORQUE_ENABLE                   1                   				// Value for enabling the torque
#define TORQUE_DISABLE                  0                   				// Value for disabling the torque

#define ESC_ASCII_VALUE                 	0x1b

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
uint8_t dxl_error = 0;                            // Dynamixel error

int getch() {
    #if defined(__linux__) || defined(__APPLE__)
      struct termios oldt, newt;
      int ch;
      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
      ch = getchar();
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
      return ch;
    #elif defined(_WIN32) || defined(_WIN64)
      return _getch();
    #endif
}

/**************************************************/
int calibration(int arr[], int size) {
	int dxl_comm_result = COMM_TX_FAIL;  
	int32_t posM [size] = {};
	for(int i = 0; i < size; i++) {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, arr[i], 20, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
        else if (dxl_error != 0)
                printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
        /*else
                printf("%d motor is OK! \n", arr[i]);*/
        usleep(10000);
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], ADDR_MX_PRESENT_POSITION, (uint32_t*)&posM[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
        else if (dxl_error != 0)
            printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
        else
            printf("%d motors present position is  \n", arr[i]);
        usleep(10000);
        printf("CALIBRATION CHECK:\n");
        printf("%d: %i\n", i + 3, posM[i]);
        printf("CALIBRATION CHECK DONE!\n");
        // Routine check every motor
        if (abs(posM[i]) > 10) packetHandler->write4ByteTxRx(portHandler, arr[i], 20, -posM[i], &dxl_error);
        usleep(10000);
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], ADDR_MX_PRESENT_POSITION, (uint32_t*)&posM[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
        else if (dxl_error != 0)
                printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
        /*else
                printf("%d motor is OK! \n", arr[i]);*/
        printf("CALIBRATION RE-CHECK:\n");
        printf("%d: %i\n", i + 3, posM[i]);
        usleep(10000);
        if (abs(posM[i])>1000) {
            printf("Bad Callibration!!!\n Press ESC to quit!)\n");
            if (getch() == ESC_ASCII_VALUE) {
                // Close port
                portHandler->closePort();
                return -1;
            }
        }
	}
	printf("DONE\n\n\n");
	return 0;
}

void arming(int arr[], int size) {
	int dxl_comm_result = COMM_TX_FAIL;  
	digitalWrite(17,1); digitalWrite(23,1);
	for(int i=0; i<size; i++) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, arr[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            printf("Arming error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
        else if (dxl_error != 0)
            printf("Arming error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
        else
            printf("Arming %d motor is OK! \n", arr[i]);
        usleep(10000);
	}
  	printf("Motors ARMED!\n");
}

void disarming(int arr[], int size) {
	int dxl_comm_result = COMM_TX_FAIL; 
	digitalWrite(17,0); digitalWrite(23,0); 
	for(int i=0; i<size; i++) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, arr[i], ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            printf("Disarming error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
        else if (dxl_error != 0)
            printf("Disarming error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
        else
            printf("Disarming %d motor is OK! \n", arr[i]);
        usleep(10000);
	}
	printf("Motors DISARMED!\n");
}

void IntrptFunc() {
  printf("Hello from buttom!'\n");
}


int client() {
	int sock;
	char buffer[2] = {0};
	struct sockaddr_in addr; // ????????? ? ???????
    	sock = socket(AF_INET, SOCK_STREAM, 0); // ???????? TCP-??????
    	if(sock < 0)
    	{
        	perror("socket");
        	exit(1);
    	}
    	addr.sin_family = AF_INET; // ?????? Internet
	addr.sin_port = htons(PORT); // ??? ????? ?????? ????...
    	addr.sin_addr.s_addr = inet_addr("10.1.198.173"); //127.0.0.1
	printf("Try to connect\n");
	connect(sock, (struct sockaddr *)&addr, sizeof(addr));// ????????? ?????????? ? ????????
	recv(sock , buffer , 2, 0);
	printf("received data: %s\n", buffer);
	close(sock);
    int result = 0;
    for (int i = 0; i < sizeof(buffer); i++) {
        result += (*(buffer+i)-'0') * pow(10, (sizeof(buffer)-i-1));
    }
    return result;
}

//Function for adjust the robot's initial position
void motors_pos(int m_arr[], int *goal_position) {
/*
    int dxl_comm_result = COMM_TX_FAIL;  // Communication result
    bool dxl_addparam_result = false;    // addParam result
    uint8_t p_position[4];
    uint8_t n_position[4];

    while(1) {
        int x = 0;
        int y = 0;
        int z = 0;
        x = getch();
        if (x == 27) {
            y = getch();
            z = getch();
        }
        if (x == 27 && y == 91) {
            switch (z) {
            case 65:
*/
                *goal_position = 11 * client(); // get position from client
/*
                break;
            case 66:
                *goal_position -= 11;
                break;
            }
        }
        else if(x == 27 && y == 27 && z == 27) {
            printf("ESC!!!\n");
            break;
        }
*/
        // Allocate goal position value into byte array
        p_position[0] = DXL_LOBYTE(DXL_LOWORD((*goal_position)));
        p_position[1] = DXL_HIBYTE(DXL_LOWORD((*goal_position)));
        p_position[2] = DXL_LOBYTE(DXL_HIWORD((*goal_position)));
        p_position[3] = DXL_HIBYTE(DXL_HIWORD((*goal_position)));
    /* This was already commented
        n_position[0] = DXL_LOBYTE(DXL_LOWORD((-1)*(*goal_position)));
        n_position[1] = DXL_HIBYTE(DXL_LOWORD((-1)*(*goal_position)));
        n_position[2] = DXL_LOBYTE(DXL_HIWORD((-1)*(*goal_position)));
        n_position[3] = DXL_HIBYTE(DXL_HIWORD((-1)*(*goal_position)));
    */
        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(m_arr[0], p_position);
        if (dxl_addparam_result != true) {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_arr[0]);
            break;
        }
    /* This was already commented
        dxl_addparam_result = groupSyncWrite.addParam(m_arr[1], n_position);
            if (dxl_addparam_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_arr[1]);
                break;
            }
    */
        // Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        // Clear syncwrite parameter storage
        groupSyncWrite.clearParam();
        printf("Position: %d\n", *goal_position);
    //}
}



/**************************************************/

int main() {
	wiringPiSetupGpio();
	pinMode(17, OUTPUT);
	pinMode(23, OUTPUT);
	pinMode(18, INPUT);
	pullUpDnControl(18, PUD_UP);
	wiringPiISR(18, INT_EDGE_FALLING, &IntrptFunc); //IntrptFunc

  	// Open port
	if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
	} else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
  	}

  	// Set port baudrate
  	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
  	} else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
  	}

	int motors[1] = {2};
	int pos = 0;
	int dxl_res = COMM_TX_FAIL;

	//int Calib_error = calibration(motors, 1);
	//if (Calib_error<0) return 0;
	arming(motors, 1);
	
	printf("Press ECS to stop the experiment, or press SPACE to continue\n"); 
	int ExStop = getch();

	if(ExStop == 27) {
		disarming(motors, 1);
  		// Close port
  		portHandler->closePort();

		return 0;
	}
	printf("Now you can adjust high level of robot. Use arrow keys to adjust\n Press ESC three times to quit\n");			
	motors_pos(motors, &pos);
	
	disarming(motors, 1);
  	// Close port
  	portHandler->closePort();
  	return 0;
}