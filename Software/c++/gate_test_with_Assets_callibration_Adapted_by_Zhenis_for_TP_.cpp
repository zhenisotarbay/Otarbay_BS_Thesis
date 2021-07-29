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

#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
 #include <vector>
#include <algorithm>  // copy()
#include <iterator>   // istream_iterator, back_inserter
#include <sstream>    // istringstream
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
// Control table address
#define ADDR_PRO_TORQUE_ENABLE          	562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          	596
#define ADDR_PRO_PRESENT_POSITION       611
#define HARDWARE_ERROR				518

#define ADDR_MX_TORQUE_ENABLE          	64                 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION          	116
#define ADDR_MX_PRESENT_POSITION       	132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           	4
#define LEN_PRO_PRESENT_POSITION        	4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        	1000000			//57600
#define DEVICENAME                      	"/dev/ttyUSB0"      // Check which port is being used on your controller // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
                                                            
#define PORT 					8080
#define TORQUE_ENABLE                   1                   				// Value for enabling the torque
#define TORQUE_DISABLE                  0                   				// Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  	// Dynamixel moving status threshold

#define ConvCoef 					843		//from  degrees to motor units H42-20-S300-R        //   old 48340
#define ESC_ASCII_VALUE                 	0x1b

int client(){
	int sock;
	char buffer[36] = {0};
	struct sockaddr_in addr; // ????????? ? ???????
    	sock = socket(AF_INET, SOCK_STREAM, 0); // ???????? TCP-??????
    	if(sock < 0)
    	{
        	perror("socket");
        	exit(1);
    	}
    	addr.sin_family = AF_INET; // ?????? Internet
	addr.sin_port = htons(PORT); // ??? ????? ?????? ????...
    	addr.sin_addr.s_addr = inet_addr("10.3.21.42"); //127.0.0.1
	printf("Try to connect\n");
	connect(sock, (struct sockaddr *)&addr, sizeof(addr));// ????????? ?????????? ? ????????
	recv(sock , buffer , 36, 0);
	printf("received data: %s\n", buffer);
	close(sock);
     int result = 0;
     int sign = 1;
     int sign_pos = 0;

     if (buffer[0] == '-'){
 	sign = -1;
 	sign_pos = 1;
     }
     int len = sign_pos;
     while ('0' <= buffer[len] && '9' >= buffer[len]){
	len ++;
	}

     for (int i = sign_pos; i < len; i++) {
	if('0' <= buffer[i] && '9' >= buffer[i]){
		result += (buffer[i]-'0') * pow(10, (len - i - 1));
	}else{
		break;
	}
      }

    return result * sign;

}

using namespace std;


// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWritePro(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncWrite groupSyncWriteMX(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

uint8_t dxl_error = 0;                            // Dynamixel error

int ConvertDegreesToMV(float degrees)
{
  int MotorsValue = 0;
  if(degrees > 0)
  {
	if(degrees > 30)
		degrees = 30;
	MotorsValue = round(2047.5 + degrees * 11.375);
  }
  else
  {
	if(degrees < -30)
		degrees = -30;
	MotorsValue = round(2047.5 + degrees * 11.375);
  }
 
  return MotorsValue;
}

void changeTrajectory(vector<float> &vec, float offset)
{
	for(int i = 0; i < vec.size(); i++)
	{
		if(vec[i] > -6)
		{
//			printf("Vector size is %f\n", vec[i]);
			vec[i] = vec[i] + offset;
		}
	}
}

void changeVal(vector <vector<float> >& vec, int joint, float coef, float offset)
{
	for(int i = 0; i < vec[joint].size(); i++)
	{
		vec[joint][i] = vec[joint][i] + offset;
		vec[joint][i] = vec[joint][i] * coef;
	}
}

void changeValStartHome(vector <vector<float> >& vec1, vector <vector<float> >& vec2, int joint, float coef)
{
	for(int i = 0; i < vec1[joint].size(); i++)
	{
		vec1[joint][i] = vec1[joint][i] * coef;
		vec2[joint][i] = vec2[joint][i] * coef;
	}
}

int getch()
{
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

int calibrationMX(int arr[], int size)
{
	int dxl_comm_result = COMM_TX_FAIL;  
	int32_t posM = 0;
	int32_t HO = 0;
	int32_t checkHO = 0;
	int32_t resPos = 0;
	int32_t lastPos = 0;
	for(int i = 0; i < size; i++)
	{
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], 20, (uint32_t*)&HO, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	else
    		printf("%d motors HO is %d \n", arr[i], HO);
	usleep(1000);
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], ADDR_MX_PRESENT_POSITION, (uint32_t*)&posM, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	else
    		printf("%d motors present position is %d \n", arr[i], posM);
	resPos = HO - posM + 2048;
	printf("Result pos is %d\n", resPos);
	usleep(1000);
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, arr[i], 20, resPos, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	else
    		printf("%d motor is OK! Sent val: %d\n", arr[i], resPos);
	usleep(1000);
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], 20, (uint32_t*)&checkHO, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	else
    		printf("%d motors HO is %d \n", arr[i], checkHO);
	usleep(1000);

	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], ADDR_MX_PRESENT_POSITION, (uint32_t*)&lastPos, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	else
    		printf("%d motors present position is %d \n", arr[i], lastPos);
	usleep(1000);

	if (abs(lastPos)>2058 && abs(lastPos)<2038)
	{
    		printf("Bad Callibration!!!\n Press ESC to quit!)\n");
    		if (getch() == ESC_ASCII_VALUE) 
		{
		  	// Close port
		  	portHandler->closePort();
			return -1;
		}
	}
	}
	printf("DONE\n");
	return 0;
}

int calibration(int arr[], int size)
{
	int dxl_comm_result = COMM_TX_FAIL;  
	int32_t posM [size] = {};
	
	for(int i = 0; i < size; i++)
	{
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, arr[i], 13, 0, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	/*else
    		printf("%d motor is OK! \n", arr[i]);*/
	usleep(10000);
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], ADDR_PRO_PRESENT_POSITION, (uint32_t*)&posM[i], &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	/*else
    		printf("%d motor is OK! \n", arr[i]);*/
	usleep(10000);
	printf("CALIBRATION CHECK:\n");
	printf("%d: %i\n", i + 3, posM[i]);
	printf("CALIBRATION CHECK DONE!\n");
	// Routine check every motor
	if (abs(posM[i]) > 1000) packetHandler->write4ByteTxRx(portHandler, arr[i], 13, -posM[i], &dxl_error);
	usleep(10000);
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, arr[i], ADDR_PRO_PRESENT_POSITION, (uint32_t*)&posM[i], &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
   		printf("Error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
  	else if (dxl_error != 0)
    		printf("Error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
  	/*else
    		printf("%d motor is OK! \n", arr[i]);*/
	printf("CALIBRATION RE-CHECK:\n");
	printf("%d: %i\n", i + 3, posM[i]);
	usleep(10000);
	if (abs(posM[i])>1000)
	{
    		printf("Bad Callibration!!!\n Press ESC to quit!)\n");
    		if (getch() == ESC_ASCII_VALUE) 
		{
		  	// Close port
		  	portHandler->closePort();
			return -1;
		}
	}
	}

	printf("DONE\n\n\n");
	return 0;
}

void arming(int arr[], int size)
{
	int dxl_comm_result = COMM_TX_FAIL;  
	uint8_t error_val = 0;
	digitalWrite(17,1); digitalWrite(23,1);
	for(int i=0; i<size; i++)
	{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, arr[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	 	printf("Arming error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
	 else if (dxl_error != 0)
	{
	    	printf("Arming error on %d motor! %s. Error code %hhu\n", arr[i], packetHandler->getRxPacketError(dxl_error), dxl_error);
		packetHandler->read1ByteTxRx(portHandler, arr[i], HARDWARE_ERROR, (uint8_t*)&error_val, &dxl_error);
		printf("Error code: %hhu\n", error_val);
	}
	 else
	 	printf("Arming %d motor is OK! \n", arr[i]);
	usleep(10000);
	}
  	printf("Motors ARMED!\n");
}

void disarming(int arr[], int size)
{
	int dxl_comm_result = COMM_TX_FAIL; 
	uint8_t m_error = 0;
	digitalWrite(17,0); digitalWrite(23,0); 
	for(int i=0; i<size; i++)
	{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, arr[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &m_error);
	if (dxl_comm_result != COMM_SUCCESS)
	 	printf("Disarming error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
	 else if (m_error != 0)
	    	printf("Disarming error on %d motor! %s. Error code: %hhu\n", arr[i], packetHandler->getRxPacketError(m_error), m_error);
	 else
	 	printf("Disarming %d motor is OK! \n", arr[i]);
	usleep(10000);
	}
	printf("Motors DISARMED!\n");
}

void armingMX(int arr[], int size)
{
	int dxl_comm_result = COMM_TX_FAIL;  
	for(int i=0; i<size; i++)
	{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, arr[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	 	printf("Arming error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
	 else if (dxl_error != 0)
	    	printf("Arming error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
	 else
	 	printf("Arming %d motor is OK! \n", arr[i]);
	usleep(1000);
	}
  	printf("Motors ARMED!\n");
}

void disarmingMX(int arr[], int size)
{
	int dxl_comm_result = COMM_TX_FAIL; 
	for(int i=0; i<size; i++)
	{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, arr[i], ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	 	printf("Disarming error on %d motor! %s\n", arr[i], packetHandler->getTxRxResult(dxl_comm_result));
	 else if (dxl_error != 0)
	    	printf("Disarming error on %d motor! %s\n", arr[i], packetHandler->getRxPacketError(dxl_error));
	 else
	 	printf("Disarming %d motor is OK! \n", arr[i]);
	usleep(1000);
	}
	printf("Motors DISARMED!\n");
}

void test_move(int m_arr[],  int *goal_position)
{
  printf("You are in test move mode. Use arrow keys to move motors and press ESC three times to exit\n");
  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  uint8_t p_position[4];
  uint8_t n_position[4];

while(1)
{
	int x = 0;
 	int y = 0;
 	int z = 0;
 	x = getch();
 	if (x == 27)
 	{
  		y = getch();
  		z = getch();
 	}

 	if (x == 27 && y == 91)
 	{
  		switch (z)
  		{
   		case 65:
   			*goal_position += 1; // increase
   			break;

   		case 66:
   			*goal_position -= 1;
   			break;
  		}
 	}
 	else if(x == 27 && y == 27 && z == 27)
 	{
		printf("ESC!!!\n");
		break;
 	}

	printf("Value of the motor %d\n", ConvertDegreesToMV(*goal_position));

    	// Allocate goal position value into byte array
    	p_position[0] = DXL_LOBYTE(DXL_LOWORD((ConvertDegreesToMV(*goal_position))));
    	p_position[1] = DXL_HIBYTE(DXL_LOWORD((ConvertDegreesToMV(*goal_position))));
    	p_position[2] = DXL_LOBYTE(DXL_HIWORD((ConvertDegreesToMV(*goal_position))));
    	p_position[3] = DXL_HIBYTE(DXL_HIWORD((ConvertDegreesToMV(*goal_position))));

    	n_position[0] = DXL_LOBYTE(DXL_LOWORD((ConvertDegreesToMV(*goal_position))));
    	n_position[1] = DXL_HIBYTE(DXL_LOWORD((ConvertDegreesToMV(*goal_position))));
    	n_position[2] = DXL_LOBYTE(DXL_HIWORD((ConvertDegreesToMV(*goal_position))));
    	n_position[3] = DXL_HIBYTE(DXL_HIWORD((ConvertDegreesToMV(*goal_position))));

    	// Add Dynamixel#1 goal position value to the Syncwrite storage
   	dxl_addparam_result = groupSyncWriteMX.addParam(m_arr[0], p_position);
    	if (dxl_addparam_result != true)
    	{
      		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_arr[0]);
      		break;
    	}

	dxl_addparam_result = groupSyncWriteMX.addParam(m_arr[1], n_position);
    	if (dxl_addparam_result != true)
    	{
      		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_arr[1]);
      		break;
    	}


    	// Syncwrite goal position
    	dxl_comm_result = groupSyncWriteMX.txPacket();
    	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    	// Clear syncwrite parameter storage
    	groupSyncWriteMX.clearParam();
	printf("Position: %d\n", *goal_position);
}
}

void move_motor(int m_ID,  int *goal_position)
{
  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  uint8_t position[4];

while(1)
{
	int x = 0;
 	int y = 0;
 	int z = 0;
 	x = getch();
 	if (x == 27)
 	{
  		y = getch();
  		z = getch();
 	}

 	if (x == 27 && y == 91)
 	{
  		switch (z)
  		{
   		case 65:
   			*goal_position += 1000; // increase
   			break;

   		case 66:
   			*goal_position -= 1000;
   			break;
  		}
 	}
 	else if(x == 27 && y == 27 && z == 27)
 	{
		printf("ESC!!!\n");
		break;
 	}
    	// Allocate goal position value into byte array
    	position[0] = DXL_LOBYTE(DXL_LOWORD((*goal_position)));
    	position[1] = DXL_HIBYTE(DXL_LOWORD((*goal_position)));
    	position[2] = DXL_LOBYTE(DXL_HIWORD((*goal_position)));
    	position[3] = DXL_HIBYTE(DXL_HIWORD((*goal_position)));

    	// Add Dynamixel#1 goal position value to the Syncwrite storage
    	dxl_addparam_result = groupSyncWritePro.addParam(m_ID, position);
    	if (dxl_addparam_result != true)
    	{
      		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_ID);
      		break;
    	}

    	// Syncwrite goal position
    	dxl_comm_result = groupSyncWritePro.txPacket();
    	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    	// Clear syncwrite parameter storage
    	groupSyncWritePro.clearParam();
	printf("Current position: %d\n", *goal_position);
}
}

bool interrupt_state = false;
void IntrptFunc()
{
  interrupt_state = true;
}

//FILE *file;
int main()
{
    /*if ((file = fopen("/home/pi/HumanoidRobot/network_comm/PID_Exp/belt_test.txt", "w")) == NULL)
        	printf("Error! Can not create a file\n");
  	else
  	{
		printf("File created!\n");
  	}
	*/
    string line;
    vector < vector <float> > trajectory;
    vector < vector <float> > toStart;
    vector < vector <float> > toHome;
    ifstream main("/home/pi/JTEdited.txt"); // окрываем файл для чтения
    ifstream start("/home/pi/start.txt");
    ifstream home("/home/pi/home.txt");
    if (main.is_open())
    {
        while (getline(main, line))
        {
		vector <float> temp;
		istringstream ss( line );
  		copy(
   		istream_iterator <float> ( ss ),
    		istream_iterator <float> (),
    		back_inserter( temp )
    		);
		trajectory.push_back(temp);
        }
    }

    if (start.is_open())
    {
        while (getline(start, line))
        {
		vector <float> temp;
		istringstream ss( line );
  		copy(
   		istream_iterator <float> ( ss ),
    		istream_iterator <float> (),
    		back_inserter( temp )
    		);
		toStart.push_back(temp);
        }
    }

    if (home.is_open())
    {
        while (getline(home, line))
        {
		vector <float> temp;
		istringstream ss( line );
  		copy(
   		istream_iterator <float> ( ss ),
    		istream_iterator <float> (),
    		back_inserter( temp )
    		);
		toHome.push_back(temp);
        }
    }

    //changeTrajectory(trajectory[5], -2);
    //changeTrajectory(trajectory[11], 2);

    for(int i=0; i< trajectory[0].size(); i++)
    {
	trajectory[5][i] = (abs(trajectory[3][i]) - abs(trajectory[2][i]))*(-1);
	trajectory[11][i] = (abs(trajectory[9][i]) - abs(trajectory[8][i]))*(-1);
    }

    changeVal(trajectory, 0, 1.2, 0); //Left hip yaw (Must be negative)
    changeVal(trajectory, 1, -1.2, 0); //Left hip roll
    changeVal(trajectory, 2, -1.2, 0); //Left hip pitch 
    changeVal(trajectory, 3, -1.2, 0); //Left knee 
    changeVal(trajectory, 4, -1.2, 0); // Left ankle roll   4.86
    changeVal(trajectory, 5, -1.2, 0); // Left ankle pitch
    changeVal(trajectory, 6, 1.2, 0); //Right hip yaw (Must be negative)
    changeVal(trajectory, 7, 1.2, 0); //Right hip roll 
    changeVal(trajectory, 8, 1.2, 0); //Right hip pitch (Must be negative)
    changeVal(trajectory, 9, 1.2, 0); //Right knee (Must be negative)
    changeVal(trajectory, 10, 1.2, 0); //Right ankle roll   2.37
    changeVal(trajectory, 11, 1.2, 0); //Right ankle pitch (Must be negative)

    cout<<"To start size "<<trajectory[0].size()<<endl;
    //cout<<"To start size row"<<toStart[0].size()<<endl;

    main.close();     // закрываем файл
    start.close();
    home.close();
	struct timeval timestamp[2];

	wiringPiSetupGpio();
	pinMode(17, OUTPUT);
	pinMode(23, OUTPUT);
	pinMode(18, INPUT);
	pullUpDnControl(18, PUD_UP);
	wiringPiISR(18, INT_EDGE_FALLING, &IntrptFunc); //IntrptFunc

  	// Open port
	if (portHandler->openPort())
  	{
    		printf("Succeeded to open the port!\n");
	}
  	else
  	{
    		printf("Failed to open the port!\n");
    		printf("Press any key to terminate...\n");
    		getch();
    		return 0;
  	}

  	// Set port baudrate
  	if (portHandler->setBaudRate(BAUDRATE))
  	{
    		printf("Succeeded to change the baudrate!\n");
  	}
  	else
  	{
    		printf("Failed to change the baudrate!\n");
    		printf("Press any key to terminate...\n");
    		getch();
    		return 0;
  	}
	long duration;
	int motor_pos = 0; 
	int pos = 0;
	int motors[10] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
	int MXmotors[2] = {1, 2};
	int orderedMID[12] = {2, 6, 4, 8, 12, 10, 1, 5, 3, 7, 11, 9};
	uint8_t controlP[4];
	uint8_t controlN[4];
	int dxl_res = COMM_TX_FAIL;

	int Calib_error = calibration(motors, 10);
	Calib_error = calibrationMX(MXmotors, 2);
	if (Calib_error<0) return 0;
	arming(motors, 10);
	armingMX(MXmotors, 2);
	//test_move(MXmotors, &pos);
	/*
	printf("Press ECS to stop the experiment, or press SPACE to continue\n"); 
	int ExStop = getch();

	if(ExStop == 27)
	{
		disarming(motors, 10);
  		// Close port
  		portHandler->closePort();
		//fclose(file);
		return 0;
	}
	
	move_motor(11, &motor_pos);
	*/
	printf("Press ECS to stop the experiment, or press SPACE to continue\n"); 
	int ExStop = getch();

	if(ExStop == 27)
	{
		disarming(motors, 10);
		disarmingMX(MXmotors, 2);
  		// Close port
  		portHandler->closePort();
		//fclose(file);
		return 0;
	}
    toStart=client();
	for(int i=0; i<toStart[0].size(); i++)
	{
		usleep(20000);
		for(int j=0; j<toStart.size(); j++)
		{
			motor_pos = floor(toStart[j][i] * ConvCoef);
			
			controlP[0] = DXL_LOBYTE(DXL_LOWORD(motor_pos));  //(int)floor(PID_val * ConvCoef)       dxl_goal_position[index]  	//
    			controlP[1] = DXL_HIBYTE(DXL_LOWORD(motor_pos));
			controlP[2] = DXL_LOBYTE(DXL_HIWORD(motor_pos));
		    	controlP[3] = DXL_HIBYTE(DXL_HIWORD(motor_pos));

			if(orderedMID[j] == 1 || orderedMID[j] ==2)
			{
				controlP[0] = DXL_LOBYTE(DXL_LOWORD(ConvertDegreesToMV(toStart[j][i])));  //(int)floor(PID_val * ConvCoef)       dxl_goal_position[index]  	//
	    			controlP[1] = DXL_HIBYTE(DXL_LOWORD(ConvertDegreesToMV(toStart[j][i])));
				controlP[2] = DXL_LOBYTE(DXL_HIWORD(ConvertDegreesToMV(toStart[j][i])));
			    	controlP[3] = DXL_HIBYTE(DXL_HIWORD(ConvertDegreesToMV(toStart[j][i])));

				groupSyncWriteMX.addParam(orderedMID[j], controlP);

			}
		
			groupSyncWritePro.addParam(orderedMID[j], controlP);
			
		}
		dxl_res = groupSyncWritePro.txPacket();
		if (dxl_res != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_res));
	    	dxl_res = groupSyncWriteMX.txPacket();
		if (dxl_res != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_res));
		groupSyncWritePro.clearParam();
    		groupSyncWriteMX.clearParam();
	}
	getch();
	/***********************************/
	gettimeofday(&timestamp[0], NULL);
	for(int k=0; k<3; k++)
	{
	for(int i=0; i<trajectory[0].size(); i++) //trajectory[0].size()
	{
		//gettimeofday(&timestamp[0], NULL);
		usleep(20000);
		for(int j=0; j<trajectory.size(); j++)
		{
			motor_pos = floor(trajectory[j][i] * ConvCoef);
			
			controlP[0] = DXL_LOBYTE(DXL_LOWORD(motor_pos));  //(int)floor(PID_val * ConvCoef)       dxl_goal_position[index]  	//
    			controlP[1] = DXL_HIBYTE(DXL_LOWORD(motor_pos));
			controlP[2] = DXL_LOBYTE(DXL_HIWORD(motor_pos));
		    	controlP[3] = DXL_HIBYTE(DXL_HIWORD(motor_pos));
			if(orderedMID[j] == 1 || orderedMID[j] ==2)
			{
				controlP[0] = DXL_LOBYTE(DXL_LOWORD(ConvertDegreesToMV(trajectory[j][i])));  //(int)floor(PID_val * ConvCoef)       dxl_goal_position[index]  	//
	    			controlP[1] = DXL_HIBYTE(DXL_LOWORD(ConvertDegreesToMV(trajectory[j][i])));
				controlP[2] = DXL_LOBYTE(DXL_HIWORD(ConvertDegreesToMV(trajectory[j][i])));
			    	controlP[3] = DXL_HIBYTE(DXL_HIWORD(ConvertDegreesToMV(trajectory[j][i])));

				groupSyncWriteMX.addParam(orderedMID[j], controlP);

			}
			groupSyncWritePro.addParam(orderedMID[j], controlP);
			
		}
		dxl_res = groupSyncWritePro.txPacket();
		if (dxl_res != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_res));
	    	dxl_res = groupSyncWriteMX.txPacket();
		if (dxl_res != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_res));
		groupSyncWritePro.clearParam();
    		groupSyncWriteMX.clearParam();
		//gettimeofday(&timestamp[1], NULL);
		//duration = (timestamp[1].tv_usec+1000000-timestamp[0].tv_usec)%1000000; //timestamp[1].tv_usec - timestamp[0].tv_usec; 
		//printf("Loop time %ld\n", duration);
		//fprintf(file,"%ld\t%ld\t%f\t%f\t%f\t%f\n", timestamp[0].tv_sec * 1000000 + timestamp[0].tv_usec, duration, angle, frontalforce, error_dot, PID_val);

	}
	}
	/**************************************/
	getch();

	for(int i=0; i<toHome[0].size(); i++)
	{
		usleep(20000);
		for(int j=0; j<toHome.size(); j++)
		{
			motor_pos = floor(toHome[j][i] * ConvCoef);
			
			controlP[0] = DXL_LOBYTE(DXL_LOWORD(motor_pos));  //(int)floor(PID_val * ConvCoef)       dxl_goal_position[index]  	//
    			controlP[1] = DXL_HIBYTE(DXL_LOWORD(motor_pos));
			controlP[2] = DXL_LOBYTE(DXL_HIWORD(motor_pos));
		    	controlP[3] = DXL_HIBYTE(DXL_HIWORD(motor_pos));

			if(orderedMID[j] == 1 || orderedMID[j] ==2)
			{
				controlP[0] = DXL_LOBYTE(DXL_LOWORD(ConvertDegreesToMV(toHome[j][i])));  //(int)floor(PID_val * ConvCoef)       dxl_goal_position[index]  	//
	    			controlP[1] = DXL_HIBYTE(DXL_LOWORD(ConvertDegreesToMV(toHome[j][i])));
				controlP[2] = DXL_LOBYTE(DXL_HIWORD(ConvertDegreesToMV(toHome[j][i])));
			    	controlP[3] = DXL_HIBYTE(DXL_HIWORD(ConvertDegreesToMV(toHome[j][i])));

				groupSyncWriteMX.addParam(orderedMID[j], controlP);

			}
		
			groupSyncWritePro.addParam(orderedMID[j], controlP);
			
		}
		dxl_res = groupSyncWritePro.txPacket();
		if (dxl_res != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_res));
	    	dxl_res = groupSyncWriteMX.txPacket();
		if (dxl_res != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_res));
		groupSyncWritePro.clearParam();
    		groupSyncWriteMX.clearParam();
	}

	gettimeofday(&timestamp[1], NULL);
	duration = (timestamp[1].tv_sec * 1000000 + timestamp[1].tv_usec - (timestamp[0].tv_sec * 1000000 + timestamp[0].tv_usec)); //timestamp[1].tv_usec - timestamp[0].tv_usec; 
	printf("Execution time %ld\n", duration);
	getch();
	disarming(motors, 10);
	disarmingMX(MXmotors, 2);
  	// Close port
  	portHandler->closePort();
	//fclose(file);

  	return 0;
}
