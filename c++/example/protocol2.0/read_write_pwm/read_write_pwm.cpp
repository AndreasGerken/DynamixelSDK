// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          3                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
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

int kbhit(void)
{
#ifdef __linux__
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}

int main()
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int dxl_goal_pwm[2] = { -100, 100 };

	uint8_t dxl_error = 0;                          // Dynamixel error
	int16_t dxl_present_pwm = 0;

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

	// Enable Dynamixel Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->getTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->getRxPacketError(dxl_error);
	}
	else
	{
		printf("Dynamixel #1 has been successfully connected \n");
	}

	while (1)
	{
		// printf("Press key_1  to continue! (or press ESC to quit!)\n");
		// if (getch() == ESC_ASCII_VALUE)
		// 	break;
		// Write goal position
		char pressed_key;
		printf("Press key to continue! (or press ESC to quit!)\n");
		pressed_key = getch();
		if (pressed_key == ESC_ASCII_VALUE)
			break;

		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, 100, dxl_goal_pwm[index], &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->getTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
			packetHandler->getRxPacketError(dxl_error);
		}

		// Read present position
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, 124, (uint16_t*)&dxl_present_pwm, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->getTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
			packetHandler->getRxPacketError(dxl_error);
		}

		printf("[ID:%03d] GoalPWM:%03d  PresPWM:%03d\n", DXL_ID, dxl_goal_pwm[index], dxl_present_pwm);

		// Change goal position
		if (index == 0)
		{
			index = 1;
		}
		else
		{
			index = 0;
		}
	}
	// Disable Dynamixel Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->getTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->getRxPacketError(dxl_error);
	}

	// Close port
	portHandler->closePort();

	return 0;

}
