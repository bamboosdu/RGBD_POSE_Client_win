/* 虚拟环境通讯的接口 */
#include <iostream>


#include <Windows.h>  
#include <thread>  
#include <mutex>  
// OpenCV
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

//socket 用于和虚拟环境交换数据：从虚拟环境获取RGBD和pose，给虚拟环境传移动路径
SOCKET sockClient;
//SOCKET sockSender;
//string serverIp = "192.168.1.2";
unsigned short portNo33 = 3333; // sockClient
unsigned short portNo66 = 6666; // sockSender
const int MAXRECV = 10240;
string serverIp = "192.168.1.107";
bool success = false;
const int rbt_num = 1;
float pose[rbt_num][7];
float set_pose[rbt_num][7];

//float rbtdir[rbt_num];
vector<cv::Mat> depth;
vector<cv::Mat> rgb;

//vector<PositionInGazebo> send_tasks;

/* initialize */

int initializeDataEngine(){

	{/* sockClient */// recv socket
		WORD wVersionRequested;
		WSADATA wsaData;
		int err;
		wVersionRequested = MAKEWORD(1, 1);
		err = WSAStartup(wVersionRequested, &wsaData);
		if (err != 0) {
			return false;
		}
		if (LOBYTE(wsaData.wVersion) != 1 ||
			HIBYTE(wsaData.wVersion) != 1) {
			WSACleanup();
			return false;
		}
		sockClient = socket(AF_INET, SOCK_STREAM, 0);
		SOCKADDR_IN addrSrv;
		addrSrv.sin_addr.S_un.S_addr = inet_addr(serverIp.c_str());
		addrSrv.sin_family = AF_INET;
		addrSrv.sin_port = htons(portNo33);
		printf("send connection...\n");
		if (connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR)) == -1)
			return -1;
		printf("connected\n");
		// 禁用NAGLE算法
		const char chOpt = 1;
		setsockopt(sockClient, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(char));
	}

	// no delay
	BOOL bNodelay = TRUE;
	setsockopt(sockClient, SOL_SOCKET, TCP_NODELAY, (char*)&bNodelay, sizeof(BOOL));
	//if (SOCKET_ERROR == setsockopt(sockClient, SOL_SOCKET, TCP_NODELAY, (char*)&bNodelay, sizeof(BOOL)))
	//	return false;
	int nBuffLen = 0;
	SOCKET_ERROR == setsockopt(sockClient, SOL_SOCKET, SO_SNDBUF, (char*)&nBuffLen, sizeof(int));
	//if (SOCKET_ERROR == setsockopt(sockClient, SOL_SOCKET, SO_SNDBUF, (char*)&nBuffLen, sizeof(int)))
	//	return false;
	// set timeout
	int timeout = 3000; //3s
	setsockopt(sockClient, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));
	setsockopt(sockClient, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

	return 0;
}

// send thread
void socketSendThread(char* data, int data_lengh)
{
	int rtn_num = send(sockClient, data, data_lengh, 0);
	cerr << "socket_send_thread: send data to server ... ";
	/*while (!success)
	{
	send(sockSender, data, data_lengh, 0);
	}*/
}

// get robot depth 获取depth后暂停移动准备规划路径
bool getDepthFromServer(){
	// ask for data
	char sendData[1];
	sendData[0] = '0';
	send(sockClient, sendData, 1, 0);
	// rcv data
	int data_len = (480 * 640 * (sizeof(short)) * rbt_num);
	char* rcvData = new char[data_len];
	int n = 0;
	int count = 0;
	char pkgData[MAXRECV];
	// rbt0
	n = 0;
	count = 0;
	while (1){
		if (count + MAXRECV >= data_len){
			n = recv(sockClient, pkgData, data_len - count, 0);
			int rst = n;
			while (rst < data_len - count){
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				rst = data_len - count;
				rst = recv(sockClient, pkgData, rst, 0);
			}
			memcpy(&rcvData[count], &pkgData, rst);
			count += rst;
			break;
		}
		n = recv(sockClient, pkgData, MAXRECV, 0);
		memcpy(&rcvData[count], &pkgData, n);
		count += n;
	}
	// cpy to depth
	int ind = 0;
	for (int id = 0; id < rbt_num; id++)
	{
		for (int i = 0; i < 480; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				memcpy(&depth[id].ptr<short>(i)[j], &rcvData[ind], sizeof(short));
				ind += sizeof(short);
			}
		}
		//cv::imshow("get depth", depth[id]);
		//cv::waitKey(0);
	}

	delete rcvData;
	return true;
}



// rcv depth 接收depth
bool rcvDepthFromServer(){
	// rcv data
	int data_len = (480 * 640 * (sizeof(short)) * rbt_num);
	char* rcvData = new char[data_len];
	int n = 0;
	int count = 0;
	char pkgData[MAXRECV];
	// rbt0
	n = 0;
	count = 0;
	while (1){
		if (count + MAXRECV >= data_len){
			n = recv(sockClient, pkgData, data_len - count, 0);
			int rst = n;
			while (rst < data_len - count){
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				rst = data_len - count;
				rst = recv(sockClient, pkgData, rst, 0);
			}
			memcpy(&rcvData[count], &pkgData, rst);
			count += rst;
			break;
		}
		n = recv(sockClient, pkgData, MAXRECV, 0);
		memcpy(&rcvData[count], &pkgData, n);
		count += n;
	}
	// cpy to depth
	int ind = 0;
	for (int id = 0; id < rbt_num; id++)
	{
		for (int i = 0; i < 480; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				memcpy(&depth[id].ptr<short>(i)[j], &rcvData[ind], sizeof(short));
				ind += sizeof(short);
			}
		}
		//cv::imshow("rcv depth", depth[id]);
		//cv::waitKey(0);
	}
	delete rcvData;
	return true;
}



/* 改为RGBD的接口 */ // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//// get a rgbd frame of robot_index
//bool get_a_frame_from_server(int robot_index){
//
//}

// get rgbd from server
bool getRGBDFromServer(){
	cout << "getting rgbd data ... ";
	// ask for data
	char sendData[1];
	sendData[0] = '3';
	//send(sockClient, sendData, 1, 0);
	thread send_task(socketSendThread, sendData, 1);
	send_task.detach();
	// recive data
	// rgb
	{
		// rcv data
		int data_len = (480 * 640 * 3 * (sizeof(uchar)) * rbt_num);
		char* rcvData = new char[data_len];
		int n = 0;
		int count = 0;
		char pkgData[MAXRECV];
		// rbt0
		n = 0;
		count = 0;
		while (1){
			//if (count == data_len)
			//	break;
			if (count + MAXRECV >= data_len){
				n = recv(sockClient, pkgData, data_len - count, 0);
				//cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
				int rst = n;
				while (rst < data_len - count){
					memcpy(&rcvData[count], &pkgData, rst);
					count += rst;
					rst = data_len - count;
					rst = recv(sockClient, pkgData, rst, 0);
				}
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				break;
			}
			n = recv(sockClient, pkgData, MAXRECV, 0);
			//if (count + n <= data_len)
			memcpy(&rcvData[count], &pkgData, n);
			//else
			//	memcpy(&rcvData[count], &pkgData, data_len - count);
			count += n;
		}
		// cpy to rgb
		int ind = 0;
		for (int id = 0; id < rbt_num; id++)
		{
			for (int i = 0; i < 480; i++)
			{
				for (int j = 0; j < 640; j++)
				{
					memcpy(&rgb[id].ptr<cv::Vec3b>(i)[j][2], &rcvData[ind], sizeof(uchar));
					ind += sizeof(uchar);
					memcpy(&rgb[id].ptr<cv::Vec3b>(i)[j][1], &rcvData[ind], sizeof(uchar));
					ind += sizeof(uchar);
					memcpy(&rgb[id].ptr<cv::Vec3b>(i)[j][0], &rcvData[ind], sizeof(uchar));
					ind += sizeof(uchar);
				}
			}
		}
		delete rcvData;
	}

	// depth
	{
		// rcv data
		int data_len = (480 * 640 * (sizeof(short)) * rbt_num);
		char* rcvData = new char[data_len];
		int n = 0;
		int count = 0;
		char pkgData[MAXRECV];
		// rbt0
		n = 0;
		count = 0;
		while (1){
			if (count + MAXRECV >= data_len){
				n = recv(sockClient, pkgData, data_len - count, 0);
				int rst = n;
				while (rst < data_len - count){
					memcpy(&rcvData[count], &pkgData, rst);
					count += rst;
					rst = data_len - count;
					rst = recv(sockClient, pkgData, rst, 0);
				}
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				break;
			}
			n = recv(sockClient, pkgData, MAXRECV, 0);
			memcpy(&rcvData[count], &pkgData, n);
			count += n;
		}
		// cpy to depth
		int ind = 0;
		for (int id = 0; id < rbt_num; id++)
		{
			for (int i = 0; i < 480; i++)
			{
				for (int j = 0; j < 640; j++)
				{
					memcpy(&depth[id].ptr<ushort>(i)[j], &rcvData[ind], sizeof(short));
					ind += sizeof(short);
				}
			}
		}
		delete rcvData;
	}

	//// test show img
	//for (int id = 0; id < rbt_num; id++)
	//{
	//	cv::imshow("get rgb", rgb[id]);
	//	cv::imshow("get depth", depth[id]);
	//	cv::waitKey(0);
	//}

	cout << "done" << endl;
	return true;
}

// rcv rgbd from server
bool rcvRGBDFromServer(){
	cout << "waitting for rgbd data ... ";
	// recive data
	// rgb
	{
		// rcv data
		int data_len = (480 * 640 * 3 * (sizeof(uchar)) * rbt_num);
		char* rcvData = new char[data_len];
		int n = 0;
		int count = 0;
		char pkgData[MAXRECV];
		// rbt0
		n = 0;
		count = 0;
		while (1){
			//if (count == data_len)
			//	break;
			if (count + MAXRECV >= data_len){
				n = recv(sockClient, pkgData, data_len - count, 0);
				//cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
				int rst = n;
				while (rst < data_len - count){
					memcpy(&rcvData[count], &pkgData, rst);
					count += rst;
					rst = data_len - count;
					rst = recv(sockClient, pkgData, rst, 0);
				}
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				break;
			}
			n = recv(sockClient, pkgData, MAXRECV, 0);
			//if (count + n <= data_len)
			memcpy(&rcvData[count], &pkgData, n);
			//else
			//	memcpy(&rcvData[count], &pkgData, data_len - count);
			count += n;
		}
		// cpy to rgb
		int ind = 0;
		for (int id = 0; id < rbt_num; id++)
		{
			for (int i = 0; i < 480; i++)
			{
				for (int j = 0; j < 640; j++)
				{
					memcpy(&rgb[id].ptr<cv::Vec3b>(i)[j][2], &rcvData[ind], sizeof(uchar));
					ind += sizeof(uchar);
					memcpy(&rgb[id].ptr<cv::Vec3b>(i)[j][1], &rcvData[ind], sizeof(uchar));
					ind += sizeof(uchar);
					memcpy(&rgb[id].ptr<cv::Vec3b>(i)[j][0], &rcvData[ind], sizeof(uchar));
					ind += sizeof(uchar);
				}
			}
		}
		delete rcvData;
	}

	{// test show img
		for (int rid = 0; rid < rbt_num; rid++)
		{
			char name[10];
			sprintf(name, "rgb_%d", rid);
			cv::imshow(name, rgb[rid]);
			cv::waitKey(0);
		}
	}

	// depth
	{
		// rcv data
		int data_len = (480 * 640 * (sizeof(short)) * rbt_num);
		char* rcvData = new char[data_len];
		int n = 0;
		int count = 0;
		char pkgData[MAXRECV];
		// rbt0
		n = 0;
		count = 0;
		while (1){
			//if (count == data_len)
			//	break;
			if (count + MAXRECV >= data_len){
				n = recv(sockClient, pkgData, data_len - count, 0);
				//cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
				int rst = n;
				while (rst < data_len - count){
					memcpy(&rcvData[count], &pkgData, rst);
					count += rst;
					rst = data_len - count;
					rst = recv(sockClient, pkgData, rst, 0);
				}
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				break;
			}
			n = recv(sockClient, pkgData, MAXRECV, 0);
			//if (count + n <= data_len)
			memcpy(&rcvData[count], &pkgData, n);
			//else
			//	memcpy(&rcvData[count], &pkgData, data_len - count);
			count += n;
		}
		// cpy to depth
		int ind = 0;
		for (int id = 0; id < rbt_num; id++)
		{
			for (int i = 0; i < 480; i++)
			{
				for (int j = 0; j < 640; j++)
				{
					memcpy(&depth[id].ptr<ushort>(i)[j], &rcvData[ind], sizeof(short));
					ind += sizeof(short);
				}
			}
		}
		delete rcvData;
	}

	// test show img
	for (int id = 0; id < rbt_num; id++)
	{
		cv::imshow("get rgb", rgb[id]);
		cv::imshow("get depth", depth[id]);
		cv::waitKey(0);
	}

	cout << "done" << endl;
	return true;
}

// ask to set up surroundings, use to initialization
void scanSurroundingsCmd(){
	// ask for data
	char sendData[1];
	sendData[0] = '5';
	send(sockClient, sendData, 1, 0);
}

// rand pose
void randDisturbPose(){
	for (int rid = 0; rid < rbt_num; rid++)
	{
		if (rand() % 2 == 1)
			pose[rid][0] += (double)(rand() % 10) / 100;
		else
			pose[rid][0] -= (double)(rand() % 10) / 100;
		if (rand() % 2 == 1)
			pose[rid][1] += (double)(rand() % 10) / 100;
		else
			pose[rid][1] -= (double)(rand() % 10) / 100;
	}
}

// move to views
bool socket_move_to_views(vector<vector<double>> poses){

	/* ask to set poses */
	char sendData[1];
	sendData[0] = 'm';
	send(sockClient, sendData, 1, 0);
	Sleep(100);

	/* send poses data */
	int data_len = rbt_num * 7 * sizeof(float);
	char* poseData = new char[data_len];
	int ind = 0;
	for (int rid = 0; rid < rbt_num; rid++)
	{
		for (int i = 0; i < 7; i++)
		{
			float pass = (float)poses[rid][i];
			memcpy(&poseData[ind], &pass, sizeof(float));
			ind += sizeof(float);
		}
	}
	//success = false;
	//thread send_task(socketSendThread, poseData, data_len);
	//send_task.detach();
	int rtn_num = send(sockClient, poseData, data_len, 0);

	/* debug check */
	//{
	//	ind = 0;
	//	for (int rid = 0; rid < rbt_num; rid++)
	//	{
	//		cerr << "pose for robot" << rid << ": ";
	//		for (int i = 0; i < 7; i++)
	//		{
	//			float test;
	//			memcpy(&test, &poseData[ind], sizeof(float));
	//			cerr << test << " ";
	//			ind += sizeof(float);
	//		}
	//		cerr << endl;
	//	}
	//}

	/* free */
	delete poseData;
	return true;

}