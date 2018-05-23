

/* 虚拟环境通讯的接口 */
#include <iostream>
#include <Windows.h>  
#include <thread>  
#include <mutex>  
// socket
#include <Winsock.h>
#pragma comment(lib, "ws2_32.lib")
// OpenCV
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

const int rbt_num = 1;
//rgbd image
vector<cv::Mat> depth;
vector<cv::Mat> rgb;
//socket set
unsigned short portNo33 = 3333; // sockClient
unsigned short portNo66 = 6666; // sockSender
const int MAXRECV = 10240;
string serverIp = "192.168.1.114";


SOCKET initializeDataEngine(string ip, unsigned short port){

	/* sockClient */// recv socket
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
		SOCKET sockClient = socket(AF_INET, SOCK_STREAM, 0);
		SOCKADDR_IN addrSrv;//存储套接字地址信息
		addrSrv.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
		addrSrv.sin_family = AF_INET;
		addrSrv.sin_port = htons(port);
		printf("send connection...\n");
		printf("IP=%s    PORT=%d\n", ip.c_str(), port);
		if (connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR)) == -1){
			printf("fail connected\n");
			getchar();
		}
		printf("connected\n");
		// 禁用NAGLE算法
		const char chOpt = 1;
		setsockopt(sockClient, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(char));
	

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

	return sockClient;
}

bool rcvRGBDFromServer(SOCKET s){
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
		//delete pkgData;
		// rbt0
		n = 0;
		count = 0;
		while (1){
			//if (count == data_len)
			//	break;
			//printf("%d,%d,%d\n", n, count, data_len);
			if (count + MAXRECV >= data_len){//内存还有空间
				n = recv(s, pkgData, data_len - count, 0);
				
				//cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
				int rst = n;
				while (rst < data_len - count){
					memcpy(&rcvData[count], &pkgData, rst);
					count += rst;
					rst = data_len - count;
					rst = recv(s, pkgData, rst, 0);
				}
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				break;
			}
			n = recv(s, pkgData, MAXRECV, 0);
			//printf("%d\n", n);
			//if (count + n <= data_len)
				memcpy(&rcvData[count], &pkgData, n);
			//else
			//	memcpy(&rcvData[count], &pkgData, data_len - count);
			count += n;
			//delete pkgData;
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
				n = recv(s, pkgData, data_len - count, 0);
				//cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
				int rst = n;
				while (rst < data_len - count){
					memcpy(&rcvData[count], &pkgData, rst);
					count += rst;
					rst = data_len - count;
					rst = recv(s, pkgData, rst, 0);
				}
				memcpy(&rcvData[count], &pkgData, rst);
				count += rst;
				break;
			}
			n = recv(s, pkgData, MAXRECV, 0);
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

void thread_img(string IP, unsigned short port){
	
	/* init rgb */
	//for (int i = 0; i < rbt_num; i++) //初始化rgb序列
		rgb.push_back(cv::Mat(480, 640, CV_8UC3));

	/* init depth */
	//for (int i = 0; i < rbt_num; i++) //初始化depth序列
		depth.push_back(cv::Mat(480, 640, CV_16UC1));

	printf("Connect information IP=%s    PORT=%d\n", IP.c_str(), port);

	SOCKET Client = initializeDataEngine(IP, port);

	rcvRGBDFromServer(Client);

	//return rgb, depth;
}


int main(int argc, char* argv[]){

	
	//for (int i = 0; i < 3; i++){
		thread task(thread_img, serverIp, portNo33);
	//}
	//thread task(thread_img, serverIp, portNo33);
		task.join();
	return 0;


}
// rcv rgbd from server
