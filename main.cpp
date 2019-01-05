/*************************************************************************
    > File Name: image_fusion
    > Author: HengZhang
    > Created Time: 2018-04-04 09:24:20
 ************************************************************************/
#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <string.h>
#include <cstdlib>
#include <vector>
#include "unistd.h"
#include "CCarCamCal.h"
#include "calmcar_interface.h"
#include <ctime>
#include <math.h>  
#include <fstream> 



#define PI 3.1415926 

using namespace std;
using namespace cv;
using namespace calmcarinterface;

VCI_BOARD_INFO pInfo;//用来获取设备信息。

struct feature
{
	int id;
	float distance_x;
	float distance_y;
	float speed;
	float angle;
};

std::vector<feature> information_list;

void *receive_func()  
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int j;
    int ind=0;
	if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,5))>0)//调用接收函数，如果有数据，进行数据处理显示,返回实际读取的帧数
	{
		printf("I am going to receive dada!\n");
		for(j=0;j<reclen;j++)
		{   
			if (rec[j].ID == 0x0000060A)
			{
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//CAN ID
				printf("\n");
				int Object_NofObjects;
				Object_NofObjects=rec[j].Data[0];
				printf("the number of Object is %d\n",Object_NofObjects);
			}
			
			
			
			else if (rec[j].ID == 0x0000060B)
			{	
				feature information;
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//CAN ID
				printf("\n");
				int Object_ID;
				Object_ID = rec[j].Data[0];
				printf("Object_ID is %d\n",Object_ID);
				information.id = Object_ID;
				union {
					unsigned short Object_DistLong;
					unsigned short Object_DistLat;
					unsigned short Object_VrelLong;
					unsigned short Object_VrelLat;
					unsigned char  Object_DistLong_tmp[2];
					unsigned char  Object_DistLat_tmp[2];
					unsigned char  Object_VrelLong_tmp[2];
					unsigned char  Object_VrelLat_tmp[2];
				}trans;

				trans.Object_DistLong_tmp[1] = rec[j].Data[1];
				trans.Object_DistLong_tmp[0] = rec[j].Data[2];
				trans.Object_DistLong >>= 3;
				printf("data1: %x,data2: %x,trans.Object_DistLong: %x\n", rec[j].Data[1], rec[j].Data[2], trans.Object_DistLong);
				printf("Object_DistLong is %f\n",trans.Object_DistLong * 0.2 - 500);
				information.distance_y = trans.Object_DistLong * 0.2 - 500;
				

				trans.Object_DistLat_tmp[1] = rec[j].Data[2];
				trans.Object_DistLat_tmp[0] = rec[j].Data[3];
				trans.Object_DistLat &= 0x07FF;
				printf("data2: %x,data3: %x,trans.Object_DistLat: %x\n", rec[j].Data[2], rec[j].Data[3], trans.Object_DistLat);
				printf("Object_DistLat is %f\n", trans.Object_DistLat * 0.2 - 204.6);
			    information.distance_x = trans.Object_DistLat * 0.2 - 204.6;
			    
				trans.Object_VrelLong_tmp[1] = rec[j].Data[4];
				trans.Object_VrelLong_tmp[0] = rec[j].Data[5]; 
				trans.Object_VrelLong &= 0xFFC0;
				trans.Object_VrelLong >>= 6;
				printf("data4: %x,data5: %x,trans.Object_VrelLong: %x\n", rec[j].Data[4], rec[j].Data[5], trans.Object_VrelLong);
				printf("Object_VrelLong is %f\n", trans.Object_VrelLong * 0.25 - 128);
				information.speed = trans.Object_VrelLong * 0.25 - 128;

				trans.Object_VrelLat_tmp[1] = rec[j].Data[5];
				trans.Object_VrelLat_tmp[0] = rec[j].Data[6];
				trans.Object_VrelLat &= 0x3FD0;
				trans.Object_VrelLat >>= 5;
				printf("data5: %x,data6: %x,trans.Object_VrelLat: %x\n", rec[j].Data[5], rec[j].Data[6], trans.Object_VrelLat);
				printf("Object_VrelLat is %f\n", trans.Object_VrelLat * 0.25 - 64);

				unsigned char Object_DynProp;
				Object_DynProp = rec[j].Data[6];
				Object_DynProp &= 0x07;	
				printf("data7: %x,Object_DynProp: %x\n", rec[j].Data[6], Object_DynProp);
				switch(Object_DynProp)
				{
					case 0x0:
						printf("moving\n");
						break;
					case 0x1:
						printf("stationary\n");
						break;
					case 0x2:
						printf("oncoming\n");
						break;
					case 0x3:
						printf("stationary candidate\n");
						break;
					case 0x4:
						printf("unknown\n");
						break;
					case 0x5:
						printf("crossing stationary\n");
						break;
					case 0x6:
						printf("crossing moving\n");
						break;
					case 0x7:
						printf("stopped\n");
						break;
					default:
						break;		
				}
                float Object_RCS = 0;
				Object_RCS = rec[j].Data[7];
				printf("Object_RCS is %f\n\n\n\n", Object_RCS * 0.5 - 64);
                information_list.push_back(information);

			}
		}
	}
	return 0;			
}


void Init_Send()
{
	//初始化参数，严格参数二次开发函数库说明书
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
    if(VCI_ClearBuffer(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Clearbuffer CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	//需要发送的帧，结构体设置	
	VCI_CAN_OBJ send[1];
	send[0].ID=0x00000200;
	send[0].SendType=1;//二次开发,建议SendType=1,提高发送的响应速度
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;
	
	int count =0;
	send[0].Data[0] = 0x3D;	
	send[0].Data[1] = 0xFF;
	send[0].Data[2] = 0xC0;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0x68;
	send[0].Data[5] = 0x00;//0C接收扩展信息
	send[0].Data[6] = 0x00;
	send[0].Data[7] = 0x00;

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
		printf("Index:%04d  ",count);count++;
		printf("CAN1 TX ID:0x%08X",send[0].ID);
		if(send[0].ExternFlag==0) printf(" Standard ");
		if(send[0].ExternFlag==1) printf(" Extend   ");
		if(send[0].RemoteFlag==0) printf(" Data   ");
		if(send[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send[0].DataLen;i++)
		{
			printf(" %02X",send[0].Data[i]);
		}

		printf("\n");
	}
	usleep(5000);

	//0x00000202	
    VCI_CAN_OBJ send_Distance[1];
	send_Distance[0].ID=0x00000202;
	send_Distance[0].SendType=1;
	send_Distance[0].RemoteFlag=0;
	send_Distance[0].ExternFlag=0;
	send_Distance[0].DataLen=5;
	
	int count1 =0;
	send_Distance[0].Data[0] = 0x8E;	
	send_Distance[0].Data[1] = 0x00;
	send_Distance[0].Data[2] = 0x00;
	send_Distance[0].Data[3] = 0x03;
	send_Distance[0].Data[4] = 0x52;//现在是过滤85米,0x03;0x52;//现在是过滤200米,0x07;0xD0;/若过滤100米0x03;0xE8;
	
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Distance, 1) == 1)
	{
		printf("Index:%04d  ",count1);count1++;
		printf("CAN1 TX ID:0x%08X",send_Distance[0].ID);
		if(send_Distance[0].ExternFlag==0) printf(" Standard ");
		if(send_Distance[0].ExternFlag==1) printf(" Extend   ");
		if(send_Distance[0].RemoteFlag==0) printf(" Data   ");
		if(send_Distance[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Distance[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Distance[0].DataLen;i++)
		{
			printf(" %02X",send_Distance[0].Data[i]);
		}

		printf("\n");
	}
	usleep(5000);
    //0x202配置扫描的方位角（范围）
	VCI_CAN_OBJ send_Azimuth[1];
	send_Azimuth[0].ID=0x00000202;
	send_Azimuth[0].SendType=1;
	send_Azimuth[0].RemoteFlag=0;
	send_Azimuth[0].ExternFlag=0;
	send_Azimuth[0].DataLen=5;
	
	int count2 =0;
	send_Azimuth[0].Data[0] = 0x96;	
	send_Azimuth[0].Data[1] = 0x03;
	send_Azimuth[0].Data[2] = 0x20;
	send_Azimuth[0].Data[3] = 0x0C;
	send_Azimuth[0].Data[4] = 0x80;//现在filter的是±30°,0x96;0x03;0x20;0x0C;0x80;//±45°,0x96;0x00;0xC8;0x0E;0xD8;若是filter一个车道，应该为±10°,0x96,0x06,0x40,0x09,0x60,//±15°:0x96,0x05,0x78,0x0A,0x28,

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Azimuth, 1) == 1)
	{
		printf("Index:%04d  ",count2);count2++;
		printf("CAN1 TX ID:0x%08X",send_Azimuth[0].ID);
		if(send_Azimuth[0].ExternFlag==0) printf(" Standard ");
		if(send_Azimuth[0].ExternFlag==1) printf(" Extend   ");
		if(send_Azimuth[0].RemoteFlag==0) printf(" Data   ");
		if(send_Azimuth[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Azimuth[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Azimuth[0].DataLen;i++)
		{
			printf(" %02X",send_Azimuth[0].Data[i]);
		}

		printf("\n");
	}
    usleep(5000);
    
    
    //0x00000202
	VCI_CAN_OBJ send_RCS[1];
	send_RCS[0].ID=0x00000202;
	send_RCS[0].SendType=1;
	send_RCS[0].RemoteFlag=0;
	send_RCS[0].ExternFlag=0;
	send_RCS[0].DataLen=5;
	
	int count3 =0;
	send_RCS[0].Data[0] = 0xAE;	
	send_RCS[0].Data[1] = 0x04;
	send_RCS[0].Data[2] = 0xB0;
	send_RCS[0].Data[3] = 0x0C;
	send_RCS[0].Data[4] = 0x80;//最小值为0的话:0xAE,0x07,0xD0;最小值为-15的话:0xAE,0x05,0x78;现在filter的是5.1dBm2:/AE,08,9C,/和30dBm2:/0C,80/之间的以上的object.8dBm2://AE,09,10,/

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_RCS, 1) == 1)
	{
		printf("Index:%04d  ",count3);count3++;
		printf("CAN1 TX ID:0x%08X",send_RCS[0].ID);
		if(send_RCS[0].ExternFlag==0) printf(" Standard ");
		if(send_RCS[0].ExternFlag==1) printf(" Extend   ");
		if(send_RCS[0].RemoteFlag==0) printf(" Data   ");
		if(send_RCS[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_RCS[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_RCS[0].DataLen;i++)
		{
			printf(" %02X",send_RCS[0].Data[i]);
		}

		printf("\n");
	}
	usleep(5000);

	//0x00000202
	VCI_CAN_OBJ send_Lifetime[1];
	send_Lifetime[0].ID=0x00000202;
	send_Lifetime[0].SendType=1;
	send_Lifetime[0].RemoteFlag=0;
	send_Lifetime[0].ExternFlag=0;
	send_Lifetime[0].DataLen=3;
	
	int count4 =0;
	send_Lifetime[0].Data[0] = 0xB6;	
	send_Lifetime[0].Data[1] = 0x00;
	send_Lifetime[0].Data[2] = 0x02;//200ms

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Lifetime, 1) == 1)
	{
		printf("Index:%04d  ",count4);count4++;
		printf("CAN1 TX ID:0x%08X",send_Lifetime[0].ID);
		if(send_Lifetime[0].ExternFlag==0) printf(" Standard ");
		if(send_Lifetime[0].ExternFlag==1) printf(" Extend   ");
		if(send_Lifetime[0].RemoteFlag==0) printf(" Data   ");
		if(send_Lifetime[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Lifetime[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Lifetime[0].DataLen;i++)
		{
			printf(" %02X",send_Lifetime[0].Data[i]);
		}

		printf("\n");
	}
	usleep(5000);
    /*
	//0x00000202
	VCI_CAN_OBJ send_Size[1];
	send_Size[0].ID=0x00000202;
	send_Size[0].SendType=1;
	send_Size[0].RemoteFlag=0;
	send_Size[0].ExternFlag=0;
	send_Size[0].DataLen=5;
	
	int count5 =0;
	send_Size[0].Data[0] = 0xBE;	
	send_Size[0].Data[1] = 0x00;
	send_Size[0].Data[2] = 0x28;
	send_Size[0].Data[3] = 0x1F;
	send_Size[0].Data[4] = 0xFF;//现在filter的是1㎡以上的object

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Size, 1) == 1)
	{
		printf("Index:%04d  ",count5);count5++;
		printf("CAN1 TX ID:0x%08X",send_Size[0].ID);
		if(send_Size[0].ExternFlag==0) printf(" Standard ");
		if(send_Size[0].ExternFlag==1) printf(" Extend   ");
		if(send_Size[0].RemoteFlag==0) printf(" Data   ");
		if(send_Size[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Size[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Size[0].DataLen;i++)
		{
			printf(" %02X",send_Size[0].Data[i]);
		}

		printf("\n");
	}
	*/
}

bool Radar_Calibration(cv::Point2f radarpoint_in,float incline_angle,cv::Point3f worldpoint_out)
{
	//此处对应的是逆时针旋转角度为正，顺时针为负
	float incline_ag = 0;
	incline_ag = incline_angle*PI/180;
    if(incline_angle>=0)
    {
    	worldpoint_out = cv::Point3f(radarpoint_in.x*cos(incline_ag)+radarpoint_in.y*sin(incline_ag),radarpoint_in.y*cos(incline_ag)-radarpoint_in.x*sin(incline_ag),0);
    }
    else
    {
    	worldpoint_out = cv::Point3f(radarpoint_in.x*cos(incline_ag)-radarpoint_in.y*sin(incline_ag),radarpoint_in.y*cos(incline_ag)+radarpoint_in.x*sin(incline_ag),0);
    }
    return true;
}

int main(int argc, char* argv[])
{
	printf(">>this is hello !\r\n");

	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)
	{
		printf(">>open deivce success!\n");
	}
	else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}

	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)
	{
        printf(">>Get VCI_ReadBoardInfo success!\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");
		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");	
	}
	else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}	
    Init_Send();

	printf("I am Start!\n");

    calmcar_interface *interface = new calmcar_interface();

	interface->parsedbc(argv[1]);

	int connectresult = interface->connect("192.168.198.93", 8000);
	std::cout << connectresult << std::endl;
	if (connectresult == -1)
		return -1;
	cv::Mat image;
	cv::namedWindow("Test", CV_WINDOW_AUTOSIZE);
	DataFrame dataframe;

    double rate = 12.0;//fps
	Size videoSize(1280,1080);
	VideoWriter writer("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, videoSize);

	const char *intrinsicXmlName = "cameraIntrinsic.xml";
    const char *extrinsicXmlName = "cameraExtrinsic.xml";

    VisualDistance *vd = new VisualDistance;
    vd->LoadCamPara(intrinsicXmlName,extrinsicXmlName);

    ofstream out("out.txt");
    ofstream out_ldw("out_ldw.txt");
	// if(out.is_open())
	// {
	//     out<<"HELLO WORLD!"<<endl;
	//     out.close();
	// }
	int frameindex = 1;
	while(true)
	{
		printf("\n\n\nI'm going to start reading radar!\n");
			
		receive_func();		
		for(unsigned int i=0;i<information_list.size();i++)
		{
			printf("%d %f %f %f\n", information_list[i].id,information_list[i].distance_x,information_list[i].distance_y,information_list[i].speed);
		}	

		int result = interface->getframe();
		if (result == -1)
			continue;
		result = interface->getinfo(dataframe);
		if (result == -1)
		{
			std::cout << "getinfo returned -1" << std::endl;
			continue;
		}
		int readimg = interface->getimage(image);
		if (readimg == -1)
			continue;
		//printf("The size of objects detected by camera is:%d.\n",dataframe.objects.size());
        //cv::circle(image, cvPoint(663,877),12,Scalar( 0, 0, 255),-1,8);
        char text_gps1[100];
        sprintf(text_gps1, "Long,%10.6f,Lat,%10.6f,H,%4.2f",dataframe.gps.longitude,dataframe.gps.latitude,dataframe.gps.height);
        char text_gps[100];
		char text_speed[100];
		char text_date[100];
		sprintf(text_gps, "Long:%10.6f Lat:%10.6f H:%4.2f",dataframe.gps.longitude,dataframe.gps.latitude,dataframe.gps.height);
		sprintf(text_speed, "Vehicle Speed:%4.2fKm/h",dataframe.carInfo.vehicle_speed);
		sprintf(text_date, "%d-%d-%d %d:%d:%d",dataframe.gps.Year,dataframe.gps.Month,dataframe.gps.Date,dataframe.gps.Hour,dataframe.gps.Minute,dataframe.gps.Second);
		cv::putText(image, text_gps, cvPoint(860,20), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(0,255,0), 1, 8);
        cv::putText(image, text_speed, cvPoint(860,20+22), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(0,255,0), 1, 8);
        cv::putText(image, text_date, cvPoint(860,20+44), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(0,255,0), 1, 8);
        for (const auto& itor : dataframe.ldws_curve)
        {
        	out_ldw <<"frame_index,"<< frameindex <<",C0," << itor.C0 << ",C1," << itor.C1 << ",C2," << itor.C2 << ",C3," << itor.C3 << ",top_x," << itor.top_x << ",top_y," << itor.top_y << ",bottom_x," << itor.bottom_x << ",bottom_y," << itor.bottom_y << ",linetype," << (int)itor.linetype << endl;
        }
		for (const auto& itor : dataframe.objects)
		{
			bool processed = false;
			std::vector<feature> list;
            for(unsigned int i = 0;i< information_list.size();i++)  
	        {  
	       	    cv::Point3f object_world = cv::Point3f(-information_list[i].distance_x*1000,information_list[i].distance_y*1000,0);
	       	    printf("雷达传入的世界坐标为:(%f,%f,%f)\n", object_world.x,object_world.y,object_world.z);
	       	    cv::Point2f headXY = vd-> m_headLocation;
	       	    object_world.x = object_world.x + headXY.x;
	       	    object_world.y = object_world.y + headXY.y;
	       	    cv::Point2f object_image;
	       	    vd->GetImgPointInWorld(object_world,object_image);
	       	    printf("雷达投影到图像上的目标点坐标为：(%d,%d)\n",(int)object_image.x,(int)object_image.y);
	       	    printf("该目标摄像机检测到的图像上坐标为:(%d,%d)\n\n",(int)itor.x,(int)itor.y); 
	        	if(((int)object_image.x>=(int)itor.x+6) && ((int)object_image.x<=((int)itor.x +(int)itor.width-6)) && ((int)object_image.y>=(int)itor.y) && ((int)object_image.y<=((int)itor.y +(int)itor.height*1.9)))
	        	{
	        		list.push_back(information_list[i]);
	        		processed = true;
	        	}
	        } 
            if(processed)//successful match
            {
            	float min = 0;
            	min = list[0].distance_y;
	        	int k = 0;
	        	for(unsigned int j = 0; j<list.size();j++)
	        	{
	        		if(list[j].distance_y<min)
	        		{
	        			k = j;
	        			min =  list[j].distance_y;
	        		}
	        	}
	        	std::cout << "Radar Object dis_x:" << -list[k].distance_x << "Radar Object dis_y:"<<list[k].distance_y << "Radar speed:"<< list[k].speed << "Radar angle:" << list[k].angle << endl;
                char text[100];
                char text1[100];
                char text2[100];
        	    sprintf(text, "Y:%3.2f",list[k].distance_y);
        	    sprintf(text1, "X:%3.2f",-list[k].distance_x);
        	    sprintf(text2, "V:%3.2f",list[k].speed);
                cv::circle(image, cv::Point((int)itor.x+(int)itor.width/2,(int)itor.y+(int)itor.height/2),10,Scalar( 0, 0, 255),-1,8);
	        	cv::putText(image, text, cv::Point((int)itor.x,(int)itor.y-44), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8);
	        	cv::putText(image, text1, cv::Point((int)itor.x,(int)itor.y-24), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8);
	        	cv::putText(image, text2, cv::Point((int)itor.x,(int)itor.y-4), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8);
	        	std::cout << "Class ID:"<< (int)itor.class_id <<"Object ID:" << (int)itor.object_id << "	Object dis_x:" << -(float)itor.distance_x / 100.0 << "	Object dis_y:" << (float)itor.distance_y / 100.0 << (float)itor.object_width << std::endl;
				cv::Rect rect((int)itor.x, (int)itor.y, (int)itor.width, (int)itor.height);
				if((int)itor.class_id == 3)
                {
                	cv::rectangle(image, rect, cv::Scalar(18,153,255), 2);
                }
                else
                {
                	cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 2);
                }
                out <<"frame_index,"<< frameindex <<",ID,"<< (int)itor.object_id <<",Y," << list[k].distance_y << ",X,"<< -list[k].distance_x << ",relative_speed,"<< list[k].speed <<","<<(int)itor.x<<","<<(int)itor.y<<","<<(int)itor.width<<","<<(int)itor.height <<",object_width," << (float)itor.object_width << ",vehicle speed," << (float)dataframe.carInfo.vehicle_speed <<","<< text_gps1<< endl;
                /*
                char text_calmcar[100];
			    char text_calmcar1[100];
        	    sprintf(text_calmcar, "Y:%3.2f",(float)itor.distance_y / 100.0);
        	    sprintf(text_calmcar1, "X:%3.2f",-(float)itor.distance_x / 100.0);
        	    cv::putText(image, text_calmcar, cv::Point((int)itor.x+99,(int)itor.y-43), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0,0,255), 2, 8);
        	    cv::putText(image, text_calmcar1, cv::Point((int)itor.x+99,(int)itor.y-24), cv::FONT_HERSHEY_SIMPLEX,0.8, CV_RGB(0,0,255), 2, 8);
	        	*/
	        	/*
	        	//输出转换后的点
	        	cv::Point3f obj_world = cv::Point3f(-list[k].distance_x,list[k].distance_y,0);
	        	cv::Point2f obj_image;
	        	cv::Point2f head = vd-> m_headLocation;
	       	    obj_world.x = obj_world.x + head.x;
	       	    obj_world.y = obj_world.y + head.y;
	       	    vd->GetImgPointInWorld(obj_world,obj_image);
	       	    cv::circle(image,obj_image,10,Scalar( 0, 0, 0),-1,8);
	       	    cv::putText(image, text, cv::Point(obj_image.x+4,obj_image.y), cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0), 0.1, 8);
	       	   */ 
            }  
            list.clear();  	
            if(!processed)//match failure
            {
            	if(((float)itor.distance_x / 100.0 <= 4.5) && ((float)itor.distance_x / 100.0 >= -4.5) && ((float)itor.distance_y / 100.0 <= 85))
            	{
            		char txt[100];
	                char txt1[100];
	                char txt2[100];
	        	    sprintf(txt, "Y:%3.2f",(float)itor.distance_y / 100.0);
	        	    sprintf(txt1, "X:%3.2f",-(float)itor.distance_x / 100.0);
	        	    sprintf(txt2, "V:%3.2f",(float)itor.relative_speed);
	                cv::circle(image, cv::Point((int)itor.x+(int)itor.width/2,(int)itor.y+(int)itor.height/2),10,Scalar(0,0,255),-1,8);
		        	cv::putText(image, txt, cv::Point((int)itor.x,(int)itor.y-44), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255,0,0), 2, 8);
		        	cv::putText(image, txt1, cv::Point((int)itor.x,(int)itor.y-24), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255,0,0), 2, 8);
		        	cv::putText(image, txt2, cv::Point((int)itor.x,(int)itor.y-4), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255,0,0), 2, 8);
		        	//std::cout << "Class ID:"<< (int)itor.class_id <<"Object ID:" << (int)itor.object_id << " Object dis_x:" << (float)itor.distance_x / 100.0 << "	Object dis_y:" << (float)itor.distance_y / 100.0 << (float)itor.object_width << std::endl;
					cv::Rect rect((int)itor.x, (int)itor.y, (int)itor.width, (int)itor.height);
					if((int)itor.class_id == 3)
	                {
	                	cv::rectangle(image, rect, cv::Scalar(18,153,255), 2);
	                }
	                else
	                {
	                	cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 2);
	                }
	                out <<"frame_index,"<< frameindex <<",ID,"<< (int)itor.object_id <<",Y," << (float)itor.distance_y/ 100.0 << ",X,"<< -(float)itor.distance_x / 100.0 << ",relative_speed,"<< (float)itor.relative_speed <<","<<(int)itor.x<<","<<(int)itor.y<<","<<(int)itor.width<<","<<(int)itor.height <<",object_width," << (float)itor.object_width << ",vehicle speed," << (float)dataframe.carInfo.vehicle_speed <<","<< text_gps1<< endl;
	            }
            }
		}
		printf("vehicle speed:%f\n",dataframe.carInfo.vehicle_speed);
		printf("GPS longitude:%f\n",dataframe.gps.longitude);
		information_list.clear(); 	
        frameindex++;
        writer << image;
        if(!image.empty())
        	cv::imshow("Test", image);
		cv::waitKey(20);		
    }

    usleep(5000);
	VCI_CloseDevice(VCI_USBCAN2,0);//close device。
	return 0;
}
