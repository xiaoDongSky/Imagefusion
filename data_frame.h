/*************************************************************************
    > File Name: data_frame.h
    > Author: zgp
    > Mail: hebzgp@foxmail.com
    > Created Time: 2018-2-8 10:12:31
 ************************************************************************/
#ifndef DATA_FRAME_H
#define DATA_FRAME_H
#include <vector>
#include <opencv2/core/core.hpp>
namespace calmcarinterface {
typedef enum { FUGA = 0, GS = 1, RX5 = 2, GOLF = 3, MAGOTAN = 4 } CarType;

struct ObjectAttr {
  char class_id;
  unsigned char object_id;
  unsigned short distance_y;
  short distance_x;
  unsigned short object_width;
  short relative_speed;
#ifndef TJU
  char ttc;
  char score;
  short x;
  short y;
  short width;
  short height;
  short angle_left;
  short angle_right;
#endif
};

struct LdwInfoCurve {
    float C0 = 0.0;
    float C1 = 0.0;
    float C2 = 0.0;
    float C3 = 0.0;
    short top_x = 0;
    short top_y = 0;
    short bottom_x = 0;
    short bottom_y = 0;
    char linetype = 0x0;
#ifndef TJU
    char quality = 0x0;
    char dis_to_car = 0x0;
    char line_width = 0x0;
    char line_index = 0x0;
    char line_start_dis = 0x0;
    char line_end_dis = 0x0;
#endif
};

struct CarInfo {
  float vehicle_speed = -1;
  float steer_angle = 0.0;
  short brake = 0;
  unsigned short engine_speed = 0;
  float accelerated_speed = 0.0;
  unsigned int mileage = 0;
  unsigned short oil_consumption = 0;  //油耗
  short wiper = 0;                     //雨刷
  short accelerograph = 0;             //油门
  char* gear;
  char* indication;
  short brake_info = 0;
};

struct GpsInfo
{
  float longitude = -1.0;
  float latitude = -1.0;
  float height = -1.0;
  float angle = -1.0;
  short Year = -1;
  short Month = -1;
  short Date = -1;
  short Hour = -1;
  short Minute = -1;
  short Second = -1;
  short Millisecond = -1;
  float speed = -1.0;
};

struct DataFrame
{
    DataFrame();
    ~DataFrame();
	DataFrame(const DataFrame&);
	DataFrame operator=(const DataFrame&);
    std::vector<ObjectAttr> objects;
    std::vector<LdwInfoCurve> ldws_curve;
    CarInfo carInfo;
    GpsInfo gps;
#ifndef TJU
    cv::Mat img;
#endif
};
}
#endif // DATA_FRAME_H
