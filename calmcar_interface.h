/*************************************************************************
    > File Name: calmcar_interface.h
    > Author: zgp
    > Mail: hebzgp@foxmail.com
    > Created Time: 2018-2-8 10:10:49
 ************************************************************************/
#ifndef CALMCAR_INTERFACE_H
#define CALMCAR_INTERFACE_H
#include <string>
#include "data_frame.h"
#include <opencv2/core/core.hpp>

namespace calmcarinterface{
class calmcar_parser;
class TCPClient;
class calmcar_data;
class calmcar_interface
{
public:
    calmcar_interface();
    ~calmcar_interface();
    int connect(const std::string&,const short&);
    int parsedbc(const std::string&);
    int getframe();
    int getimage(cv::Mat&);
	int getinfo(DataFrame&);
private:
    calmcar_parser* parser=nullptr;
    TCPClient* client=nullptr;
    calmcar_data* calmcardata=nullptr;
};
}


#endif // CALMCAR_INTERFACE_H
