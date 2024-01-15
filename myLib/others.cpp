#include "others.hpp"



Other::Other()
{
    this->num = -1;
    this->apper_times = 0;
    this->disap_times = 0;
}

Other::~Other()
{
}

void Other::update(cv::Rect newRect)
{
    this->disap_times = 0;
    this->apper_times++;
    this->rect = rect;
}

void Other::disapper()
{
    this->apper_times = 0;
    this->disap_times++;
}
