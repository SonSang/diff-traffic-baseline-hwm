#ifndef _CAR_MODELS_HPP_
#define _CAR_MODELS_HPP_

#include <vector>
#include <string>
#include "network.hpp"

struct bodycolor
{
    float rgb[3];
};

struct car_model
{
    bool load_from_xml(const char *filename);
    bool xml_read(xmlTextReaderPtr reader);

    std::string id;
    std::string make;
    std::string model;
    std::string year;

    std::string body_mesh;
    std::string wheel_mesh;
    std::string wheel_flip_mesh;

    float wheel_diameter;
    float wheel_points[4][3];

    float z_offset;

    std::vector<bodycolor> body_colors;
};

#endif
