#ifndef _CAR_MODELS_HPP_
#define _CAR_MODELS_HPP_

#include <vector>
#include <string>
#include <map>
#include "network.hpp"

struct bodycolor
{
    float rgb[3];
};

struct car_model
{
    bool load_from_xml(const char *filename);
    bool xml_read(xmlTextReaderPtr reader);

    void swap(car_model &o);

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

struct car_db
{
    bool add_file(const char *file);
    int  add_dir(const char *dir);

    std::map<const std::string, car_model> db;
};

#endif
