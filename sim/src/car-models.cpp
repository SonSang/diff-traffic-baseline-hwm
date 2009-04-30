#include "car-models.hpp"
#include "boost/algorithm/string.hpp"
#include <sstream>

bool car_model::load_from_xml(const char *filename)
{
    xmlTextReaderPtr reader = xmlReaderForFile(filename, NULL, XML_PARSE_XINCLUDE);
    if(!reader)
    {
        fprintf(stderr, "Couldn't open %s for reading\n", filename);
        return false;
    }

    int ret = xmlTextReaderRead(reader);
    while (ret == 1 && !is_opening_element(reader, "car-model"))
        ret = xmlTextReaderRead(reader);

    if(ret != 1)
    {
        if(ret == -1)
            fprintf(stderr, "Parsing error in %s\n", filename);
        else if(ret == 0)
            fprintf(stderr, "Couldn't find car-model element in %s\n", filename);

        xmlFreeTextReader(reader);
        return false;
    }

    xmlNodePtr np = xmlTextReaderCurrentNode(reader);
    if(!np)
    {
        fprintf(stderr, "Bad node pointer!\n");
        xmlFreeTextReader(reader);
        return false;
    }

    if(xmlXIncludeProcessTree(np) == -1)
    {
        fprintf(stderr, "Couldn't include!\n");
        xmlFreeTextReader(reader);
        return false;
    }

    bool status = xml_read(reader);

    xmlFreeTextReader(reader);
    xmlCleanupParser();

    return status;
}

static bool read_info   (void *item, xmlTextReaderPtr reader);
static bool read_parts  (void *item, xmlTextReaderPtr reader);
static bool read_metrics(void *item, xmlTextReaderPtr reader);
static bool read_colors (void *item, xmlTextReaderPtr reader);

bool car_model::xml_read(xmlTextReaderPtr reader)
{
    boost::fusion::vector<list_matcher<std::string> >
        vl(lm("id", &id));

    if(!read_attributes(vl, reader))
        return false;

    xml_elt read[] =
        {{0,
          BAD_CAST "info",
          this,
          read_info},
         {0,
          BAD_CAST "parts",
          this,
          read_parts},
         {0,
          BAD_CAST "metrics",
          this,
          read_metrics},
         {0,
          BAD_CAST "body-colors",
          this,
          read_colors}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "car-model"))
        return false;

    for(size_t c = 0; c < sizeof(read)/sizeof(read[0]); ++c)
        if(read[c].count != 1)
            return false;

    boost::trim(id);

    boost::trim(make);
    boost::trim(model);
    boost::trim(year);

    boost::trim(body_mesh);
    boost::trim(wheel_mesh);
    boost::trim(wheel_flip_mesh);

    return true;
}

// INFO reading code

static bool read_make(void *item, xmlTextReaderPtr reader);
static bool read_model(void *item, xmlTextReaderPtr reader);
static bool read_year(void *item, xmlTextReaderPtr reader);

static bool read_info(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "make",
          item,
          read_make},
         {0,
          BAD_CAST "model",
          item,
          read_model},
         {0,
          BAD_CAST "year",
          item,
          read_year}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "info"))
        return false;

    for(size_t c = 0; c < sizeof(read)/sizeof(read[0]); ++c)
        if(read[c].count != 1)
            return false;

    return true;
}

static bool read_make(void *item, xmlTextReaderPtr reader)
{
    return read_leaf_text(reinterpret_cast<car_model*>(item)->make, reader, "make");
}

static bool read_model(void *item, xmlTextReaderPtr reader)
{
    return read_leaf_text(reinterpret_cast<car_model*>(item)->model, reader, "model");
}

static bool read_year(void *item, xmlTextReaderPtr reader)
{
    return read_leaf_text(reinterpret_cast<car_model*>(item)->year, reader, "year");
}

// PARTS reading code

static bool read_body      (void *item, xmlTextReaderPtr reader);
static bool read_wheel     (void *item, xmlTextReaderPtr reader);
static bool read_wheel_flip(void *item, xmlTextReaderPtr reader);

static bool read_parts(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "body",
          item,
          read_body},
         {0,
          BAD_CAST "wheel",
          item,
          read_wheel},
         {0,
          BAD_CAST "wheel-flip",
          item,
          read_wheel_flip}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "parts"))
        return false;

    for(size_t c = 0; c < sizeof(read)/sizeof(read[0]); ++c)
        if(read[c].count != 1)
            return false;

    return true;
}

static bool read_body(void *item, xmlTextReaderPtr reader)
{
    return read_leaf_text(reinterpret_cast<car_model*>(item)->body_mesh, reader, "body");
}

static bool read_wheel(void *item, xmlTextReaderPtr reader)
{
    return read_leaf_text(reinterpret_cast<car_model*>(item)->wheel_mesh, reader, "wheel");
}

static bool read_wheel_flip(void *item, xmlTextReaderPtr reader)
{
    return read_leaf_text(reinterpret_cast<car_model*>(item)->wheel_flip_mesh, reader, "wheel-flip");
}

// METRICS reading code

static bool read_wheel_diameter(void *item, xmlTextReaderPtr reader);
static bool read_wheel_points  (void *item, xmlTextReaderPtr reader);
static bool read_z_offset      (void *item, xmlTextReaderPtr reader);

static bool read_metrics(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "wheel-diameter",
          item,
          read_wheel_diameter},
         {0,
          BAD_CAST "wheel-points",
          item,
          read_wheel_points},
         {0,
          BAD_CAST "z-offset",
          item,
          read_z_offset}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "metrics"))
        return false;

    for(size_t c = 0; c < sizeof(read)/sizeof(read[0]); ++c)
        if(read[c].count != 1)
            return false;

    return true;
}

static bool read_wheel_diameter(void *item, xmlTextReaderPtr reader)
{
    std::string res;
    if(!read_leaf_text(res, reader, "wheel-diameter"))
        return false;

    return (sscanf(res.c_str(), "%f", &(reinterpret_cast<car_model*>(item)->wheel_diameter)) == 1);
}

static bool read_wheel_points(void *item, xmlTextReaderPtr reader)
{
    std::string res;
    if(!read_leaf_text(res, reader, "wheel-points"))
        return false;

    car_model &cm = *reinterpret_cast<car_model*>(item);
    std::stringstream ss(res);
    for(int i = 0; i < 4; ++i)
        ss >> cm.wheel_points[i][0] >> cm.wheel_points[i][1] >> cm.wheel_points[i][2];

    return true;
}

static bool read_z_offset(void *item, xmlTextReaderPtr reader)
{
    std::string res;
    if(!read_leaf_text(res, reader, "z-offset"))
        return false;

    return (sscanf(res.c_str(), "%f", &(reinterpret_cast<car_model*>(item)->z_offset)) == 1);
}

// COLORS reading code

static bool read_color(void *item, xmlTextReaderPtr reader);

static bool read_colors(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "color",
          item,
          read_color}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "body-colors"))
        return false;

    return !reinterpret_cast<car_model*>(item)->body_colors.empty();
}

static bool read_color(void *item, xmlTextReaderPtr reader)
{
    std::string res;
    if(!read_leaf_text(res, reader, "color"))
        return false;

    std::vector<bodycolor> &bc = reinterpret_cast<car_model*>(item)->body_colors;
    bc.push_back(bodycolor());
    return (sscanf(res.c_str(), "%f %f %f", bc.back().rgb, bc.back().rgb+1, bc.back().rgb+2) == 3);
}
