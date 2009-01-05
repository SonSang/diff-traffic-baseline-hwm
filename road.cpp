#include "network.hpp"

float line_rep::offset_length(float offset) const
{
    return clengths.back() + offset*cmitres.back();
}

int line_rep::locate(point *pt, float t, float offset) const
{
    int seg = find_segment(t, offset);

    t = t*(clengths.back() + 2*offset*cmitres.back()) - (clengths[seg] + offset*(cmitres[seg] + cmitres[seg-1]));

    pt->x = (t - offset*(cmitres[seg]-cmitres[seg-1])) * normals[seg].x - offset*normals[seg].y + points[seg].x;
    pt->y = (t - offset*(cmitres[seg]-cmitres[seg-1])) * normals[seg].y + offset*normals[seg].x + points[seg].y;
    return seg;
}

int line_rep::locate_vec(point *pt, point *vec, float t, float offset) const
{
    int seg = locate(pt, t, offset);

    vec->x = normals[seg].x;
    vec->y = normals[seg].y;

    return seg;
}

void line_rep::lane_mesh(std::vector<point> & vrts, std::vector<quad>  & faces, const float range[2], float center_offs, const float offsets[2]) const
{
    int start = find_segment(range[0], center_offs);
    int end   = find_segment(range[1], center_offs);

    float start_t = range[0]*(clengths.back() + 2*center_offs*cmitres.back()) - (clengths[start] + 2*center_offs*cmitres[start]);
    float end_t   = range[1]*(clengths.back() + 2*center_offs*cmitres.back()) - (clengths[end]   + 2*center_offs*cmitres[end]);

    vrts.push_back(point(start_t*normals[start].x - offsets[0]*normals[start].y + points[start].x,
                         start_t*normals[start].y + offsets[0]*normals[start].x + points[start].y));

    vrts.push_back(point(start_t*normals[start].x - offsets[1]*normals[start].y + points[start].x,
                         start_t*normals[start].y + offsets[1]*normals[start].x + points[start].y));

    for(int c = start+1; c <= end; ++c)
    {
        float mitre = cmitres[c]-cmitres[c-1];
        vrts.push_back(point(offsets[0]*(mitre*normals[c-1].x - normals[c-1].y) + points[c].x,
                             offsets[0]*(mitre*normals[c-1].y + normals[c-1].x) + points[c].y));

        vrts.push_back(point(offsets[1]*(mitre*normals[c-1].x - normals[c-1].y) + points[c].x,
                             offsets[1]*(mitre*normals[c-1].y + normals[c-1].x) + points[c].y));
    }

    vrts.push_back(point(end_t*normals[end].x - offsets[0]*normals[end].y + points[end].x,
                         end_t*normals[end].y + offsets[0]*normals[end].x + points[end].y));

    vrts.push_back(point(end_t*normals[end].x - offsets[1]*normals[end].y + points[end].x,
                         end_t*normals[end].y + offsets[1]*normals[end].x + points[end].y));

    faces.resize(1+end-start);
    for(int i = 0; i < 1+end-start; ++i)
    {
        faces[i].v[0] = 2*i;
        faces[i].v[1] = 2*i+1;
        faces[i].v[2] = 2*(i+1)+1;
        faces[i].v[3] = 2*(i+1);
    }
}

int line_rep::find_segment(float x, float offset) const
{
    assert(x >= 0 && x <= 1.0);

    x *= clengths.back() + 2*offset*cmitres.back();

    int current = 1;
    while(current < static_cast<int>(clengths.size()))
    {
        if(x-(clengths[current] + offset*(cmitres[current] + cmitres[current-1])) < 0.0f)
            return current-1;
        ++current;
    }
    return clengths.size()-2;
}

void line_rep::calc_rep()
{
    /*!
      strictly speaking, this will work with points = 1,
      but the class doesn't make much sense then.
    */
    assert(points.size() > 1);

    clengths.resize(points.size());
    normals.resize(points.size()-1);
    cmitres.resize(points.size());

    clengths[0] = 0.0f;
    for(int i = 1; i < static_cast<int>(points.size()); ++i)
    {
        normals[i-1].x = points[i].x-points[i-1].x;
        normals[i-1].y = points[i].y-points[i-1].y;

        float len = std::sqrt(normals[i-1].x*normals[i-1].x + normals[i-1].y*normals[i-1].y);

        clengths[i] = clengths[i-1] + len;

        float invlen = 1.0f/len;
        normals[i-1].x *= invlen;
        normals[i-1].y *= invlen;
    }

    // for the time being, set start mitre to 0.
    cmitres[0] = 0.0f;

    for(int i = 0; i < static_cast<int>(cmitres.size()-2); ++i)
    {
        float dot = normals[i].x*normals[i+1].x + normals[i].y*normals[i+1].y;
        float orient =  normals[i].y*normals[i+1].x - normals[i].x*normals[i+1].y;

        float mitre = orient*std::sqrt((1.0f - dot)/(1.0f + dot));

        // see write-up for interpretation of this formula.
        cmitres[i+1] += cmitres[i] + mitre;
    }

    // for the time being, set end mitre to 0.
    cmitres[cmitres.size()-1] = cmitres[cmitres.size()-2] + 0.0f;
}

int line_rep::to_string(char buff[], int len) const
{
    int offs = 0;
    for(int i = 0; i < static_cast<int>(points.size()) && offs < len; ++i)
        offs += snprintf(buff+offs, len-offs, "%d: (%f, %f) - %f\n", i, points[i].x, points[i].y, clengths[i]);

    return offs;
}

void line_rep::points_string(const char *str)
{
    while(*str)
    {
        while(isspace(*str))
            ++str;

        float x, y;
        if(sscanf(str, "%f %f", &x, &y) != 2)
            break;

        points.push_back(point(x, y));

        while(*str && *str != '\n')
            ++str;
    }
}

bool line_rep::xml_read(xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name || !xmlStrEqual(name, BAD_CAST "points"))
                return false;

            int ret = xmlTextReaderRead(reader);
            if(ret != 1 || xmlTextReaderNodeType(reader) != XML_READER_TYPE_TEXT)
                return false;

            const xmlChar *pts = xmlTextReaderConstValue(reader);
            points_string((const char*) pts);

            if(points.size() == 0)
            {
                fprintf(stderr, "No points in line-rep!\n");
                return false;
            }

            do
            {
                ret = xmlTextReaderRead(reader);
                if(ret != 1)
                    return false;
            }
            while(!is_closing_element(reader, "points"));
        }
    }
    while(!is_closing_element(reader, "line_rep"));

    calc_rep();

    return true;
}

road::road(const road &r)
{
    name = strdup(r.name);
    rep = r.rep;
}

road::~road()
{
    free(name);
}

bool road::xml_read(xmlTextReaderPtr reader)
{
    boost::fusion::vector<list_matcher<char*> > vl(lm("name", &name));

    if(!read_attributes(vl, reader))
        return false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *na = xmlTextReaderConstName(reader);
            if(!na || !xmlStrEqual(na, BAD_CAST "line_rep"))
                return false;

            rep.xml_read(reader);
        }
    }
    while(!is_closing_element(reader, "road"));

    return true;
}
