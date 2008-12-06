#include "road.hpp"

void line_rep::locate(point *pt, float t, float offset) const
{
    int seg = find_segment(t, offset);

    t = t*(clengths.back() + offset*cmitres.back()) - (clengths[seg] + offset*cmitres[seg]);

    pt->x = t * normals[seg].x - offset*normals[seg].y + points[seg].x;
    pt->y = t * normals[seg].y + offset*normals[seg].x + points[seg].y;
}

void line_rep::lane_mesh(float range[2], float center_offs, float offsets[2]) const
{
    std::vector<point> vrts(points.size()*2);
    std::vector<quad>  faces(points.size()-1);

    float last_mitre = 0.0f;
    for(int i = 0; i < static_cast<int>(points.size()-1); ++i)
    {
        float mitre = cmitres[i]-last_mitre;

        vrts[2*i].x = offsets[0]*(mitre*normals[i].x - normals[i].y) + points[i].x;
        vrts[2*i].y = offsets[0]*(mitre*normals[i].y + normals[i].x) + points[i].y;

        vrts[2*i+1].x = offsets[1]*(mitre*normals[i].x - normals[i].y) + points[i].x;
        vrts[2*i+1].y = offsets[1]*(mitre*normals[i].y + normals[i].x) + points[i].y;

        last_mitre = cmitres[i];
    }
    {
        int i = points.size()-1;
        float mitre = cmitres[i]-last_mitre;

        vrts[2*i].x = offsets[0]*(mitre*normals[i-1].x - normals[i-1].y) + points[i].x;
        vrts[2*i].y = offsets[0]*(mitre*normals[i-1].y + normals[i-1].x) + points[i].y;

        vrts[2*i+1].x = offsets[1]*(mitre*normals[i-1].x - normals[i-1].y) + points[i].x;
        vrts[2*i+1].y = offsets[1]*(mitre*normals[i-1].y + normals[i-1].x) + points[i].y;
    }

    for(int i = 0; i < static_cast<int>(points.size()-1); ++i)
    {
        faces[i].v[0] = 2*i;
        faces[i].v[1] = 2*i+1;
        faces[i].v[2] = 2*(i+1)+1;
        faces[i].v[3] = 2*(i+1);
    }

    //    printf("%zu\n", vrts.size());
    for(int i = 0; i < static_cast<int>(vrts.size()); ++i)
        printf("v %f %f 0.0\n", vrts[i].x, vrts[i].y);
    //    printf("%zu\n", faces.size());
    for(int i = 0; i < static_cast<int>(faces.size()); ++i)
        printf("f %d %d %d %d\n", faces[i].v[0]+1, faces[i].v[1]+1, faces[i].v[2]+1, faces[i].v[3]+1);
}

void line_rep::draw(float start, float stop, float offset) const
{
}

int line_rep::find_segment(float x, float offset) const
{
    assert(x >= 0 && x <= 1.0);

    x *= clengths.back() + offset*cmitres.back();

    int current = 1;
    while(current < static_cast<int>(clengths.size()))
    {
        if(x-(clengths[current] + offset*cmitres[current]) < 0.0f)
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
        float orient =  normals[i].x*normals[i+1].y - normals[i].y*normals[i+1].x;

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
