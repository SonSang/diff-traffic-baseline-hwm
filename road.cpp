#include "road.hpp"

void line_rep::locate(point *pt, float t, float offset) const
{
    int seg = find_segment(t, offset);

    t = t*(clengths.back() + offset*cmitres.back()) - (clengths[seg] + offset*cmitres[seg]);

    pt->x = t * normals[seg].x - offset*normals[seg].y + points[seg].x;
    pt->y = t * normals[seg].y + offset*normals[seg].x + points[seg].y;
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
