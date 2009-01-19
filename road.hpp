#ifndef _ROAD_HPP_
#define _ROAD_HPP_

//! A simple 2d point.
struct point
{
    point()
    {}

    point(float inx, float iny)
        : x(inx), y(iny)
    {}

    float x;
    float y;
};

//! A four-sided simple polygon.
struct quad
{
    int v[4];
};

inline void intersect_lines(point &res,
                            const point &o0, const point &n0,
                            const point &o1, const point &n1)
{
    float a[2] = { n0.y,  n1.y};
    float b[2] = {-n0.x, -n1.x};
    float c[2] = {o0.x*n0.y - o0.y*n0.x,
                  o1.x*n1.y - o1.y*n1.x};
    if(std::abs(n0.y) < 1e-6)
    {
        if(std::abs(n1.y) < 1e-6)
        {
            res.x = 0.5f*(o0.x + o1.x);
            res.y = 0.5f*(o0.y + o1.y);
            return;
        }
        std::swap(a[0], a[1]);
        std::swap(b[0], b[1]);
        std::swap(c[0], c[1]);
    }

    float inva0 = 1.0f/a[0];
    float denom = b[1] - a[1]*inva0*b[0];
    if(std::abs(denom) < 1e-6)
    {
        res.x = 0.5f*(o0.x + o1.x);
        res.y = 0.5f*(o0.y + o1.y);
        return;
    }

    res.y = (c[1] - a[1]*inva0 * c[0])/denom;
    res.x = (c[0] - b[0]*res.y)*inva0;
}

//! A representation of a road's shape.
struct line_rep
{
    //! Compute length of line displaced from polyline.
    /*!
      \param offset An offset from the polyline along which to place the point.
                    The convention that the parameterization increases with x,
                    and the offset increaseses with y.
      \returns the length of the line
    */
    float offset_length(float offset) const;

    //! Determine where a point along the parameterization is.
    /*!
      \param pt A pointer to a point where the results will be stored.
      \param x  A float in [0, 1] specifying the parameter to extract
      \param offset An offset from the polyline along which to place the point.
                    The convention that the parameterization increases with x,
                    and the offset increaseses with y
    */
    int locate(point *pt, float x, float offset) const;

    int locate_vec(point *pt, point *vec, float x, float offset) const;

    //! Extract a strip of quads.
    /*!
      \param range A pair of ordered (<) floats in [0, 1] representing the parameteric range of the road to extract.
      \param center_offset The offset from the center of the road along which to take the paramterization.
      \param offsets A pair of offsets defining the extents of the sweep in 'y'.
      This function isn't air-tight and can generate some bogus meshes, but it's a start
    */
    void lane_mesh(std::vector<point> & pts, std::vector<quad> & faces, const float range[2], float center_offset, const float offsets[2]) const;

    void draw() const;

    int draw_data(draw_type dtype, const road_membership *rom, float &lenused, int count, const lane *la, float gamma_c) const;

    //! Find the segment containing x.
    /*!
      \param x A float in [0.0, 1.0].
      \param offset A float representing offset from centerline (+ is 'left')
      \returns The segment containing x
    */
    int find_segment(float x, float offset) const;

    //! Builds lengths, normals, and mitres from points
    /*!
      This assumes that points has at least 2 elements in it;
      we overwrite any existing quantities with new calculations.
    */
    void calc_rep();

    //! Create a string representation of line
    /*!
      Fills buff with the first len characters of rep
      \param buff An array of at least len characters.
      \param len  The max. number of characters to write in buff (including trailing \0).
      \returns The number of written characters (not inluding trailing \0).
    */
    int to_string(char buff[], int len) const;

    void points_string(const char *str);

    bool xml_read(xmlTextReaderPtr reader);

    float lane_width; //!< Width of lanes.

    std::vector<point> points;  //!< Points defining a polyline.

    std::vector<float> clengths; //!< Cumulative lengths of segments. (#points)
    std::vector<point> normals; //!< (#points-1) normalized vectors describing segment direction.
    std::vector<float> cmitres;  //!< Cumulative quantities describing mitre joints at each point. (#points)
};

//! Road data structure.
/*!
  Groups bundles of lanes together, giving them a name and with
  line_rep, a shape
*/
struct road
{
    road() : name(0) {}
    road(const road &r);

    ~road();

    bool xml_read(xmlTextReaderPtr reader);

    char * name;  //!< Road name
    line_rep rep; //!< A description of the road's shape
};

#endif
