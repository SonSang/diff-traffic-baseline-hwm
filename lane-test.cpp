#include "lane.hpp"

int main(int argc, char * argv[])
{
    line_rep r;

    r.points.push_back(point(0.0f, 0.0f));
    r.points.push_back(point(0.5f, 0.0f));
    r.points.push_back(point(2.5f, 0.0f));
    r.points.push_back(point(3.0f, 0.0f));

    r.populate_lengths();

    //0     1      2      3
    //[0, 1) [1, 2) [2, 3)
    //   0      1     2

    printf("find seg: %f = %d\n", 0.0, r.find_segment(0.0));
    printf("find seg: %f = %d\n", 0.05, r.find_segment(0.05));
    printf("find seg: %f = %d\n", 0.40, r.find_segment(0.40));
    printf("find seg: %f = %d\n", 0.70, r.find_segment(0.70));
    printf("find seg: %f = %d\n", 0.95, r.find_segment(0.95));
    printf("find seg: %f = %d\n", 1.0, r.find_segment(1.0));

    float f = 0.5f;
    point p;
    r.locate(&p, f, 0.0f);
    printf("locate %f = (%f %f)\n", f, p.x, p.y);

    char buff[1024];
    r.to_string(buff, 1024);
    printf("%s", buff);


    return 0;
}
