#include "intervals.hpp"

void insert(intervals<float> & x, float num)
{
    printf("Inserting %f = %f\n", num, num*100);
    x.insert(num, num*100);
    char buff[1024];
    x.to_string(buff, 1024);
    printf("%s\n", buff);
}

int main(int argc, char * argv[])
{
    intervals<float> speeds;
    speeds.base_data = 0.0f;

    insert(speeds, 0.5);
    insert(speeds, 0.2);
    insert(speeds, 0.1);
    insert(speeds, 0.4);
    insert(speeds, 0.9);
    insert(speeds, 0.95);
    insert(speeds, 0.975);
    insert(speeds, 0.45);
    insert(speeds, 0.05);

    printf("%f ? %f\n", 0.99, speeds[0.99]);

    return 0;
}
