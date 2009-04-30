#include "car-models.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        fprintf(stderr, "Usage: %s <dir>\n", argv[0]);
        exit(1);
    }

    car_db cd;
    int res = cd.add_dir(argv[1]);
    printf("Add %d entries to db\n", res);

    return 0;
}
