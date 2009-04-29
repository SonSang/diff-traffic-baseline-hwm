#include "car-models.hpp"

int main(int argc, char **argv)
{
    car_model cm;

    if(argc < 2)
    {
        fprintf(stderr, "Usage: %s <xml-file>\n", argv[0]);
        exit(1);
    }

    cm.load_from_xml(argv[1]);

    return 0;
}
