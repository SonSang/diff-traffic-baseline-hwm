#include "network.hpp"

int main(int argc, char **argv)
{
    LIBXML_TEST_VERSION;

    network n;

    if(n.load_from_xml(argv[1]))
        printf("Network name is: %s\n", n.name);

    return 0;
}

