#include "car-models.hpp"
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

namespace bf = boost::filesystem;

int main(int argc, char **argv)
{
    if(argc == 2)
    {
        bf::path srcdir(argv[1]);
        if(bf::is_directory(srcdir))
        {
            boost::regex re(".*\\.xml");
            bf::directory_iterator end_itr;
            for( bf::directory_iterator itr(srcdir);
                 itr != end_itr;
                 ++itr)
            {
                if(itr->path().has_filename() && boost::regex_match(itr->path().filename(), re))
                {
                    std::cout << "Loading " << itr->path().file_string() << "...";

                    car_model cm;
                    if(cm.load_from_xml(itr->path().file_string().c_str()))
                        std::cout << "Done." << std::endl;
                    else
                    {
                        std::cout << "Error" << std::endl;
                        exit(1);
                    }
                }
            }

            return 0;
        }
    }
    fprintf(stderr, "Usage: %s <dir>\n", argv[0]);

    return 1;
}
