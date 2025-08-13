
#include <boost/config.hpp>
#include <exception>
#include <stdexcept>
#include <iostream>

// Custom implementation for boost::throw_exception
namespace boost {
    void throw_exception(std::exception const & e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        throw e;
    }
}
