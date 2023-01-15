#include <listener.hpp>
#include <iostream>

size_t count{0};

void listener::on_data_available (::DDS::DataReader_ptr reader) {
    std::cout << "on data called " << ++count << " times\n";
}
