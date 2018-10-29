#include <iostream>

#include "placeholder.hpp"
#include "generic/hello.hpp"

using namespace GooBalls;

int main(int argc, char *argv[]){
    std::cout << "Hi there!" << std::endl;
    anotherHello();
    hello();
    return 0;
}
