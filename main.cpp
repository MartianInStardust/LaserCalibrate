#include <iostream>
#include "include/Utils.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    std::cout << "this is my first modification" << std::endl;
    auto a = new Utils();
    a->print();

    return 0;
}
