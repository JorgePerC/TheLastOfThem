#include <stdio.h>
#include <iostream>

//Include our library
    // Only file name, no directory
#include "myLib.hpp"

int main (int argc, char *argv[])
{
    std::cout << "Hello World!" << std::endl;
    
    // Usar la libreria
    long i = factorial(5);

    std::cout << "Factorial de 5: " << i << std::endl;

    std::cout << "Suma de 5 y 6: " << suma(5,6) << std::endl;

    return 0;
}