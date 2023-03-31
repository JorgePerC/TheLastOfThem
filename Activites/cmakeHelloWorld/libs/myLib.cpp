//#include "myLib.hpp"
    // Need to comment it oyt if compiling alone

float suma (float a, float b){
    return a+b;
}

float resta (float a, float b){
    return a-b;
}

float multiplicacion (float a, float b){
    return a*b;
}

float division (float a, float b){
    return a/b;
}

long factorial (int a){
    long fact = 1;

    for (int i = 1; i <= a; i++){
        fact *= i;
    }

    return fact;
}
