#include <iostream>
#include "PrimeChecker.hpp"

using namespace std;

int main(int argc, char** argv) {
    if (argc == 2) {
        int number = stoi(argv[1]);
        PrimeChecker pc;
        cout << "botusan, miruna; " << number << " is a prime number? " << pc.isPrime(number) << endl;
    }
    return 0;
}