#include <iostream>

class Interface {
    protected:
        double cmd[2];

    public:
        virtual int readFromInterface() = 0;
        virtual int writeToInterface() = 0;
};