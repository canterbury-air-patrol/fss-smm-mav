#include <bits/stdint-uintn.h>
#include <string>
#include <list>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include "fmu.hpp"

bool running = true;

void sigIntHandler(__attribute__((unused)) int signum)
{
    running = false;
}

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cout << "Usage: " << argv[0] << " client.json addr port" << std::endl;
        return -1;
    }
    /* Watch out for sigint */
    signal (SIGINT, sigIntHandler);
    /* Ignore SIGPIPE */
    signal (SIGPIPE, SIG_IGN);

    FSS *fss = new FSS(argv[1]);
    SMM *smm = new SMM();
    MAV *mav = new MAV(argv[2], atoi(argv[3]));

    while (running)
    {
        sleep (1);
    }
}