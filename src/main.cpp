#include <bits/stdint-uintn.h>
#include <string>
#include <list>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include "fmu.hpp"
#include "fss/fmu-fss-types.hpp"

bool running = true;

void sigIntHandler(__attribute__((unused)) int signum)
{
    running = false;
}

static void
fss_command_cb (void *priv, FSSCommand command)
{
    if (priv != nullptr)
    {
        FMUStateMachine *state_machine = (FMUStateMachine *)priv;
        state_machine->FSSNewCommand(command);
    }
}

static void
fss_comms_status_cb (void *priv, FSSCommsStatus status)
{
    if (priv != nullptr)
    {
        FMUStateMachine *state_machine = (FMUStateMachine *)priv;
        state_machine->setCommsFailure((status == fss_comms_failure));
    }
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

    /* Setup the State Machine */
    FMUStateMachine *state_machine = new FMUStateMachine(mav, smm, fss);

    /* Connect up the notifications */
    fss->registerCommandCB(fss_command_cb, state_machine);
    fss->registerCommsStatusCB(fss_comms_status_cb, state_machine);

    while (running)
    {
        sleep (1);
    }
}