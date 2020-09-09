#include <bits/stdint-uintn.h>
#include <string>
#include <list>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include "fmu-types.hpp"
#include "fmu.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"

std::string asset_name = "";

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

static void
smm_settings_cb (void *priv, SMMSettings settings)
{
    if (priv != nullptr)
    {
        SMM *smm = (SMM *)priv;
        smm->connect(settings.getURL(), settings.getUsername(), settings.getPassword(), asset_name);
    }
}

struct fss_smm_s {
    FSS *fss;
    SMM *smm;
};

static void
mav_position_cb (void *priv, double t_lat, double t_lng, double alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver)
{
    struct fss_smm_s *fss_smm = (struct fss_smm_s *)priv;
    if (fss_smm->fss != nullptr)
    {
        fss_smm->fss->reportPosition(t_lat, t_lng, alt, t_hdg, t_vel_hor, t_vel_ver);
    }
    if (fss_smm->smm != nullptr)
    {
        fss_smm->smm->reportPosition(t_lat, t_lng, alt, t_hdg / 100);
    }
}

static void
mav_reached_cb (void *priv, int point)
{
    struct fss_smm_s *fss_smm = (struct fss_smm_s *)priv;

    if (fss_smm->fss != nullptr)
    {
        fss_smm->fss->reachedPoint(point, fss_smm->smm->currentSearchPoints());
    }
    if (fss_smm->smm != nullptr)
    {
        fss_smm->smm->reachedPoint(point);
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
    MAV *mav = new MAV(argv[2], atoi(argv[3]));
    SMM *smm = new SMM(mav);

    /* Get the asset name */
    asset_name = fss->getAssetName();

    /* Setup the State Machine */
    FMUStateMachine *state_machine = new FMUStateMachine(mav, smm, fss);

    /* Connect up the notifications */
    fss->registerCommandCB(fss_command_cb, state_machine);
    fss->registerCommsStatusCB(fss_comms_status_cb, state_machine);
    fss->registerSMMSettingsCB(smm_settings_cb, smm);

    struct fss_smm_s *fss_smm = (struct fss_smm_s *) calloc (1, sizeof (struct fss_smm_s));
    fss_smm->fss = fss;
    fss_smm->smm = smm;

    mav->registerPositionCB(mav_position_cb, fss_smm);
    mav->registerReachedCB(mav_reached_cb, fss_smm);

    while (running)
    {
        sleep (1);
    }

    fss_smm->fss = nullptr;
    fss_smm->smm = nullptr;

    delete state_machine;
    delete smm;
    delete fss;
    delete mav;
    free (fss_smm);
}