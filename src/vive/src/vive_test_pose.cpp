#include <survive.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy)
{
    if (keepRunning == 0)
        exit(-1);
    keepRunning = 0;
}

#endif

SURVIVE_EXPORT void button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
                                   const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals)
{
    if (buttonId == SURVIVE_BUTTON_MENU)
    {
        survive_reset_lighthouse_positions(so->ctx);
    }
}

SURVIVE_EXPORT void pose_process(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose)
{
    printf("Pose: [%ld][%s][% 08.8f,% 08.8f,% 08.8f] [ang:%08.2f %08.2f %08.2f %08.2f]\n", timecode, so->codename,
           pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

int main(int argc, char **argv)
{
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    SurviveContext *ctx = survive_init(argc, argv);
    if (ctx == 0) // implies -help or similiar
        return 0;

    survive_startup(ctx);

    survive_install_button_fn(ctx, button_process);
    survive_install_pose_fn(ctx, pose_process);
    while (keepRunning && survive_poll(ctx) == 0)
    {
    }

    survive_close(ctx);
    return 0;
}
