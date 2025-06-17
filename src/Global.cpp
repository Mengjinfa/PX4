#include "Global.h"

CGazebo_camera *gzcam;

void initializeSingleton()
{
    gzcam = GazeboCamera::Instance();
}