#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h> 
#include "PhysicsDirectC_API.h"
#include "bullet.h"

int b3CreateBody(b3PhysicsClientHandle client, double pos[3], double orientation[4], double halfExtents[3], double mass, int shape)
{
    b3SharedMemoryCommandHandle h = b3CreateBoxShapeCommandInit(client);
    b3CreateBoxCommandSetStartPosition(h, pos[0], pos[1], pos[2]);
    b3CreateBoxCommandSetStartOrientation(h, orientation[0], orientation[1], orientation[2], orientation[3]);
    b3CreateBoxCommandSetHalfExtents(h, halfExtents[0], halfExtents[1], halfExtents[2]);
    b3CreateBoxCommandSetMass(h, mass);
    b3CreateBoxCommandSetCollisionShapeType(h, shape);
    b3SharedMemoryStatusHandle status = b3SubmitClientCommandAndWaitStatus(client, h);
    int bodyIndex = b3GetStatusBodyIndex(status);
    return b3GetBodyUniqueId(client, bodyIndex);
}

void b3SetBodyMass(b3PhysicsClientHandle client, int bodyId, double mass)
{
    b3SharedMemoryCommandHandle h = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetMass(h, bodyId, -1, mass);
    b3SubmitClientCommandAndWaitStatus(client, h);
}

void b3GetBodyPosition(b3PhysicsClientHandle client, int bodyId, double* out)
{
    b3SharedMemoryCommandHandle h = b3RequestActualStateCommandInit(client, bodyId);
    b3SharedMemoryStatusHandle status = b3SubmitClientCommandAndWaitStatus(client, h);
    struct b3LinkState state;
    b3GetLinkState(client, status, 0, &state);
    memcpy(out, &state.m_worldPosition, sizeof(state.m_worldPosition));
}

void b3GetBodyOrientation(b3PhysicsClientHandle client, int bodyId, double* out)
{
    b3SharedMemoryCommandHandle h = b3RequestActualStateCommandInit(client, bodyId);
    b3SharedMemoryStatusHandle status = b3SubmitClientCommandAndWaitStatus(client, h);
    struct b3LinkState state;
    b3GetLinkState(client, status, 0, &state);
    memcpy(out, &state.m_worldOrientation, sizeof(state.m_worldOrientation));
}

void b3ApplyImpulse(b3PhysicsClientHandle client, int bodyId, double force[3], double position[3])
{
    b3SharedMemoryCommandHandle h = b3ApplyExternalForceCommandInit(client);
    b3ApplyExternalForce(h, bodyId, -1, force, position, 0);
    b3SubmitClientCommandAndWaitStatus(client, h);
}

void b3SetTimeStep(b3PhysicsClientHandle client, double step)
{
    b3SharedMemoryCommandHandle h = b3InitPhysicsParamCommand(client);
    b3PhysicsParamSetTimeStep(h, step);
    b3SubmitClientCommandAndWaitStatus(client, h);
}

void b3SetGravity(b3PhysicsClientHandle physicsClient, double x, double y, double z)
{
    b3SharedMemoryCommandHandle h = b3InitPhysicsParamCommand(physicsClient);
    b3PhysicsParamSetGravity(h, x, y, z);
    b3SubmitClientCommandAndWaitStatus(physicsClient, h);
}

void b3ResetSimulation(b3PhysicsClientHandle client)
{
    b3SharedMemoryCommandHandle h = b3InitResetSimulationCommand(client);
    b3SubmitClientCommandAndWaitStatus(client, h);
}

void b3StepSimulation(b3PhysicsClientHandle client)
{
    b3SharedMemoryCommandHandle h = b3InitStepSimulationCommand(client);
    b3SubmitClientCommandAndWaitStatus(client, h);
}

#ifdef __cplusplus
}
#endif