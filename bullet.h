#ifndef __GO_BULLET_H__
#define __GO_BULLET_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "PhysicsClientC_API.h"

int b3CreateBody(b3PhysicsClientHandle client, double pos[3], double orientation[4], double halfExtents[3], double mass, int shape);
void b3SetBodyMass(b3PhysicsClientHandle client, int bodyId, double mass);
void b3GetBodyPosition(b3PhysicsClientHandle client, int bodyId, double* out);
void b3GetBodyOrientation(b3PhysicsClientHandle client, int bodyId, double* out);
void b3ApplyImpulse(b3PhysicsClientHandle client, int bodyId, double force[3], double position[3]);

void b3SetTimeStep(b3PhysicsClientHandle client, double step);
void b3SetGravity(b3PhysicsClientHandle physicsClient, double x, double y, double z);
void b3ResetSimulation(b3PhysicsClientHandle client);
void b3StepSimulation(b3PhysicsClientHandle client);

#ifdef __cplusplus
}
#endif
#endif