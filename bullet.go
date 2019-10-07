package bullet3

// #cgo CFLAGS: -I"./bullet3/lib/include/bullet" -I"./bullet3/lib/include/bullet_robotics" -I"./bullet3/lib/include/bullet" -I"./bullet3/lib/include"
// #cgo LDFLAGS: -L./bullet3/lib/lib -lBulletRobotics -lBulletInverseDynamics -lBulletInverseDynamicsUtils -l BulletFileLoader -l BulletWorldImporter -lBulletSoftBody  -lBulletDynamics -lBulletCollision -lLinearMath -lBullet3Common -lm -lstdc++ -ldl
// #include "bullet.h"
// #include "PhysicsDirectC_API.h"
import "C"
import "github.com/go-gl/mathgl/mgl64"

type Param func(w *World)

func Gravity(g mgl64.Vec3) Param {
	return func(w *World) {
		w.SetGravity(g)
	}
}

func TimeStep(step float64) Param {
	return func(w *World) {
		w.SetTimeStep(step)
	}
}

type World struct {
	handle C.b3PhysicsClientHandle
}

func NewWorld(params ...Param) *World {
	w := &World{C.b3ConnectPhysicsDirect()}
	for _, p := range params {
		p(w)
	}
	return w
}

func (w *World) Destroy() {
	C.b3DisconnectSharedMemory(w.handle)
}

func (w *World) SetTimeStep(step float64) {
	C.b3SetTimeStep(w.handle, C.double(step))
}

func (w *World) SetGravity(g mgl64.Vec3) {
	C.b3SetGravity(w.handle, C.double(g[0]), C.double(g[1]), C.double(g[2]))
}

func (w *World) ResetSimulation() {
	C.b3ResetSimulation(w.handle)
}

func (w *World) Step() {
	C.b3StepSimulation(w.handle)
}

func (w *World) CreateRigidBody(pos mgl64.Vec3, ori mgl64.Quat, halfExtents mgl64.Vec3, mass float64, shape CollisionShape) *RigidBody {
	o := []float64{ori.V[0], ori.V[1], ori.V[2], ori.W}
	id := C.b3CreateBody(w.handle, (*C.double)(&pos[0]), (*C.double)(&o[0]), (*C.double)(&halfExtents[0]), C.double(mass), C.int(shape.ShapeType()))
	return &RigidBody{
		id: int(id),
		mass: mass,
		collisionShape: shape,
		world: w.handle,
	}
}

type CollisionShape interface {
	ShapeType() int
}

type RigidBody struct {
	id int
	mass float64
	collisionShape CollisionShape
	world C.b3PhysicsClientHandle
}

func (r *RigidBody) ID() int {
	return r.id
}

func (r *RigidBody) Mass() float64 {
	return r.mass
}

func (r *RigidBody) Position() mgl64.Vec3 {
	var v mgl64.Vec3
	C.b3GetBodyPosition(r.world, C.int(r.id), (*C.double)(&v[0]))
	return v
}

func (r *RigidBody) Orientation() mgl64.Quat {
	var v [4]float64
	C.b3GetBodyOrientation(r.world, C.int(r.id), (*C.double)(&v[0]))
	return mgl64.Quat{
		V: mgl64.Vec3{v[0], v[1], v[2]},
		W: v[3],
	}
}

func (r *RigidBody) SetMass(mass float64) {
	C.b3SetBodyMass(r.world, C.int(r.id), C.double(mass))
	r.mass = mass
}

func (r *RigidBody) ApplyImpulse(impulse, worldPosition mgl64.Vec3) {
	C.b3ApplyImpulse(r.world, C.int(r.id), (*C.double)(&impulse[0]), (*C.double)(&worldPosition[0]));
}