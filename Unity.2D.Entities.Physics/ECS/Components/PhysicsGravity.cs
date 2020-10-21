using Unity.Entities;

namespace Unity.U2D.Entities.Physics
{
    // Optional gravity scale applied to a physics body during each simulation step.
    // This scales the gravity vector supplied to the simulation step.
    [GenerateAuthoringComponent]
    public struct PhysicsGravity : IComponentData
    {
        public float Scale;
    }
}
