using Unity.Entities;

namespace Unity.U2D.Entities.Physics
{
    // A component that encapsulates the PhysicsSettings.
    public struct PhysicsSettingsComponent : IComponentData
    {
        public PhysicsSettings Value;
    }
}
