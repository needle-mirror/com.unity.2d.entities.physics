using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    // The mass properties of a physics body.
    // If not present, then the physics body has infinite mass and inertia.
    [GenerateAuthoringComponent]
    public struct PhysicsMass : IComponentData
    {
        // Inverse mass. Zero indicates infinite mass.
        public float InverseMass;

        // Invert rotational inertia. Zero indicates infinite inertia.
        public float InverseInertia;

        // Local center of mass.
        public float2 LocalCenterOfMass;

        // Create a Dynamic body with the specified mass.
        public static PhysicsMass CreateDynamic(MassProperties massProperties, float mass)
        {
            if (!(mass <= 0f) && math.isfinite(mass))
                return new PhysicsMass
                {
                    InverseMass = math.rcp(mass),
                    InverseInertia = massProperties.MassDistribution.InverseInertia,
                    LocalCenterOfMass = massProperties.MassDistribution.LocalCenterOfMass,
                };
            
            SafetyChecks.ThrowArgumentException("Cannot specify less than zero or Infinite/NaN.", "mass");
            return default;

        }

        // Create a Kinematic body.
        public static PhysicsMass CreateKinematic(MassProperties massProperties)
        {
            if (math.any(!math.isfinite(massProperties.MassDistribution.LocalCenterOfMass)))
            {
                SafetyChecks.ThrowArgumentException("Cannot specify less than zero or Infinite/NaN.", "localCenterOfMass");
            }

            return new PhysicsMass
            {
                InverseMass = 0.0f,
                InverseInertia = 0.0f,
                LocalCenterOfMass = massProperties.MassDistribution.LocalCenterOfMass
            };
        }
    }
}
