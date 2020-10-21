using System.Runtime.CompilerServices;

using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    // Describes how the mass is distributed.
    public struct MassDistribution
    {
        // The (local) center of mass.
        public float2 LocalCenterOfMass;

        // The inverse rotational inertia.
        public float InverseInertia;
    }

    public struct MassProperties
    {
        public MassDistribution MassDistribution;

        public float Area;
        public float AngularExpansionFactor;

        public static MassProperties Default => new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                LocalCenterOfMass = float2.zero,
                InverseInertia = 1.0f
            },

            Area = 1.0f,
            AngularExpansionFactor = 0.0f
        };

        public MassProperties(float2 localCenterOfMass, float inertia, float area, float angularExpansionFactor)
        {
            MassDistribution = new MassDistribution
            {
                LocalCenterOfMass = localCenterOfMass,
                InverseInertia = inertia > 0f ? math.rcp(inertia) : 1f
            };

            Area = area;
            AngularExpansionFactor = angularExpansionFactor;
        }
    }

    // Provides an upper bound on change in a body's extents in any direction during a step.
    // Used to determine how far away from the body to look for collisions.
    struct MotionExpansion
    {
        public float2 Linear;   // how far to look ahead of the object
        public float Uniform;   // how far to look around the object

        public float MaxDistance => math.length(Linear) + Uniform;

        public static readonly MotionExpansion Zero = new MotionExpansion
        {
            Linear = new float2(0f),
            Uniform = 0.0f
        };

        // Expand an AABB.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Aabb ExpandAabb(Aabb aabb) => new Aabb
        {
            Max = math.max(aabb.Max, aabb.Max + Linear) + Uniform,
            Min = math.min(aabb.Min, aabb.Min + Linear) - Uniform
        };
    }

}
