using System;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    [Serializable]
    public struct PhysicsSettings
    {
        public static PhysicsSettings Default => new PhysicsSettings
        {
            Gravity = new float2(0.0f, -9.81f),
            AabbInflation = 0.1f,
            NumberOfThreadsHint = 8,
            SimulationType = SimulationType.Default
        };

        // World gravity.
        public float2 Gravity;

        // Expands the Aabb when building the bounding area hierarchy tree.
        public float AabbInflation;

        // The number of available threads.
        public int NumberOfThreadsHint;
        
        // The type of simulation to be used when simulating this physics world.
        public SimulationType SimulationType;
        
        // Constants.
        public struct Constants
        {
            // A small length used as a collision and constraint tolerance.
            public const float LinearSlop = 0.005f;

            // Default Convex Radius.
            public const float MinimumConvexRadius = LinearSlop * 2.0f;

            // Maximum iterations allowed for GJK.
            public const int MaxGJKInterations = 20;

            // Contacts are always created between PhysicsBody if they are closer than this distance threshold.
            public const float CollisionTolerance = 0.01f;
        }

        public void Validate()
        {
            var defaultSettings = Default;

            if (math.any(!math.isfinite(Gravity)))
                Gravity = defaultSettings.Gravity;

            if (!math.isfinite(AabbInflation))
                AabbInflation = defaultSettings.AabbInflation;

            if (NumberOfThreadsHint < 0)
                NumberOfThreadsHint = defaultSettings.NumberOfThreadsHint;
        }
    }
}
