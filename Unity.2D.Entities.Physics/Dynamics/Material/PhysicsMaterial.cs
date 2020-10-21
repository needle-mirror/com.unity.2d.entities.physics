using System;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    // Describes how an object should respond to collisions with other objects.
    public struct PhysicsMaterial
    {
        public MaterialFlags Flags;
        public float Friction;
        public float Restitution;
        public CombinePolicy FrictionCombinePolicy;
        public CombinePolicy RestitutionCombinePolicy;

        // If true, the object does not collide but raises trigger events instead
        public bool IsTrigger => (Flags & MaterialFlags.IsTrigger) != 0;
        
        // If true, the object raises collision events if an impulse is applied during solving
        public bool EnableCollisionEvents => (Flags & MaterialFlags.EnableCollisionEvents) != 0;

        [Flags]
        public enum MaterialFlags
        {
            IsTrigger = 1 << 0,
            EnableCollisionEvents = 1 << 1,
        }

        // Describes how to mix material properties.
        public enum CombinePolicy
        {
            // sqrt(a * b)
            GeometricMean,

            // min(a, b)
            Minimum,
                       
            // max(a, b)
            Maximum,

            // (a + b) / 2
            ArithmeticMean
        }

        public static readonly PhysicsMaterial Default = new PhysicsMaterial
        {
            Friction = 0.4f,
            Restitution = 0.0f,
            FrictionCombinePolicy = CombinePolicy.GeometricMean,
            RestitutionCombinePolicy = CombinePolicy.Maximum
        };

        // Get a combined friction value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static float GetCombinedFriction(in PhysicsMaterial materialA, in PhysicsMaterial materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.FrictionCombinePolicy, (int)materialB.FrictionCombinePolicy);
            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Friction * materialB.Friction);

                case CombinePolicy.Minimum:
                    return math.min(materialA.Friction, materialB.Friction);

                case CombinePolicy.Maximum:
                    return math.max(materialA.Friction, materialB.Friction);

                case CombinePolicy.ArithmeticMean:
                    return (materialA.Friction + materialB.Friction) * 0.5f;

                default:
                    return 0;
            }
        }

        // Get a combined restitution value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static float GetCombinedRestitution(in PhysicsMaterial materialA, in PhysicsMaterial materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.RestitutionCombinePolicy, (int)materialB.RestitutionCombinePolicy);

            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Restitution * materialB.Restitution);

                case CombinePolicy.Minimum:
                    return math.min(materialA.Restitution, materialB.Restitution);

                case CombinePolicy.Maximum:
                    return math.max(materialA.Restitution, materialB.Restitution);

                case CombinePolicy.ArithmeticMean:
                    return (materialA.Restitution + materialB.Restitution) * 0.5f;

                default:
                    return 0;
            }
        }

        public bool Equals(PhysicsMaterial other)
        {
            return
                Flags == other.Flags &&
                FrictionCombinePolicy == other.FrictionCombinePolicy &&
                RestitutionCombinePolicy == other.RestitutionCombinePolicy &&
                Friction.Equals(other.Friction) &&
                Restitution.Equals(other.Restitution);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                unchecked((uint)(
                    (byte)Flags
                    | ((byte)FrictionCombinePolicy << 4)
                    | ((byte)RestitutionCombinePolicy << 8))
                ),
                math.hash(new float2(Friction, Restitution))
            )));
        }
    }
}
