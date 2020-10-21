using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct PhysicsTransform : IEquatable<PhysicsTransform>
    {
        public float2 Translation;
        public float2x2 Rotation;

        public static PhysicsTransform Identity => new PhysicsTransform { Translation = float2.zero, Rotation = float2x2.identity };

        public PhysicsTransform(float2 translation)
        {
            Translation = translation;
            Rotation = float2x2.identity;
        }

        public PhysicsTransform(float2 translation, float rotation)
        {
            Translation = translation;
            Rotation = float2x2.Rotate(rotation);
        }

        public PhysicsTransform(float2 translation, float2x2 rotation)
        {
            Translation = translation;
            Rotation = rotation;
        }

        public PhysicsTransform(float3 translation, quaternion rotation)
        {
            Translation = translation.xy;
            Rotation = float2x2.Rotate(PhysicsMath.ZRotationFromQuaternion(rotation));
        }

        public PhysicsTransform(RigidTransform rigidTransform)
        {
            Translation = rigidTransform.pos.xy;
            Rotation = float2x2.Rotate(PhysicsMath.ZRotationFromQuaternion(rigidTransform.rot));
        }
       
        public bool Equals(PhysicsTransform other)
        {
            return Translation.Equals(other.Translation) && Rotation.Equals(other.Rotation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetQuaternionRotation(quaternion rotation)
        {
            Rotation = float2x2.Rotate(PhysicsMath.ZRotationFromQuaternion(rotation));
        }
    }

    public static partial class PhysicsMath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PhysicsTransform inverse(PhysicsTransform transform)
        {
            var inverseRotation = math.transpose(transform.Rotation);

            return new PhysicsTransform
            {
                Translation = math.mul(inverseRotation, -transform.Translation),
                Rotation = inverseRotation
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 mul(PhysicsTransform transform, float2 value)
        {
            return math.mul(transform.Rotation, value) + transform.Translation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 mul(float2x2 rotation, float2 value)
        {
            return math.mul(rotation, value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PhysicsTransform mul(PhysicsTransform transform1, PhysicsTransform transform2)
        {
            return new PhysicsTransform
            {
                Translation = math.mul(transform1.Rotation, transform2.Translation) + transform1.Translation,
                Rotation = math.mul(transform1.Rotation, transform2.Rotation)
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float angle(float2x2 rotation)
        {
            return math.atan2(rotation.c0.y, rotation.c0.x);
        }
    }
}
