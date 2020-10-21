using System;
using System.Runtime.CompilerServices;

using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct PhysicsBody : IQueryable
    {
        public struct Constants
        {
            public const int InvalidBodyIndex = -1;
        }

        // The Collider associated with this PhysicsBody (can be NULL).
        public BlobAssetReference<Collider> Collider;

        // The World-Space Transform of this PhysicsBody.
        public PhysicsTransform WorldTransform;

        // The Entity that this PhysicsBody represents.
        public Entity Entity;

        // Returns a PhysicsBody at the origin, with no rotation or attached Collider.
        public static readonly PhysicsBody Zero = new PhysicsBody
        {
            Collider = default,
            WorldTransform = PhysicsTransform.Identity,
            Entity = Entity.Null
        };

        public PhysicsBody(BlobAssetReference<Collider> colliderBlob)
        {
            Collider = colliderBlob;
            WorldTransform = PhysicsTransform.Identity;
            Entity = Entity.Null;
        }

        public void SetCollider(BlobAssetReference<Collider> colliderBlob)
        {
            Collider = colliderBlob;
        }

        // A Dynamic PhysicsBody "cold" motion data.
        public struct MotionData
        {
            public float2 WorldPosition;
            public float WorldAngle;

            public float2 LocalCenterOfMass;

            // Damping applied to the motion during each simulation step.
            public float LinearDamping;
            public float AngularDamping;

            // A scaling applied to the simulation step's gravity.
            public float GravityScale;
        }

        // A Dynamic PhysicsBody "hot" motion data, used during solving.
        public struct MotionVelocity
        {
            public float2 LinearVelocity;
            public float AngularVelocity;

            public float InverseMass;
            public float InverseInertia;

            public float AngularExpansionFactor;

            public static readonly MotionVelocity Zero = new MotionVelocity
            {
                LinearVelocity = new float2(0f),
                AngularVelocity = 0f,
                InverseInertia = 0f,
                InverseMass = 0f,
                AngularExpansionFactor = 0f
            };
            
            internal bool HasInfiniteInertiaAndMass => InverseInertia < float.Epsilon && InverseMass < float.Epsilon;
            
            // Apply a linear impulse (in world space)
            public void ApplyLinearImpulse(float2 impulse)
            {
                LinearVelocity += impulse * InverseMass;
            }

            // Apply an angular impulse (in motion space)
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ApplyAngularImpulse(float impulse)
            {
                AngularVelocity += impulse * InverseInertia;
            }

            // Calculate the distances by which to expand collision tolerances based on the speed of the object.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            internal MotionExpansion CalculateExpansion(float timeStep) => new MotionExpansion
            {
                Linear = LinearVelocity * timeStep,
                Uniform = math.min(math.length(AngularVelocity) * timeStep * AngularExpansionFactor, AngularExpansionFactor)
            };
        }

        // A pair of physics body indices.
        public struct IndexPair : IEquatable<IndexPair>
        {
            public int PhysicsBodyIndexA;
            public int PhysicsBodyIndexB;

            public bool IsValid => PhysicsBodyIndexA != -1 && PhysicsBodyIndexB != -1;

            public static IndexPair Invalid => new IndexPair { PhysicsBodyIndexA = -1, PhysicsBodyIndexB = -1 };

            internal IndexPair GetSwapped()
            {
                return new IndexPair
                {
                    PhysicsBodyIndexA = PhysicsBodyIndexB,
                    PhysicsBodyIndexB = PhysicsBodyIndexA
                };
            }
            
            public bool Equals(IndexPair other)
            {
                return
                    PhysicsBodyIndexA.Equals(other.PhysicsBodyIndexA) &&
                    PhysicsBodyIndexB.Equals(other.PhysicsBodyIndexB);
            }

            public override int GetHashCode()
            {
                return unchecked((int)math.hash(new int2(PhysicsBodyIndexA, PhysicsBodyIndexB)));
            }

            public override string ToString()
            {
                return $"A:{PhysicsBodyIndexA}, B:{PhysicsBodyIndexB}";
            }
        }

        #region IQueryable

        public Aabb CalculateAabb()
        {
            return Collider.IsCreated ?
                Collider.Value.CalculateAabb(WorldTransform) :
                new Aabb { Min = WorldTransform.Translation, Max = WorldTransform.Translation };
        }

        public Aabb CalculateAabb(PhysicsTransform transform)
        {
            return Collider.IsCreated ?
                Collider.Value.CalculateAabb(PhysicsMath.mul(transform, WorldTransform)) :
                new Aabb { Min = WorldTransform.Translation, Max = WorldTransform.Translation };
        }

        // Check a point against this body.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            return Collider.IsCreated && Collider.Value.OverlapPoint(input, ref collector);
        }

        // Check a collider against this body.
        public bool OverlapCollider(OverlapColliderInput input) => QueryWrappers.OverlapCollider(ref this, input);
        public bool OverlapCollider(OverlapColliderInput input, out OverlapColliderHit hit) => QueryWrappers.OverlapCollider(ref this, input, out hit);
        public bool OverlapCollider(OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits) => QueryWrappers.OverlapCollider(ref this, input, ref allHits);
        public bool OverlapCollider<T>(OverlapColliderInput input, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        {
            return Collider.IsCreated && Collider.Value.OverlapCollider(input, ref collector);
        }

        // Casts a ray against this body.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return Collider.IsCreated && Collider.Value.CastRay(input, ref collector);
        }

        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return Collider.IsCreated && Collider.Value.CastCollider(input, ref collector);
        }

        // Calculate distance of point collider(s)
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        // Calculate distance of collider(s).
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        #endregion
    }
}
