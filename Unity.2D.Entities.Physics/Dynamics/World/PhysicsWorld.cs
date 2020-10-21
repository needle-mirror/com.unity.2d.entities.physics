using System;
using Unity.Collections;

namespace Unity.U2D.Entities.Physics
{
    public struct PhysicsWorld : IQueryable, IDisposable
    {
        public CollisionWorld CollisionWorld;
        public DynamicsWorld DynamicsWorld;
        public PhysicsSettings Settings;
        public float TimeStep;

        public int StaticBodyCount => CollisionWorld.StaticBodyCount;
        public int DynamicBodyCount => CollisionWorld.DynamicBodyCount;
        public int BodyCount => CollisionWorld.BodyCount;
        public int JointCount => DynamicsWorld.JointCount;
        public int GroundBodyIndex => BodyCount - 1;

        public NativeArray<PhysicsBody> AllBodies => CollisionWorld.AllBodies;
        public NativeArray<PhysicsBody> StaticBodies => CollisionWorld.StaticBodies;
        public NativeArray<PhysicsBody> DynamicBodies => CollisionWorld.DynamicBodies;
        public NativeArray<PhysicsBody.MotionData> BodyMotionData => DynamicsWorld.BodyMotionData;
        public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity => DynamicsWorld.BodyMotionVelocity;
        public NativeArray<Joint> Joints => DynamicsWorld.Joints;

        public PhysicsWorld(int staticBodyCount, int dynamicBodyCount, int jointCount)
        {
            CollisionWorld = new CollisionWorld(staticBodyCount : staticBodyCount, dynamicBodyCount : dynamicBodyCount);
            DynamicsWorld = new DynamicsWorld(bodyMotionCount : dynamicBodyCount, jointCount : jointCount);
            Settings = PhysicsSettings.Default;
            TimeStep = 1 / 60.0f;
        }

        public void Reset(int staticBodyCount, int dynamicBodyCount, int jointCount)
        {
            CollisionWorld.Reset(staticBodyCount : staticBodyCount, dynamicBodyCount : dynamicBodyCount);
            DynamicsWorld.Reset(bodyMotionCount : dynamicBodyCount, jointCount : jointCount);
        }

        public bool OverlapAabb(OverlapAabbInput input, NativeList<int> physicsBodyIndices)
        {
            return CollisionWorld.OverlapAabb(input, physicsBodyIndices);
        }

        #region IQueryable

        public Aabb CalculateAabb()
        {
            return CollisionWorld.CalculateAabb();
        }

        public Aabb CalculateAabb(PhysicsTransform transform)
        {
            return CollisionWorld.CalculateAabb(transform);
        }

        // Check a point in the world.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            return CollisionWorld.OverlapPoint(input, ref collector);
        }

        // Check a collider against this body.
        public bool OverlapCollider(OverlapColliderInput input) => QueryWrappers.OverlapCollider(ref this, input);
        public bool OverlapCollider(OverlapColliderInput input, out OverlapColliderHit hit) => QueryWrappers.OverlapCollider(ref this, input, out hit);
        public bool OverlapCollider(OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits) => QueryWrappers.OverlapCollider(ref this, input, ref allHits);
        public bool OverlapCollider<T>(OverlapColliderInput input, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        {
            return CollisionWorld.OverlapCollider(input, ref collector);
        }

        // Cast a ray in the world.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return CollisionWorld.CastRay(input, ref collector);
        }

        // Cast collider
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return CollisionWorld.CastCollider(input, ref collector);
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return CollisionWorld.CalculateDistance(input, ref collector);
        }

        // Collider distance
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return CollisionWorld.CalculateDistance(input, ref collector);
        }
        #endregion

        #region Cloneable

        public PhysicsWorld Clone()
        {
            return new PhysicsWorld()
            {
                CollisionWorld = CollisionWorld.Clone(),
                DynamicsWorld = DynamicsWorld.Clone(),
                Settings = Settings,
                TimeStep = TimeStep
            };
        }

        #endregion

        #region IDisposable

        public void Dispose()
        {
            CollisionWorld.Dispose();
            DynamicsWorld.Dispose();
        }

        #endregion
    }
}
