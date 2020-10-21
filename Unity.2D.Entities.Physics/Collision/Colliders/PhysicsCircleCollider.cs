using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct CircleGeometry : IEquatable<CircleGeometry>
    {
        public float2 Center { get => m_Center; set => m_Center = value; }
        float2 m_Center;

        public float Radius { get => m_Radius; set => m_Radius = value; }
        float m_Radius;

        public bool Equals(CircleGeometry other)
        {
            return m_Center.Equals(other.m_Center)
                && m_Radius.Equals(other.m_Radius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new float3(m_Center, m_Radius)));
        }

        internal void Validate()
        {
            if (math.any(!math.isfinite(m_Center)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Center");

            if (!math.isfinite(m_Radius) || m_Radius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "Radius");
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysicsCircleCollider : IConvexCollider
    {
        // Must be first member.
        ConvexColliderHeader m_Header;
        // Note: Must be placed immediately after header.
        internal ConvexHull m_ConvexHull;

        float2 m_Center;
        float2 m_DummyNormal;

        public float2 Center => m_Center;
        public float Radius => m_ConvexHull.ConvexRadius;

        public CircleGeometry Geometry
        {
            get => new CircleGeometry
            {
                Center = m_Center,
                Radius = m_ConvexHull.ConvexRadius
            };
            set
            {
                if (!value.Equals(Geometry))
                {
                    SetGeometry(value);
                }
            }
        }

        #region ICollider

        public ColliderType ColliderType => m_Header.ColliderType;
        public CollisionType CollisionType => m_Header.CollisionType;
        public uint UserData { get => m_Header.UserData; set { if (!m_Header.UserData.Equals(value)) m_Header.UserData = value; m_Header.SetDirty(); } }
        public MassProperties MassProperties { get; private set; }
        public int MemorySize => UnsafeUtility.SizeOf<PhysicsCircleCollider>();

        #endregion

        #region IConvexCollider

        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) m_Header.Filter = value; m_Header.SetDirty(); } }
        public PhysicsMaterial Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) m_Header.Material = value; m_Header.SetDirty(); } }

        public int VertexCount => m_ConvexHull.Length;
        public ConvexHull.ConvexArray.Accessor Vertices => m_ConvexHull.Vertices;
        public ConvexHull.ConvexArray.Accessor Normals => m_ConvexHull.Normals;

        #endregion

        #region IQueryable

        public Aabb CalculateAabb()
        {
            return CalculateAabb(PhysicsTransform.Identity);
        }

        public Aabb CalculateAabb(PhysicsTransform transform)
        {
            var center = PhysicsMath.mul(transform, m_Center);

            return new Aabb()
            {
                Min = center - Radius,
                Max = center + Radius
            };
        }

        // Check a point against this collider.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public unsafe bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            fixed (PhysicsCircleCollider* target = &this)
            {
                return OverlapQueries.OverlapPoint(input, (Collider*)target, ref collector);
            }
        }

        // Check a collider against this body.
        public bool OverlapCollider(OverlapColliderInput input) => QueryWrappers.OverlapCollider(ref this, input);
        public bool OverlapCollider(OverlapColliderInput input, out OverlapColliderHit hit) => QueryWrappers.OverlapCollider(ref this, input, out hit);
        public bool OverlapCollider(OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits) => QueryWrappers.OverlapCollider(ref this, input, ref allHits);
        public unsafe bool OverlapCollider<T>(OverlapColliderInput input, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        {
            fixed (PhysicsCircleCollider* target = &this)
            {
                return OverlapQueries.OverlapCollider(input, (Collider*)target, ref collector);
            }
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (PhysicsCircleCollider* target = &this)
            {
                return RaycastQueries.RayCollider(input, (Collider*)target, ref collector);
            }
        }

        // Cast another collider against this one.
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public unsafe bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            fixed (PhysicsCircleCollider* target = &this)
            {
                return ColliderCastQueries.CastCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (PhysicsCircleCollider* target = &this)
            {
                return DistanceQueries.PointDistance(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from another collider to this one.
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (PhysicsCircleCollider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, (Collider*)target, ref collector);
            }
        }

        #endregion

        #region Construction

        public static BlobAssetReference<Collider> Create(CircleGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CircleGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CircleGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(CircleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var physicsCollider = new PhysicsCircleCollider();
            physicsCollider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&physicsCollider, UnsafeUtility.SizeOf<PhysicsCircleCollider>());
        }

        public void Initialize(CircleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Center = geometry.Center;

            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Circle,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(CircleGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            m_Center = geometry.Center;

            fixed(PhysicsCircleCollider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, (byte*)&collider->m_Center, (byte*)&collider->m_DummyNormal, 1, geometry.Radius);
            }

            var radiusSqr = geometry.Radius * geometry.Radius;
            var area = math.PI * radiusSqr;
            var mass = area;
            var localCenterOfMass = m_Center;
            var inertia = mass * ((radiusSqr * 0.5f) + math.dot(localCenterOfMass, localCenterOfMass));

            // Set mass properties.
            MassProperties = new MassProperties(
                localCenterOfMass : localCenterOfMass,
                inertia : inertia,
                area : area,
                angularExpansionFactor : 0.0f);
        }

        #endregion
    }
}
