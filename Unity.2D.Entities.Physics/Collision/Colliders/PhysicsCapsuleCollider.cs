using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct CapsuleGeometry : IEquatable<CapsuleGeometry>
    {
        public float2 Vertex0 { get => m_Vertex0; set => m_Vertex0 = value; }
        float2 m_Vertex0;

        public float2 Vertex1 { get => m_Vertex1; set => m_Vertex1 = value; }
        float2 m_Vertex1;

        public float Radius { get => m_Radius; set => m_Radius = value; }
        float m_Radius;

        public bool Equals(CapsuleGeometry other)
        {
            return m_Vertex0.Equals(other.m_Vertex0)
                && m_Vertex1.Equals(other.m_Vertex1)
                && m_Radius.Equals(other.m_Radius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                math.hash(m_Vertex0),
                math.hash(new float3(m_Vertex1, m_Radius))
                )));
        }

        internal void Validate()
        {
            if (math.any(!math.isfinite(m_Vertex0)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Vertex0");

            if (math.any(!math.isfinite(m_Vertex1)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Vertex1");

            if (!math.isfinite(m_Radius) || m_Radius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "Radius");
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysicsCapsuleCollider : IConvexCollider
    {
        // Must be first member.
        ConvexColliderHeader m_Header;
        // Note: Must be placed immediately after header.
        internal ConvexHull m_ConvexHull;

        float2 m_Vertex0;
        float2 m_Vertex1;
        float2 m_Normal0;
        float2 m_Normal1;

        public float2 Vertex0 => m_Vertex0;
        public float2 Vertex1 => m_Vertex1;
        public float Radius => m_ConvexHull.ConvexRadius;

        public CapsuleGeometry Geometry
        {
            get => new CapsuleGeometry
            {
                Vertex0 = m_Vertex0,
                Vertex1 = m_Vertex1,
                Radius = Radius
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
        public int MemorySize => UnsafeUtility.SizeOf<PhysicsCapsuleCollider>();

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
            var vertex0 = PhysicsMath.mul(transform, m_Vertex0);
            var vertex1 = PhysicsMath.mul(transform, m_Vertex1);

            return new Aabb()
            {
                Min = math.min(vertex0, vertex1) - Radius,
                Max = math.max(vertex0, vertex1) + Radius
            };
        }

        // Check a point against this collider.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public unsafe bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            fixed (PhysicsCapsuleCollider* target = &this)
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
            fixed (PhysicsCapsuleCollider* target = &this)
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
            fixed (PhysicsCapsuleCollider* target = &this)
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
            fixed (PhysicsCapsuleCollider* target = &this)
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
            fixed (PhysicsCapsuleCollider* target = &this)
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
            fixed (PhysicsCapsuleCollider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, (Collider*)target, ref collector);
            }
        }

        #endregion

        #region Construction

        public static BlobAssetReference<Collider> Create(CapsuleGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CapsuleGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CapsuleGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(CapsuleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var physicsCollider = new PhysicsCapsuleCollider();
            physicsCollider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&physicsCollider, UnsafeUtility.SizeOf<PhysicsCapsuleCollider>());
        }

        public void Initialize(CapsuleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Capsule,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            m_Vertex0 = geometry.Vertex0;
            m_Vertex1 = geometry.Vertex1;

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(CapsuleGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            m_Vertex0 = geometry.Vertex0;
            m_Vertex1 = geometry.Vertex1;

            fixed(PhysicsCapsuleCollider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, (byte*)&collider->m_Vertex0, (byte*)&collider->m_Normal0, 2, geometry.Radius);
            }

            var radiusSqr = geometry.Radius * geometry.Radius;
            var bodyLength = math.distance(m_Vertex0, m_Vertex1);
            var bodyArea = bodyLength * geometry.Radius * 2.0f;
            var bodyMass = bodyArea;
            var bodyInertia = bodyMass * (bodyLength * bodyLength + radiusSqr) / 12.0f;

            var capsArea = math.PI * radiusSqr;
            var capsMass = capsArea;
            var capsInertia = capsMass * (0.5f * radiusSqr + bodyLength * bodyLength * 0.25f);

            var mass = bodyMass + capsArea;
            var localCenterOfMass = 0.5f * (m_Vertex0 + m_Vertex1);
            var area = bodyArea + capsArea;
            var inertia = bodyInertia + capsInertia + mass * math.dot(localCenterOfMass, localCenterOfMass);
            var angularExpansionFactor = math.length(m_Vertex1 - m_Vertex0) * 0.5f;

            m_Normal0 = PhysicsMath.perp(math.normalize(m_Vertex1 - m_Vertex0));
            m_Normal1 = -m_Normal0;

            // Set mass properties.
            MassProperties = new MassProperties(
                localCenterOfMass : localCenterOfMass,
                inertia : inertia,
                area : area,
                angularExpansionFactor : angularExpansionFactor);
        }

        #endregion
    }
}
