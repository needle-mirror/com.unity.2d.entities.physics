using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct PolygonGeometry : IEquatable<PolygonGeometry>
    {
        public NativeArray<float2> Vertices { get => m_Vertices; set => m_Vertices = value; }
        NativeArray<float2> m_Vertices;

        public float BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        float m_BevelRadius;

        public bool Equals(PolygonGeometry other)
        {
            if (m_Vertices.Length != other.m_Vertices.Length ||
                !m_BevelRadius.Equals(other.m_BevelRadius))
                return false;

            var length = m_Vertices.Length;
            for(var i = 0; i < length; ++i)
            {
                if (!m_Vertices[i].Equals(other.m_Vertices[i]))
                    return false;
            }

            return true;
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                (uint)m_Vertices.GetHashCode(),
                math.hash(new float2(BevelRadius))
                )));
        }

        internal void Validate()
        {
            if (m_Vertices.Length < 3 || m_Vertices.Length > PhysicsPolygonCollider.Constants.MaxVertexCount)
                SafetyChecks.ThrowArgumentException("Invalid number of vertices specified.", "Vertices (length)");

            for(var i = 0; i < m_Vertices.Length; ++i)
            {
                if (math.any(!math.isfinite(m_Vertices[i])))
                    SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Vertices");
            }

            if (!math.isfinite(m_BevelRadius) || m_BevelRadius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "BevelRadius");
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysicsPolygonCollider : IConvexCollider
    {
        public struct Constants
        {
            public const int MaxVertexCount = 16;
        }

        // Note: Must be first member.
        ConvexColliderHeader m_Header;
        // Note: Must be placed immediately after header.
        internal ConvexHull m_ConvexHull;

        unsafe fixed byte m_Vertices[(sizeof(float) * 2) * Constants.MaxVertexCount];
        unsafe fixed byte m_Normals[(sizeof(float) * 2) * Constants.MaxVertexCount];

        public float BevelRadius => m_ConvexHull.ConvexRadius;

        public PolygonGeometry Geometry
        {
            set => SetGeometry(value);
        }

        #region ICollider

        public ColliderType ColliderType => m_Header.ColliderType;
        public CollisionType CollisionType => m_Header.CollisionType;
        public uint UserData { get => m_Header.UserData; set { if (!m_Header.UserData.Equals(value)) m_Header.UserData = value; m_Header.SetDirty(); } }
        public MassProperties MassProperties { get; private set; }
        public int MemorySize => UnsafeUtility.SizeOf<PhysicsPolygonCollider>();

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
            return m_ConvexHull.CalculateAabb(transform);
        }

        // Check a point against this collider.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public unsafe bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            fixed (PhysicsPolygonCollider* target = &this)
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
            fixed (PhysicsPolygonCollider* target = &this)
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
            fixed (PhysicsPolygonCollider* target = &this)
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
            fixed (PhysicsPolygonCollider* target = &this)
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
            fixed (PhysicsPolygonCollider* target = &this)
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
            fixed (PhysicsPolygonCollider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, (Collider*)target, ref collector);
            }
        }

        #endregion

        #region Construction

        public static BlobAssetReference<Collider> Create(PolygonGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(PolygonGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(PolygonGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(PolygonGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var physicsCollider = new PhysicsPolygonCollider();
            physicsCollider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&physicsCollider, UnsafeUtility.SizeOf<PhysicsPolygonCollider>());
        }

        public void Initialize(PolygonGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Polygon,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(PolygonGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            fixed(PhysicsPolygonCollider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, collider->m_Vertices, collider->m_Normals, geometry.Vertices.Length, geometry.BevelRadius);
                m_ConvexHull.SetAndGiftWrap(geometry.Vertices);
            }

            MassProperties = m_ConvexHull.GetMassProperties();
        }

        #endregion
    }
}
