using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct BoxGeometry : IEquatable<BoxGeometry>
    {
        public float2 Size { get => m_Size; set => m_Size = value; }
        float2 m_Size;

        public float2 Center { get => m_Center; set => m_Center = value; }
        float2 m_Center;

        public float Angle { get => m_Angle; set => m_Angle = value; }
        float m_Angle;

        public float BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        float m_BevelRadius;

        public bool Equals(BoxGeometry other)
        {
            return m_Size.Equals(other.m_Size)
                && m_Center.Equals(other.m_Center)
                && m_Angle.Equals(other.m_Angle)
                && m_BevelRadius.Equals(other.m_BevelRadius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint3(
                math.hash(m_Size),
                math.hash(m_Center),
                math.hash(new float2(m_Angle, m_BevelRadius))
                )));
        }

        internal void Validate()
        {
            if (math.any(!math.isfinite(m_Size)) || math.any(m_Size <= 0.0f))
                SafetyChecks.ThrowArgumentException("Cannot specify less than zero or Infinite/NaN.", "Size");

            if (math.any(!math.isfinite(m_Center)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Center");

            if (!math.isfinite(Angle))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Angle");

            if (!math.isfinite(m_BevelRadius) || m_BevelRadius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "BevelRadius");
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysicsBoxCollider : IConvexCollider
    {
        public struct Constants
        {
            public const int MaxVertexCount = 4;
        }

        // Note: Must be first member.
        ConvexColliderHeader m_Header;
        // Note: Must be placed immediately after header.
        internal ConvexHull m_ConvexHull;

        unsafe fixed byte m_Vertices[(sizeof(float) * 2) * Constants.MaxVertexCount];
        unsafe fixed byte m_Normals[(sizeof(float) * 2) * Constants.MaxVertexCount];

        float2 m_Size;
        float2 m_Center;
        float m_Angle;

        public float2 Size => m_Size;
        public float2 Center => m_Center;
        public float Angle => m_Angle;
        public float BevelRadius => m_ConvexHull.ConvexRadius;

        public BoxGeometry Geometry
        {
            get => new BoxGeometry
            {
                Size = m_Size,
                Center = m_Center,
                Angle = m_Angle,
                BevelRadius = m_ConvexHull.ConvexRadius
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
        public int MemorySize => UnsafeUtility.SizeOf<PhysicsBoxCollider>();

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
            fixed (PhysicsBoxCollider* target = &this)
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
            fixed (PhysicsBoxCollider* target = &this)
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
            fixed (PhysicsBoxCollider* target = &this)
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
            fixed (PhysicsBoxCollider* target = &this)
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
            fixed (PhysicsBoxCollider* target = &this)
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
            fixed (PhysicsBoxCollider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, (Collider*)target, ref collector);
            }
        }

        #endregion

        #region Construction

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var physicsCollider = new PhysicsBoxCollider();
            physicsCollider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&physicsCollider, UnsafeUtility.SizeOf<PhysicsBoxCollider>());
        }

        void Initialize(BoxGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Box,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(BoxGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            m_Size = geometry.Size;
            m_Center = geometry.Center;
            m_Angle = geometry.Angle;

            fixed(PhysicsBoxCollider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, collider->m_Vertices, collider->m_Normals, Constants.MaxVertexCount, geometry.BevelRadius);
            }

            var extents = m_Size * 0.5f;

            var xf = new PhysicsTransform(m_Center, m_Angle);

            var vertices = m_ConvexHull.Vertices.GetUnsafePtr();
            vertices[0] = PhysicsMath.mul(xf, -extents);
            vertices[1] = PhysicsMath.mul(xf, new float2(extents.x, -extents.y));
            vertices[2] = PhysicsMath.mul(xf, extents);
            vertices[3] = PhysicsMath.mul(xf, new float2(-extents.x, extents.y));

            var normals = m_ConvexHull.Normals.GetUnsafePtr();
            normals[0] = PhysicsMath.mul(xf.Rotation, new float2(0.0f, -1.0f));
            normals[1] = PhysicsMath.mul(xf.Rotation, new float2(1.0f, 0.0f));
            normals[2] = PhysicsMath.mul(xf.Rotation, new float2(0.0f, 1.0f));
            normals[3] = PhysicsMath.mul(xf.Rotation, new float2(-1.0f, 0.0f));

            MassProperties = m_ConvexHull.GetMassProperties();
        }

        #endregion
    }
}
