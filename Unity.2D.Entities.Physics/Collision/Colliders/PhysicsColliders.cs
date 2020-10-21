using System;
using System.Runtime.InteropServices;

using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public enum ColliderType
    {
        Invalid     = 0,

        // Convex types.
        Box         = 1,
        Polygon     = 2,
        Circle      = 3,
        Capsule     = 4,

        // Composite types.
        Compound    = 5
    }

    public enum CollisionType
    {
        Invalid     = 0,

        Convex      = 1,
        Composite   = 2
    }

    interface ICollider : IQueryable
    {       
        ColliderType ColliderType { get; }
        CollisionType CollisionType { get; }
        uint UserData { get; set; }
        MassProperties MassProperties { get; }

        // The total size of the collider in memory
        int MemorySize { get; }
    }

    interface IConvexCollider: ICollider
    {
        CollisionFilter Filter { get; set; }
        PhysicsMaterial Material { get; set; }

        int VertexCount { get; }
        ConvexHull.ConvexArray.Accessor Vertices { get; }
        ConvexHull.ConvexArray.Accessor Normals { get; }
    }

    interface ICompositeCollider : ICollider
    {
        // The combined filter of all the child colliders.
        CollisionFilter Filter { get; }

        // The maximum number of bits needed to identify a child of this collider.
        uint NumColliderKeyBits { get; }

        // Get a child of this collider.
        // Return false if the key is not valid.
        bool GetChild(ref ColliderKey key, out ChildCollider child);

        // Get a leaf of this collider.
        // Return false if the key is not valid.
        bool GetLeaf(ColliderKey key, out ChildCollider leaf);

        // Get all the leaves of this collider.
        void GetLeaves<T>(ref T collector) where T : struct, ILeafColliderCollector;
    }

    public interface ILeafColliderCollector
    {
        void AddLeaf(ColliderKey key, ref ChildCollider leaf);

        void PushCompositeCollider(ColliderKeyPath compositeKey, PhysicsTransform parentFromComposite, out PhysicsTransform worldFromParent);

        void PopCompositeCollider(uint numCompositeKeyBits, PhysicsTransform worldFromParent);
    }

    // Header common to all colliders.
    struct ColliderHeader
    {
        public ColliderType ColliderType;
        public CollisionType CollisionType;
        public uint UserData;   // User data. Not used by the physics system itself.
        public byte Version;    // increment whenever the collider data has changed
        public byte Magic;      // always = 0xff (for validation)

        public CollisionFilter Filter;

        public struct Constants
        {
            public const byte Magic = 0xff;
        }

        public void SetDirty()
        {
            Version += 1;
        }
    };    

    // Convex colliders only.
    struct ConvexColliderHeader
    {
        public ColliderType ColliderType;
        public CollisionType CollisionType;
        public uint UserData;   // User data. Not used by the physics system itself.
        public byte Version;    // increment whenever the collider data has changed
        public byte Magic;      // always = 0xff (for validation)

        public CollisionFilter Filter;
        public PhysicsMaterial Material;

        public void SetDirty()
        {
            Version += 1;
        }
    };   
    
    [StructLayout(LayoutKind.Sequential)]
    public struct Collider : ICompositeCollider
    {
        // Note: Must be first member.
        ColliderHeader m_Header;

        #region ICollider

        public ColliderType ColliderType => m_Header.ColliderType;
        public CollisionType CollisionType => m_Header.CollisionType;
        public uint UserData { get => m_Header.UserData; set { if (!m_Header.UserData.Equals(value)) m_Header.UserData = value; m_Header.SetDirty(); } }

        public unsafe MassProperties MassProperties
        {
            get
            {
                fixed (Collider* collider = &this)
                {
                    switch (ColliderType)
                    {
                        case ColliderType.Box:
                            return ((PhysicsBoxCollider*)collider)->MassProperties;

                        case ColliderType.Polygon:
                            return ((PhysicsPolygonCollider*)collider)->MassProperties;

                        case ColliderType.Circle:
                            return ((PhysicsCircleCollider*)collider)->MassProperties;

                        case ColliderType.Capsule:
                            return ((PhysicsCapsuleCollider*)collider)->MassProperties;

                        case ColliderType.Compound:
                            return ((PhysicsCompoundCollider*)collider)->MassProperties;

                        case ColliderType.Invalid:
                        default:
                            SafetyChecks.ThrowInvalidOperationException("Unknown or invalid physics collider.");
                            return default;
                    }
                }
            }
        }

        public CollisionFilter Filter
        {
            get => m_Header.Filter;
            set
            {
                // Disallow changing the filter of composite types directly, since that is a combination of its children
                if(m_Header.CollisionType == CollisionType.Convex)
                {
                    m_Header.Version++;
                    m_Header.Filter = value;
                }
            }
        }

        public unsafe int MemorySize
        {
            get
            {
                fixed (Collider* collider = &this)
                {
                    switch (ColliderType)
                    {
                        case ColliderType.Box:
                            return ((PhysicsBoxCollider*)collider)->MemorySize;

                        case ColliderType.Polygon:
                            return ((PhysicsPolygonCollider*)collider)->MemorySize;

                        case ColliderType.Circle:
                            return ((PhysicsCircleCollider*)collider)->MemorySize;

                        case ColliderType.Capsule:
                            return ((PhysicsCapsuleCollider*)collider)->MemorySize;

                        case ColliderType.Compound:
                            return ((PhysicsCompoundCollider*)collider)->MemorySize;

                        case ColliderType.Invalid:
                        default:
                            SafetyChecks.ThrowInvalidOperationException("Unknown or invalid physics collider.");
                            return default;
                    }
                }
            }
        }

        #endregion

        #region ICompositeCollider

        public unsafe uint NumColliderKeyBits
        {
            get
            {
                fixed (Collider* collider = &this)
                {
                    switch (collider->ColliderType)
                    {
                        case ColliderType.Compound:
                            return ((PhysicsCompoundCollider*)collider)->NumColliderKeyBits;
                        default:
                            return 0;
                    }
                }
            }
        }

        public unsafe bool GetChild(ref ColliderKey key, out ChildCollider child)
        {
            fixed (Collider* collider = &this)
            {
                switch (collider->ColliderType)
                {
                    case ColliderType.Compound:
                        return ((PhysicsCompoundCollider*)collider)->GetChild(ref key, out child);
                    default:
                        child = new ChildCollider();
                        return false;
                }
            }
        }

        public unsafe bool GetLeaf(ColliderKey key, out ChildCollider leaf)
        {
            fixed (Collider* collider = &this)
            {
                return GetLeafCollider(collider, PhysicsTransform.Identity, key, out leaf);
            }
        }

        public unsafe void GetLeaves<T>(ref T collector) where T : struct, ILeafColliderCollector
        {
            fixed (Collider* collider = &this)
            {
                switch (collider->ColliderType)
                {
                    case ColliderType.Compound:
                        ((PhysicsCompoundCollider*)collider)->GetLeaves(ref collector);
                        break;
                }
            }
        }

        // Get a leaf of a collider hierarchy.
        // Return false if the key is not valid for the collider.
        public static unsafe bool GetLeafCollider(Collider* root, PhysicsTransform rootTransform, ColliderKey key, out ChildCollider leaf)
        {
            leaf = new ChildCollider(root, rootTransform);
            while (leaf.Collider != null)
            {
                if (!leaf.Collider->GetChild(ref key, out var child))
                {
                    break;
                }
                leaf = new ChildCollider(leaf, child);
            }
            return (leaf.Collider == null || leaf.Collider->CollisionType == CollisionType.Convex);
        }

        #endregion

        #region IQueryable

        public Aabb CalculateAabb()
        {
            return CalculateAabb(PhysicsTransform.Identity);
        }

        public unsafe Aabb CalculateAabb(PhysicsTransform transform)
        {
            fixed(Collider* collider = &this)
            {
                switch (ColliderType)
                {
                    case ColliderType.Box:
                        return ((PhysicsBoxCollider*)collider)->CalculateAabb(transform);

                    case ColliderType.Polygon:
                        return ((PhysicsPolygonCollider*)collider)->CalculateAabb(transform);

                    case ColliderType.Circle:
                        return ((PhysicsCircleCollider*)collider)->CalculateAabb(transform);

                    case ColliderType.Capsule:
                        return ((PhysicsCapsuleCollider*)collider)->CalculateAabb(transform);

                    case ColliderType.Compound:
                        return ((PhysicsCompoundCollider*)collider)->CalculateAabb(transform);

                    case ColliderType.Invalid:
                    default:
                        SafetyChecks.ThrowInvalidOperationException("Unknown or invalid physics collider.");
                        return default;
                }
            }
        }

        // Overlap point.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public unsafe bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            fixed (Collider* target = &this)
            {
                return OverlapQueries.OverlapPoint(input, target, ref collector);
            }
        }

        // Check a collider against this body.
        public bool OverlapCollider(OverlapColliderInput input) => QueryWrappers.OverlapCollider(ref this, input);
        public bool OverlapCollider(OverlapColliderInput input, out OverlapColliderHit hit) => QueryWrappers.OverlapCollider(ref this, input, out hit);
        public bool OverlapCollider(OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits) => QueryWrappers.OverlapCollider(ref this, input, ref allHits);
        public unsafe bool OverlapCollider<T>(OverlapColliderInput input, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        {
            fixed (Collider* target = &this)
            {
                return OverlapQueries.OverlapCollider(input, target, ref collector);
            }
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (Collider* target = &this)
            {
                return RaycastQueries.RayCollider(input, target, ref collector);
            }
        }

        // Cast another collider against this one.
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public unsafe bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            fixed (Collider* target = &this)
            {
                return ColliderCastQueries.CastCollider(input, target, ref collector);
            }
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (Collider* target = &this)
            {
                return DistanceQueries.PointDistance(input, target, ref collector);
            }
        }

        // Calculate the distance from another collider to this one.
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (Collider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, target, ref collector);
            }
        }

        #endregion
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ConvexCollider : IConvexCollider
    {
        // Header
        ConvexColliderHeader m_Header;
        internal ConvexHull m_ConvexHull;

        #region ICollider

        public ColliderType ColliderType => m_Header.ColliderType;
        public CollisionType CollisionType => m_Header.CollisionType;
        public uint UserData { get => m_Header.UserData; set { if (!m_Header.UserData.Equals(value)) m_Header.UserData = value; m_Header.SetDirty(); } }
        public int MemorySize { get; }
        public MassProperties MassProperties { get; }

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

        // Overlap point.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public unsafe bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
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
            fixed (ConvexCollider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, (Collider*)target, ref collector);
            }
        }

        #endregion
    }

    // An opaque key which packs a path to a specific leaf of a collider hierarchy into a single integer.
    public struct ColliderKey : IEquatable<ColliderKey>
    {
        public uint Value { get; internal set; }

        public static readonly ColliderKey Empty = new ColliderKey { Value = uint.MaxValue };

        internal ColliderKey(uint numSubKeyBits, uint subKey)
        {
            Value = uint.MaxValue;
            PushSubKey(numSubKeyBits, subKey);
        }

        public bool Equals(ColliderKey other)
        {
            return Value == other.Value;
        }

        // Append a sub key to the front of the path
        // "numSubKeyBits" is the maximum number of bits required to store any value for this sub key.
        // Returns false if the key is empty.
        public void PushSubKey(uint numSubKeyBits, uint subKey)
        {
            var parentPart = (uint)((ulong)subKey << 32 - (int)numSubKeyBits);
            var childPart = Value >> (int)numSubKeyBits;
            Value = parentPart | childPart;
        }

        // Extract a sub key from the front of the path.
        // "numSubKeyBits" is the maximum number of bits required to store any value for this sub key.
        // Returns false if the key is empty.
        public bool PopSubKey(uint numSubKeyBits, out uint subKey)
        {
            if (Value != uint.MaxValue)
            {
                subKey = Value >> (32 - (int)numSubKeyBits);
                Value = ((1 + Value) << (int)numSubKeyBits) - 1;
                return true;
            }

            subKey = uint.MaxValue;
            return false;
        }
    }

    // Stores a ColliderKey along with the number of bits in it that are used.
    // This is useful for building keys from root to leaf, the bit count shows where to place the child key bits
    public struct ColliderKeyPath
    {
        ColliderKey m_Key;
        uint m_NumKeyBits;

        public ColliderKey Key => m_Key;

        public static ColliderKeyPath Empty => new ColliderKeyPath(ColliderKey.Empty, 0);

        public ColliderKeyPath(ColliderKey key, uint numKeyBits)
        {
            m_Key = key;
            m_NumKeyBits = numKeyBits;
        }

        // Append the local key for a child of the shape referenced by this path
        public void PushChildKey(ColliderKeyPath child)
        {
            m_Key.Value &= (uint)(child.m_Key.Value >> (int)m_NumKeyBits | (ulong)0xffffffff << (int)(32 - m_NumKeyBits));
            m_NumKeyBits += child.m_NumKeyBits;
        }

        // Remove the most leafward shape's key from this path
        public void PopChildKey(uint numChildKeyBits)
        {
            m_NumKeyBits -= numChildKeyBits;
            m_Key.Value |= (uint)((ulong)0xffffffff >> (int)m_NumKeyBits);
        }

        // Get the collider key for a leaf shape that is a child of the shape referenced by this path
        public ColliderKey GetLeafKey(ColliderKey leafKeyLocal)
        {
            var leafPath = this;
            leafPath.PushChildKey(new ColliderKeyPath(leafKeyLocal, 0));
            return leafPath.Key;
        }
    }

    // A pair of collider keys.
    public struct ColliderKeyPair
    {
        public ColliderKey ColliderKeyA;
        public ColliderKey ColliderKeyB;

        public static readonly ColliderKeyPair Empty = new ColliderKeyPair { ColliderKeyA = ColliderKey.Empty, ColliderKeyB = ColliderKey.Empty };

        internal ColliderKeyPair GetSwapped()
        {
            return new ColliderKeyPair
            {
                ColliderKeyA = ColliderKeyB,
                ColliderKeyB = ColliderKeyA
            };
        }
    }

    // A child/leaf collider.
    public unsafe struct ChildCollider
    {
        readonly Collider* m_Collider;

        // The transform of the child collider in whatever space it was queried from
        public PhysicsTransform TransformFromChild;

        public Collider* Collider
        {
            get
            {
                fixed (ChildCollider* self = &this)
                {
                    // Accessing uninitialized Collider.
                    SafetyChecks.IsTrue(m_Collider != null);
                    return self->m_Collider;
                }
            }
        }

        // Create from collider
        public ChildCollider(Collider* collider)
        {
            m_Collider = collider;
            TransformFromChild = PhysicsTransform.Identity;
        }

        // Create from body
        public ChildCollider(Collider* collider, PhysicsTransform transform)
        {
            m_Collider = collider;
            TransformFromChild = transform;
        }

        // Combine a parent ChildCollider with another ChildCollider describing one of its children
        public ChildCollider(ChildCollider parent, ChildCollider child)
        {
            m_Collider = child.m_Collider;
            TransformFromChild = PhysicsMath.mul(parent.TransformFromChild, child.TransformFromChild);
        }
    }

    // Extension method to make accessing a collider easier from an asset reference.
    public static unsafe class BlobAssetReferenceColliderExtension
    {
        public static Collider* GetColliderPtr(this Unity.Entities.BlobAssetReference<Collider> assetReference)
        {
            return (Collider*)assetReference.GetUnsafePtr();
        }

        public static ref T GetColliderRef<T>(this Unity.Entities.BlobAssetReference<Collider> assetReference)
            where T : struct
        {
            return ref UnsafeUtility.AsRef<T>(assetReference.GetUnsafePtr());
        }
    }
}
