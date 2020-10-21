using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    [StructLayout(LayoutKind.Sequential)]
    public struct PhysicsCompoundCollider : ICompositeCollider
    {
        // Must be first member.
        ColliderHeader m_Header;

        // A child collider, within the same blob as the compound collider.
        // Warning: This references the collider via a relative offset, so must always be passed by reference.
        public struct Child
        {
            public PhysicsTransform CompoundFromChild;
            internal int m_ColliderOffset;

            public unsafe Collider* Collider
            {
                get
                {
                    fixed (int* offsetPtr = &m_ColliderOffset)
                    {
                        return (Collider*)((byte*)offsetPtr + *offsetPtr);
                    }
                }
            }
        }

        // The array of child colliders
        BlobArray m_ChildrenBlob;
        public int NumChildren => m_ChildrenBlob.Length;
        public BlobArray.Accessor<Child> Children => new BlobArray.Accessor<Child>(ref m_ChildrenBlob);

        // The bounding volume hierarchy
        // TODO: Store node filters array too, for filtering queries within the BVH
        BlobArray m_BvhNodesBlob;
        internal unsafe BoundingVolumeHierarchy BoundingVolumeHierarchy
        {
            get
            {
                fixed (BlobArray* blob = &m_BvhNodesBlob)
                {
                    var firstNode = (BoundingVolumeHierarchy.Node*)((byte*)&(blob->Offset) + blob->Offset);
                    return new BoundingVolumeHierarchy(firstNode, nodeFilters: null);
                }
            }
        }

        #region IQueryable

        public Aabb CalculateAabb()
        {
            return CalculateAabb(PhysicsTransform.Identity);
        }

        public Aabb CalculateAabb(PhysicsTransform transform)
        {
            // TODO: Store a convex hull wrapping all the children, and use that to calculate tighter AABBs?
            return PhysicsMath.mul(transform, BoundingVolumeHierarchy.Domain);
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (PhysicsCompoundCollider* target = &this)
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
            fixed (PhysicsCompoundCollider* target = &this)
            {
                return ColliderCastQueries.CastCollider(input, (Collider*)target, ref collector);
            }
        }

        // Check a point against this collider.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public unsafe bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            fixed (PhysicsCompoundCollider* target = &this)
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
            fixed (PhysicsCompoundCollider* target = &this)
            {
                return OverlapQueries.OverlapCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (PhysicsCompoundCollider* target = &this)
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
            fixed (PhysicsCompoundCollider* target = &this)
            {
                return DistanceQueries.ColliderDistance(input, (Collider*)target, ref collector);
            }
        }

        #endregion

        #region ICollider

        public ColliderType ColliderType => m_Header.ColliderType;
        public CollisionType CollisionType => m_Header.CollisionType;
        public uint UserData { get => m_Header.UserData; set { if (!m_Header.UserData.Equals(value)) m_Header.UserData = value; m_Header.SetDirty(); } }
        public int MemorySize { get; private set; }
        public MassProperties MassProperties { get; private set; }

        #endregion

        #region ICompositeCollider

        public CollisionFilter Filter => m_Header.Filter;

        public uint NumColliderKeyBits => (uint)(32 - math.lzcnt(NumChildren));

        public unsafe bool GetChild(ref ColliderKey key, out ChildCollider child)
        {
            if (key.PopSubKey(NumColliderKeyBits, out var childIndex))
            {
                ref var c = ref Children[(int)childIndex];
                child = new ChildCollider(c.Collider) { TransformFromChild = c.CompoundFromChild };
                return true;
            }

            child = new ChildCollider();
            return false;
        }

        public unsafe bool GetLeaf(ColliderKey key, out ChildCollider leaf)
        {
            fixed (PhysicsCompoundCollider* root = &this)
            {
                return Collider.GetLeafCollider((Collider*)root, PhysicsTransform.Identity, key, out leaf);
            }
        }

        public unsafe void GetLeaves<T>(ref T collector) where T : struct, ILeafColliderCollector
        {
            for (uint i = 0; i < NumChildren; i++)
            {
                ref var c = ref Children[(int)i];
                var childKey = new ColliderKey(NumColliderKeyBits, i);
                if (c.Collider->CollisionType == CollisionType.Composite)
                {
                    collector.PushCompositeCollider(new ColliderKeyPath(childKey, NumColliderKeyBits), c.CompoundFromChild, out var worldFromCompound);
                    c.Collider->GetLeaves(ref collector);
                    collector.PopCompositeCollider(NumColliderKeyBits, worldFromCompound);
                }
                else
                {
                    var child = new ChildCollider(c.Collider) { TransformFromChild = c.CompoundFromChild };
                    collector.AddLeaf(childKey, ref child);
                }
            }
        }

        #endregion

        #region Construction

        public struct ColliderBlobInstance
        {
            public PhysicsTransform CompoundFromChild;
            public BlobAssetReference<Collider> Collider;
        }

         // Create a compound collider containing an array of other colliders.
        // The source colliders are copied into the compound, so that it becomes one blob.
        public static unsafe BlobAssetReference<Collider> Create(NativeArray<ColliderBlobInstance> children, uint userData = 0)
        {
            if (children.Length == 0)
                SafetyChecks.ThrowArgumentException("Cannot create a PhysicsCompoundCollider with no children.");

            // Get the total required memory size for the compound plus all its children,
            // and the combined filter of all children
            // TODO: Verify that the size is enough
            var totalSize = PhysicsMath.NextMultipleOf16(UnsafeUtility.SizeOf<PhysicsCompoundCollider>());
            var filter = children[0].Collider.Value.Filter;
            var srcToDestInstanceAddrs = new NativeHashMap<long, long>(children.Length, Allocator.Temp);
            for (var childIndex = 0; childIndex < children.Length; childIndex++)
            {
                var child = children[childIndex];
                var instanceKey = (long)child.Collider.GetUnsafePtr();
                if (srcToDestInstanceAddrs.ContainsKey(instanceKey))
                    continue;
                totalSize += PhysicsMath.NextMultipleOf16(child.Collider.Value.MemorySize);
                filter = CollisionFilter.CreateUnion(filter, child.Collider.Value.Filter);
                srcToDestInstanceAddrs.Add(instanceKey, 0L);
            }
            totalSize += (children.Length + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches) * UnsafeUtility.SizeOf<BoundingVolumeHierarchy.Node>();

            // Allocate the collider
            var compoundCollider = (PhysicsCompoundCollider*)UnsafeUtility.Malloc(totalSize, 16, Allocator.Temp);
            UnsafeUtility.MemClear(compoundCollider, totalSize);
            compoundCollider->m_Header.ColliderType = ColliderType.Compound;
            compoundCollider->m_Header.CollisionType = CollisionType.Composite;
            compoundCollider->m_Header.UserData = userData;
            compoundCollider->m_Header.Version = 0;
            compoundCollider->m_Header.Magic = 0xff;
            compoundCollider->m_Header.Filter = filter;

            // Initialize children array
            var childrenPtr = (Child*)((byte*)compoundCollider + UnsafeUtility.SizeOf<PhysicsCompoundCollider>());
            compoundCollider->m_ChildrenBlob.Offset = (int)((byte*)childrenPtr - (byte*)(&compoundCollider->m_ChildrenBlob.Offset));
            compoundCollider->m_ChildrenBlob.Length = children.Length;
            var end = (byte*)childrenPtr + UnsafeUtility.SizeOf<Child>() * children.Length;
            end = (byte*)PhysicsMath.NextMultipleOf16((ulong)end);

            // Copy children
            for (var i = 0; i < children.Length; i++)
            {
                var collider = (Collider*)children[i].Collider.GetUnsafePtr();
                var srcInstanceKey = (long)collider;
                var dstAddr = srcToDestInstanceAddrs[srcInstanceKey];
                if (dstAddr == 0L)
                {
                    dstAddr = (long)end;
                    srcToDestInstanceAddrs[srcInstanceKey] = dstAddr;
                    UnsafeUtility.MemCpy(end, collider, collider->MemorySize);
                    end += PhysicsMath.NextMultipleOf16(collider->MemorySize);
                }
                childrenPtr[i].m_ColliderOffset = (int)((byte*)dstAddr - (byte*)(&childrenPtr[i].m_ColliderOffset));
                childrenPtr[i].CompoundFromChild = children[i].CompoundFromChild;
            }

            // Build mass properties
            compoundCollider->MassProperties = compoundCollider->BuildMassProperties();

            // Build bounding volume
            var numNodes = compoundCollider->BuildBoundingVolume(out var nodes);
            var bvhSize = numNodes * UnsafeUtility.SizeOf<BoundingVolumeHierarchy.Node>();
            compoundCollider->m_BvhNodesBlob.Offset = (int)(end - (byte*)(&compoundCollider->m_BvhNodesBlob.Offset));
            compoundCollider->m_BvhNodesBlob.Length = numNodes;
            UnsafeUtility.MemCpy(end, nodes.GetUnsafeReadOnlyPtr(), bvhSize);
            end += bvhSize;

            // Copy to blob asset
            var usedSize = (int)(end - (byte*)compoundCollider);

            if (usedSize > totalSize)
                SafetyChecks.ThrowInvalidOperationException("PhysicsCompoundCollider exceeded its allocated size.");

            compoundCollider->MemorySize = usedSize;
            var blob = BlobAssetReference<Collider>.Create(compoundCollider, usedSize);
            UnsafeUtility.Free(compoundCollider, Allocator.Temp);

            return blob;
        }

        // Build mass descriptor representing a union of all the child collider mass descriptors.
        unsafe MassProperties BuildMassProperties()
        {
            var children = Children;

            // Calculate compound area and center of mass.
            var combinedCenterOfMass = float2.zero;
            var combinedArea = 0f;
            for (var i = 0; i < NumChildren; ++i)
            {
                ref var child = ref children[i];
                var collider = child.Collider;
                if (collider->CollisionType == CollisionType.Convex && ((ConvexCollider*)child.Collider)->Material.IsTrigger)
                    continue;

                var massProperties = collider->MassProperties;

                combinedCenterOfMass += PhysicsMath.mul(child.CompoundFromChild, massProperties.MassDistribution.LocalCenterOfMass) * massProperties.Area;
                combinedArea += massProperties.Area;
            }

            // Calculate the center of mass.
            if (combinedArea > 0f)
            {
                combinedCenterOfMass /= combinedArea;
            }
            else
            {
                combinedArea = 1f;
            }

            // Calculate compound inertia.
            var combinedInertia = 0f;
            for (var i = 0; i < NumChildren; ++i)
            {
                ref var child = ref children[i];
                var collider = child.Collider;
                if (collider->CollisionType == CollisionType.Convex && ((ConvexCollider*)child.Collider)->Material.IsTrigger)
                    continue;

                var massProperties = collider->MassProperties;

                var shiftedCenterOfMass = PhysicsMath.mul(child.CompoundFromChild, massProperties.MassDistribution.LocalCenterOfMass) - combinedCenterOfMass;
                combinedInertia += math.rcp(massProperties.MassDistribution.InverseInertia) + (massProperties.Area * math.dot(shiftedCenterOfMass, shiftedCenterOfMass));
            }
            
            if (Math.Abs(combinedInertia) < float.Epsilon)
                combinedInertia = 0.0f;

            // Calculate combined angular expansion factor, relative to new center of mass.
            var combinedAngularExpansionFactor = 0f;
            for (var i = 0; i < NumChildren; ++i)
            {
                ref var child = ref children[i];
                var collider = child.Collider;
                var massProperties = collider->MassProperties;

                var shiftedCenterOfMass = PhysicsMath.mul(child.CompoundFromChild, massProperties.MassDistribution.LocalCenterOfMass) - combinedCenterOfMass;
                combinedAngularExpansionFactor = math.max(combinedAngularExpansionFactor, massProperties.AngularExpansionFactor + math.length(shiftedCenterOfMass));
            }

            return new MassProperties(
                localCenterOfMass : combinedCenterOfMass,
                inertia : combinedInertia,
                area : combinedArea,
                angularExpansionFactor : combinedAngularExpansionFactor);
        }

        unsafe int BuildBoundingVolume(out NativeArray<BoundingVolumeHierarchy.Node> nodes)
        {
            // Create inputs
            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (var i = 0; i < NumChildren; ++i)
            {
                points[i] = new BoundingVolumeHierarchy.PointAndIndex { Position = Children[i].CompoundFromChild.Translation, Index = i };
                aabbs[i] = Children[i].Collider->CalculateAabb(Children[i].CompoundFromChild);
            }

            // Build BVH
            // Todo: cleanup, better size of nodes array
            nodes = new NativeArray<BoundingVolumeHierarchy.Node>(2 + NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory)
            {
                [0] = BoundingVolumeHierarchy.Node.Empty,
                [1] = BoundingVolumeHierarchy.Node.Empty
            };

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out var numNodes);

            points.Dispose();
            aabbs.Dispose();

            return numNodes;
        }

#endregion
    }
}
