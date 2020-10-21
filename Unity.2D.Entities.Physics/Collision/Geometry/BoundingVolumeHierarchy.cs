using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    // A 4-way bounding volume hierarchy
    public partial struct BoundingVolumeHierarchy
    {
        readonly unsafe Node* m_Nodes;
        readonly unsafe CollisionFilter* m_NodeFilters;

        public unsafe Aabb Domain => m_Nodes[1].Bounds.GetCompoundAabb();

        public unsafe BoundingVolumeHierarchy(Node* nodes, CollisionFilter* nodeFilters)
        {
            m_Nodes = nodes;
            m_NodeFilters = nodeFilters;
        }

        public unsafe BoundingVolumeHierarchy(NativeArray<Node> nodes, NativeArray<CollisionFilter> nodeFilters)
        {
            m_Nodes = (Node*)nodes.GetUnsafeReadOnlyPtr();
            m_NodeFilters = (CollisionFilter*)nodeFilters.GetUnsafeReadOnlyPtr();
        }

        public unsafe BoundingVolumeHierarchy(NativeArray<Node> nodes)
        {
            m_Nodes = (Node*)nodes.GetUnsafeReadOnlyPtr();
            m_NodeFilters = null;
        }

        // A node in the hierarchy
        [DebuggerDisplay("{IsLeaf?\"Leaf\":\"Internal\"}, [ {Data[0]}, {Data[1]}, {Data[2]}, {Data[3]} ]")]
        [StructLayout(LayoutKind.Sequential, Size = 128)]
        public struct Node
        {
            public FourAabb Bounds;
            public int4 Data;
            public int Flags;

            public static Node Empty => new Node
            {
                Bounds = FourAabb.Empty,
                Data = int4.zero,
                IsLeaf = false
            };

            public bool IsInternal { get => Flags == 0; set => Flags = value ? 0 : 1; }
            public bool IsLeaf { get => Flags != 0; set => Flags = value ? 1 : 0; }

            static readonly int4 minusOne4i = new int4(-1);

            public bool4 AreLeavesValid => (Data != minusOne4i);
            public bool4 AreInternalsValid => (Data != int4.zero);

            public bool IsChildValid(int index)
            {
                if (IsLeaf && Data[index] == -1) return false;
                if (!IsLeaf && Data[index] == 0) return false;
                return true;
            }

            public int NumValidChildren()
            {
                var cnt = 0;
                for (var i = 0; i < 4; i++)
                {
                    cnt += IsChildValid(i) ? 1 : 0;
                }

                return cnt;
            }

            public bool IsLeafValid(int index) => Data[index] != -1;
            public bool IsInternalValid(int index) => Data[index] != 0;

            public void ClearLeafData(int index) => Data[index] = -1;
            public void ClearInternalData(int index) => Data[index] = 0;
        }

        // Utility function
        static void Swap<T>(ref T a, ref T b) where T : struct { var t = a; a = b; b = t; }

        #region Self overlap query

        public interface ITreeOverlapCollector
        {
            void AddPairs(int l, int4 r, int countR);
            void AddPairs(int4 l, int4 r, int count, bool swapped = false);
            void FlushIfNeeded();
        }

        internal unsafe void BvhOverlap<T>(ref T pairWriter, BoundingVolumeHierarchy other, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            TreeOverlap(ref pairWriter, m_Nodes, other.m_Nodes, m_NodeFilters, other.m_NodeFilters, rootA, rootB);
        }

        internal unsafe void SelfBvhOverlap<T>(ref T pairWriter, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            TreeOverlap(ref pairWriter, m_Nodes, m_Nodes, m_NodeFilters,  m_NodeFilters, rootA, rootB);
        }

        internal static unsafe void TreeOverlap<T>(
            ref T pairWriter,
            Node* treeA, Node* treeB,
            CollisionFilter* collisionFilterA = null, CollisionFilter* collisionFilterB = null,
            int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        {
            var binaryStackA = stackalloc int[Constants.BinaryStackSize];
            var binaryStackB = stackalloc int[Constants.BinaryStackSize];
            var stackA = binaryStackA;
            var stackB = binaryStackB;

            var compressedDataBuffer = stackalloc int4[4];

            if (treeA == treeB && rootA == rootB)
            {
                var unaryStack = stackalloc int[Constants.UnaryStackSize];
                var stack = unaryStack;
                *stack++ = rootA;

                do
                {
                    var nodeIndex = *(--stack);
                    if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndex], collisionFilterB[nodeIndex]))
                    {
                        ProcessAA(ref treeA[nodeIndex], compressedDataBuffer, ref stack, ref stackA, ref stackB, ref pairWriter);
                    }
                    pairWriter.FlushIfNeeded();

                    while (stackA > binaryStackA)
                    {
                        var nodeIndexA = *(--stackA);
                        var nodeIndexB = *(--stackB);

                        if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
                        {
                            ProcessAB(&treeA[nodeIndexA], &treeA[nodeIndexB], treeA, treeA, compressedDataBuffer, ref stackA, ref stackB, ref pairWriter);
                        }
                        pairWriter.FlushIfNeeded();
                    }
                } while (stack > unaryStack);
            }
            else
            {
                *stackA++ = rootA;
                *stackB++ = rootB;

                do
                {
                    var nodeIndexA = *(--stackA);
                    var nodeIndexB = *(--stackB);
                    if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
                    {
                        ProcessAB(&treeA[nodeIndexA], &treeB[nodeIndexB], treeA, treeB, compressedDataBuffer, ref stackA, ref stackB, ref pairWriter);
                    }

                    pairWriter.FlushIfNeeded();
                } while (stackA > binaryStackA);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void ProcessAA<T>(ref Node node, int4* compressedData, ref int* stack, ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        {
            var nodeData = node.Data;
            var nodeBounds = node.Bounds;

            var aabbT = stackalloc FourAabb[3];
            aabbT[0] = nodeBounds.GetAabbT(0);
            aabbT[1] = nodeBounds.GetAabbT(1);
            aabbT[2] = nodeBounds.GetAabbT(2);

            var masks = stackalloc bool4[3];
            masks[0] = new bool4(false, true, true, true);
            masks[1] = new bool4(false, false, true, true);
            masks[2] = new bool4(false, false, false, true);

            var compressedCounts = int3.zero;
            compressedCounts[0] = math.compress((int*)(compressedData)    , 0, nodeData, aabbT[0].Overlap1Vs4(ref nodeBounds) & masks[0]);
            compressedCounts[1] = math.compress((int*)(compressedData + 1), 0, nodeData, aabbT[1].Overlap1Vs4(ref nodeBounds) & masks[1]);
            compressedCounts[2] = math.compress((int*)(compressedData + 2), 0, nodeData, aabbT[2].Overlap1Vs4(ref nodeBounds) & masks[2]);

            if (node.IsLeaf)
            {
                for (var i = 0; i < 3; i++)
                {
                    pairWriter.AddPairs(nodeData[i], compressedData[i], compressedCounts[i]);
                }
            }
            else
            {
                int4 internalNodes;
                var numInternals = math.compress((int*)&internalNodes, 0, nodeData, node.AreInternalsValid);
                *((int4*)stack) = internalNodes;
                stack += numInternals;

                for (var i = 0; i < 3; i++)
                {
                    *((int4*)stackA) = new int4(nodeData[i]);
                    *((int4*)stackB) = compressedData[i];
                    stackA += compressedCounts[i];
                    stackB += compressedCounts[i];
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void ProcessAB<T>(
            Node* nodeA, Node* nodeB,
            Node* treeA, Node* treeB,
            int4* compressedData,
            ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        {
            var swapped = false;
            if (nodeA->IsInternal && nodeB->IsLeaf)
            {
                var tmp = nodeA;
                nodeA = nodeB;
                nodeB = tmp;

                treeB = treeA;

                swapped = true;
            }

            var overlapMask = stackalloc bool4[4];
            var aabbTA = nodeA->Bounds.GetAabbT(0);
            overlapMask[0] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
            aabbTA = nodeA->Bounds.GetAabbT(1);
            overlapMask[1] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
            aabbTA = nodeA->Bounds.GetAabbT(2);
            overlapMask[2] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
            aabbTA = nodeA->Bounds.GetAabbT(3);
            overlapMask[3] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);

            var compressedCount = int4.zero;

            compressedCount[0] = math.compress((int*)&compressedData[0], 0, nodeB->Data, overlapMask[0]);
            compressedCount[1] = math.compress((int*)&compressedData[1], 0, nodeB->Data, overlapMask[1]);
            compressedCount[2] = math.compress((int*)&compressedData[2], 0, nodeB->Data, overlapMask[2]);
            compressedCount[3] = math.compress((int*)&compressedData[3], 0, nodeB->Data, overlapMask[3]);

            if (nodeA->IsLeaf && nodeB->IsLeaf)
            {
                for (var i = 0; i < 4; i++)
                {
                    pairWriter.AddPairs(nodeA->Data[i], compressedData[i], compressedCount[i]);
                }
            }
            else if (nodeA->IsInternal && nodeB->IsInternal)
            {
                for (var i = 0; i < 4; i++)
                {
                    *((int4*)stackA) = new int4(nodeA->Data[i]);
                    *((int4*)stackB) = compressedData[i];
                    stackA += compressedCount[i];
                    stackB += compressedCount[i];
                }
            }
            else
            {
                for (var i = 0; i < 4; i++)
                {
                    if (compressedCount[i] > 0)
                    {
                        *((int4*)stackA) = compressedData[i];
                        var internalStack = stackA + compressedCount[i];
                        var aabbT = nodeA->Bounds.GetAabbT(i);
                        var leafA = new int4(nodeA->Data[i]);

                        do
                        {
                            var internalNode = treeB + *(--internalStack);
                            int4 internalCompressedData;
                            var internalCount = math.compress((int*)&internalCompressedData, 0, internalNode->Data, aabbT.Overlap1Vs4(ref internalNode->Bounds));
                            if (internalCount > 0)
                            {
                                if (internalNode->IsLeaf)
                                {
                                    pairWriter.AddPairs(leafA, internalCompressedData, internalCount, swapped);
                                    pairWriter.FlushIfNeeded();
                                }
                                else
                                {
                                    *((int4*)internalStack) = internalCompressedData;
                                    internalStack += internalCount;
                                }
                            }
                        }
                        while (internalStack != stackA);
                    }
                }
            }
        }          

        #endregion

        #region Point overlap query

        public interface IPointOverlapLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool PointLeaf<T>(OverlapPointInput input, int leafData, ref T collector) where T : struct, ICollector<OverlapPointHit>;
        }

        public unsafe bool OverlapPoint<TProcessor, TCollector>(OverlapPointInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IPointOverlapLeafProcessor
            where TCollector : struct, ICollector<OverlapPointHit>
        {
            var hadHit = false;
            int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
            *top++ = 1;
            do
            {
                var node = m_Nodes + *(--top);
                var hitMask = node->Bounds.OverlapPoint(input.Position);
                int4 hitData;
                var hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < hitCount; i++)
                    {
                        var index = hitData[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            hadHit |= leafProcessor.PointLeaf(input, index, ref collector);
                            if (collector.EarlyOutOnHit && hadHit)
                            {
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    *((int4*)top) = hitData;
                    top += hitCount;
                }
            } while (top > stack);

            return hadHit;
        }

        #endregion

        #region Collider overlap query

        public interface IColliderOverlapLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool ColliderLeaf<T>(OverlapColliderInput input, int leafData, ref T collector) where T : struct, ICollector<OverlapColliderHit>;
        }

        public unsafe bool OverlapCollider<TProcessor, TCollector>(OverlapColliderInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IColliderOverlapLeafProcessor
            where TCollector : struct, ICollector<OverlapColliderHit>
        {
            var hadHit = false;
            int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
            *top++ = 1;
            var colliderAabb = new FourAabb();
            colliderAabb.SetAllAabbs(input.Collider.Value.CalculateAabb(input.Transform));
            do
            {
                var node = m_Nodes + *(--top);
                var hitMask = node->Bounds.Overlap(ref colliderAabb);
                int4 hitData;
                var hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < hitCount; i++)
                    {
                        var index = hitData[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            hadHit |= leafProcessor.ColliderLeaf(input, index, ref collector);
                            if (collector.EarlyOutOnHit && hadHit)
                            {
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    *((int4*)top) = hitData;
                    top += hitCount;
                }
            } while (top > stack);

            return hadHit;
        }

        #endregion

        #region AABB overlap query

        public interface IAabbOverlapLeafProcessor
        {
            // Called when the query overlaps a leaf node of the bounding volume hierarchy
            void AabbLeaf<T>(OverlapAabbInput input, int leafData, ref T collector) where T : struct, IOverlapCollector;
        }

        public unsafe void AabbOverlap<TProcessor, TCollector>(OverlapAabbInput input, ref TProcessor processor, ref TCollector collector, int root = 1)
            where TProcessor : struct, IAabbOverlapLeafProcessor
            where TCollector : struct, IOverlapCollector
        {
            var binaryStack = stackalloc int[Constants.BinaryStackSize];
            var stack = binaryStack;
            *stack++ = root;

            FourAabb aabbT;
            (&aabbT)->SetAllAabbs(input.Aabb);
            do
            {
                var nodeIndex = *(--stack);
                var node = m_Nodes + nodeIndex;
                var overlap = aabbT.Overlap1Vs4(ref node->Bounds);
                int4 compressedValues;
                var compressedCount = math.compress((int*)(&compressedValues), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < compressedCount; i++)
                    {
                        var index = compressedValues[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            processor.AabbLeaf(input, index, ref collector);
                        }
                    }
                }
                else
                {
                    *((int4*)stack) = compressedValues;
                    stack += compressedCount;
                }
            } while (stack > binaryStack);
        }

        #endregion

        #region Ray cast query

        public interface IRaycastLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool RayLeaf<T>(RaycastInput input, int leafData, ref T collector) where T : struct, ICollector<RaycastHit>;
        }

        public unsafe bool Raycast<TProcessor, TCollector>(RaycastInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IRaycastLeafProcessor
            where TCollector : struct, ICollector<RaycastHit>
        {
            var hadHit = false;
            int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
            *top++ = 1;
            do
            {
                var node = m_Nodes + *(--top);
                var hitMask = node->Bounds.Raycast(input.Ray, collector.MaxFraction, out var hitFractions);
                int4 hitData;
                var hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < hitCount; i++)
                    {
                        var index = hitData[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            hadHit |= leafProcessor.RayLeaf(input, index, ref collector);
                            if (collector.EarlyOutOnHit && hadHit)
                            {
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    *((int4*)top) = hitData;
                    top += hitCount;
                }
            } while (top > stack);

            return hadHit;
        }

        #endregion

        #region Collider cast query

        public interface IColliderCastLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool ColliderCastLeaf<T>(ColliderCastInput input, int leafData, ref T collector) where T : struct, ICollector<ColliderCastHit>;
        }

        public unsafe bool ColliderCast<TProcessor, TCollector>(ColliderCastInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IColliderCastLeafProcessor
            where TCollector : struct, ICollector<ColliderCastHit>
        {
            float2 aabbExtents;
            Ray aabbRay;
            {
                var aabb = input.Collider.Value.CalculateAabb(new PhysicsTransform { Translation = input.Start, Rotation = input.Rotation });
                aabbExtents = aabb.Extents;
                aabbRay = input.Ray;
                aabbRay.Origin = aabb.Min;
            }

            var hadHit = false;

            int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
            *top++ = 1;
            do
            {
                var node = m_Nodes + *(--top);
                var bounds = node->Bounds;
                bounds.MinX -= aabbExtents.x;
                bounds.MinY -= aabbExtents.y;
                var hitMask = bounds.Raycast(aabbRay, collector.MaxFraction, out var hitFractions);
                int4 hitData;
                var hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < hitCount; i++)
                    {
                        var index = hitData[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            hadHit |= leafProcessor.ColliderCastLeaf(input, index, ref collector);
                            if (collector.EarlyOutOnHit && hadHit)
                            {
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    *((int4*)top) = hitData;
                    top += hitCount;
                }
            } while (top > stack);

            return hadHit;
        }

        #endregion

        #region Point distance query

        public interface IPointDistanceLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool DistanceLeaf<T>(PointDistanceInput input, int leafData, ref T collector) where T : struct, ICollector<DistanceHit>;
        }

        public unsafe bool Distance<TProcessor, TCollector>(PointDistanceInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IPointDistanceLeafProcessor
            where TCollector : struct, ICollector<DistanceHit>
        {
            SafetyChecks.IsTrue(collector.MaxFraction <= input.MaxDistance);

            var hadHit = false;

            var binaryStack = stackalloc int[Constants.BinaryStackSize];
            var stack = binaryStack;
            *stack++ = 1;

            var pointT = new FourPoints(input.Position);
            var maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);

            do
            {
                var nodeIndex = *(--stack);
                var node = m_Nodes + nodeIndex;
                var distanceToNodesSquared = node->Bounds.DistanceFromPointSquared(ref pointT);
                var overlap = (node->Bounds.MinX <= node->Bounds.MaxX) & (distanceToNodesSquared <= maxDistanceSquared);
                int4 hitData;
                var hitCount = math.compress((int*)(&hitData), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < hitCount; i++)
                    {
                        var index = hitData[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            hadHit |= leafProcessor.DistanceLeaf(input, index, ref collector);

                            if (collector.EarlyOutOnHit && hadHit)
                            {
                                return true;
                            }
                        }
                    }

                    maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);
                }
                else
                {
                    *((int4*)stack) = hitData;
                    stack += hitCount;
                }
            } while (stack > binaryStack);

            return hadHit;
        }

        #endregion

        #region Collider distance query

        public interface IColliderDistanceLeafProcessor
        {
            // Called when the query hits a leaf node of the bounding volume hierarchy
            bool DistanceLeaf<T>(ColliderDistanceInput input, int leafData, ref T collector) where T : struct, ICollector<DistanceHit>;
        }

        public unsafe bool Distance<TProcessor, TCollector>(ColliderDistanceInput input, ref TProcessor leafProcessor, ref TCollector collector)
            where TProcessor : struct, IColliderDistanceLeafProcessor
            where TCollector : struct, ICollector<DistanceHit>
        {
            SafetyChecks.IsTrue(collector.MaxFraction <= input.MaxDistance);

            var hadHit = false;

            var binaryStack = stackalloc int[Constants.BinaryStackSize];
            var stack = binaryStack;
            *stack++ = 1;

            var aabb = input.Collider.Value.CalculateAabb(input.Transform);
            FourAabb aabbT;
            (&aabbT)->SetAllAabbs(aabb);
            var maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);

            do
            {
                var nodeIndex = *(--stack);
                var node = m_Nodes + nodeIndex;
                var distanceToNodesSquared = node->Bounds.DistanceFromAabbSquared(ref aabbT);
                var overlap = (node->Bounds.MinX <= node->Bounds.MaxX) & (distanceToNodesSquared <= maxDistanceSquared);
                int4 hitData;
                var hitCount = math.compress((int*)(&hitData), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (var i = 0; i < hitCount; i++)
                    {
                        var index = hitData[i];
                        if (!(input.Ignore.IgnoreBody && input.Ignore.IgnoreBodyIndex == index))
                        {
                            hadHit |= leafProcessor.DistanceLeaf(input, index, ref collector);
                            if (collector.EarlyOutOnHit && hadHit)
                            {
                                return true;
                            }
                        }

                        maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);
                    }
                }
                else
                {
                    *((int4*)stack) = hitData;
                    stack += hitCount;
                }
            } while (stack > binaryStack);

            return hadHit;
        }

        #endregion
    }
}
