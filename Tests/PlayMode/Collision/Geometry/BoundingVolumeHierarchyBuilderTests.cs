using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class BoundingVolumeHierarchyBuilderTests
    {
        public void InitializeInputArrays(NativeArray<BoundingVolumeHierarchy.PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> filters)
        {           
            var random = new Mathematics.Random(1234);

            const float posRange = 1000f;
            const float radiusRangeMin = 1f;
            const float radiusRangeMax = 10f;

            for (int i = 0; i < points.Length; i++)
            {
                float2 pos;
                pos.x = random.NextFloat(-posRange, posRange);
                pos.y = random.NextFloat(-posRange, posRange);
                points[i] = new BoundingVolumeHierarchy.PointAndIndex { Position = pos, Index = i };

                var radius = new float2(random.NextFloat(radiusRangeMin, radiusRangeMax));
                aabbs[i] = new Aabb { Min = pos - radius, Max = pos + radius };

                filters[i] = CollisionFilter.Default;
            }
        }

        public void InitializeInputWithCopyArrays(NativeArray<BoundingVolumeHierarchy.PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> filters)
        {
            var random = new Mathematics.Random(1234);

            const float posRange = 1000f;
            const float radiusRangeMin = 1f;
            const float radiusRangeMax = 10f;

            for (int i = 0; i < points.Length; i++)
            {
                var pos = random.NextFloat2(-posRange, posRange);
                points[i] = new BoundingVolumeHierarchy.PointAndIndex { Position = pos, Index = i };

                var radius = new float2(random.NextFloat(radiusRangeMin, radiusRangeMax));
                aabbs[i] = new Aabb { Min = pos - radius, Max = pos + radius };

                points[i + 1] = new BoundingVolumeHierarchy.PointAndIndex { Position = pos, Index = i + 1 };

                aabbs[i + 1] = new Aabb { Min = pos - radius, Max = pos + radius };

                filters[i] = new CollisionFilter
                {
                    GroupIndex = 0,
                    BelongsTo = random.NextUInt(0, 16),
                    CollidesWith = random.NextUInt(0, 16)
                };

                filters[i + 1] = new CollisionFilter
                {
                    GroupIndex = 0,
                    BelongsTo = random.NextUInt(0, 16),
                    CollidesWith = random.NextUInt(0, 16)
                };

                i++;
            }
        }

        [Test]
        public void BuildTree([Values(2, 10, 100, 1000)] int elementCount)
        {
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            InitializeInputArrays(points, aabbs, filters);

            var nodes = new NativeArray<BoundingVolumeHierarchy.Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodesOut);
            bvh.CheckIntegrity();

            points.Dispose();
            filters.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
        }

        [Test]
        public void BuildTreeByBranches([Values(2, 10, 33, 100, 1000)] int elementCount)
        {
            const int threadCount = 8;
            int numNodes = elementCount + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;

            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            InitializeInputArrays(points, aabbs, filters);

            var nodes = new NativeArray<BoundingVolumeHierarchy.Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var branchNodeOffsets = new NativeArray<int>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);

            bvh.BuildFirstNLevels(points, ranges, branchNodeOffsets, threadCount, out int branchCount);

            int minBranchNodeIndex = branchNodeOffsets[0];
            for (int i = 0; i < branchCount; i++)
            {
                bvh.BuildBranch(points, aabbs, ranges[i], branchNodeOffsets[i]);
                minBranchNodeIndex = math.min(branchNodeOffsets[i], minBranchNodeIndex);
            }

            bvh.Refit(aabbs, 1, minBranchNodeIndex);

            bvh.CheckIntegrity();

            points.Dispose();
            filters.Dispose();
            aabbs.Dispose();
            nodes.Dispose();

            ranges.Dispose();
            branchNodeOffsets.Dispose();
        }

        [Test]
        public unsafe void BuildTreeTasks([Values(2, 10, 33, 100, 1000)] int elementCount)
        {
            const int threadCount = 8;
            int numNodes = elementCount + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;

            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            InitializeInputArrays(points, aabbs, filters);

            var nodes = new NativeArray<BoundingVolumeHierarchy.Node>(numNodes, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(BoundingVolumeHierarchy.Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchNodeOffset = new NativeArray<int>(BoundingVolumeHierarchy.Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var branchCount = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var shouldDoWork = new NativeArray<int>(1, Allocator.Persistent);
            shouldDoWork[0] = 1;
            int oldBranchCount = branchCount[0];

            var handle = new BoundingVolumeHierarchy.BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = (BoundingVolumeHierarchy.Node*)nodes.GetUnsafePtr(),
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount,
                ThreadCount = threadCount,
                ShouldDoWork = shouldDoWork
            }.Schedule();

            handle = new BoundingVolumeHierarchy.BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = filters,
                Nodes = (BoundingVolumeHierarchy.Node*)nodes.GetUnsafePtr(),
                NodeFilters = null,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffset,
                BranchCount = branchCount
            }.ScheduleUnsafeIndex0(branchCount, 1, handle);

            new BoundingVolumeHierarchy.FinalizeTreeJob
            {
                Aabbs = aabbs,
                Nodes = (BoundingVolumeHierarchy.Node*)nodes.GetUnsafePtr(),
                BranchNodeOffsets = branchNodeOffset,
                NumNodes = nodes.Length,
                LeafFilters = filters,
                BranchCount = branchCount,
                OldBranchCount = oldBranchCount,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle).Complete();

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.CheckIntegrity();

            filters.Dispose();
            nodes.Dispose();
            ranges.Dispose();
            branchCount.Dispose();
            shouldDoWork.Dispose();
        }

        struct PairBuffer : BoundingVolumeHierarchy.ITreeOverlapCollector, IDisposable
        {
            public NativeList<PhysicsBody.IndexPair> Pairs;

            public void AddPairs(int l, int4 r, int countR)
            {
                for (int i = 0; i < countR; i++)
                {
                    Pairs.Add(new PhysicsBody.IndexPair { PhysicsBodyIndexA = l, PhysicsBodyIndexB = r[i] });
                }
            }

            public void AddPairs(int4 pairLeft, int4 r, int count, bool swapped = false)
            {
                for (int i = 0; i < count; i++)
                {
                    Pairs.Add(new PhysicsBody.IndexPair { PhysicsBodyIndexA = pairLeft[i], PhysicsBodyIndexB = r[i] });
                }
            }

            public void FlushIfNeeded()
            {
            }

            public void Dispose()
            {
                if (Pairs.IsCreated)
                    Pairs.Dispose();
            }

            public int MaxId;
        }

        [Test]
        public unsafe void BuildTreeAndOverlap([Values(2, 10, 33, 100)] int elementCount)
        {
            elementCount *= 2;
            int numNodes = elementCount / 3 * 2 + 4;
            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var filters = new NativeArray<CollisionFilter>(elementCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            InitializeInputWithCopyArrays(points, aabbs, filters);

            var nodes = new NativeArray<BoundingVolumeHierarchy.Node>(numNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodesOut);
            bvh.CheckIntegrity();

            var buffer = new PairBuffer { Pairs = new NativeList<PhysicsBody.IndexPair>(0, Allocator.Temp) };
            buffer.MaxId = elementCount - 1;

            BoundingVolumeHierarchy.Node* nodesPtr = (BoundingVolumeHierarchy.Node*)nodes.GetUnsafePtr();
            BoundingVolumeHierarchy.TreeOverlap(ref buffer, nodesPtr, nodesPtr);

            int numCollidingPairs = buffer.Pairs.Length;
            Assert.AreEqual(elementCount / 2, numCollidingPairs);

            buffer.Dispose();
            filters.Dispose();
            points.Dispose();
            aabbs.Dispose();
            nodes.Dispose();
        }
    }
}
