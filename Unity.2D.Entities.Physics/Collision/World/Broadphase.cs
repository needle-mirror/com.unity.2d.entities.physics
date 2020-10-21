using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using static Unity.U2D.Entities.Physics.BoundingVolumeHierarchy;

namespace Unity.U2D.Entities.Physics
{
    public struct Broadphase : IDisposable
    {
        Tree m_StaticTree;
        Tree m_DynamicTree;

        public Tree StaticTree => m_StaticTree;
        public Tree DynamicTree => m_DynamicTree;
        public Aabb Domain => Aabb.Union(m_StaticTree.BoundingVolumeHierarchy.Domain, m_DynamicTree.BoundingVolumeHierarchy.Domain);
        public int StaticBodyCount => m_StaticTree.BodyCount;
        public int DynamicBodyCount => m_DynamicTree.BodyCount;

        public Broadphase(int staticBodyCount, int dynamicBodyCount)
        {
            m_StaticTree = new Tree(staticBodyCount);
            m_DynamicTree = new Tree(dynamicBodyCount);
        }

        public void Reset(int staticBodyCount, int dynamicBodyCount)
        {
            m_StaticTree.Reset(staticBodyCount);
            m_DynamicTree.Reset(dynamicBodyCount);
        }

        #region Queries

        public bool OverlapAabb(OverlapAabbInput input, NativeArray<PhysicsBody> rigidBodies, NativeList<int> physicsBodyIndices)
        {
            if (input.Filter.IsEmpty)
                return false;

            var hitsBefore = physicsBodyIndices.Length;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            var leafCollector = new PhysicsBodyOverlapsCollector { PhysicsBodyIndices = physicsBodyIndices };

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            m_StaticTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            m_DynamicTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);

            return physicsBodyIndices.Length > hitsBefore;
        }

        public bool OverlapPoint<T>(OverlapPointInput input, NativeArray<PhysicsBody> rigidBodies, ref T collector)
            where T : struct, ICollector<OverlapPointHit>
        {
            if (input.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            var hasHit = m_StaticTree.BoundingVolumeHierarchy.OverlapPoint(input, ref leafProcessor, ref collector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.OverlapPoint(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public bool OverlapCollider<T>(OverlapColliderInput input, NativeArray<PhysicsBody> rigidBodies, ref T collector)
            where T : struct, ICollector<OverlapColliderHit>
        {
            if (input.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            var hasHit = m_StaticTree.BoundingVolumeHierarchy.OverlapCollider(input, ref leafProcessor, ref collector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.OverlapCollider(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public bool CastRay<T>(RaycastInput input, NativeArray<PhysicsBody> rigidBodies, ref T collector)
            where T : struct, ICollector<RaycastHit>
        {
            if (input.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            var hasHit = m_StaticTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public bool CastCollider<T>(ColliderCastInput input, NativeArray<PhysicsBody> rigidBodies, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            SafetyChecks.IsTrue(input.Collider.IsCreated);

            if (input.Collider.Value.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            var hasHit = m_StaticTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public bool CalculateDistance<T>(PointDistanceInput input, NativeArray<PhysicsBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            if (input.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            var hasHit = m_StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        public bool CalculateDistance<T>(ColliderDistanceInput input, NativeArray<PhysicsBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            SafetyChecks.IsTrue(input.Collider.IsCreated);

            if (input.Collider.Value.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            // Offset to Static bodies.
            leafProcessor.BasePhysicsBodyIndex = m_DynamicTree.BodyCount;
            var hasHit = m_StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            // No offset to Dynamic bodies.
            leafProcessor.BasePhysicsBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
            return hasHit;
        }

        internal struct PhysicsBodyOverlapsCollector : IOverlapCollector
        {
            public NativeList<int> PhysicsBodyIndices;

            public unsafe void AddPhysicsBodyIndices(int* indices, int count)
            {
                PhysicsBodyIndices.AddRange(indices, count);
            }

            public unsafe void AddColliderKeys(ColliderKey* keys, int count)
            {
                SafetyChecks.ThrowNotSupportedException();
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey)
            {
                SafetyChecks.ThrowNotSupportedException();
            }

            public void PopCompositeCollider(uint numCompositeKeyBits)
            {
                SafetyChecks.ThrowNotSupportedException();
            }
        }

        internal struct BvhLeafProcessor :
            IRaycastLeafProcessor,
            IColliderCastLeafProcessor,
            IPointOverlapLeafProcessor,
            IColliderOverlapLeafProcessor,
            IColliderDistanceLeafProcessor,
            IPointDistanceLeafProcessor,
            IAabbOverlapLeafProcessor
        {
            readonly NativeArray<PhysicsBody> m_Bodies;
            public int BasePhysicsBodyIndex;

            public BvhLeafProcessor(NativeArray<PhysicsBody> bodies)
            {
                m_Bodies = bodies;
                BasePhysicsBodyIndex = 0;
            }

            public bool AabbOverlap(int physicsBodyIndex, ref NativeList<int> allHits)
            {
                allHits.Add(BasePhysicsBodyIndex + physicsBodyIndex);
                return true;
            }

            public bool RayLeaf<T>(RaycastInput input, int physicsBodyIndex, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                var body = m_Bodies[physicsBodyIndex];

                var worldFromBody = body.WorldTransform;

                // Transform the ray into body space
                var inputLs = input;
                {
                    var bodyFromWorld = PhysicsMath.inverse(worldFromBody);
                    inputLs.Start = PhysicsMath.mul(bodyFromWorld, input.Start);
                    inputLs.End = PhysicsMath.mul(bodyFromWorld, input.End);
                    inputLs.QueryContext = new QueryContext(physicsBodyIndex, body.Entity, worldFromBody);
                }

                return body.CastRay(inputLs, ref collector);
            }

            public bool ColliderCastLeaf<T>(ColliderCastInput input, int physicsBodyIndex, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                var body = m_Bodies[physicsBodyIndex];

                // Transform the input into body space
                var worldFromBody = body.WorldTransform;
                var bodyFromWorld = PhysicsMath.inverse(worldFromBody);
                var inputLs = new ColliderCastInput
                {
                    Collider = input.Collider,
                    Ignore = input.Ignore,
                    Rotation = math.mul(math.inverse(body.WorldTransform.Rotation), input.Rotation),
                    Start = PhysicsMath.mul(bodyFromWorld, input.Start),
                    End = PhysicsMath.mul(bodyFromWorld, input.End),
                    QueryContext = new QueryContext(physicsBodyIndex, body.Entity, worldFromBody)
                };

                return body.CastCollider(inputLs, ref collector);
            }

            public bool PointLeaf<T>(OverlapPointInput input, int physicsBodyIndex, ref T collector) where T : struct, ICollector<OverlapPointHit>
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                var body = m_Bodies[physicsBodyIndex];

                var worldFromBody = body.WorldTransform;

                // Transform the ray into body space
                var inputLs = input;
                {
                    var bodyFromWorld = PhysicsMath.inverse(worldFromBody);
                    inputLs.Position = PhysicsMath.mul(bodyFromWorld, input.Position);
                    inputLs.QueryContext = new QueryContext(physicsBodyIndex, body.Entity, worldFromBody);
                }

                return body.OverlapPoint(inputLs, ref collector);
            }

            public bool ColliderLeaf<T>(OverlapColliderInput input, int physicsBodyIndex, ref T collector) where T : struct, ICollector<OverlapColliderHit>
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                var body = m_Bodies[physicsBodyIndex];

                var worldFromBody = body.WorldTransform;

                // Transform the ray into body space
                var inputLs = input;
                {
                    var bodyFromWorld = PhysicsMath.inverse(worldFromBody);
                    inputLs.Transform = PhysicsMath.mul(bodyFromWorld, input.Transform);
                    inputLs.QueryContext = new QueryContext(physicsBodyIndex, body.Entity, worldFromBody);
                }

                return body.OverlapCollider(inputLs, ref collector);
            }

            public bool DistanceLeaf<T>(PointDistanceInput input, int physicsBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                var body = m_Bodies[physicsBodyIndex];

                // Transform the input into body space
                var worldFromBody = body.WorldTransform;
                var bodyFromWorld = PhysicsMath.inverse(worldFromBody);
                var inputLs = new PointDistanceInput
                {
                    Position = PhysicsMath.mul(bodyFromWorld, input.Position),
                    MaxDistance = input.MaxDistance,
                    Filter = input.Filter,
                    QueryContext = new QueryContext(physicsBodyIndex, body.Entity, worldFromBody)
                };

                return body.CalculateDistance(inputLs, ref collector);
            }

            public bool DistanceLeaf<T>(ColliderDistanceInput input, int physicsBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                if (physicsBodyIndex > m_Bodies.Length)
                {
                }
                var body = m_Bodies[physicsBodyIndex];

                // Transform the input into body space
                var worldFromBody = body.WorldTransform;
                var bodyFromWorld = PhysicsMath.inverse(worldFromBody);
                var inputLs = new ColliderDistanceInput
                {
                    Collider = input.Collider,
                    Transform = new PhysicsTransform
                    {
                        Translation = PhysicsMath.mul(bodyFromWorld, input.Transform.Translation),
                        Rotation = math.mul(math.inverse(body.WorldTransform.Rotation), input.Transform.Rotation)
                    },
                    MaxDistance = input.MaxDistance,
                    QueryContext = new QueryContext(physicsBodyIndex, body.Entity, worldFromBody)
                };

                return body.CalculateDistance(inputLs, ref collector);
            }

            public unsafe void AabbLeaf<T>(OverlapAabbInput input, int physicsBodyIndex, ref T collector)
                where T : struct, IOverlapCollector
            {
                physicsBodyIndex += BasePhysicsBodyIndex;
                var body = m_Bodies[physicsBodyIndex];
                if (body.Collider.IsCreated && CollisionFilter.IsCollisionEnabled(input.Filter, body.Collider.Value.Filter))
                {
                    collector.AddPhysicsBodyIndices(&physicsBodyIndex, 1);
                }
            }
        }

        #endregion

        #region Cloneable

        public Broadphase Clone()
        {
            return new Broadphase
            {
                m_StaticTree = m_StaticTree.Clone(),
                m_DynamicTree = m_DynamicTree.Clone()
            };
        }

        #endregion

        #region IDisposable

        public void Dispose()
        {
            m_StaticTree.Dispose();
            m_DynamicTree.Dispose();
        }

        #endregion

        #region Build

        /// <summary>
        /// Build the broadphase based on the given world.
        /// </summary>
        public void Build(
            NativeArray<PhysicsBody> staticBodies, NativeArray<PhysicsBody> dynamicBodies,
            NativeArray<PhysicsBody.MotionData> motionDatas, NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
            float collisionTolerance,
            float timeStep,
            float2 gravity,
            bool buildStaticTree = true)
        {
            var aabbMargin = collisionTolerance * 0.5f; // each body contributes half

            if (buildStaticTree)
            {
                m_StaticTree.Reset(staticBodies.Length);
                BuildStaticTree(staticBodies, aabbMargin);
            }

            m_DynamicTree.Reset(dynamicBodies.Length);
            BuildDynamicTree(dynamicBodies, motionDatas, motionVelocities, gravity, timeStep, aabbMargin);
        }

        /// <summary>
        /// Build the static tree of the broadphase based on the given array of rigid bodies.
        /// </summary>
        public void BuildStaticTree(NativeArray<PhysicsBody> staticBodies, float aabbMargin)
        {
            SafetyChecks.IsTrue(staticBodies.Length == m_StaticTree.BodyCount);

            if (staticBodies.Length == 0)
            {
                return;
            }

            // Read bodies
            var aabbs = new NativeArray<Aabb>(staticBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(staticBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (var i = 0; i < staticBodies.Length; i++)
            {
                PrepareStaticBodyDataJob.Execute(i, aabbMargin, staticBodies, aabbs, points, m_StaticTree.BodyFilters);
            }

            // Build tree
            m_StaticTree.BoundingVolumeHierarchy.Build(points, aabbs, out var nodeCount);

            // Build node filters
            m_StaticTree.BoundingVolumeHierarchy.BuildCombinedCollisionFilter(m_StaticTree.BodyFilters, 1, nodeCount - 1);
        }

        /// <summary>
        /// Build the dynamic tree of the broadphase based on the given array of rigid bodies and motions.
        /// </summary>
        public void BuildDynamicTree(
            NativeArray<PhysicsBody> dynamicBodies,
            NativeArray<PhysicsBody.MotionData> motionDatas, NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
            float2 gravity, float timeStep, float aabbMargin)
        {
            SafetyChecks.IsTrue(dynamicBodies.Length == m_DynamicTree.BodyCount);

            if (dynamicBodies.Length == 0)
            {
                return;
            }

            // Read bodies
            var aabbs = new NativeArray<Aabb>(dynamicBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(dynamicBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (var i = 0; i < dynamicBodies.Length; i++)
            {
                PrepareDynamicBodyDataJob.Execute(i, aabbMargin, gravity, timeStep, dynamicBodies, motionDatas, motionVelocities, aabbs, points, m_DynamicTree.BodyFilters);
            }

            // Build tree
            m_DynamicTree.BoundingVolumeHierarchy.Build(points, aabbs, out var nodeCount);

            // Build node filters
            m_DynamicTree.BoundingVolumeHierarchy.BuildCombinedCollisionFilter(m_DynamicTree.BodyFilters, 1, nodeCount - 1);
        }

        #endregion

        #region Find overlaps

        // Write all overlapping body pairs to the given streams,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public void FindOverlaps(ref NativeStream.Writer dynamicVsDynamicPairsWriter, ref NativeStream.Writer staticVsDynamicPairsWriter)
        {
            // Dynamic-dynamic
            {
                dynamicVsDynamicPairsWriter.BeginForEachIndex(0);
                DynamicVsDynamicFindOverlappingPairsJob.Execute(
                    new int2(1, 1), m_DynamicTree, ref dynamicVsDynamicPairsWriter);
                dynamicVsDynamicPairsWriter.EndForEachIndex();
            }

            // Static-dynamic
            {
                staticVsDynamicPairsWriter.BeginForEachIndex(0);
                StaticVsDynamicFindOverlappingPairsJob.Execute(
                    new int2(1, 1), m_StaticTree, m_DynamicTree, ref staticVsDynamicPairsWriter);
                staticVsDynamicPairsWriter.EndForEachIndex();
            }
        }

        // Schedule a set of jobs which will write all overlapping body pairs to the given steam,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, int threadCountHint = 0)
        {
            SimulationJobHandles returnHandles = default;

            if (threadCountHint <= 0)
            {
                dynamicVsDynamicPairsStream = new NativeStream(1, Allocator.TempJob);
                staticVsDynamicPairsStream = new NativeStream(1, Allocator.TempJob);
                returnHandles.FinalExecutionHandle = new FindOverlapsJob
                {
                    Broadphase = this,
                    DynamicVsDynamicPairsWriter = dynamicVsDynamicPairsStream.AsWriter(),
                    StaticVsDynamicPairsWriter = staticVsDynamicPairsStream.AsWriter()
                    
                }.Schedule(inputDeps);

                return returnHandles;
            }

            var dynamicVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);
            var staticVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);

            var allocateDeps = new AllocateDynamicVsStaticNodePairs
            {
                dynamicVsDynamicNodePairIndices = dynamicVsDynamicNodePairIndices,
                staticVsDynamicNodePairIndices = staticVsDynamicNodePairIndices,
                dynamicBranchCount = m_DynamicTree.BranchCount,
                staticBranchCount = m_StaticTree.BranchCount
            }.Schedule(inputDeps);

            // Build pairs of branch node indices
            var dynamicVsDynamicPairs = new DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = m_DynamicTree.Ranges,
                NumBranches = m_DynamicTree.BranchCount,
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
                
            }.Schedule(allocateDeps);

            var staticVsDynamicPairs = new StaticVsDynamicBuildBranchNodePairsJob
            {
                DynamicRanges = m_DynamicTree.Ranges,
                StaticRanges = m_StaticTree.Ranges,
                NumStaticBranches = m_StaticTree.BranchCount,
                NumDynamicBranches = m_DynamicTree.BranchCount,
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
                
            }.Schedule(allocateDeps);

            //@TODO: We only need a dependency on allocateDeps, but the safety system doesn't understand that we can not change length list in DynamicVsDynamicBuildBranchNodePairsJob & StaticVsDynamicBuildBranchNodePairsJob
            //       if this is a performance issue we can use [NativeDisableContainerSafetyRestriction] on DynamicVsDynamicBuildBranchNodePairsJob & StaticVsDynamicBuildBranchNodePairsJob 
            var dynamicConstruct = NativeStream.ScheduleConstruct(out dynamicVsDynamicPairsStream, dynamicVsDynamicNodePairIndices, dynamicVsDynamicPairs, Allocator.TempJob);
            var staticConstruct = NativeStream.ScheduleConstruct(out staticVsDynamicPairsStream, staticVsDynamicNodePairIndices, staticVsDynamicPairs, Allocator.TempJob);

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            var dynamicVsDynamicHandle = new DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicTree = m_DynamicTree,
                PairWriter = dynamicVsDynamicPairsStream.AsWriter(),
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
                
            }.Schedule(dynamicVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(dynamicVsDynamicPairs, dynamicConstruct));

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            var staticVsDynamicHandle = new StaticVsDynamicFindOverlappingPairsJob
            {
                StaticTree = m_StaticTree,
                DynamicTree = m_DynamicTree,
                PairWriter = staticVsDynamicPairsStream.AsWriter(),
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
                
            }.Schedule(staticVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(staticVsDynamicPairs, staticConstruct));

            // Dispose node pair lists
            var disposeOverlapPairs0 = NativeListUtility.DisposeHotFix(ref dynamicVsDynamicNodePairIndices, dynamicVsDynamicHandle);
            var disposeOverlapPairs1 = NativeListUtility.DisposeHotFix(ref staticVsDynamicNodePairIndices, staticVsDynamicHandle);
            returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(disposeOverlapPairs0, disposeOverlapPairs1);
            returnHandles.FinalExecutionHandle = JobHandle.CombineDependencies(dynamicVsDynamicHandle, staticVsDynamicHandle);

            return returnHandles;
        }
        
        // An implementation of IOverlapCollector which filters and writes body pairs to a native stream
        unsafe struct BodyPairWriter : ITreeOverlapCollector
        {
            const int k_Capacity = 256;
            const int k_Margin = 64;
            const int k_Threshold = k_Capacity - k_Margin;

            fixed int m_PairsLeft[k_Capacity];
            fixed int m_PairsRight[k_Capacity];

            readonly NativeStream.Writer* m_CollidingPairs;
            readonly CollisionFilter* m_BodyFiltersLeft;
            readonly CollisionFilter* m_BodyFiltersRight;
            readonly int m_BodyAIndexBase;
            readonly int m_BodyBIndexBase;
            int m_Count;

            public BodyPairWriter(NativeStream.Writer* collidingPairs, CollisionFilter* bodyFiltersLeft, CollisionFilter* bodyFiltersRight,
                int bodyAIndexBase, int bodyBIndexBase)
            {
                m_CollidingPairs = collidingPairs;
                m_BodyFiltersLeft = bodyFiltersLeft;
                m_BodyFiltersRight = bodyFiltersRight;
                m_BodyAIndexBase = bodyAIndexBase;
                m_BodyBIndexBase = bodyBIndexBase;
                m_Count = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int4 pairsLeft, int4 pairsRight, int count, bool swapped = false)
            {
                if (swapped)
                {
                    fixed (int* l = m_PairsRight)
                    {
                        *((int4*)(l + m_Count)) = pairsLeft;
                    }

                    fixed (int* r = m_PairsLeft)
                    {
                        *((int4*)(r + m_Count)) = pairsRight;
                    }
                }
                else
                {
                    fixed (int* l = m_PairsLeft)
                    {
                        *((int4*)(l + m_Count)) = pairsLeft;
                    }

                    fixed (int* r = m_PairsRight)
                    {
                        *((int4*)(r + m_Count)) = pairsRight;
                    }
                }

                m_Count += count;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int pairLeft, int4 pairsRight, int countR)
            {
                fixed (int* l = m_PairsLeft)
                {
                    *((int4*)(l + m_Count)) = new int4(pairLeft);
                }

                fixed (int* r = m_PairsRight)
                {
                    *((int4*)(r + m_Count)) = pairsRight;
                }

                m_Count += countR;
            }

            public void Close()
            {
                Flush();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void FlushIfNeeded()
            {
                if (m_Count >= k_Threshold)
                {
                    Flush();
                }
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            void Flush()
            {
                if (m_Count != 0)
                {
                    fixed (int* l = m_PairsLeft)
                    {
                        fixed (int* r = m_PairsRight)
                        {
                            for (var i = 0; i < m_Count; i++)
                            {
                                var bodyALocalIndex = l[i];
                                var bodyBLocalIndex = r[i];
                                if (CollisionFilter.IsCollisionEnabled(m_BodyFiltersLeft[bodyALocalIndex], m_BodyFiltersRight[bodyBLocalIndex]))
                                {
                                    m_CollidingPairs->Write(new PhysicsBody.IndexPair
                                    {
                                        PhysicsBodyIndexA = bodyALocalIndex + m_BodyAIndexBase,
                                        PhysicsBodyIndexB = bodyBLocalIndex + m_BodyBIndexBase
                                    });
                                }
                            }
                        }
                    }

                    m_Count = 0;
                }
            }
        }
        
        // Writes pairs of overlapping broadphase AABBs to a stream, in a single job.
        [BurstCompile]
        struct FindOverlapsJob : IJob
        {
            [ReadOnly] public Broadphase Broadphase;
            public NativeStream.Writer DynamicVsDynamicPairsWriter;
            public NativeStream.Writer StaticVsDynamicPairsWriter;

            public void Execute()
            {
                Broadphase.FindOverlaps(ref DynamicVsDynamicPairsWriter, ref StaticVsDynamicPairsWriter);
            }
        }
        
        // Allocate memory for pair indices
        [BurstCompile]
        struct AllocateDynamicVsStaticNodePairs : IJob
        {
            [ReadOnly] public NativeArray<int> dynamicBranchCount;
            [ReadOnly] public NativeArray<int> staticBranchCount;

            public NativeList<int2> dynamicVsDynamicNodePairIndices;
            public NativeList<int2> staticVsDynamicNodePairIndices;

            public void Execute()
            {
                var numDynamicVsDynamicBranchOverlapPairs = dynamicBranchCount[0] * (dynamicBranchCount[0] + 1) / 2;
                dynamicVsDynamicNodePairIndices.ResizeUninitialized(numDynamicVsDynamicBranchOverlapPairs);

                var numStaticVsDynamicBranchOverlapPairs = staticBranchCount[0] * dynamicBranchCount[0];
                staticVsDynamicNodePairIndices.ResizeUninitialized(numStaticVsDynamicBranchOverlapPairs);
            }
        }
        
        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        struct DynamicVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> NumBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                var numBranches = NumBranches[0];

                var arrayIndex = 0;

                // First add all branch self overlaps.
                // Start with largest branch
                for (var i = 0; i < numBranches; i++)
                {
                    NodePairIndices[arrayIndex++] = new int2(Ranges[i].Root, Ranges[i].Root);
                }

                for (var i = 0; i < numBranches; i++)
                {
                    for (var j = i + 1; j < numBranches; j++)
                    {
                        var pair = new int2 { x = Ranges[i].Root, y = Ranges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        struct StaticVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> StaticRanges;
            [ReadOnly] public NativeArray<Builder.Range> DynamicRanges;
            [ReadOnly] public NativeArray<int> NumStaticBranches;
            [ReadOnly] public NativeArray<int> NumDynamicBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                var numStaticBranches = NumStaticBranches[0];
                var numDynamicBranches = NumDynamicBranches[0];

                var arrayIndex = 0;
                for (var i = 0; i < numStaticBranches; i++)
                {
                    for (var j = 0; j < numDynamicBranches; j++)
                    {
                        var pair = new int2 { x = StaticRanges[i].Root, y = DynamicRanges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }        
        
        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        struct DynamicVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public Tree DynamicTree;
            [ReadOnly] public NativeArray<int2> NodePairIndices;
            public NativeStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                var pair = NodePairIndices[index];
                Execute(pair, DynamicTree, ref PairWriter);

                PairWriter.EndForEachIndex();
            }

            internal static unsafe void Execute(int2 pair, Tree dynamicTree, ref NativeStream.Writer pairWriter)
            {
                var bodyFilters = (CollisionFilter*)dynamicTree.BodyFilters.GetUnsafeReadOnlyPtr();
                var bufferedPairs = new BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref pairWriter), bodyFilters, bodyFilters, 0, 0);
                new BoundingVolumeHierarchy(dynamicTree.Nodes, dynamicTree.NodeFilters).SelfBvhOverlap(ref bufferedPairs, pair.x, pair.y);
                bufferedPairs.Close();
            }
        }
        
        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        struct StaticVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public Tree StaticTree;
            [ReadOnly] public Tree DynamicTree;
            [ReadOnly] public NativeArray<int2> NodePairIndices;
            public NativeStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                var pair = NodePairIndices[index];
                Execute(pair, StaticTree, DynamicTree, ref PairWriter);

                PairWriter.EndForEachIndex();
            }

            internal static unsafe void Execute(int2 pair, Tree staticTree, Tree dynamicTree, ref NativeStream.Writer pairWriter)
            {
                var staticBvh = new BoundingVolumeHierarchy(staticTree.Nodes, staticTree.NodeFilters);
                var dynamicBvh = new BoundingVolumeHierarchy(dynamicTree.Nodes, dynamicTree.NodeFilters);

                var bodyPairWriter = new BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref pairWriter),
                    (CollisionFilter*)staticTree.BodyFilters.GetUnsafeReadOnlyPtr(), (CollisionFilter*)dynamicTree.BodyFilters.GetUnsafeReadOnlyPtr(),
                    dynamicTree.BodyCount, 0);

                staticBvh.BvhOverlap(ref bodyPairWriter, dynamicBvh, pair.x, pair.y);

                bodyPairWriter.Close();
            }
        }
        
        #endregion
        
        #region Tree

        public struct Tree : IDisposable
        {
            public NativeArray<Node> Nodes; // The nodes of the bounding volume
            public NativeArray<CollisionFilter> NodeFilters; // The collision filter for each node (a union of all its children)
            public NativeArray<CollisionFilter> BodyFilters; // A copy of the collision filter of each body
            internal NativeArray<Builder.Range> Ranges; // Used during building
            internal NativeArray<int> BranchCount; // Used during building

            public BoundingVolumeHierarchy BoundingVolumeHierarchy => new BoundingVolumeHierarchy(Nodes, NodeFilters);

            public int BodyCount => BodyFilters.Length;

            public Tree(int bodyCount)
            {
                this = default;
                SetCapacity(bodyCount);
                Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(
                    BoundingVolumeHierarchy.Constants.MaxNumTreeBranches, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                BranchCount = new NativeArray<int>(1, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            }

            public void Reset(int bodyCount)
            {
                if (bodyCount != BodyFilters.Length)
                {
                    SetCapacity(bodyCount);
                }
            }

            void SetCapacity(int bodyCount)
            {
                var nodeCount = bodyCount + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;

                if (Nodes.IsCreated)
                {
                    Nodes.Dispose();
                }
                Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(nodeCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory)
                {
                    // Always initialize first 2 nodes as empty, to gracefully return from queries on an empty tree
                    [0] = BoundingVolumeHierarchy.Node.Empty,
                    [1] = BoundingVolumeHierarchy.Node.Empty
                };

                if (NodeFilters.IsCreated)
                {
                    NodeFilters.Dispose();
                }
                NodeFilters = new NativeArray<CollisionFilter>(nodeCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory)
                {
                    // All queries should descend past these special root nodes
                    [0] = CollisionFilter.Default,
                    [1] = CollisionFilter.Default
                };

                if (BodyFilters.IsCreated)
                {
                    BodyFilters.Dispose();
                }
                BodyFilters = new NativeArray<CollisionFilter>(bodyCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            public Tree Clone()
            {
                return new Tree
                {
                    Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(Nodes, Allocator.Persistent),
                    NodeFilters = new NativeArray<CollisionFilter>(NodeFilters, Allocator.Persistent),
                    BodyFilters = new NativeArray<CollisionFilter>(BodyFilters, Allocator.Persistent),
                    Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(Ranges, Allocator.Persistent),
                    BranchCount = new NativeArray<int>(BranchCount, Allocator.Persistent)
                };
            }

            public void Dispose()
            {
                if (Nodes.IsCreated)
                {
                    Nodes.Dispose();
                }

                if (NodeFilters.IsCreated)
                {
                    NodeFilters.Dispose();
                }

                if (BodyFilters.IsCreated)
                {
                    BodyFilters.Dispose();
                }

                if (Ranges.IsCreated)
                {
                    Ranges.Dispose();
                }

                if (BranchCount.IsCreated)
                {
                    BranchCount.Dispose();
                }
            }
        }

        #endregion

        #region Tree Build

        internal JobHandle ScheduleBuildJobs(ref PhysicsWorld world, NativeArray<int> buildStaticTree, JobHandle inputDeps)
        {
            if (world.Settings.NumberOfThreadsHint <= 0)
            {
                return new BuildBroadphaseJob
                {
                    StaticBodies = world.StaticBodies,
                    DynamicBodies = world.DynamicBodies,
                    MotionDatas = world.BodyMotionData,
                    MotionVelocities = world.BodyMotionVelocity,
                    CollisionTolerance = PhysicsSettings.Constants.CollisionTolerance,
                    TimeStep = world.TimeStep,
                    Gravity = world.Settings.Gravity,
                    BuildStaticTree = buildStaticTree,
                    Broadphase = this

                }.Schedule(inputDeps);
            }

            return JobHandle.CombineDependencies(
                ScheduleStaticTreeBuildJobs(ref world, buildStaticTree, inputDeps),
                ScheduleDynamicTreeBuildJobs(ref world, inputDeps));        
        }

        /// <summary>
        /// Schedule a set of jobs to build the static tree of the broadphase based on the given world.
        /// </summary>
        JobHandle ScheduleStaticTreeBuildJobs(
            ref PhysicsWorld world,
            NativeArray<int> shouldDoWork,
            JobHandle inputDeps)
        {
            SafetyChecks.IsTrue(world.StaticBodyCount == m_StaticTree.BodyCount);
            
            if (world.StaticBodyCount == 0)
                return inputDeps;

            var aabbs = new NativeArray<Aabb>(world.StaticBodyCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(world.StaticBodyCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var staticBodyCountArray = new NativeArray<int>(1, Allocator.TempJob);
            var handle = new PrepareStaticBodyCountJob
            {
                StaticBodyCount = world.StaticBodyCount,
                BuildStaticTree = shouldDoWork,
                StaticBodyCountArray = staticBodyCountArray

            }.Schedule(inputDeps);

            var staticBodyDataJobHandle = new PrepareStaticBodyDataJob
            {
                PhysicsBodies = world.StaticBodies,
                Aabbs = aabbs,
                Points = points,
                FiltersOut = m_StaticTree.BodyFilters,
                AabbMargin = PhysicsSettings.Constants.CollisionTolerance * 0.5f, // each body contributes half

            }.ScheduleUnsafeIndex0(staticBodyCountArray, 32, handle);

            handle = JobHandle.CombineDependencies(staticBodyDataJobHandle, staticBodyCountArray.Dispose(handle));

            return m_StaticTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                points, aabbs, m_StaticTree.BodyFilters, shouldDoWork, world.Settings.NumberOfThreadsHint, handle,
                m_StaticTree.Nodes.Length, m_StaticTree.Ranges, m_StaticTree.BranchCount);
        }

        internal JobHandle ScheduleDynamicTreeBuildJobs(ref PhysicsWorld world, JobHandle inputDeps)
        {
            SafetyChecks.IsTrue(world.DynamicBodyCount == m_DynamicTree.BodyCount);
            if (world.DynamicBodyCount == 0)
            {
                return inputDeps;
            }

            var aabbs = new NativeArray<Aabb>(world.DynamicBodyCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(world.DynamicBodyCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var handle = new PrepareDynamicBodyDataJob
            {
                PhysicsBodies = world.DynamicBodies,
                MotionVelocities = world.BodyMotionVelocity,
                MotionDatas = world.BodyMotionData,
                Aabbs = aabbs,
                Points = points,
                FiltersOut = m_DynamicTree.BodyFilters,
                AabbMargin = PhysicsSettings.Constants.CollisionTolerance * 0.5f, // each body contributes half
                TimeStep = world.TimeStep,
                Gravity = world.Settings.Gravity

            }.Schedule(world.DynamicBodyCount, 32, inputDeps);

            var shouldDoWork = new NativeArray<int>(1, Allocator.TempJob);
            shouldDoWork[0] = 1;

            handle = m_DynamicTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                points, aabbs, m_DynamicTree.BodyFilters, shouldDoWork, world.Settings.NumberOfThreadsHint, handle,
                m_DynamicTree.Nodes.Length, m_DynamicTree.Ranges, m_DynamicTree.BranchCount);

            return shouldDoWork.Dispose(handle);
        }

        [BurstCompile]
        struct PrepareStaticBodyCountJob : IJob
        {
            public int StaticBodyCount;
            public NativeArray<int> BuildStaticTree;
            public NativeArray<int> StaticBodyCountArray;

            public void Execute()
            {
                if (BuildStaticTree[0] == 1)
                {
                    StaticBodyCountArray[0] = StaticBodyCount;
                }
                else
                {
                    StaticBodyCountArray[0] = 0;
                }
            }
        }

        // Reads broadphase data from static rigid bodies
        [BurstCompile]
        struct PrepareStaticBodyDataJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<PhysicsBody> PhysicsBodies;
            [ReadOnly] public float AabbMargin;

            public NativeArray<Aabb> Aabbs;
            public NativeArray<PointAndIndex> Points;
            public NativeArray<CollisionFilter> FiltersOut;

            public void Execute(int index)
            {
                Execute(index, AabbMargin, PhysicsBodies, Aabbs, Points, FiltersOut);
            }

            internal static void Execute(
                int index,
                float aabbMargin,
                NativeArray<PhysicsBody> physicsBodies,
                NativeArray<Aabb> aabbs,
                NativeArray<PointAndIndex> points,
                NativeArray<CollisionFilter> filtersOut)
            {
                var physicsBody = physicsBodies[index];

                Aabb aabb;
                if (physicsBody.Collider.IsCreated)
                {
                    aabb = physicsBody.Collider.Value.CalculateAabb(physicsBody.WorldTransform);
                    aabb.Inflate(aabbMargin);

                    filtersOut[index] = physicsBodies[index].Collider.Value.Filter;
                }
                else
                {
                    aabb.Min = aabb.Max = physicsBody.WorldTransform.Translation;

                    filtersOut[index] = CollisionFilter.Default;
                }

                aabbs[index] = aabb;
                points[index] = new BoundingVolumeHierarchy.PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }


        // Reads broadphase data from dynamic rigid bodies
        [BurstCompile]
        struct PrepareDynamicBodyDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<PhysicsBody> PhysicsBodies;
            [ReadOnly] public NativeArray<PhysicsBody.MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeArray<PhysicsBody.MotionData> MotionDatas;
            [ReadOnly] public float AabbMargin;
            [ReadOnly] public float2 Gravity;
            [ReadOnly] public float TimeStep;

            public NativeArray<PointAndIndex> Points;
            public NativeArray<Aabb> Aabbs;
            public NativeArray<CollisionFilter> FiltersOut;

            public void Execute(int index)
            {
                Execute(index, AabbMargin, Gravity, TimeStep, PhysicsBodies, MotionDatas, MotionVelocities, Aabbs, Points, FiltersOut);
            }

            internal static void Execute(int index, float aabbMargin, float2 gravity, float timeStep,
                NativeArray<PhysicsBody> physicsBodies, NativeArray<PhysicsBody.MotionData> bodyMotionData, NativeArray<PhysicsBody.MotionVelocity> bodyMotionVelocity,
                NativeArray<Aabb> aabbs, NativeArray<PointAndIndex> points, NativeArray<CollisionFilter> filtersOut)
            {
                var body = physicsBodies[index];

                Aabb aabb;
                if (body.Collider.IsCreated)
                {
                    var motionData = bodyMotionData[index];
                    var motionVelocity = bodyMotionVelocity[index];

                    // Apply gravity only on a copy to get proper expansion for the AABB,
                    // actual applying of gravity will be done later in the physics step
                    motionVelocity.LinearVelocity += gravity * timeStep * motionData.GravityScale;
                    var expansion = motionVelocity.CalculateExpansion(timeStep);

                    // Inflate the collider AABB by the body motion.
                    aabb = expansion.ExpandAabb(body.Collider.Value.CalculateAabb(body.WorldTransform));
                    aabb.Inflate(aabbMargin);

                    filtersOut[index] = body.Collider.Value.Filter;
                }
                else
                {
                    aabb.Min = aabb.Max = body.WorldTransform.Translation;

                    filtersOut[index] = CollisionFilter.Zero;
                }

                aabbs[index] = aabb;
                points[index] = new BoundingVolumeHierarchy.PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }

        // Builds the broadphase in a single job.
        [BurstCompile]
        struct BuildBroadphaseJob : IJob
        {
            [ReadOnly] public NativeArray<PhysicsBody> StaticBodies;
            [ReadOnly] public NativeArray<PhysicsBody> DynamicBodies;
            [ReadOnly] public NativeArray<PhysicsBody.MotionData> MotionDatas;
            [ReadOnly] public NativeArray<PhysicsBody.MotionVelocity> MotionVelocities;
            [ReadOnly] public float CollisionTolerance;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public float2 Gravity;
            [ReadOnly] public NativeArray<int> BuildStaticTree;

            public Broadphase Broadphase;

            public void Execute()
            {
                Broadphase.Build(StaticBodies, DynamicBodies, MotionDatas, MotionVelocities, CollisionTolerance, TimeStep, Gravity, BuildStaticTree[0] == 1);
            }
        }

        #endregion
    }
}
