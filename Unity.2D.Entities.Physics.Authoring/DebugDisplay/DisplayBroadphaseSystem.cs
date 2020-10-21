using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

using UnityEngine;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateAfter(typeof(PhysicsDebugStreamSystem))]
    [UpdateBefore(typeof(PhysicsWorldSystem))]
    internal class DisplayBroadphaseSystem : SystemBase
    {
        PhysicsWorldSystem m_PhysicsWorldSystem;
        PhysicsDebugStreamSystem m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_PhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<PhysicsDebugStreamSystem>();

            RequireSingletonForUpdate<PhysicsDebugDisplay>();
        }

        protected override void OnUpdate()
        {
            var debugDisplay = GetSingleton<PhysicsDebugDisplay>();
            if (debugDisplay.DrawBroadphase == 0)
                return;

            JobHandle callback(ref PhysicsWorld world, JobHandle deps)
            {
                return new DisplayBroadphaseJob
                {
                    OutputStream = m_DebugStreamSystem.GetContext(1),
                    DebugDisplay = debugDisplay,
                    StaticNodes = world.CollisionWorld.Broadphase.StaticTree.Nodes,
                    DynamicNodes = world.CollisionWorld.Broadphase.DynamicTree.Nodes,

                }.Schedule(deps);
            }

            m_PhysicsWorldSystem.ScheduleCallback(PhysicsCallbacks.Phase.PreStepSimulation, callback);
        }
    }

    // Job which walks the broadphase tree and writes the
    // bounding box of leaf nodes to a PhysicsDebugStreamSystem.
    [BurstCompile]
    internal struct DisplayBroadphaseJob : IJob
    {
        public PhysicsDebugStreamSystem.Context OutputStream;
        public PhysicsDebugDisplay DebugDisplay;

        [ReadOnly]
        public NativeArray<BoundingVolumeHierarchy.Node> StaticNodes;

        [ReadOnly]
        public NativeArray<BoundingVolumeHierarchy.Node> DynamicNodes;

        public void DrawLeavesRecursive(NativeArray<BoundingVolumeHierarchy.Node> nodes, UnityEngine.Color color, int nodeIndex)
        {
            if (nodes[nodeIndex].IsLeaf)
            {
                bool4 leavesValid = nodes[nodeIndex].AreLeavesValid;
                for (int l = 0; l < 4; l++)
                {
                    if (leavesValid[l])
                    {
                        var aabb = nodes[nodeIndex].Bounds.GetAabb(l);
                        OutputStream.Box(aabb.Center, aabb.Extents, color);
                    }
                }

                return;
            }

            for (int i = 0; i < 4; i++)
            {
                if (nodes[nodeIndex].IsChildValid(i))
                {
                    DrawLeavesRecursive(nodes, color, nodes[nodeIndex].Data[i]);
                }
            }
        }

        public void Execute()
        {
            UnityEngine.Color staticColor = (Vector4)DebugDisplay.StaticBroadphaseColor;
            UnityEngine.Color dynamicColor = (Vector4)DebugDisplay.DynamicBroadphaseColor;

            OutputStream.Begin(0);
            DrawLeavesRecursive(StaticNodes, staticColor, 1);
            DrawLeavesRecursive(DynamicNodes, dynamicColor, 1);
            OutputStream.End();
        }
    }
}