using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

using UnityEngine;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateAfter(typeof(PhysicsDebugStreamSystem))]
    [UpdateBefore(typeof(PhysicsWorldSystem))]
    internal partial class DisplayColliderAabbSystem : SystemBase
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
            if (m_PhysicsWorldSystem.PhysicsWorld.BodyCount == 0)
                return;

            var debugDisplay = GetSingleton<PhysicsDebugDisplay>();
            if (debugDisplay.DrawColliderAabbs == 0)
                return;

            JobHandle callback(ref PhysicsWorld world, JobHandle deps)
            {
                return new DisplayColliderAabbJob
                {
                    OutputStream = m_DebugStreamSystem.GetContext(1),
                    DebugDisplay = debugDisplay,
                    PhysicsBodies = m_PhysicsWorldSystem.PhysicsWorld.AllBodies
                    
                }.Schedule(deps);
            }

            m_PhysicsWorldSystem.ScheduleCallback(PhysicsCallbacks.Phase.PreStepSimulation, callback);
        }
    }

    // Job to iterate over all the bodies in a scene, for any
    // which have a collider, calculate the bounding box and
    // write it to a debug stream.
    [BurstCompile]
    internal unsafe struct DisplayColliderAabbJob : IJob
    {
        public PhysicsDebugStreamSystem.Context OutputStream;
        public PhysicsDebugDisplay DebugDisplay;

        [ReadOnly] public NativeSlice<PhysicsBody> PhysicsBodies;

        public void Execute()
        {
            OutputStream.Begin(0);

            Color colliderAabbColor = (Vector4)DebugDisplay.ColliderAabbColor;

            for (var i = 0; i < PhysicsBodies.Length; ++i)
            {
                var physicsBody = PhysicsBodies[i];
                var collider = physicsBody.Collider;
                if (collider.IsCreated)
                {
                    var aabb = collider.Value.CalculateAabb(physicsBody.WorldTransform);
                    OutputStream.Box(aabb.Center, aabb.Extents, colliderAabbColor);
                }
            }
            OutputStream.End();
        }
    }
}
