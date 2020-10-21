using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

namespace Unity.U2D.Entities.Physics
{
    [UpdateAfter(typeof(PhysicsWorldSystem))]
    [UpdateAfter(typeof(StepPhysicsWorldSystem))]
    [UpdateAfter(typeof(ExportPhysicsWorldSystem))]
    public class EndFramePhysicsSystem : SystemBase
    {
        public NativeList<JobHandle> HandlesToWaitFor;

        // A combined handle of all built-in and user physics jobs
        public JobHandle FinalJobHandle { get; private set; }

        PhysicsWorldSystem m_PhysicsWorldSystem;
        StepPhysicsWorldSystem m_SimulateWorldSystem;
        ExportPhysicsWorldSystem m_ExportPhysicsWorld;

        protected override void OnCreate()
        {
            base.OnCreate();
            
            HandlesToWaitFor = new NativeList<JobHandle>(16, Allocator.Persistent);
            FinalJobHandle = default;

            m_PhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            m_SimulateWorldSystem = World.GetOrCreateSystem<StepPhysicsWorldSystem>();
            m_ExportPhysicsWorld = World.GetOrCreateSystem<ExportPhysicsWorldSystem>();
        }

        protected override void OnDestroy()
        {
            CombineDependencies().Complete();
            HandlesToWaitFor.Dispose();
            
            base.OnDestroy();
        }

        protected override void OnUpdate()
        {
            m_PhysicsWorldSystem.Callbacks.Clear();
            
            Dependency = FinalJobHandle = JobHandle.CombineDependencies(CombineDependencies(), Dependency);
        }

        JobHandle CombineDependencies()
        {
            // Add built-in jobs
            HandlesToWaitFor.Add(m_PhysicsWorldSystem.FinalJobHandle);
            HandlesToWaitFor.Add(m_SimulateWorldSystem.FinalJobHandle);
            HandlesToWaitFor.Add(m_ExportPhysicsWorld.FinalJobHandle);
            var handle = JobHandle.CombineDependencies(HandlesToWaitFor);
            HandlesToWaitFor.Clear();
            return handle;
        }
    }
}
