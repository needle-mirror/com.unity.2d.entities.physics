using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.U2D.Entities.Physics
{
    [UpdateAfter(typeof(StepPhysicsWorldSystem)), UpdateBefore(typeof(EndFramePhysicsSystem)), UpdateBefore(typeof(TransformSystemGroup))]
    public partial class ExportPhysicsWorldSystem : SystemBase
    {
        public JobHandle FinalJobHandle { get; private set; }

        PhysicsWorldSystem m_PhysicsWorldSystem;
        StepPhysicsWorldSystem m_StepPhysicsWorldSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            
            FinalJobHandle = default;

            m_PhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorldSystem>();
        }

        protected override void OnUpdate()
        {
            var handle = JobHandle.CombineDependencies(m_StepPhysicsWorldSystem.FinalSimulationJobHandle, Dependency);

            ref var world = ref m_PhysicsWorldSystem.PhysicsWorld;

            var translationType = GetComponentTypeHandle<Translation>();
            var rotationType = GetComponentTypeHandle<Rotation>();
            var velocityType = GetComponentTypeHandle<PhysicsVelocity>();

            handle = new ExportDynamicBodiesJob
            {
                BodyMotionData = world.DynamicsWorld.BodyMotionData,
                BodyMotionVelocity = world.DynamicsWorld.BodyMotionVelocity,

                TranslationType = translationType,
                RotationType = rotationType,
                VelocityType = velocityType

            }.Schedule(m_PhysicsWorldSystem.DynamicEntityGroup, handle);
            
            // Schedule phase callback.
            Dependency = FinalJobHandle = m_PhysicsWorldSystem.Callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostExport, ref m_PhysicsWorldSystem.PhysicsWorld, handle);
        }

        [BurstCompile]
        struct ExportDynamicBodiesJob : IJobChunk
        {
            [ReadOnly] public NativeArray<PhysicsBody.MotionData> BodyMotionData;
            [ReadOnly] public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;

            public ComponentTypeHandle<Translation> TranslationType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                var chunkTranslations = chunk.GetNativeArray(TranslationType);
                var chunkRotations = chunk.GetNativeArray(RotationType);
                var chunkVelocities = chunk.GetNativeArray(VelocityType);

                for(int i = 0, bodyMotionIndex = firstEntityIndex; i < chunk.Count; ++i, ++bodyMotionIndex)
                {
                    var motionData = BodyMotionData[bodyMotionIndex];
                    var motionVelocity = BodyMotionVelocity[bodyMotionIndex];

                    var translation = motionData.WorldPosition - PhysicsMath.mul(float2x2.Rotate(motionData.WorldAngle), motionData.LocalCenterOfMass);
                    var rotation = PhysicsMath.QuaternionFromZRotation(motionData.WorldAngle);

                    chunkTranslations[i] = new Translation { Value = new float3(translation, 0.0f) };
                    chunkRotations[i] = new Rotation { Value = rotation };
                    chunkVelocities[i] = new PhysicsVelocity
                    {
                        Linear = motionVelocity.LinearVelocity,
                        Angular = motionVelocity.AngularVelocity
                    };
                }
            }
        }
    }
}
