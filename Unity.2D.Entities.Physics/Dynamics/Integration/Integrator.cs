using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    static class Integrator
    {
        internal static JobHandle ScheduleIntegrateJobs(ref PhysicsWorld physicsWorld, JobHandle inputDeps, int threadCountHint = 0)
        {
            var job = new ParallelIntegrateMotionsJob
            {
                BodyMotionData = physicsWorld.BodyMotionData,
                BodyMotionVelocity = physicsWorld.BodyMotionVelocity,
                TimeStep = physicsWorld.TimeStep,
            };
            return job.Schedule(physicsWorld.DynamicsWorld.BodyMotionCount, 64, inputDeps);
        }

        [BurstCompile]
        struct ParallelIntegrateMotionsJob : IJobParallelFor
        {
            public NativeArray<PhysicsBody.MotionData> BodyMotionData;
            public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;
            public float TimeStep;

            public void Execute(int index)
            {
                Execute(index, BodyMotionData, BodyMotionVelocity, TimeStep);
            }
            
            public static void Execute(
                int index,
                NativeArray<PhysicsBody.MotionData> bodyMotionData,
                NativeArray<PhysicsBody.MotionVelocity> bodyMotionVelocity,
                float timestep
                )
            {
                var motionData = bodyMotionData[index];
                var motionVelocity = bodyMotionVelocity[index];

                // Orientation.
                motionData.WorldPosition += motionVelocity.LinearVelocity * timestep;
                motionData.WorldAngle += motionVelocity.AngularVelocity * timestep;
                
                // Damping.
                motionVelocity.LinearVelocity *= math.clamp(1.0f - motionData.LinearDamping * timestep, 0.0f, 1.0f);
                motionVelocity.AngularVelocity *= math.clamp(1.0f - motionData.AngularDamping * timestep, 0.0f, 1.0f);

                bodyMotionData[index] = motionData;
                bodyMotionVelocity[index] = motionVelocity;
            }
        }
      
    }
}
