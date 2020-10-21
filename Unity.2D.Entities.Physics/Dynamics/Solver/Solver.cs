using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public static class Solver
    {
        // Schedule the job to apply gravity to all dynamic bodies and copy input velocities
        internal static JobHandle ScheduleApplyGravityAndCopyInputVelocitiesJob(
            ref DynamicsWorld world,
            NativeArray<PhysicsVelocity> inputVelocities,
            float2 gravityAcceleration,
            JobHandle inputDeps, int threadCountHint = 0)
        {
            if (threadCountHint <= 0)
            {
                var job = new ApplyGravityAndCopyInputVelocitiesJob
                {
                    BodyMotionData = world.BodyMotionData,
                    BodyMotionVelocity = world.BodyMotionVelocity,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration
                };

                return job.Schedule(inputDeps);
            }
            else
            {
                var job = new ParallelApplyGravityAndCopyInputVelocitiesJob
                {
                    BodyMotionData = world.BodyMotionData,
                    BodyMotionVelocity = world.BodyMotionVelocity,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration
                };

                return job.Schedule(world.BodyMotionCount, 64, inputDeps);
            }
        }
        
        // Apply gravity to all dynamic bodies and copy input velocities
        internal static void ApplyGravityAndCopyInputVelocities(
            NativeArray<PhysicsBody.MotionData> bodyMotionData,
            NativeArray<PhysicsBody.MotionVelocity> bodyMotionVelocity,
            NativeArray<PhysicsVelocity> inputVelocities,
            float2 gravity)
        {
            for (var i = 0; i < bodyMotionData.Length; i++)
            {
                ParallelApplyGravityAndCopyInputVelocitiesJob.Execute(i, gravity, bodyMotionData, bodyMotionVelocity, inputVelocities);
            }
        }
        
        #region Jobs
        
        [BurstCompile]
        struct ParallelApplyGravityAndCopyInputVelocitiesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<PhysicsBody.MotionData> BodyMotionData;
            public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;
            public NativeArray<PhysicsVelocity> InputVelocities;
            public float2 GravityAcceleration;

            public void Execute(int index)
            {
                Execute(index, GravityAcceleration, BodyMotionData, BodyMotionVelocity, InputVelocities);
            }

            internal static void Execute(
                int index,
                float2 gravityAcceleration,
                NativeArray<PhysicsBody.MotionData> bodyMotionData,
                NativeArray<PhysicsBody.MotionVelocity> bodyMotionVelocity,
                NativeArray<PhysicsVelocity> inputVelocities)
            {
                var motionData = bodyMotionData[index];
                var motionVelocity = bodyMotionVelocity[index];

                // Apply gravity
                motionVelocity.LinearVelocity += gravityAcceleration * motionData.GravityScale;

                // Write back
                bodyMotionVelocity[index] = motionVelocity;

                // Make a copy
                inputVelocities[index] = new PhysicsVelocity
                {
                    Linear = motionVelocity.LinearVelocity,
                    Angular = motionVelocity.AngularVelocity
                };
            }
        }
        
        [BurstCompile]
        struct ApplyGravityAndCopyInputVelocitiesJob : IJob
        {
            [ReadOnly] public NativeArray<PhysicsBody.MotionData> BodyMotionData;
            public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;
            public NativeArray<PhysicsVelocity> InputVelocities;
            public float2 GravityAcceleration;

            public void Execute()
            {
                ApplyGravityAndCopyInputVelocities(BodyMotionData, BodyMotionVelocity, InputVelocities, GravityAcceleration);
            }
        }        

        #endregion
    }
}
