using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace Unity.U2D.Entities.Physics
{
    // Default simulation implementation
    public class DefaultSimulation : ISimulation
    {
        public SimulationType Type => SimulationType.Default;
        public JobHandle FinalSimulationJobHandle => m_StepHandles.FinalExecutionHandle;
        public JobHandle FinalJobHandle => JobHandle.CombineDependencies(FinalSimulationJobHandle, m_StepHandles.FinalDisposeHandle);

        internal SimulationContext SimulationContext;

        SimulationJobHandles m_StepHandles = new SimulationJobHandles(new JobHandle());

        // Schedule all the jobs for the simulation step.
        public SimulationJobHandles ScheduleStepJobs(PhysicsWorld physicsWorld, PhysicsCallbacks physicsCallbacks, JobHandle inputDeps)
        {
            var physicsSettings = physicsWorld.Settings;
            
            SafetyChecks.IsFalse(physicsWorld.TimeStep < 0f);

            SimulationContext.Reset(ref physicsWorld, false);
            SimulationContext.TimeStep = physicsWorld.TimeStep;

            if (physicsWorld.DynamicBodyCount == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(inputDeps);
                return m_StepHandles;
            }
            
            // Execute phase callback.
            var handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PreStepSimulation, ref physicsWorld, inputDeps);

            // Apply gravity and copy input velocities at this point (in parallel with the scheduler, but before the callbacks)
            handle = Solver.ScheduleApplyGravityAndCopyInputVelocitiesJob(
                ref physicsWorld.DynamicsWorld, SimulationContext.InputVelocities, physicsWorld.TimeStep * physicsSettings.Gravity, handle, physicsSettings.NumberOfThreadsHint);

            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateOverlapBodies, ref physicsWorld, handle);
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateContacts, ref physicsWorld, handle);
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateConstraints, ref physicsWorld, handle);
            
            // Integrate motions.
            handle = Integrator.ScheduleIntegrateJobs(ref physicsWorld, handle, physicsSettings.NumberOfThreadsHint);

            // Schedule phase callback.
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostIntegrate, ref physicsWorld, handle);
            
            m_StepHandles.FinalExecutionHandle = handle;
            m_StepHandles.FinalDisposeHandle = handle;
            
            return m_StepHandles;
        }
        
        public void Dispose()
        {
            SimulationContext.Dispose();
        }        
    }
    
    // Holds temporary data in a storage that lives as long as simulation lives
    // and is only re-allocated if necessary.
    public struct SimulationContext : IDisposable
    {
        int m_InputVelocityCount;
        NativeArray<PhysicsVelocity> m_InputVelocities;
        
        internal NativeArray<PhysicsVelocity> InputVelocities => m_InputVelocities.GetSubArray(0, m_InputVelocityCount);        

        internal float TimeStep;
        
        internal void Reset(ref PhysicsWorld world, bool allocateEventDataStreams)
        {
            m_InputVelocityCount = world.DynamicBodyCount;
            if (!m_InputVelocities.IsCreated || m_InputVelocities.Length < m_InputVelocityCount)
            {
                if (m_InputVelocities.IsCreated)
                {
                    m_InputVelocities.Dispose();
                }
                m_InputVelocities = new NativeArray<PhysicsVelocity>(m_InputVelocityCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }
        }

        public void Dispose()
        {
            if (m_InputVelocities.IsCreated)
            {
                m_InputVelocities.Dispose();
            }
        }
    }
}
