using System;
using Unity.Jobs;

namespace Unity.U2D.Entities.Physics
{
    // Implementations of ISimulation
    public enum SimulationType
    {
        // A dummy physics implementation that performs no simulation.
        None,
        
        // The standard stateless default physics implementation.
        Default
    }
    
    // Result of ISimulation.ScheduleStepJobs()
    public struct SimulationJobHandles
    {
        public JobHandle FinalExecutionHandle;
        public JobHandle FinalDisposeHandle;

        public SimulationJobHandles(JobHandle handle)
        {
            FinalExecutionHandle = handle;
            FinalDisposeHandle = handle;
        }
    }

    // Defines a simulation.
    public interface ISimulation : IDisposable
    {
        // The implementation type.
        SimulationType Type { get; }

        // Schedule a set of jobs to step the simulation.
        SimulationJobHandles ScheduleStepJobs(PhysicsWorld physicsWorld, PhysicsCallbacks callbacks, JobHandle inputDeps);

        // The final scheduled simulation job.
        // Jobs which use the simulation results should depend on this.
        JobHandle FinalSimulationJobHandle { get; }

        // The final scheduled job, including all simulation and cleanup.
        // The end of each step should depend on this.
        JobHandle FinalJobHandle { get; }
    }
}
