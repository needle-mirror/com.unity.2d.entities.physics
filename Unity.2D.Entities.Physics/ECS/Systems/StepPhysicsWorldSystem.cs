using Unity.Entities;
using Unity.Jobs;

namespace Unity.U2D.Entities.Physics
{
    [UpdateAfter(typeof(PhysicsWorldSystem)), UpdateBefore(typeof(ExportPhysicsWorldSystem))]
    public partial class StepPhysicsWorldSystem : SystemBase
    {
        // The simulation implementation
        public ISimulation Simulation { get; private set; }

        // The final job handle produced by this system.
        // This includes all simulation jobs as well as array disposal jobs.
        public JobHandle FinalJobHandle => Simulation.FinalJobHandle;

        // The final simulation job handle produced by this system.
        // Systems which read the simulation results should depend on this.
        public JobHandle FinalSimulationJobHandle => Simulation.FinalSimulationJobHandle;
        
        delegate ISimulation SimulationCreator();
        const int k_SimulationTypeCount = 2;
        readonly SimulationCreator[] m_SimulationCreators = new SimulationCreator[k_SimulationTypeCount];

        SimulationType m_SimulationType; 
        SimulationType SimulationType
        {
            get => m_SimulationType;
            set
            {
                // Change the simulation implementation if the physics world simulation type changed.
                if (value != m_SimulationType)
                {
                    m_SimulationType = value;
                    Simulation.Dispose();
                    Simulation = m_SimulationCreators[(int)m_SimulationType]();
                }
            }
        }

        PhysicsWorldSystem m_PhysicsWorldSystem;
        
        protected override void OnCreate()
        {
            base.OnCreate();

#if !NET_DOTS
            SafetyChecks.IsTrue(k_SimulationTypeCount == System.Enum.GetValues(typeof(SimulationType)).Length);
            SafetyChecks.IsTrue(k_SimulationTypeCount == m_SimulationCreators.Length);
#endif
            m_SimulationCreators[(int)SimulationType.None] = () => new NoSimulation();
            m_SimulationCreators[(int)SimulationType.Default] = () => new DefaultSimulation();
            
            // Set the default simulation type.
            Simulation = new DefaultSimulation();
            m_SimulationType = SimulationType.Default;
            
            m_PhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
        }

        protected override void OnDestroy()
        {
            Simulation?.Dispose();

            base.OnDestroy();
        }
        
        protected override void OnUpdate()
        {
            ref var physicsWorld = ref m_PhysicsWorldSystem.PhysicsWorld;
            var physicsCallbacks = m_PhysicsWorldSystem.Callbacks;
            
            // Set the simulation type.
            SimulationType = physicsWorld.Settings.SimulationType;

            // Yes, so set up the dependencies.
            var handle = JobHandle.CombineDependencies(m_PhysicsWorldSystem.FinalJobHandle, Dependency);
            
            // Schedule the simulation jobs
            Simulation.ScheduleStepJobs(
                physicsWorld,
                physicsCallbacks,
                handle);
            
            Dependency = JobHandle.CombineDependencies(FinalSimulationJobHandle, handle);
        }
    }
}
