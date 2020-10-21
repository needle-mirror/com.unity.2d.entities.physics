using System.Collections.Generic;
using Unity.Jobs;

namespace Unity.U2D.Entities.Physics
{
    public class PhysicsCallbacks
    {
        // This should match the number of members in the "Phase" enum below!
        internal const int k_PhaseCount = 7;

        public enum Phase
        {
            PreBuild = 0,
            PreStepSimulation,
            PostCreateOverlapBodies,
            PostCreateContacts,
            PostCreateConstraints,
            PostIntegrate,
            PostExport
        }

        public delegate JobHandle Callback(ref PhysicsWorld world, JobHandle inputDeps);

        struct CallbackAndDependency
        {
            public Callback Callback;
            public JobHandle Dependency;
        }

        readonly List<CallbackAndDependency>[] m_Callbacks = new List<CallbackAndDependency>[k_PhaseCount];

        internal PhysicsCallbacks()
        {
            for (var i = 0; i < k_PhaseCount; ++i)
            {
                m_Callbacks[i] = new List<CallbackAndDependency>(8);
            }
        }

        internal void Enqueue(Phase phase, Callback callback, JobHandle dependency)
        {
            m_Callbacks[(int)phase].Add(new CallbackAndDependency { Callback = callback, Dependency = dependency });
        }

        internal JobHandle ScheduleCallbacksForPhase(Phase phase, ref PhysicsWorld physicsWorld, JobHandle inputDeps)
        {
            ref var callbacks = ref m_Callbacks[(int)phase];
            var callbackCount = callbacks.Count;
            if (callbackCount > 0)
            {
                for (var i = 0; i < callbackCount; ++i)
                {
                    var callback = callbacks[i];
                    inputDeps = callback.Callback(ref physicsWorld, JobHandle.CombineDependencies(inputDeps, callback.Dependency));
                }
            }

            return inputDeps;
        }

        internal void Clear()
        {
            for (var i = 0; i < k_PhaseCount; i++)
            {
                m_Callbacks[i].Clear();
            }
        }
    }
}