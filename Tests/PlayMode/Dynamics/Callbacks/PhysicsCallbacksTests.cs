using Unity.Jobs;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsCallbacksTests
    {
        [Test]
        public void SanityCheckOnPhaseCount()
        {
            Assert.AreEqual(7, PhysicsCallbacks.k_PhaseCount, "Invalid number of PhysicsCallback phases.");
        }

        struct PhaseCounter
        {
            public int PreBuild;
            public int PreStepSimulation;
            public int PostCreateOverlapBodies;
            public int PostCreateContacts;
            public int PostCreateConstraints;
            public int PostIntegrate;
            public int PostExport;
        };

        [Test]
        public void ScheduleCallbackPhases()
        {
            PhaseCounter phaseCount = default;

            Assert.AreEqual(0, phaseCount.PreBuild);
            Assert.AreEqual(0, phaseCount.PreStepSimulation);
            Assert.AreEqual(0, phaseCount.PostCreateOverlapBodies);
            Assert.AreEqual(0, phaseCount.PostCreateContacts);
            Assert.AreEqual(0, phaseCount.PostCreateConstraints);
            Assert.AreEqual(0, phaseCount.PostIntegrate);
            Assert.AreEqual(0, phaseCount.PostExport);

            JobHandle preBuildCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PreBuild++;
                return dependencyHandle;
            }

            JobHandle preStepSimulationCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PreStepSimulation++;
                return dependencyHandle;
            }
            
            JobHandle postCreateOverlapBodiesCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PostCreateOverlapBodies++;
                return dependencyHandle;
            }
            
            JobHandle postCreateContactsCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PostCreateContacts++;
                return dependencyHandle;
            }

            JobHandle postCreateConstraintsCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PostCreateConstraints++;
                return dependencyHandle;
            }

            JobHandle postIntegrateCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PostIntegrate++;
                return dependencyHandle;
            }
            
            JobHandle postExportCallback(ref PhysicsWorld physicsWorld, JobHandle dependencyHandle)
            {
                phaseCount.PostExport++;
                return dependencyHandle;
            }

            {
                var handle = new JobHandle();
                var physicsWorld = new PhysicsWorld();

                var callbacks = new  PhysicsCallbacks();

                // Enqueue each phase once.
                callbacks.Enqueue(PhysicsCallbacks.Phase.PreBuild, preBuildCallback, handle);
                callbacks.Enqueue(PhysicsCallbacks.Phase.PreStepSimulation, preStepSimulationCallback, handle);
                callbacks.Enqueue(PhysicsCallbacks.Phase.PostCreateOverlapBodies, postCreateOverlapBodiesCallback, handle);
                callbacks.Enqueue(PhysicsCallbacks.Phase.PostCreateContacts, postCreateContactsCallback, handle);
                callbacks.Enqueue(PhysicsCallbacks.Phase.PostCreateConstraints, postCreateConstraintsCallback, handle);
                callbacks.Enqueue(PhysicsCallbacks.Phase.PostIntegrate, postIntegrateCallback, handle);
                callbacks.Enqueue(PhysicsCallbacks.Phase.PostExport, postExportCallback, handle);

                PerformCallbacks(ref callbacks, ref physicsWorld);

                // Should all be called once.
                Assert.AreEqual(1, phaseCount.PreBuild);
                Assert.AreEqual(1, phaseCount.PreStepSimulation);
                Assert.AreEqual(1, phaseCount.PostCreateOverlapBodies);
                Assert.AreEqual(1, phaseCount.PostCreateContacts);
                Assert.AreEqual(1, phaseCount.PostCreateConstraints);
                Assert.AreEqual(1, phaseCount.PostIntegrate);
                Assert.AreEqual(1, phaseCount.PostExport);

                callbacks.Clear();
                phaseCount = default;
                PerformCallbacks(ref callbacks, ref physicsWorld);

                // No callbacks should happen.
                Assert.AreEqual(0, phaseCount.PreBuild);
                Assert.AreEqual(0, phaseCount.PreStepSimulation);
                Assert.AreEqual(0, phaseCount.PostCreateOverlapBodies);
                Assert.AreEqual(0, phaseCount.PostCreateContacts);
                Assert.AreEqual(0, phaseCount.PostCreateConstraints);
                Assert.AreEqual(0, phaseCount.PostIntegrate);
                Assert.AreEqual(0, phaseCount.PostExport);

				for(var n = 0; n < 2; ++n)
					callbacks.Enqueue(PhysicsCallbacks.Phase.PreBuild, preBuildCallback, handle);

				for(var n = 0; n < 3; ++n)
                	callbacks.Enqueue(PhysicsCallbacks.Phase.PreStepSimulation, preStepSimulationCallback, handle);
				
				for(var n = 0; n < 4; ++n)
                	callbacks.Enqueue(PhysicsCallbacks.Phase.PostCreateOverlapBodies, postCreateOverlapBodiesCallback, handle);

				for(var n = 0; n < 5; ++n)
                	callbacks.Enqueue(PhysicsCallbacks.Phase.PostCreateContacts, postCreateContactsCallback, handle);

				for(var n = 0; n < 6; ++n)
                	callbacks.Enqueue(PhysicsCallbacks.Phase.PostCreateConstraints, postCreateConstraintsCallback, handle);

				for(var n = 0; n < 7; ++n)
                	callbacks.Enqueue(PhysicsCallbacks.Phase.PostIntegrate, postIntegrateCallback, handle);

				for(var n = 0; n < 8; ++n)
                	callbacks.Enqueue(PhysicsCallbacks.Phase.PostExport, postExportCallback, handle);

                PerformCallbacks(ref callbacks, ref physicsWorld);

                // Should all be called multiple times.
                Assert.AreEqual(2, phaseCount.PreBuild);
                Assert.AreEqual(3, phaseCount.PreStepSimulation);
                Assert.AreEqual(4, phaseCount.PostCreateOverlapBodies);
                Assert.AreEqual(5, phaseCount.PostCreateContacts);
                Assert.AreEqual(6, phaseCount.PostCreateConstraints);
                Assert.AreEqual(7, phaseCount.PostIntegrate);
                Assert.AreEqual(8, phaseCount.PostExport);
            }
        }

        #region Utility

        static void PerformCallbacks(ref PhysicsCallbacks callbacks, ref PhysicsWorld physicsWorld)
        {
            var handle = new JobHandle();

            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PreBuild, ref physicsWorld, handle);
            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PreStepSimulation, ref physicsWorld, handle);
            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateOverlapBodies, ref physicsWorld, handle);
            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateContacts, ref physicsWorld, handle);
            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateConstraints, ref physicsWorld, handle);
            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostIntegrate, ref physicsWorld, handle);
            callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostExport, ref physicsWorld, handle);
        }

        #endregion
    }
}
