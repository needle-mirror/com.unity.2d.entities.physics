using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsWorldTests
    {
        [Test]
        public void WorldTest()
        {
            {
                var world = new PhysicsWorld(staticBodyCount: 4, dynamicBodyCount: 10, jointCount: 23);

                Assert.AreEqual(4, world.StaticBodyCount);
                Assert.AreEqual(10, world.DynamicBodyCount);
                Assert.AreEqual(14, world.BodyCount);
                Assert.AreEqual(23, world.JointCount);

                world.Reset(0, 0, 0);
                Assert.AreEqual(0, world.StaticBodyCount);
                Assert.AreEqual(0, world.DynamicBodyCount);
                Assert.AreEqual(0, world.BodyCount);
                Assert.AreEqual(0, world.JointCount);

                world.Dispose();
            }
       
            {
                // World clone.
                var world = new PhysicsWorld(staticBodyCount: 40, dynamicBodyCount: 100, jointCount: 200);
                Assert.AreEqual(40, world.StaticBodyCount);
                Assert.AreEqual(100, world.DynamicBodyCount);
                Assert.AreEqual(140, world.BodyCount);
                Assert.AreEqual(200, world.JointCount);
                
                var worldClone = world.Clone();
                Assert.AreEqual(40, worldClone.StaticBodyCount);
                Assert.AreEqual(100, worldClone.DynamicBodyCount);
                Assert.AreEqual(140, worldClone.BodyCount);
                Assert.AreEqual(200, worldClone.JointCount);

                worldClone.Dispose();
                world.Dispose();
            }
        }
    }
}
