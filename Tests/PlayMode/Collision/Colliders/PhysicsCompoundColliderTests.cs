using Unity.Collections;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class CompoundColliderTests
    {
        [Test]
        public void MassProperties_BuiltFromChildren_MatchesExpected()
        {
            void TestCompoundBox(PhysicsTransform transform)
            {
                // Create a unit box
                var boxCollider = PhysicsBoxCollider.Create(new BoxGeometry
                {
                    Size = new float2(1f),
                    Center = transform.Translation,
                    Angle = 0f,
                    BevelRadius = 0.0f
                });

                // Create a compound of mini boxes, matching the volume of the single box
                var miniBox = PhysicsBoxCollider.Create(new BoxGeometry
                {
                    Size = new float2(0.5f),
                    Center = float2.zero,
                    Angle = 0f,
                    BevelRadius = 0.0f
                });

                const uint UserData = 0xDEADBEEF;
                const int ChildrenCount = 4;

                var childrenTransforms = new NativeArray<PhysicsTransform>(ChildrenCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory)
                {
                    [0] = new PhysicsTransform(new float2(-0.25f, -0.25f), float2x2.identity),
                    [1] = new PhysicsTransform(new float2(0.25f, -0.25f), float2x2.identity),
                    [2] = new PhysicsTransform(new float2(0.25f, 0.25f), float2x2.identity),
                    [3] = new PhysicsTransform(new float2(-0.25f, 0.25f), float2x2.identity),
                };

                var children = new NativeArray<PhysicsCompoundCollider.ColliderBlobInstance>(ChildrenCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                for(var i = 0; i < ChildrenCount; ++i)
                {
                    children[i] = new PhysicsCompoundCollider.ColliderBlobInstance
                    {
                        Collider = miniBox,
                        CompoundFromChild = PhysicsMath.mul(transform, childrenTransforms[i])
                    };
                }

                var colliderBlob = PhysicsCompoundCollider.Create(children, UserData);
                childrenTransforms.Dispose();
                children.Dispose();

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCompoundCollider>();
                Assert.AreEqual(ColliderType.Compound, collider.ColliderType);
                Assert.AreEqual(CollisionType.Composite, collider.CollisionType);
                Assert.AreEqual(UserData, collider.UserData);

                var boxMassProperties = boxCollider.Value.MassProperties;
                var compoundMassProperties = colliderBlob.Value.MassProperties;
                
                Assert.AreEqual(boxMassProperties.Area, compoundMassProperties.Area, 1e-3f, "Area incorrect.");
                Assert.AreEqual(boxMassProperties.AngularExpansionFactor, compoundMassProperties.AngularExpansionFactor, 1e-3f, "AngularExpansionFactor incorrect.");
                PhysicsAssert.AreEqual(boxMassProperties.MassDistribution.LocalCenterOfMass, compoundMassProperties.MassDistribution.LocalCenterOfMass, 1e-3f, "LocalCenterOfMass incorrect.");
                Assert.AreEqual(boxMassProperties.MassDistribution.InverseInertia, compoundMassProperties.MassDistribution.InverseInertia, 1e-3f, "InverseInertia incorrect.");

                boxCollider.Dispose();
                colliderBlob.Dispose();
            }

            // Compare box with compound at various transforms.
            TestCompoundBox(PhysicsTransform.Identity);
            TestCompoundBox(new PhysicsTransform(new float2(2.0f, 3.0f), float2x2.identity));
            TestCompoundBox(new PhysicsTransform(float2.zero, float2x2.Rotate(0.5f)));
            TestCompoundBox(new PhysicsTransform(new float2(3.0f, 4.0f), float2x2.Rotate(2.5f)));
        }
    }
}
