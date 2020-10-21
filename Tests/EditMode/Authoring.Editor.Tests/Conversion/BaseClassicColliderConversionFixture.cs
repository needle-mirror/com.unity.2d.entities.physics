using NUnit.Framework;

using Unity.Collections;
using Unity.Entities;
using Unity.U2D.Entities.Physics;

abstract class BaseClassicColliderConversionFixture : AuthoringTestFixture
{
    protected delegate void TestValidationFunction(BlobAssetReference<Collider> colliderBlob);

    protected void RunConversionTest(TestValidationFunction validationFunc)
    {
        // Run the GameObject Conversion.
        RunConversion(Root);

        // Verify the Conversion.
        using (var query = EntityManager.CreateEntityQuery(typeof(PhysicsColliderBlob)))
        {
            Assert.AreEqual(1, query.CalculateEntityCount(), "Was expecting a single entity!");

            // Fetch the entity.
            using (var entities = query.ToEntityArray(Allocator.TempJob))
            {
                var entity = entities[0];

                // Fetch the collider blob.
                using (var colliderBlob = GetComponentData<PhysicsColliderBlob>(entity).Collider)
                {
                    validationFunc(colliderBlob);
                }
            }
        }      
    }
}

