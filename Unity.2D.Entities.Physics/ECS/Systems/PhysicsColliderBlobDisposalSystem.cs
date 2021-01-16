using Unity.Entities;

namespace Unity.U2D.Entities.Physics
{
    // Handle disposing of Collider blobs when the entity has been destroyed
    // and the presence of a PhysicsColliderBlobOwner is detected.
    [UpdateBefore(typeof(PhysicsWorldSystem))]
    partial class PhysicsColliderBlobDisposalSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Entities
                .WithNone<PhysicsColliderBlob>()
                .WithStructuralChanges()
                .ForEach((Entity entity, ref PhysicsColliderBlobOwner colliderBlobOwner) =>
                {
                    var colliderBlob = colliderBlobOwner.Collider;
                    if (colliderBlob.IsCreated)
                    {
                        colliderBlob.Dispose();
                    }

                    EntityManager.RemoveComponent<PhysicsColliderBlobOwner>(entity);
                    
                }).Run();
        }
    }
}