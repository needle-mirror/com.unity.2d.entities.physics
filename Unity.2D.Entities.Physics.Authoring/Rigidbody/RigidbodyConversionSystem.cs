using Unity.Entities;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateInGroup(typeof(GameObjectConversionGroup))]
    [ConverterVersion("2d", 1)]
    internal class RigidbodyConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.Rigidbody2D rigidbody) =>
            {
                // We don't convert a Rigidbody2D if it's not Simulated.
                if (!rigidbody.simulated)
                    return;

                var entity = GetPrimaryEntity(rigidbody);

                var bodyType = rigidbody.bodyType;

                // There's no components to define a Static rigidbody or its properties.
                if (bodyType != UnityEngine.RigidbodyType2D.Static)
                {
                    // Velocity.
                    if (!DstEntityManager.HasComponent<PhysicsVelocity>(entity))
                    {
                        DstEntityManager.AddComponentData(entity,
                            new PhysicsVelocity
                            {
                                Linear = rigidbody.velocity,
                                Angular = rigidbody.angularVelocity
                            });
                    }

                    var massProperties = MassProperties.Default;

                    // Fetch mass properties from any available collider.
                    if (DstEntityManager.HasComponent<PhysicsColliderBlob>(entity))
                    {
                        var collider = DstEntityManager.GetComponentData<PhysicsColliderBlob>(entity).Collider;
                        massProperties = collider.IsCreated ? collider.Value.MassProperties : MassProperties.Default;
                    }

                    // Dynamic.
                    if (bodyType == UnityEngine.RigidbodyType2D.Dynamic)
                    {
                        DstEntityManager.AddOrSetComponent(entity,
                            PhysicsMass.CreateDynamic(massProperties, rigidbody.mass));

                        if (!DstEntityManager.HasComponent<PhysicsGravity>(entity))
                            DstEntityManager.AddComponentData(entity,
                                new PhysicsGravity { Scale = rigidbody.gravityScale });

                        if (!DstEntityManager.HasComponent<PhysicsDamping>(entity))
                            DstEntityManager.AddComponentData(entity,
                                new PhysicsDamping
                                {
                                    Linear = rigidbody.drag,
                                    Angular = rigidbody.angularDrag
                                });
                    }             
                    // Kinematic.
                    else
                    {
                        DstEntityManager.AddOrSetComponent(entity,
                            PhysicsMass.CreateKinematic(massProperties));
                    }
                }

                // Create any colliders associated with this rigidbody entity.
                m_ColliderConversionSystem.CreateCollider(entity);
            });
        }
    }
}
