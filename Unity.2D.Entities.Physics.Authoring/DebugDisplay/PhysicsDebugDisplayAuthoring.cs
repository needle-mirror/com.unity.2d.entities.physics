using Unity.Entities;
using UnityEngine;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [DisallowMultipleComponent]
    internal sealed class PhysicsDebugDisplayAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        PhysicsDebugDisplayAuthoring() { }

        [Header("Collider Geometry")]
        public bool DrawStaticColliders = false;        
        public bool DrawDynamicColliders = false;        
        public Color StaticColliderColor = new Color(0f, 0.5f, 0f, 0.5f);
        public Color DynamicColliderColor = Color.green;

        [Header("Collider AABB")]
        public bool DrawColliderAabbs = false;
        public Color ColliderAabbColor = Color.yellow;

        [Header("Broadphase")]
        public bool DrawBroadphase = false;
        public Color StaticBroadphaseColor = Color.blue;
        public Color DynamicBroadphaseColor = Color.gray;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(
                entity,
                new PhysicsDebugDisplay
                {
                    DrawStaticColliders = DrawStaticColliders ? 1 : 0,
                    DrawDynamicColliders = DrawDynamicColliders ? 1 : 0,
                    DrawColliderAabbs = DrawColliderAabbs ? 1 : 0,
                    DrawBroadphase = DrawBroadphase ? 1 : 0,

                    StaticColliderColor = (Vector4)StaticColliderColor,
                    DynamicColliderColor = (Vector4)DynamicColliderColor,
                    ColliderAabbColor = (Vector4)ColliderAabbColor,
                    StaticBroadphaseColor = (Vector4)StaticBroadphaseColor,
                    DynamicBroadphaseColor = (Vector4)DynamicBroadphaseColor
                }
            );
        }
    }
}