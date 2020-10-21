using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct PhysicsDebugDisplay : IComponentData
    {
        public int DrawStaticColliders;
        public int DrawDynamicColliders;
        public int DrawColliderAabbs;
        public int DrawBroadphase;

        public float4 StaticColliderColor;
        public float4 DynamicColliderColor;
        public float4 ColliderAabbColor;
        public float4 StaticBroadphaseColor;
        public float4 DynamicBroadphaseColor;
    }
}
