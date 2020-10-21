using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

using UnityEngine;
using UnityEngine.Assertions;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateAfter(typeof(PhysicsDebugStreamSystem))]
    [UpdateBefore(typeof(PhysicsWorldSystem))]
    public class DisplayColliderSystem : SystemBase
    {
        PhysicsWorldSystem m_PhysicsWorldSystem;
        PhysicsDebugStreamSystem m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_PhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<PhysicsDebugStreamSystem>();

            RequireSingletonForUpdate<PhysicsDebugDisplay>();
        }

        protected override void OnUpdate()
        {
            if (m_PhysicsWorldSystem.PhysicsWorld.BodyCount == 0)
                return;

            var debugDisplay = GetSingleton<PhysicsDebugDisplay>();

            if (debugDisplay.DrawStaticColliders != 0)
            {
                JobHandle callback(ref PhysicsWorld world, JobHandle deps)
                {
                    return new DisplayColliderJob
                    {
                        OutputStream = m_DebugStreamSystem.GetContext(1),
                        ColliderColor = (Vector4)debugDisplay.StaticColliderColor,
                        PhysicsBodies = m_PhysicsWorldSystem.PhysicsWorld.StaticBodies
                    }.Schedule(deps);
                }

                m_PhysicsWorldSystem.ScheduleCallback(PhysicsCallbacks.Phase.PreStepSimulation, callback);
            }

            if (debugDisplay.DrawDynamicColliders != 0)
            {
                JobHandle callback(ref PhysicsWorld world, JobHandle deps)
                {
                    return new DisplayColliderJob
                    {
                        OutputStream = m_DebugStreamSystem.GetContext(1),
                        ColliderColor = (Vector4)debugDisplay.DynamicColliderColor,
                        PhysicsBodies = m_PhysicsWorldSystem.PhysicsWorld.DynamicBodies
                    }.Schedule(deps);
                }

                m_PhysicsWorldSystem.ScheduleCallback(PhysicsCallbacks.Phase.PreStepSimulation, callback);
            }
        }

        // Job to iterate over all the bodies in a scene, for any
        // which have a collider, fetch the geometry and
        // write it to a debug stream.
        [BurstCompile]
        private unsafe struct DisplayColliderJob : IJob
        {
            public PhysicsDebugStreamSystem.Context OutputStream;
            public Color ColliderColor;

            [ReadOnly] public NativeSlice<PhysicsBody> PhysicsBodies;

            public void Execute()
            {
                OutputStream.Begin(0);

                for (var i = 0; i < PhysicsBodies.Length; ++i)
                {
                    var physicsBody = PhysicsBodies[i];
                    var worldTransform = physicsBody.WorldTransform;
                    var colliderBlob = physicsBody.Collider;
                
                    if (colliderBlob.IsCreated)
                        DrawCollider(
                            colliderBlob.GetColliderPtr(),
                            ref worldTransform,
                            ref OutputStream,
                            ColliderColor);
                }

                OutputStream.End();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void DrawCollider(            
            Collider* collider,
            ref PhysicsTransform worldTransform,
            ref PhysicsDebugStreamSystem.Context outputStream,
            Color colliderColor)
        {
            switch (collider->CollisionType)
            {
                case CollisionType.Convex:
                    {
                        DrawConvex(collider, ref worldTransform, ref outputStream, colliderColor);
                        return;
                    }

                case CollisionType.Composite:
                    {
                        DrawComposite(collider, ref worldTransform, ref outputStream, colliderColor);
                        return;
                    }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void DrawConvex(            
            Collider* collider,
            ref PhysicsTransform worldTransform,
            ref PhysicsDebugStreamSystem.Context outputStream,
            Color colliderColor)
        {
            var convexCollider = (ConvexCollider*)collider;
            var vertexCount = convexCollider->VertexCount;
            var vertices = convexCollider->Vertices;
            var convexRadius = convexCollider->m_ConvexHull.ConvexRadius;

            switch (collider->ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                    {
                        Assert.IsTrue(vertexCount >= 3, "ConvexCollider must have >= 3 vertices.");

                        outputStream.Polygon(vertices, vertexCount, worldTransform, colliderColor);
                        return;
                    }

                case ColliderType.Circle:
                    {
                        Assert.AreEqual(1, vertexCount, "CircleCollider must have 1 vertex.");

                        var position = PhysicsMath.mul(worldTransform, vertices[0]);
                        outputStream.Circle(position, convexRadius, colliderColor);
                        return;
                    }

                case ColliderType.Capsule:
                    {
                        Assert.AreEqual(2, vertexCount, "CapsuleCollider must have 2 vertices.");

                        var vertex0 = PhysicsMath.mul(worldTransform, vertices[0]);
                        var vertex1 = PhysicsMath.mul(worldTransform, vertices[1]);
                        var offset = PhysicsMath.perp(math.normalizesafe(vertex1 - vertex0)) * new float2(convexRadius);

                        // Side Edges.
                        {
                            outputStream.Line(vertex0 - offset, vertex1 - offset, colliderColor);
                            outputStream.Line(vertex0 + offset, vertex1 + offset, colliderColor);
                        }

                        // End Caps.
                        {
                            var startAngle = math.atan2(offset.y, offset.x);
                            var endAngle = startAngle + math.PI;
                            outputStream.Arc(vertex0, convexRadius, startAngle, endAngle, colliderColor);
                            outputStream.Arc(vertex1, convexRadius, startAngle + math.PI, endAngle + math.PI, colliderColor);
                        }

                        return;
                    }

                default:
                    return;
            }
        }

        private static unsafe void DrawComposite(            
            Collider* collider,
            ref PhysicsTransform worldTransform,
            ref PhysicsDebugStreamSystem.Context outputStream,
            Color colliderColor)
        {
            switch (collider->ColliderType)
            {
                case ColliderType.Compound:
                    {
                        var compoundCollider = (PhysicsCompoundCollider*)collider;
                        var children = compoundCollider->Children;

                        for(var i = 0; i < children.Length; ++i)
                        {
                            ref PhysicsCompoundCollider.Child child = ref children[i];

                            var colliderWorldTransform = PhysicsMath.mul(worldTransform, child.CompoundFromChild);
                            DrawCollider(children[i].Collider, ref colliderWorldTransform, ref outputStream, colliderColor);
                        }

                        return;
                    }
            }
        }

    }
}