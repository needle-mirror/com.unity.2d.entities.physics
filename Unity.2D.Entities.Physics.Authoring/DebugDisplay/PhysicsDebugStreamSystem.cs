using System.Collections.Generic;
using System.Runtime.CompilerServices;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateBefore(typeof(PhysicsWorldSystem))]
    public class PhysicsDebugStreamSystem : SystemBase
    {
        readonly List<NativeStream> m_DebugStreams = new List<NativeStream>();
        PhysicsDrawComponent m_PhysicsDrawComponent;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;
        static Material m_LineMaterial;

        protected override void OnCreate()
        {
            RequireSingletonForUpdate<PhysicsDebugDisplay>();

            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();

            CreateLineMaterial();
        }

        static void CreateLineMaterial()
        {
            if (m_LineMaterial)
                return;

            // Unity has a built-in shader that is useful for drawing simple colored things.
            var shader = Shader.Find("Hidden/Internal-Colored");
            m_LineMaterial = new Material(shader)
            {
                hideFlags = HideFlags.HideAndDontSave,
            };
            // Turn on alpha blending
            m_LineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            m_LineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            // Turn back-face culling off
            m_LineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            // Turn off depth writes
            m_LineMaterial.SetInt("_ZWrite", 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BeginLineStrip(Color color)
        {
            m_LineMaterial.SetPass(0);
            GL.Begin(GL.LINE_STRIP);
            GL.Color(color);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BeginLines(Color color)
        {
            m_LineMaterial.SetPass(0);
            GL.Begin(GL.LINES);
            GL.Color(color);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DrawLineVertex(float2 vertex)
        {
            GL.Vertex3(vertex.x, vertex.y, 0f);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void EndDraw()
        {
            GL.End();
        }

        public struct Context
        {
            public void Begin(int index)
            {
                Writer.BeginForEachIndex(index);
            }

            public void End()
            {
                Writer.EndForEachIndex();
            }

            public void Point(float2 position, float size, Color color)
            {
                Writer.Write(Type.Point);
                Writer.Write(new Point { Position = position, Size = size, Color = color });
            }

            public void Line(float2 v0, float2 v1, Color color)
            {
                Writer.Write(Type.Line);
                Writer.Write(new Line { V0 = v0, V1 = v1, Color = color });
            }

            public void Box(float2 position, float2 extents, Color color)
            {
                Writer.Write(Type.Box);
                Writer.Write(new Box { Position = position, Extents = extents, Color = color });
            }

            public unsafe void Polygon(ConvexHull.ConvexArray.Accessor vertices, int vertexCount, PhysicsTransform physicsTransform, Color color)
            {
                SafetyChecks.IsTrue(vertexCount <= PhysicsPolygonCollider.Constants.MaxVertexCount);

                Writer.Write(Type.Polygon);

                var polygon = new Polygon
                {
                    Transform = physicsTransform,
                    VertexCount = vertexCount,
                    Color = color
                };

                UnsafeUtility.MemCpy(polygon.Vertices, vertices.GetUnsafePtr(), sizeof(float2) * vertexCount);
                Writer.Write(polygon);
            }

            public void Circle(float2 position, float radius, Color color)
            {
                Writer.Write(Type.Circle);
                Writer.Write(new Circle { Position = position, Radius = radius, Color = color });
            }

            public void Arc(float2 center, float radius, float startAngle, float endAngle, Color color)
            {
                Writer.Write(Type.Arc);
                Writer.Write(new Arc { Center = center, Radius = radius, StartAngle = startAngle, EndAngle = endAngle, Color = color });
            }

            public void Text(char[] text, float2 position, Color color)
            {
                Writer.Write(Type.Text);
                Writer.Write(new Text { Position = position, Color = color, Length = text.Length });

                foreach (char c in text)
                {
                    Writer.Write(c);
                }
            }

            internal NativeStream.Writer Writer;
        }

        public Context GetContext(int foreachCount)
        {
            var stream = new NativeStream(foreachCount, Allocator.TempJob);
            m_DebugStreams.Add(stream);
            return new Context { Writer = stream.AsWriter() };
        }

        public enum Type
        {
            Point,
            Line,
            Box,
            Polygon,
            Circle,
            Arc,
            Text
        }

        public struct Point
        {
            public float2 Position;
            public float Size;
            public Color Color;

            public void Draw()
            {
                var horz = new float2(Size, 0f);
                var vert = new float2(0f, Size);

                BeginLines(Color);
                    DrawLineVertex(Position - horz);
                    DrawLineVertex(Position + horz);
                    DrawLineVertex(Position - vert);
                    DrawLineVertex(Position + vert);
                EndDraw();
            }
        }

        public struct Line
        {
            public float2 V0;
            public float2 V1;
            public Color Color;

            public void Draw()
            {
                BeginLines(Color);
                    DrawLineVertex(V0);
                    DrawLineVertex(V1);
                EndDraw();
            }
        }

        public struct Box
        {
            public float2 Position;
            public float2 Extents;
            public Color Color;

            public void Draw()
            {
                if (!math.all(Extents == 0f))
                {
                    var offset = Extents * 0.5f;

                    BeginLineStrip(Color);
                        DrawLineVertex(Position - offset);
                        DrawLineVertex(Position - (offset * new float2(-1f, 1f)));
                        DrawLineVertex(Position + offset);
                        DrawLineVertex(Position - (offset * new float2(1f, -1f)));
                        DrawLineVertex(Position - offset);
                    EndDraw();
                }
            }
        }

        public struct Circle
        {
            public float2 Position;
            public float Radius;
            public Color Color;

            public void Draw()
            {
                if (Radius > 0f)
                {
                    BeginLineStrip(Color);

                    const int Sides = 16;
                    const float Tau = math.PI * 2f;
                    const float AngleStep = Tau / Sides;
                    const float AngleLimit = Tau + AngleStep;
                    for(var angle = 0f; angle <= AngleLimit; angle += AngleStep)
                    {
                        math.sincos(angle, out float sin, out float cos);
                        DrawLineVertex(Position + new float2(Radius * cos, Radius * sin));
                    }

                    EndDraw();
                }
            }
        }

        public struct Polygon
        {
            public unsafe fixed byte Vertices[(sizeof(float) * 2) * PhysicsPolygonCollider.Constants.MaxVertexCount];
            public int VertexCount;
            public PhysicsTransform Transform;
            public Color Color;

            public unsafe void Draw()
            {
                if (VertexCount < 3)
                    return;

                BeginLineStrip(Color);

                fixed (byte* array = Vertices)
                {
                    float2* vertexArray = (float2*)array;
                    for(var i = 0; i < VertexCount; ++i)
                    {
                        var vertex = PhysicsMath.mul(Transform, vertexArray[i]);
                        DrawLineVertex(vertex);
                    }
                    DrawLineVertex(PhysicsMath.mul(Transform, vertexArray[0]));
                }

                EndDraw();
            }
        }

        public struct Arc
        {
            public float2 Center;
            public float Radius;
            public float StartAngle;
            public float EndAngle;
            public Color Color;

            public void Draw()
            {
                if (Radius > 0f)
                {
                    if (StartAngle > EndAngle)
                    {
                        var temp = StartAngle;
                        StartAngle = EndAngle;
                        EndAngle = temp;
                    }

                    BeginLineStrip(Color);

                    const int Sides = 16;
                    float ArcSpan = EndAngle - StartAngle;
                    float AngleStep = ArcSpan / Sides;
                    float AngleLimit = EndAngle + AngleStep;
                    for(var angle = StartAngle; angle <= AngleLimit; angle += AngleStep)
                    {
                        math.sincos(angle, out float sin, out float cos);
                        DrawLineVertex(Center + new float2(Radius * cos, Radius * sin));
                    }

                    EndDraw();
                }
            }
        }

        struct Text
        {
            public float2 Position;
            public Color Color;
            public int Length;

            public void Draw(ref NativeStream.Reader reader)
            {
                // Read string data.
                char[] stringBuf = new char[Length];
                for (int i = 0; i < Length; i++)
                {
                    stringBuf[i] = reader.Read<char>();
                }

                GUIStyle style = new GUIStyle();
                style.normal.textColor = Color;
#if UNITY_EDITOR    
                Handles.Label(new float3(Position, 0f), new string(stringBuf), style);
#endif // UNITY_EDITOR                
            }
        }

        private void Draw()
        {
            for (int i = 0; i < m_DebugStreams.Count; i++)
            {
                NativeStream.Reader reader = m_DebugStreams[i].AsReader();
                for (int j = 0; j != reader.ForEachCount; j++)
                {
                    reader.BeginForEachIndex(j);
                    while (reader.RemainingItemCount != 0)
                    {
                        switch (reader.Read<Type>())
                        {
                            case Type.Polygon:
                                reader.Read<Polygon>().Draw();
                                continue;

                            case Type.Circle:
                                reader.Read<Circle>().Draw();
                                continue;

                            case Type.Box:
                                reader.Read<Box>().Draw();
                                continue;

                            case Type.Arc:
                                reader.Read<Arc>().Draw();
                                continue;

                            case Type.Point:
                                reader.Read<Point>().Draw();
                                continue;

                            case Type.Line:
                                reader.Read<Line>().Draw();
                                continue;

                            case Type.Text:
                                reader.Read<Text>().Draw(ref reader);
                                continue;

                            default:
                                return;
                        }
                    }
                    reader.EndForEachIndex();
                }
            }
        }

        private class PhysicsDrawComponent : MonoBehaviour
        {
            public PhysicsDebugStreamSystem DebugDraw;

            public void OnDrawGizmos()
            {
                if (DebugDraw != null)
                {
                    // Make sure all potential debug display jobs are finished
                    DebugDraw.m_EndFramePhysicsSystem.FinalJobHandle.Complete();

                    // Draw everything.
                    DebugDraw.Draw();
                }
            }
        }

        protected override void OnUpdate()
        {
            // Make sure all potential debug display jobs are finished
            m_EndFramePhysicsSystem.FinalJobHandle.Complete();

            // Reset
            for (int i = 0; i < m_DebugStreams.Count; i++)
            {
                m_DebugStreams[i].Dispose();
            }
            m_DebugStreams.Clear();

            // Set up component to draw
            if (m_PhysicsDrawComponent == null)
            {
                var drawObject = new GameObject();
                m_PhysicsDrawComponent = drawObject.AddComponent<PhysicsDrawComponent>();
                m_PhysicsDrawComponent.name = "PhysicsDebugStreamSystem.PhysicsDrawComponent";
                m_PhysicsDrawComponent.DebugDraw = this;
            }
        }

        protected override void OnDestroy()
        {
            for (int i = 0; i < m_DebugStreams.Count; i++)
                m_DebugStreams[i].Dispose();
        }
    }
}
