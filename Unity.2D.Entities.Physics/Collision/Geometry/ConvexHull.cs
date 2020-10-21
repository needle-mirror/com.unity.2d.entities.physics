// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Miscellaneous parts of the source code below are an adaption of the Box2D library.

using System;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    // Represents a convex hull of points.
    public struct ConvexHull
    {
        public struct ConvexArray
        {
            internal int Offset;
            internal int Length;

            public ConvexArray(int offset, int length)
            {
                Offset = offset;
                Length = length;
            }

            public unsafe struct Accessor
            {
                readonly int* m_OffsetPtr;
                public int Length { get; private set; }

                public Accessor(ref ConvexArray array)
                {
                    fixed (ConvexArray* ptr = &array)
                    {
                        m_OffsetPtr = &ptr->Offset;
                        Length = ptr->Length;
                    }
                }

                public ref float2 this[int index]
                {
                    get
                    {
                        SafetyChecks.CheckIndexAndThrow(index, Length);
                        return ref UnsafeUtility.ArrayElementAsRef<float2>((byte*)m_OffsetPtr + *m_OffsetPtr, index);
                    }
                }

                public float2* GetUnsafePtr() => (float2*)((byte*)m_OffsetPtr + *m_OffsetPtr);

                public Enumerator GetEnumerator() => new Enumerator(m_OffsetPtr, Length);

                public struct Enumerator
                {
                    readonly int* m_OffsetPtr;
                    readonly int m_Length;
                    int m_Index;

                    public float2 Current => UnsafeUtility.ArrayElementAsRef<float2>((byte*)m_OffsetPtr + *m_OffsetPtr, m_Index);

                    public Enumerator(int* offsetPtr, int length)
                    {
                        m_OffsetPtr = offsetPtr;
                        m_Length = length;
                        m_Index = -1;
                    }

                    public bool MoveNext()
                    {
                        return ++m_Index < m_Length;
                    }
                }
            }
        }

        public int Length { get; internal set; }
        public float ConvexRadius;

        ConvexArray m_Vertices;
        ConvexArray m_Normals;
        public ConvexArray.Accessor Vertices => new ConvexArray.Accessor(ref m_Vertices);
        public ConvexArray.Accessor Normals => new ConvexArray.Accessor(ref m_Normals);

        public unsafe NativeArray<float2> AsNativeArray(ConvexArray.Accessor accessor, Allocator allocator)
        {
            var array = new NativeArray<float2>(Length, allocator, NativeArrayOptions.UninitializedMemory);
            UnsafeUtility.MemCpy(array.GetUnsafePtr(), accessor.GetUnsafePtr(), Length * sizeof(float2));
            return array;
        }

        public unsafe ConvexHull(ref ConvexHull hull, byte* vertices, byte* normals, int arrayLength, float convexRadius)
        {
            var offsetVertices = (int)(vertices - (byte*)UnsafeUtility.AddressOf(ref hull.m_Vertices.Offset));
            var offsetNormals = (int)(normals - (byte*)UnsafeUtility.AddressOf(ref hull.m_Normals.Offset));

            m_Vertices = new ConvexArray(offsetVertices, arrayLength);
            m_Normals = new ConvexArray(offsetNormals, arrayLength);
            Length = arrayLength;
            ConvexRadius = convexRadius;
        }

        public unsafe void SetAndGiftWrap(NativeArray<float2> points)
        {
            SafetyChecks.IsTrue(Length == points.Length);

            // Find rightmost point.
            var maxX = points[0].x;
            var maxIndex = 0;
            for (var i = 0; i < Length; ++i)
            {
                var vertex = points[i];
                var x = vertex.x;
                if (x > maxX || (x == maxX && vertex.y < points[maxIndex].y))
                {
                    maxIndex = i;
                    maxX = x;
                }
            }

            // Find convex hull.
            var hullIndices = new NativeArray<int>(Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var m = 0;
            var ih = maxIndex;
            while (true)
            {
                SafetyChecks.IsTrue(m < Length);
                hullIndices[m] = ih;

                var ie = 0;
                for (var j = 1; j < Length; ++j)
                {
                    if (ie == ih)
                    {
                        ie = j;
                        continue;
                    }

                    var r = points[ie] - points[hullIndices[m]];
                    var v = points[j] - points[hullIndices[m]];
                    var crossEdge = PhysicsMath.cross(r, v);

                    // Check hull point or being collinear.
                    if (crossEdge < 0f || (math.abs(crossEdge) < float.Epsilon && math.lengthsq(v) > math.lengthsq(r)))
                    {
                        ie = j;
                    }
                }

                ++m;
                ih = ie;

                if (ie == maxIndex)
                    break;
            }

            // Trim lengths for vertices and normals.
            Length = m_Vertices.Length = m_Normals.Length = m;

            // Copy hull vertices.
            var vertices = Vertices.GetUnsafePtr();
            for (var i = 0; i < Length; ++i)
            {
                vertices[i] = points[hullIndices[i]];
            }

            hullIndices.Dispose();

            // Calculate normals.
            var normals = Normals.GetUnsafePtr();
            for (var i = 0; i < Length; ++i)
            {
                var i1 = i;
                var i2 = i + 1 < Length ? i + 1 : 0;
                var edge = vertices[i2] - vertices[i1];
                SafetyChecks.IsTrue(math.lengthsq(edge) > float.Epsilon);
                normals[i] = math.normalize(PhysicsMath.cross(edge, 1.0f));
            }
        }

        public unsafe Aabb CalculateAabb(PhysicsTransform transform)
        {
            var min = new float2(float.MaxValue);
            var max = new float2(float.MinValue);

            var vertices = Vertices.GetUnsafePtr();
            for (var i = 0; i < Length; ++i, ++vertices)
            {
                min = math.min(min, PhysicsMath.mul(transform, *vertices));
                max = math.max(max, PhysicsMath.mul(transform, *vertices));
            }

            return new Aabb
            {
                Min = min,
                Max = max
            };
        }

        public unsafe MassProperties GetMassProperties()
        {
            var vertexCount = Length;

            SafetyChecks.IsTrue(vertexCount >=3);

            var area = 0f;
            var localCenterOfMass = float2.zero;
            var inertia = 0f;

            var vertices = Vertices.GetUnsafePtr();

            // Find a reference point inside the hull.
            var referencePoint = float2.zero;
            for (var i = 0; i < vertexCount; ++i)
            {
                referencePoint += vertices[i];
            }
            referencePoint *= 1.0f / vertexCount;

            var oneThird = math.rcp(3.0f);
            var oneQuarter = math.rcp(4.0f);

            // Calculate the area, center of mass and inertia.
            for (var i = 0; i < vertexCount; ++i)
            {
                var edge1 = vertices[i] - referencePoint;
                var edge2 = (i + 1 < vertexCount ? vertices[i + 1] : vertices[0]) - referencePoint;

                var crossEdge = PhysicsMath.cross(edge1, edge2);
                var triangleArea = crossEdge * 0.5f;
                area += triangleArea;
                localCenterOfMass += triangleArea * oneThird * (edge1 + edge2);

                var intX = (edge1.x * edge1.x) + (edge2.x * edge1.x) + (edge2.x * edge2.x);
                var intY = (edge1.y * edge1.y) + (edge2.y * edge1.y) + (edge2.y * edge2.y);
                inertia += (oneQuarter * oneThird * crossEdge) * (intX + intY);
            }

            area = math.abs(area);
            SafetyChecks.IsTrue(area > float.Epsilon);
            localCenterOfMass *= math.rcp(area);
            localCenterOfMass += referencePoint;

            // Calculate the angular expansion factor.
            var angularExpansionFactor = 0f;
            for (var i = 0; i < vertexCount; ++i)
            {
                angularExpansionFactor = math.max(angularExpansionFactor, math.lengthsq(vertices[i] - localCenterOfMass));
            }
            angularExpansionFactor = math.sqrt(angularExpansionFactor);

            return new MassProperties(
                localCenterOfMass : localCenterOfMass,
                inertia : inertia,
                area : area,
                angularExpansionFactor : angularExpansionFactor);
        }
    }
}