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

using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    static class QueryUtility
    {
        // Based on Game Programming Gems 2
        // Fast, Robust Intersection of 3D Line Segments
        // Graham Rhodes, Applied Research Associates
        // Via Box2D.
        public static float2 NearestPointOnLineSegment(float2 point, float2 p1, float2 p2)
        {
            NearestPointOnLineSegment(out var result, out var unused, p1, p2 - p1, point, false);
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void NearestPointOnLineSegment(
            out float2 nearest, out float parameter,
            float2 A1, float2 L, float2 B,
            bool infinite_line)
        {
            var D = math.lengthsq(L);
            if (D < float.Epsilon * float.Epsilon)
            {
                nearest = A1;
                parameter = 0f;
                return;
            }

            var AB = B - A1;
            parameter = math.dot(L, AB) / D;
            if (!infinite_line)
            {
                parameter = math.clamp(parameter, 0.0f, 1.0f);
            }
            nearest = A1 + parameter * L;
        }

        // Collision Detection in Interactive 3D Environments by Gino van den Bergen
        // From Section 3.4.1 via Box2D.
        public static bool RaycastSegment(
            ref Ray ray, float2 vertex0, float2 vertex1,
            ref float fraction, out float2 normal)
        {
            // Cull back facing collision and ignore parallel segments.
            var rayDirection = ray.Displacement;
            var segmentNormal = PhysicsMath.cross(vertex1 - vertex0, 1.0f);
            var denominator = -math.dot(rayDirection, segmentNormal);

            var slop = float.Epsilon * 100f;
            if (denominator > slop)
            {
                // Does the segment intersect the infinite line associated with this segment?
                var offset = ray.Origin - vertex0;
                var hitFraction = math.dot(offset, segmentNormal);
                if (hitFraction >= 0f && hitFraction <= fraction * denominator)
                {
                    // Does the segment intersect this segment?
                    var mu2 = -rayDirection.x * offset.y + rayDirection.y * offset.x;
                    if (-slop * denominator <= mu2 && mu2 <= denominator * (1.0f + slop))
                    {
                        normal = math.normalize(segmentNormal);
                        fraction = hitFraction / denominator;
                        return true;
                    }
                }
            }

            normal = float2.zero;
            return false;
        }
    }
}
