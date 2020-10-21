using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    [DebuggerDisplay("{Min} : {Max}")]
    public struct Aabb
    {
        public float2 Min;
        public float2 Max;

        public float2 Center => 0.5f * (Max + Min);
        public float2 Extents => Max - Min;
        public float2 HalfExtents => 0.5f * Extents;
        public bool IsValid => math.all(Min <= Max);
        
        public static Aabb Empty => new Aabb
        {
            Min = new float2(float.MaxValue),
            Max = new float2(float.MinValue),
        };

        public Aabb(float2 point)
        {
            Min = point;
            Max = point;
        }

        public Aabb(float2 min, float2 max)
        {
            Min = min;
            Max = max;
        }

        public Aabb(Aabb aabb1, Aabb aabb2)
        {
            Min = math.min(aabb1.Min, aabb2.Min);
            Max = math.max(aabb1.Max, aabb2.Max);
        }

        public bool Contains(Aabb other)
        {
            var result = true;
            result = result && Min.x <= other.Min.x;
            result = result && Min.y <= other.Min.y;
            result = result && other.Max.x <= Max.x;
            result = result && other.Max.y <= Max.y;
            return result;
        }

        public bool Overlap(Aabb aabb)
        {
            return math.all(Max >= aabb.Min & Min <= aabb.Max);
        }

        public bool Overlap(float2 point)
        {
            return math.all(Max >= point & Min <= point);
        }

        public void Combine(Aabb other)
        {
            Min = math.min(Min, other.Min);
            Max = math.max(Max, other.Max);
        }

        public static Aabb Union(Aabb aabb1, Aabb aabb2)
        {
            aabb1.Include(aabb2);
            return aabb1;
        }

        public void Include(Aabb aabb)
        {
            Min = math.min(Min, aabb.Min);
            Max = math.max(Max, aabb.Max);
        }

        public void Include(float2 point)
        {
            Min = math.min(Min, point);
            Max = math.max(Max, point);
        }

        public void Inflate(float delta)
        {
            Min -= delta;
            Max += delta;
        }

        public void Deflate(float delta)
        {
            Min += delta;
            Max -= delta;
        }

        public float SurfaceArea
        {
            get
            {
                var diff = Max - Min;
                return diff.x * diff.y;
            }            
        }
    }

    public static partial class PhysicsMath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Aabb mul(PhysicsTransform transform, Aabb aabb)
        {
            var halfExtents = aabb.HalfExtents;
            var transformedHalfExtents = math.abs(transform.Rotation.c0 * halfExtents.x) + math.abs(transform.Rotation.c1 * halfExtents.y);
            var transformedCenter = mul(transform, aabb.Center);

            return new Aabb
            {
                Min = transformedCenter - transformedHalfExtents,
                Max = transformedCenter + transformedHalfExtents
            };
        }
    }
}
