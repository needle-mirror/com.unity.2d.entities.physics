using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    public struct FourAabb
    {
        public float4 MinX, MaxX;
        public float4 MinY, MaxY;

        public static FourAabb Empty => new FourAabb
        {
            MinX = new float4(float.MaxValue),
            MaxX = new float4(float.MinValue),
            MinY = new float4(float.MaxValue),
            MaxY = new float4(float.MinValue),
        };

        public Aabb GetAabb(int index) => new Aabb
        {
            Min = new float2(MinX[index], MinY[index]),
            Max = new float2(MaxX[index], MaxY[index])
        };

        public FourAabb GetAabbT(int index) => new FourAabb
        {
            MinX = new float4(MinX[index]),
            MinY = new float4(MinY[index]),
            MaxX = new float4(MaxX[index]),
            MaxY = new float4(MaxY[index]),
        };

        public void SetAllAabbs(Aabb aabb)
        {
            MinX = new float4(aabb.Min.x);
            MinY = new float4(aabb.Min.y);
            MaxX = new float4(aabb.Max.x);
            MaxY = new float4(aabb.Max.y);
        }

        public void SetAabb(int index, Aabb aabb)
        {
            MinX[index] = aabb.Min.x;
            MaxX[index] = aabb.Max.x;

            MinY[index] = aabb.Min.y;
            MaxY[index] = aabb.Max.y;
        }

        public Aabb GetAABB(int index) => new Aabb
        {
            Min = new float2(MinX.x, MinY.y),
            Max = new float2(MaxX.x, MaxY.y)
        };

        public bool4 Overlap(ref FourAabb aabb)
        {
            var lower4 = (aabb.MinX <= MaxX) & (aabb.MinY <= MaxY);
            var upper4 = (aabb.MaxX >= MinX) & (aabb.MaxY >= MinY);
            return lower4 & upper4;
        }

        public Aabb GetCompoundAabb() => new Aabb
        {
            Min = new float2(math.cmin(MinX), math.cmin(MinY)),
            Max = new float2(math.cmax(MaxX), math.cmax(MaxY))
        };
       
        public bool4 Overlap1Vs4(ref FourAabb aabbT)
        {
            var lc = (aabbT.MinX <= MaxX) & (aabbT.MinY <= MaxY);
            var hc = (aabbT.MaxX >= MinX) & (aabbT.MaxY >= MinY);
            var c = lc & hc;
            return c;
        }

        public bool4 Overlap1Vs4(ref FourAabb other, int index)
        {
            var aabbT = other.GetAabbT(index);
            return Overlap1Vs4(ref aabbT);
        }

        public bool4 OverlapPoint(float2 point)
        {
            var pointX = new float4(point.x);
            var pointY = new float4(point.y);

            return  (pointX >= MinX) &
                    (pointX <= MaxX) &
                    (pointY >= MinY) &
                    (pointY <= MaxY);
        }

        public bool4 Raycast(Ray ray, float maxFraction, out float4 fractions)
        {
            var lx = MinX - new float4(ray.Origin.x);
            var hx = MaxX - new float4(ray.Origin.x);
            var nearXt = lx * new float4(ray.ReciprocalDisplacement.x);
            var farXt = hx * new float4(ray.ReciprocalDisplacement.x);

            var ly = MinY - new float4(ray.Origin.y);
            var hy = MaxY - new float4(ray.Origin.y);
            var nearYt = ly * new float4(ray.ReciprocalDisplacement.y);
            var farYt = hy * new float4(ray.ReciprocalDisplacement.y);

            var nearX = math.min(nearXt, farXt);
            var farX = math.max(nearXt, farXt);

            var nearY = math.min(nearYt, farYt);
            var farY = math.max(nearYt, farYt);

            var nearMax = math.max(math.max(nearX, nearY), float4.zero);
            var farMin = math.min(math.min(farX, farY), new float4(maxFraction));

            fractions = nearMax;

            return (nearMax <= farMin) & (lx <= hx);
        }

        public float4 DistanceFromPointSquared(ref FourPoints fourPoints)
        {
            var px = math.max(fourPoints.X, MinX);
            px = math.min(px, MaxX) - fourPoints.X;

            var py = math.max(fourPoints.Y, MinY);
            py = math.min(py, MaxY) - fourPoints.Y;

            return px * px + py * py;
        }

        public float4 DistanceFromAabbSquared(ref FourAabb aabb)
        {
            var px = math.max(float4.zero, aabb.MinX - MaxX);
            px = math.min(px, aabb.MaxX - MinX);

            var py = math.max(float4.zero, aabb.MinY - MaxY);
            py = math.min(py, aabb.MaxY - MinY);

            return px * px + py * py;
        }
    }
}
