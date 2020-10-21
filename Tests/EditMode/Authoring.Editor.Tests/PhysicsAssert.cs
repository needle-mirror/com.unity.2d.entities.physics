using Unity.Mathematics;
using NUnit.Framework;

static class PhysicsAssert
{
    public static void AreEqual(float2 a, float2 b, float epsilon, string message = default)
    {
        Assert.IsTrue(math.length(a - b) < epsilon, string.Format("Expected {0} but it is {1} with epsilon {2} : {3}", a, b, epsilon, message));
    }
}
