using Unity.Mathematics;
using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
	static class PhysicsAssert
	{
	    public static void AreEqual(float2 a, float2 b, float epsilon, string message = default)
	    {
	        Assert.IsTrue(math.length(a - b) < epsilon, $"{a} is not equal to {b} with epsilon {epsilon} : {message}");
	    }
	    
	    public static void AreEqual(float a, float b, float epsilon = 0.001f, string message = default)
	    {
		    Assert.IsTrue(math.abs(a - b) < epsilon, $"{a} is not equal to {b} with epsilon {epsilon} : {message}");
	    }
	}
}
