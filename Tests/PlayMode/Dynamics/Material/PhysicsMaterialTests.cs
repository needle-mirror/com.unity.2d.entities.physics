using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsMaterialTests
    {
        [Test]
        public void FrictionCombinePolicyTest()
        {
            var mat1 = new PhysicsMaterial();
            var mat2 = new PhysicsMaterial();

            // GeometricMean Tests
            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;

            mat1.Friction = 1.0f;
            mat2.Friction = 0.0f;      
            var combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.0f);

            mat1.Friction = 0.5f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction,  0.5f);

            mat1.Friction = 1.0f;
            mat2.Friction = 0.25f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            // Minimum Tests
            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat1.Friction = 1.0f;
            mat2.Friction = 0.0f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.0f);

            mat1.Friction = 0.5f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            mat1.Friction = 1.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            // Maximum Tests
            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum;
            mat1.Friction = 1.0f;
            mat2.Friction = 0.0f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 1.0f);

            mat1.Friction = 0.5f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 2.0f);

            // ArithmeticMean Tests
            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean;
            mat1.Friction = 1.0f;
            mat2.Friction = 0.0f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            mat1.Friction = 0.25f;
            mat2.Friction = 0.75f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);

            // Mixed CombinePolicy Tests - Note that max(CombinePolicy of both materials) is used
            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean; // this policy should be used
            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);
            //switch order
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat2, mat1);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);

            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum; // this policy should be used
            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 2.0f);
            //switch order
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat2, mat1);
            PhysicsAssert.AreEqual(combinedFriction, 2.0f);

            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum; // this policy should be used
            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);
            //switch order
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat2, mat1);
            PhysicsAssert.AreEqual(combinedFriction, 0.5f);

            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum; // this policy should be used
            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 2.0f);
            //switch order
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat2, mat1);
            PhysicsAssert.AreEqual(combinedFriction, 2.0f);

            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean; // this policy should be used
            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);
            //switch order
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat2, mat1);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);

            mat1.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum;
            mat2.FrictionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean; // this policy should be used
            mat1.Friction = 2.0f;
            mat2.Friction = 0.5f;
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat1, mat2);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);
            //switch order
            combinedFriction = PhysicsMaterial.GetCombinedFriction(mat2, mat1);
            PhysicsAssert.AreEqual(combinedFriction, 1.25f);
        }


        [Test]
        public void RestitutionCombinePolicyTest()
        {
            PhysicsMaterial mat1 = new PhysicsMaterial();
            PhysicsMaterial mat2 = new PhysicsMaterial();
            float combinedRestitution;

            // GeometricMean Tests
            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;

            mat1.Restitution = 1.0f;
            mat2.Restitution = 0.0f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.0f);

            mat1.Restitution = 0.5f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            mat1.Restitution = 1.0f;
            mat2.Restitution = 0.25f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            // Minimum Tests
            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat1.Restitution = 1.0f;
            mat2.Restitution = 0.0f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.0f);

            mat1.Restitution = 0.5f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            mat1.Restitution = 1.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            // Maximum Tests
            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum;
            mat1.Restitution = 1.0f;
            mat2.Restitution = 0.0f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 1.0f);

            mat1.Restitution = 0.5f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 2.0f);

            // ArithmeticMean Tests
            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean;
            mat1.Restitution = 1.0f;
            mat2.Restitution = 0.0f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            mat1.Restitution = 0.25f;
            mat2.Restitution = 0.75f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);

            // Mixed CombinePolicy Tests - Note that max(CombinePolicy of both materials) is used
            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean; // this policy should be used
            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);
            //switch order
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat2, mat1);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);

            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum; // this policy should be used
            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 2.0f);
            //switch order
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat2, mat1);
            PhysicsAssert.AreEqual(combinedRestitution, 2.0f);

            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.GeometricMean;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum; // this policy should be used
            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);
            //switch order
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat2, mat1);
            PhysicsAssert.AreEqual(combinedRestitution, 0.5f);

            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum; // this policy should be used
            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 2.0f);
            //switch order
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat2, mat1);
            PhysicsAssert.AreEqual(combinedRestitution, 2.0f);

            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Minimum;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean; // this policy should be used
            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);
            //switch order
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat2, mat1);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);

            mat1.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.Maximum;
            mat2.RestitutionCombinePolicy = PhysicsMaterial.CombinePolicy.ArithmeticMean; // this policy should be used
            mat1.Restitution = 2.0f;
            mat2.Restitution = 0.5f;
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat1, mat2);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);
            //switch order
            combinedRestitution = PhysicsMaterial.GetCombinedRestitution(mat2, mat1);
            PhysicsAssert.AreEqual(combinedRestitution, 1.25f);
        }
    }
}
