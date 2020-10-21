using System;
using System.Diagnostics;
using Unity.Collections;

namespace Unity.U2D.Entities.Physics
{
    public static class SafetyChecks
    {
        private const string ConditionalSymbol = "ENABLE_UNITY_COLLECTIONS_CHECKS";
     
        #region Assertions
        
        [Conditional(ConditionalSymbol)]
        public static void IsTrue(bool condition)
        {
            if (!condition)
                throw new InvalidOperationException();
        }

        [Conditional(ConditionalSymbol)]
        public static void IsFalse(bool condition)
        {
            if (condition)
                throw new InvalidOperationException();
        }

        [Conditional(ConditionalSymbol)]
        public static void AreEqual<A, B>(A value1, B  value2)
            where A : IEquatable<A> where B : IEquatable<B>
        {
            if (!value1.Equals(value2))
                throw new InvalidOperationException();
        }

        [Conditional(ConditionalSymbol)]
        public static void AreNotEqual<A, B>(A value1, B  value2)
            where A : IEquatable<A> where B : IEquatable<B>
        {
            if (value1.Equals(value2))
                throw new InvalidOperationException();
        }
        
        [Conditional(ConditionalSymbol)]
        public static void CheckIndexAndThrow(int index, int length, int min = 0)
        {
            if (index < min || index >= length)
                throw new IndexOutOfRangeException($"Index {index} is out of range [{min}, {length}].");
        }

        #endregion
        
        #region Exceptions

        [Conditional(ConditionalSymbol)]
        public static void ThrowInvalidOperationException(FixedString64 message = default) => throw new InvalidOperationException($"{message}");

        [Conditional(ConditionalSymbol)]
        public static void ThrowNotImplementedException() => throw new NotImplementedException();

        [Conditional(ConditionalSymbol)]
        public static void ThrowNotSupportedException(FixedString64 message = default) => throw new NotSupportedException($"{message}");

        [Conditional(ConditionalSymbol)]
        public static void ThrowArgumentException(in FixedString32 paramName, FixedString64 message = default) =>
            throw new ArgumentException($"{message}", $"{paramName}");

        #endregion
    }
}