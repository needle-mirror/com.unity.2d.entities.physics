using System;
using Unity.Collections;
using Unity.Entities.UniversalDelegates;

namespace Unity.U2D.Entities.Physics
{
    public struct DynamicsWorld : IDisposable
    {
        // Body Motion Data/Velocities. The length of these two arrays are always equal.
        NativeArray<PhysicsBody.MotionData> m_BodyMotionData;
        NativeArray<PhysicsBody.MotionVelocity> m_BodyMotionVelocity;

        NativeArray<Joint> m_Joints;
        int m_JointCount;
        
        internal void Reset(int bodyMotionCount, int jointCount)
        {
            BodyMotionCount = bodyMotionCount;
            if (m_BodyMotionData.Length < BodyMotionCount)
            {
                if (m_BodyMotionData.IsCreated)
                {
                    m_BodyMotionData.Dispose();
                }

                m_BodyMotionData = new NativeArray<PhysicsBody.MotionData>(BodyMotionCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            if (m_BodyMotionVelocity.Length < BodyMotionCount)
            {
                if (m_BodyMotionVelocity.IsCreated)
                {
                    m_BodyMotionVelocity.Dispose();
                }

                m_BodyMotionVelocity = new NativeArray<PhysicsBody.MotionVelocity>(BodyMotionCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }
            
            m_JointCount = jointCount;
            if (m_Joints.Length < m_JointCount)
            {
                m_Joints.Dispose();
                m_Joints = new NativeArray<Joint>(m_JointCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }            
            
        }

        public NativeArray<PhysicsBody.MotionData> BodyMotionData => m_BodyMotionData.GetSubArray(0, BodyMotionCount);
        public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity => m_BodyMotionVelocity.GetSubArray(0, BodyMotionCount);

        public NativeArray<Joint> Joints => m_Joints.GetSubArray(0, m_JointCount);
        
        public int BodyMotionCount { get; private set; }

        public int JointCount => m_JointCount;
        
        public DynamicsWorld(int bodyMotionCount, int jointCount)
        {
            BodyMotionCount = bodyMotionCount;
            m_BodyMotionData = new NativeArray<PhysicsBody.MotionData>(BodyMotionCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_BodyMotionVelocity = new NativeArray<PhysicsBody.MotionVelocity>(BodyMotionCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            
            m_JointCount = jointCount;
            m_Joints = new NativeArray<Joint>(m_JointCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        #region Cloneable

        public DynamicsWorld Clone()
        {
            return new DynamicsWorld()
            {
                BodyMotionCount = BodyMotionCount,
                m_BodyMotionData = new NativeArray<PhysicsBody.MotionData>(m_BodyMotionData, Allocator.Persistent),
                m_BodyMotionVelocity = new NativeArray<PhysicsBody.MotionVelocity>(m_BodyMotionVelocity, Allocator.Persistent),
                
                m_JointCount = m_JointCount,
                m_Joints = new NativeArray<Joint>(m_Joints, Allocator.Persistent)
            };
        }

        #endregion

        #region IDisposable

        public void Dispose()
        {
            BodyMotionCount = 0;

            if (m_BodyMotionData.IsCreated)
            {
                m_BodyMotionData.Dispose();
            }

            if (m_BodyMotionVelocity.IsCreated)
            {
                m_BodyMotionVelocity.Dispose();
            }

            m_JointCount = 0;
            if (m_Joints.IsCreated)
            {
                m_Joints.Dispose();
            }
        }

        #endregion
    }
}
