using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Unity.U2D.Entities.Physics
{
    static class JobParallelForDeferExtensionsPhysics
    {
        public static unsafe JobHandle ScheduleUnsafeIndex0<T>(this T jobData, NativeArray<int> forEachCount, int innerloopBatchCount, JobHandle dependsOn = new JobHandle()) 
            where T : struct, IJobParallelForDefer
        {
            return jobData.Schedule((int*)NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(forEachCount), innerloopBatchCount, dependsOn);
        }
    }
}