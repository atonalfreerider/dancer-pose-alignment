using System.Numerics;
using ComputeSharp;

namespace dancer_pose_alignment;

public struct Ray(Vector3 origin, Vector3 direction)
{
    public readonly Vector3 Origin = origin;
    public readonly Vector3 Direction = direction;
}

[ThreadGroupSize(DefaultThreadGroupSizes.X)]
[GeneratedComputeShaderDescriptor]
public readonly partial struct MidpointFinder(
    ReadWriteBuffer<float3> minMidpointBuffer,
    ReadOnlyBuffer<float3> rayOriginsBuffer,
    ReadOnlyBuffer<float3> rayDirectionsBuffer,
    ReadOnlyBuffer<float> confidencesBuffer,
    int camCount
) : IComputeShader
{
    public void Execute()
    {
        int i = ThreadIds.X;

        // Average Origin
        float3 weightedSum = float3.Zero;
        float totalConfidence = 0;

        for (int ex = i * camCount; ex < i * camCount + camCount; ex++)
        {
            weightedSum += Hlsl.Mul(confidencesBuffer[ex], rayOriginsBuffer[ex]);
            totalConfidence += confidencesBuffer[ex];
        }

        float3 averageOrigin = Hlsl.Mul(weightedSum, 1 / totalConfidence);

        // Optimize
        const float stepSize = 0.01f;
        float3 currentPoint = averageOrigin;

        for (int j = 0; j < 1000; j++)
        {
            // Compute Gradient
            float3 gradient = float3.Zero;

            for (int k = i * camCount; k < i * camCount + camCount; k++)
            {
                float confidence = confidencesBuffer[k];
                float3 rayDirection = rayDirectionsBuffer[k];

                float3 w = currentPoint - rayOriginsBuffer[k];
                float dotProduct = Hlsl.Dot(w, rayDirection);
                float3 projection = Hlsl.Mul(dotProduct, rayDirection);
                float3 diff = w - projection;

                float3 grad = Hlsl.Mul(2,
                    Hlsl.Mul(confidence, (diff - Hlsl.Mul(Hlsl.Dot(diff, rayDirection), rayDirection))));
                gradient += grad;
            }

            if (Hlsl.IsInfinite(gradient.X) || Hlsl.IsInfinite(gradient.Y) || Hlsl.IsInfinite(gradient.Z))
            {
                gradient = float3.Zero;
            }

            float3 nextPoint = currentPoint - Hlsl.Mul(stepSize, gradient);

            // Objective Function Next Point
            float nextSum = 0;
            for (int m = i * camCount; m < i * camCount + camCount; m++)
            {
                // Calculate the shortest distance from 'point' to 'ray'
                float3 direction = rayDirectionsBuffer[m];
                float3 origin = rayOriginsBuffer[m];
                float3 w0 = nextPoint - rayOriginsBuffer[m];
                float c1 = Hlsl.Dot(w0, direction);
                float c2 = Hlsl.Dot(direction, direction);
                float b = c1 / c2;

                float3 pb = origin + Hlsl.Mul(direction, b);

                float distance = Hlsl.Distance(nextPoint, pb);

                nextSum += Hlsl.Mul(confidencesBuffer[m], Hlsl.Mul(distance, distance));
            }

            // Objective Function Current Point
            float currentSum = 0;
            for (int m = i * camCount; m < i * camCount + camCount; m++)
            {
                // Calculate the shortest distance from 'point' to 'ray'
                float3 direction = rayDirectionsBuffer[m];
                float3 origin = rayOriginsBuffer[m];
                float3 w0 = currentPoint - rayOriginsBuffer[m];
                float c1 = Hlsl.Dot(w0, direction);
                float c2 = Hlsl.Dot(direction, direction);
                float b = c1 / c2;

                float3 pb = origin + Hlsl.Mul(direction, b);

                float distance = Hlsl.Distance(currentPoint, pb);


                currentSum += Hlsl.Mul(confidencesBuffer[m], Hlsl.Mul(distance, distance));
            }

            // FINAL
            if (Hlsl.Abs(nextSum - currentSum) < 0.0001f)
            {
                j = 1000; // break
            }

            currentPoint = nextPoint;
        }


        if (Hlsl.IsNaN(currentPoint.X) || Hlsl.IsNaN(currentPoint.Y) || Hlsl.IsNaN(currentPoint.Z))
        {
            minMidpointBuffer[i] = float3.Zero;
        }

        minMidpointBuffer[i] = currentPoint;
    }
}

public static class MidpointFinderExtension
{
    public static Vector3 MinimumMidpoint(Vector3 P1, Vector3 D1, Vector3 P2, Vector3 D2)
    {
        Vector3 w0 = P1 - P2;
        float a = Vector3.Dot(D1, D1);
        float b = Vector3.Dot(D1, D2);
        float c = Vector3.Dot(D2, D2);
        float d = Vector3.Dot(D1, w0);
        float e = Vector3.Dot(D2, w0);

        float denom = a * c - b * b;

        float t = (b * e - c * d) / denom;
        float s = (a * e - b * d) / denom;

        Vector3 pointOnRay1 = P1 + t * D1;
        Vector3 pointOnRay2 = P2 + s * D2;

        return 0.5f * (pointOnRay1 + pointOnRay2);
    }
}