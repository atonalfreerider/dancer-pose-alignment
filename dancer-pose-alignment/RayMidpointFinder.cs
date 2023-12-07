using System.Numerics;

namespace dancer_pose_alignment;

public struct Ray(Vector3 origin, Vector3 direction)
{
    public readonly Vector3 Origin = origin;
    public readonly Vector3 Direction = direction;
}

public static class RayMidpointFinder
{
    const float Tolerance = 0.0001f;
    const int MaxIterations = 1000;

    public static Vector3 FindMinimumMidpoint(List<Ray> rays, List<float> confidences)
    {
        if (rays.Count != confidences.Count)
            throw new ArgumentException("Rays and confidences lists must have the same length.");

        Vector3 startPoint = AverageOrigins(rays, confidences);
        Vector3 optimize = Optimize(startPoint, rays, confidences);
        if (float.IsNaN(optimize.X) || float.IsNaN(optimize.Y) || float.IsNaN(optimize.Z))
        {
            return Vector3.Zero;
        }

        return optimize;
    }

    static Vector3 Optimize(Vector3 startPoint, List<Ray> rays, List<float> confidences)
    {
        Vector3 currentPoint = startPoint;
        const float stepSize = 0.01f;

        for (int i = 0; i < MaxIterations; i++)
        {
            Vector3 gradient = ComputeGradient(currentPoint, rays, confidences);
            Vector3 nextPoint = currentPoint - stepSize * gradient;

            if (Math.Abs(ObjectiveFunction(nextPoint, rays, confidences) -
                         ObjectiveFunction(currentPoint, rays, confidences)) < Tolerance)
                break;

            currentPoint = nextPoint;
        }

        return currentPoint;
    }

    static Vector3 ComputeGradient(Vector3 point, List<Ray> rays, List<float> confidences)
    {
        Vector3 gradient = Vector3.Zero;

        for (int i = 0; i < rays.Count; i++)
        {
            Ray ray = rays[i];
            float confidence = confidences[i];

            Vector3 w = point - ray.Origin;
            float dotProduct = Vector3.Dot(w, ray.Direction);
            Vector3 projection = dotProduct * ray.Direction;
            Vector3 diff = w - projection;

            Vector3 grad = 2 * confidence * (diff - Vector3.Dot(diff, ray.Direction) * ray.Direction);
            gradient += grad;
        }

        if (float.IsInfinity(gradient.X) || float.IsInfinity(gradient.Y) || float.IsInfinity(gradient.Z))
        {
            gradient = Vector3.Zero;
        }

        return gradient;
    }

    static float ObjectiveFunction(Vector3 point, List<Ray> rays, List<float> confidences)
    {
        float sum = 0;
        for (int i = 0; i < rays.Count; i++)
        {
            float distance = DistanceFromPointToRay(point, rays[i]);
            sum += confidences[i] * distance * distance;
        }

        return sum;
    }

    static float DistanceFromPointToRay(Vector3 point, Ray ray)
    {
        // Calculate the shortest distance from 'point' to 'ray' 
        Vector3 w0 = point - ray.Origin;
        float c1 = Vector3.Dot(w0, ray.Direction);
        float c2 = Vector3.Dot(ray.Direction, ray.Direction);
        float b = c1 / c2;

        Vector3 pb = ray.Origin + b * ray.Direction;
        return Vector3.Distance(point, pb);
    }

    static Vector3 AverageOrigins(List<Ray> rays, List<float> confidences)
    {
        Vector3 weightedSum = Vector3.Zero;
        float totalConfidence = 0;

        for (int i = 0; i < rays.Count; i++)
        {
            weightedSum += confidences[i] * rays[i].Origin;
            totalConfidence += confidences[i];
        }

        return weightedSum / totalConfidence;
    }
}