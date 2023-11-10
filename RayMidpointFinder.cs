using System.Numerics;

namespace dancer_pose_alignment;

static class RayMidpointFinder
{
    struct Ray
    {
        public readonly Vector3 Origin;
        public readonly Vector3 Direction;

        public Ray(Vector3 origin, Vector3 direction)
        {
            Origin = origin;
            Direction = direction;
        }
    }

    const float Tolerance = 0.0001f;
    const int MaxIterations = 1000;

    public static List<Vector3> Merged3DPose(List<List<Vector3>> Poses2D, IReadOnlyList<Vector3> CameraForwards)
    {
        List<Vector3> pose = new();

        int poseCount = Poses2D[0].Count;
        for (int i = 0; i < poseCount; i++)
        {
            List<Ray> rays = new();
            int camCounter = 0;
            foreach (List<Vector3> pose2D in Poses2D)
            {
                Vector3 origin = pose2D[i];
                Vector3 direction = CameraForwards[camCounter];
                rays.Add(new Ray(origin, direction));
                camCounter++;
            }

            Vector3 midpoint = FindMinimumMidpoint(rays);
            pose.Add(midpoint);
        }


        return pose;
    }

    static Vector3 FindMinimumMidpoint(List<Ray> rays)
    {
        Vector3 startPoint = AverageOrigins(rays);
        return Optimize(startPoint, rays);
    }

    static Vector3 Optimize(Vector3 startPoint, List<Ray> rays)
    {
        Vector3 currentPoint = startPoint;
        const float stepSize = 0.01f;

        for (int i = 0; i < MaxIterations; i++)
        {
            Vector3 gradient = ComputeGradient(currentPoint, rays);
            Vector3 nextPoint = currentPoint - stepSize * gradient;

            // Check if the objective function value has converged
            if (Math.Abs(ObjectiveFunction(nextPoint, rays) - ObjectiveFunction(currentPoint, rays)) < Tolerance)
                break;

            currentPoint = nextPoint;
        }

        return currentPoint;
    }

    static Vector3 ComputeGradient(Vector3 point, List<Ray> rays)
    {
        Vector3 gradient = Vector3.Zero;

        foreach (Ray ray in rays)
        {
            Vector3 w = point - ray.Origin;
            float dotProduct = Vector3.Dot(w, ray.Direction);
            Vector3 projection = dotProduct * ray.Direction;
            Vector3 diff = w - projection;

            // Calculate the gradient of the squared distance
            Vector3 grad = 2 * (diff - Vector3.Dot(diff, ray.Direction) * ray.Direction);

            // Add the gradient for this ray to the total gradient
            gradient += grad;
        }

        if (float.IsInfinity(gradient.X) || float.IsInfinity(gradient.Y) || float.IsInfinity(gradient.Z))
        {
            // blow up
            gradient = Vector3.Zero;
        }

        return gradient;
    }

    static float ObjectiveFunction(Vector3 point, IEnumerable<Ray> rays)
    {
        return rays.Select(ray => DistanceFromPointToRay(point, ray))
            .Select(distance => distance * distance)
            .Sum();
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

    static Vector3 AverageOrigins(IReadOnlyCollection<Ray> rays)
    {
        Vector3 sum = new Vector3(0, 0, 0);
        sum = rays.Aggregate(sum, (current, ray) => current + ray.Origin);

        return sum / rays.Count;
    }
}