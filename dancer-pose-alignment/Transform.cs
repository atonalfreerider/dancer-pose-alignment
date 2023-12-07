using System.Numerics;

namespace dancer_pose_alignment;

public static class Transform
{
    public static Quaternion LookAt(Vector3 targetPosition, Quaternion transformRotation, Vector3 transformPosition)
    {
        Vector3 direction = targetPosition - transformPosition;

        // Check if direction is not zero
        if (direction == Vector3.Zero)
        {
            return transformRotation;
        }

        // Normalize the direction vector
        direction = Vector3.Normalize(direction);

        // Default forward vector
        Vector3 forward = Vector3.Transform(new Vector3(0, 0, 1), transformRotation);

        // Calculate the rotation that aligns the forward vector to the direction vector
        Quaternion lookRotation = Quaternion.Identity;
        float dot = Vector3.Dot(forward, direction);

        if (Math.Abs(dot + 1.0f) <= float.Epsilon)
        {
            // Vector are opposite. Rotate 180 degrees around an arbitrary orthogonal axis. Axis normalisation can be skipped.
            Vector3 axis = Vector3.Cross(new Vector3(0, 1, 0), forward);
            if (axis.LengthSquared() <= float.Epsilon)
            {
                // Pick another if colinear
                axis = Vector3.Cross(new Vector3(1, 0, 0), forward);
            }

            lookRotation = RotateQuaternion(new Quaternion(axis.X, axis.Y, axis.Z, 0), new Vector3(0, 0, 1),
                (float)Math.PI);
        }
        else
        {
            float s = (float)Math.Sqrt((1 + dot) * 2);
            float invs = 1 / s;

            Vector3 c = Vector3.Cross(forward, direction);

            lookRotation = new Quaternion(c.X * invs, c.Y * invs, c.Z * invs, s * 0.5f);
            lookRotation = Quaternion.Normalize(lookRotation);
        }

        return lookRotation;
    }

    static Quaternion RotateQuaternion(Quaternion original, Vector3 axis, float angleRadians)
    {
        // Normalize the axis vector
        axis = Vector3.Normalize(axis);

        // Create a rotation quaternion for the specified angle around the given axis
        float halfAngle = angleRadians / 2;
        float sinHalfAngle = (float)Math.Sin(halfAngle);
        Quaternion rotation = new Quaternion(axis.X * sinHalfAngle, axis.Y * sinHalfAngle, axis.Z * sinHalfAngle,
            (float)Math.Cos(halfAngle));

        // Combine the original quaternion with the rotation quaternion
        Quaternion result = Quaternion.Multiply(rotation, original);

        return result;
    }
}