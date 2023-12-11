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

        // Global up vector
        Vector3 up = Vector3.UnitY;

        // Calculate right vector
        Vector3 right = Vector3.Normalize(Vector3.Cross(up, direction));

        // Recalculate up vector
        up = Vector3.Cross(direction, right);

        // Create a look rotation matrix
        Matrix4x4 lookAtMatrix = new Matrix4x4(
            right.X, right.Y, right.Z, 0,
            up.X, up.Y, up.Z, 0,
            direction.X, direction.Y, direction.Z, 0,
            0, 0, 0, 1
        );

        // Convert the matrix to a quaternion
        Quaternion lookRotation = Quaternion.CreateFromRotationMatrix(lookAtMatrix);

        return lookRotation;
    }

}