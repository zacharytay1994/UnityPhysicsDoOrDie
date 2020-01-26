using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MathStuff
{
    public static Vector3 AbsVec(Vector3 v)
    {
        return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
    }

    public static Vector3 RotateVector(Vector3 rotation, Vector3 input)
    {
        Matrix4x4 z_rot = Matrix4x4.identity;
        Matrix4x4 y_rot = Matrix4x4.identity;
        Matrix4x4 x_rot = Matrix4x4.identity;

        rotation.z *= (3.142f / 180.0f);
        rotation.x *= (3.142f / 180.0f);
        rotation.y *= (3.142f / 180.0f);

        z_rot.SetColumn(0, new Vector4(Mathf.Cos(rotation.z), -Mathf.Sin(rotation.z), 0, 0));
        z_rot.SetColumn(1, new Vector4(Mathf.Sin(rotation.z), Mathf.Cos(rotation.z), 0, 0));
        z_rot.SetColumn(2, new Vector4(0, 0, 1, 0));
        z_rot.SetColumn(3, new Vector4(0, 0, 0, 1));

        y_rot.SetColumn(0, new Vector4(Mathf.Cos(rotation.y), 0, Mathf.Sin(rotation.y), 0));
        y_rot.SetColumn(1, new Vector4(0, 1, 0, 0));
        y_rot.SetColumn(2, new Vector4(-Mathf.Sin(rotation.y), 0, Mathf.Cos(rotation.y), 0));
        y_rot.SetColumn(3, new Vector4(0, 0, 0, 1));

        x_rot.SetColumn(0, new Vector4(1, 0, 0, 0));
        x_rot.SetColumn(1, new Vector4(0, Mathf.Cos(rotation.x), -Mathf.Sin(rotation.x), 0));
        x_rot.SetColumn(2, new Vector4(0, Mathf.Sin(rotation.x), Mathf.Cos(rotation.x), 0));
        x_rot.SetColumn(3, new Vector4(0, 0, 0, 1));

        return y_rot.MultiplyVector(x_rot.MultiplyVector(z_rot.MultiplyVector(input)));
    }

    public static Mat3 QToMat(Quaternion q)
    {
        float qx2 = q.x + q.x;
        float qy2 = q.y + q.y;
        float qz2 = q.z + q.z;
        float qxqx2 = q.x * qx2;
        float qxqy2 = q.x * qy2;
        float qxqz2 = q.x * qz2;
        float qxqw2 = q.w * qx2;
        float qyqy2 = q.y * qy2;
        float qyqz2 = q.y * qz2;
        float qyqw2 = q.w * qy2;
        float qzqz2 = q.z * qz2;
        float qzqw2 = q.w * qz2;

        Mat3 temp = new Mat3();
        temp.x_ = new Vector3(1.0f - qyqy2 - qzqz2, qxqy2 + qzqw2, qxqz2 - qyqw2);
        temp.y_ = new Vector3(qxqy2 - qzqw2, 1.0f - qxqx2 - qzqz2, qyqz2 + qxqw2);
        temp.z_ = new Vector3(qxqz2 + qyqw2, qyqz2 - qxqw2, 1.0f - qxqx2 - qyqy2);
        return temp;
    }

    public static float Invert(float f)
    {
        return f != 0.0f ? 1.0f / f : 0.0f;
    }

    public static Quaternion AngularVelocityToQuarternion(Vector3 av, Quaternion old)
    {
        Quaternion q = new Quaternion(av.x * Time.deltaTime, av.y * Time.deltaTime, av.z * Time.deltaTime, 0.0f);
        q *= old;
        old.x += q.x * 0.5f;
        old.y += q.y * 0.5f;
        old.z += q.z * 0.5f;
        old.w += q.w * 0.5f;

        return NormalizeQuaternion(old);
    }

    public static Quaternion NormalizeQuaternion(Quaternion q)
    {
        float x = q.x;
        float y = q.y;
        float z = q.z;
        float w = q.w;
        float d = q.w * q.w + q.x * q.x + q.y * q.y * q.z * q.z;

        if (d == 0)
        {
            w = 1.0f;
        }

        d = 1.0f / Mathf.Sqrt(d);

        if (d > 0.00000001f)
        {
            x *= d;
            y *= d;
            z *= d;
            w *= d;
        }

        return new Quaternion(x, y, z, w);
    }
}

public struct Transform
{
    public Vector3 position_;
    public Mat3 rotation_;

    public Vector3 MultVec(Vector3 v)
    {
        return rotation_.VMult(v) + position_;
    }
}

// column basis vectors
public class Mat3
{
    public Vector3 x_;
    public Vector3 y_;
    public Vector3 z_;

    public Mat3()
    {
        Identity();
    }

    public Mat3(Vector3 x, Vector3 y, Vector3 z)
    {
        x_ = x;
        y_ = y;
        z_ = z;
    }

    void Identity()
    {
        x_ = new Vector3(1.0f, 0.0f, 0.0f);
        y_ = new Vector3(0.0f, 1.0f, 0.0f);
        z_ = new Vector3(0.0f, 0.0f, 1.0f);
    }

    public Vector3 VMult(Vector3 v)
    {
        return new Vector3(x_.x * v.x + y_.x * v.y + z_.x * v.z,
            x_.y * v.x + y_.y * v.y + z_.y * v.z,
            x_.z * v.x + y_.z * v.y + z_.z * v.z);
    }

    public Mat3 MMult(Mat3 rhs)
    {
        return new Mat3(VMult(rhs.x_),
            VMult(rhs.y_),
            VMult(rhs.z_));
    }

    public Mat3 Transpose()
    {
        return new Mat3(new Vector3(x_.x, y_.x, z_.x),
            new Vector3(x_.y, y_.y, z_.y),
            new Vector3(x_.z, y_.z, z_.z));
    }

    public Mat3 Abs()
    {
        return new Mat3(new Vector3(Mathf.Abs(x_.x), Mathf.Abs(x_.y), Mathf.Abs(x_.z)),
            new Vector3(Mathf.Abs(y_.x), Mathf.Abs(y_.y), Mathf.Abs(y_.z)),
            new Vector3(Mathf.Abs(z_.x), Mathf.Abs(z_.y), Mathf.Abs(z_.z)));
    }

    public Vector3 GetColumn(int index)
    {
        switch (index)
        {
            case 0:
                return new Vector3(x_.x, y_.x, z_.x);
                break;
            case 1:
                return new Vector3(x_.y, y_.y, z_.y);
                break;
            case 2:
                return new Vector3(x_.z, y_.z, z_.z);
                break;
        }
        return Vector3.zero;
    }
}
