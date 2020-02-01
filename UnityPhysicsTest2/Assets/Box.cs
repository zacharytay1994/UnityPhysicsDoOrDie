using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct VelocityState
{
    public Vector3 velocity_;
    public Vector3 angular_velocity_;
}

public class Box
{
    public Transform transform_;
    public Vector3 extents_;

    // body
    public VelocityState vs_;
    public float mass_;
    public float inv_mass_;
    public Mat3 inv_inertia_ = new Mat3();
    public Vector3 force_ = Vector3.zero;
    public Vector3 torque_ = Vector3.zero;

    public Box(Vector3 extents, float mass)
    {
        transform_.position_ = Vector3.zero;
        transform_.rotation_ = new Mat3();
        extents_ = extents;

        // body
        vs_.velocity_ = Vector3.zero;
        vs_.angular_velocity_ = Vector3.zero;
        float ih = 1.0f / 12.0f * mass * ((extents.x * 2) * (extents.x * 2) + (extents.z * 2) * (extents.z * 2));
        float iw = 1.0f / 12.0f * mass * ((extents.z * 2) * (extents.z * 2) + (extents.y * 2) * (extents.y * 2));
        float id = 1.0f / 12.0f * mass * ((extents.x * 2) * (extents.x * 2) + (extents.y * 2) * (extents.y * 2));
        inv_inertia_.x_ = new Vector3(1.0f/ih, 0.0f, 0.0f);
        inv_inertia_.y_ = new Vector3(0.0f, 1.0f/iw, 0.0f);
        inv_inertia_.z_ = new Vector3(0.0f, 0.0f, 1.0f/id);

        mass_ = mass;
        inv_mass_ = 1.0f / mass;
    }

    public void ApplyLinearForce(Vector3 force)
    {
        force_ += force * mass_;
    }
}
