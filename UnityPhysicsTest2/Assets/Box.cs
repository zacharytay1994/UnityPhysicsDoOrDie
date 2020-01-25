using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Box
{
    public Transform transform_;
    public Vector3 extents_;

    public Box(Vector3 extents)
    {
        transform_.position_ = Vector3.zero;
        transform_.rotation_ = new Mat3();
        extents_ = extents;
    }
}
