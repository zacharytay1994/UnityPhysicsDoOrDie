using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct Contact
{
    public Vector3 position_;
    public float penetration_;
    public float bias_;
    public float constraint_mass_;
}

public struct Manifold
{
    public Box A;
    public Box B;
    public Vector3 normal_;
    public Contact[] contacts_;
    public int contact_count_;
}

public class SAT
{
    public static bool OBoxToOBox(ref Manifold m, ref Box A, ref Box B)
    {
        m.contacts_ = new Contact[16];

        Mat3 A_rotation = A.transform_.rotation_;
        Mat3 B_rotation = B.transform_.rotation_;

        Vector3 A_extents = A.extents_;
        Vector3 B_extents = B.extents_;

        // rotation to represent B in A's coordinate frame C = A^T * B
        // C in qu3e
        Mat3 B_to_A_frame = A_rotation.Transpose().MMult(B_rotation);

        // absC in qu3e
        Mat3 Abs_B_to_A_frame = B_to_A_frame.Abs();

        // vector from A to B center in A space
        // t in qu3e
        Vector3 vec_AB_in_A_frame = A_rotation.Transpose().VMult(B.transform_.position_ - A.transform_.position_);

        // query states
        float s;
        float aMax = -float.MaxValue;
        float bMax = -float.MaxValue;
        int aAxis = -1;
        int bAxis = -1;
        Vector3 normalA = Vector3.zero;
        Vector3 normalB = Vector3.zero;

        // face axis checks
        s = Mathf.Abs(vec_AB_in_A_frame.x) - (A_extents.x + Vector3.Dot(Abs_B_to_A_frame.GetColumn(0), B_extents));
        if (FaceAxis(ref aAxis, 0, s, ref aMax, A_rotation.x_, ref normalA))
        {
            return false;
        }

        Vector3 testget = Abs_B_to_A_frame.GetColumn(1);
        float test = Vector3.Dot(Abs_B_to_A_frame.GetColumn(1), B_extents);
        s = Mathf.Abs(vec_AB_in_A_frame.y) - (A_extents.y + Vector3.Dot(Abs_B_to_A_frame.GetColumn(1), B_extents));
        if (FaceAxis(ref aAxis, 1, s, ref aMax, A_rotation.y_, ref normalA))
        {
            return false;
        }

        s = Mathf.Abs(vec_AB_in_A_frame.z) - (A_extents.z + Vector3.Dot(Abs_B_to_A_frame.GetColumn(2), B_extents));
        if (FaceAxis(ref aAxis, 2, s, ref aMax, A_rotation.z_, ref normalA))
        {
            return false;
        }

        s = Mathf.Abs(Vector3.Dot(vec_AB_in_A_frame, B_to_A_frame.x_)) - (B_extents.x + Vector3.Dot(Abs_B_to_A_frame.x_, A_extents));
        if (FaceAxis(ref bAxis, 3, s, ref bMax, B_rotation.x_, ref normalB))
        {
            return false;
        }

        s = Mathf.Abs(Vector3.Dot(vec_AB_in_A_frame, B_to_A_frame.y_)) - (B_extents.y + Vector3.Dot(Abs_B_to_A_frame.y_, A_extents));
        if (FaceAxis(ref bAxis, 4, s, ref bMax, B_rotation.y_, ref normalB))
        {
            return false;
        }

        s = Mathf.Abs(Vector3.Dot(vec_AB_in_A_frame, B_to_A_frame.z_)) - (B_extents.z + Vector3.Dot(Abs_B_to_A_frame.z_, A_extents));
        if (FaceAxis(ref bAxis, 5, s, ref bMax, B_rotation.z_, ref normalB))
        {
            return false;
        }

        // get max
        int axis;
        Vector3 normal = Vector3.zero;

        if (bMax > aMax)
        {
            axis = bAxis;
            normal = normalB;
        }
        else
        {
            axis = aAxis;
            normal = normalA;
        }

        // check position for inverse normal
        if (Vector3.Dot(normal, B.transform_.position_ - A.transform_.position_) < 0.0f)
        {
            normal = -normal;
        }

        if (axis == -1)
        {
            Debug.Log("AXIS IS NOT FOUND");
        }

        // calculate reference and incidence face
        Transform reference_transform;
        Transform incidence_transform;
        Vector3 reference_extent;
        Vector3 incidence_extent;
        bool flip;

        if (axis < 3)
        {
            reference_transform = A.transform_;
            incidence_transform = B.transform_;
            reference_extent = A_extents;
            incidence_extent = B_extents;
            flip = false;
        }
        else
        {
            reference_transform = B.transform_;
            incidence_transform = A.transform_;
            reference_extent = B_extents;
            incidence_extent = A_extents;
            flip = true;
            normal = -normal;
        }

        Vector3[] vertices = new Vector3[4];
        ComputeIncidentFace(incidence_transform, incidence_extent, normal, ref vertices);
        int[] clip_edges = new int[4];
        Mat3 basis = new Mat3();
        Vector3 extent = Vector3.zero;
        ComputeReferenceEdgesAndBasis(reference_extent, reference_transform, normal, axis, ref clip_edges, ref basis, ref extent);

        Vector3[] vertices_out = new Vector3[16];
        float[] depths = new float[16];
        int outNum;
        outNum = Clip(reference_transform.position_, extent, ref clip_edges, basis, ref vertices, ref vertices_out, ref depths);

        if (outNum > 0)
        {
            m.A = A;
            m.B = B;
            m.contact_count_ = outNum;
            m.normal_ = flip ? -normal : normal;

            for (int i = 0; i < outNum; i++)
            {
                Contact c = new Contact();
                c.position_ = vertices_out[i];
                c.penetration_ = depths[i];
                m.contacts_[i] = c;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    public static bool FaceAxis(ref int controlaxis, int axisindex, float s, ref float sMax, Vector3 normal, ref Vector3 axisnormal)
    {
        if (s > 0.0f)
        {
            return true;
        }
        if (s > sMax)
        {
            sMax = s;
            controlaxis = axisindex;
            axisnormal = normal;
        }
        return false;
    }

    public static void ComputeIncidentFace(Transform incidencetransform, Vector3 extent, Vector3 normal, ref Vector3[] vertices)
    {
        Vector3 rotatedN = -incidencetransform.rotation_.Transpose().VMult(normal);
        Vector3 absRN = MathStuff.AbsVec(rotatedN);

        if (absRN.x > absRN.y && absRN.x > absRN.z)
        {
            if (rotatedN.x > 0.0f)
            {
                vertices[0] = new Vector3(extent.x,  extent.y, -extent.z);
                vertices[1] = new Vector3(extent.x,  extent.y,  extent.z);
                vertices[2] = new Vector3(extent.x, -extent.y,  extent.z);
                vertices[3] = new Vector3(extent.x, -extent.y, -extent.z);
            }
            else
            {
                vertices[0] = new Vector3(-extent.x, -extent.y,  extent.z);
                vertices[1] = new Vector3(-extent.x,  extent.y,  extent.z);
                vertices[2] = new Vector3(-extent.x,  extent.y, -extent.z);
                vertices[3] = new Vector3(-extent.x, -extent.y, -extent.z);
            }
        }
        else if (absRN.y > absRN.x && absRN.y > absRN.z)
        {
            if (rotatedN.y > 0.0f)
            {
                vertices[0] = new Vector3(-extent.x,  extent.y,  extent.z);
                vertices[1] = new Vector3( extent.x,  extent.y,  extent.z);
                vertices[2] = new Vector3( extent.x,  extent.y, -extent.z);
                vertices[3] = new Vector3(-extent.x,  extent.y, -extent.z);
            }
            else
            {
                vertices[0] = new Vector3( extent.x, -extent.y,  extent.z);
                vertices[1] = new Vector3(-extent.x, -extent.y,  extent.z);
                vertices[2] = new Vector3(-extent.x, -extent.y, -extent.z);
                vertices[3] = new Vector3( extent.x, -extent.y, -extent.z);
            }
        }
        else
        {
            if (rotatedN.z > 0.0f)
            {
                vertices[0] = new Vector3(-extent.x,  extent.y,  extent.z);
                vertices[1] = new Vector3(-extent.x, -extent.y,  extent.z);
                vertices[2] = new Vector3( extent.x, -extent.y,  extent.z);
                vertices[3] = new Vector3( extent.x,  extent.y,  extent.z);
            }
            else
            {
                vertices[0] = new Vector3( extent.x, -extent.y, -extent.z);
                vertices[1] = new Vector3(-extent.x, -extent.y, -extent.z);
                vertices[2] = new Vector3(-extent.x,  extent.y, -extent.z);
                vertices[3] = new Vector3( extent.x,  extent.y, -extent.z);
            }
        }

        for (int i = 0; i < 4; i++)
        {
            vertices[i] = incidencetransform.MultVec(vertices[i]);
        }
    }

    public static void ComputeReferenceEdgesAndBasis(Vector3 referenceextent, Transform referencetransform, Vector3 normal, int axis, ref int[] output, ref Mat3 basis, ref Vector3 extent)
    {
        Vector3 rotatedN = referencetransform.rotation_.VMult(normal);

        if (axis >= 3)
        {
            axis -= 3;
        }

        switch (axis)
        {
            case 0:
                if (rotatedN.x >= 0.0f)
                {
                    output[0] = 1;
                    output[1] = 8;
                    output[2] = 7;
                    output[3] = 9;

                    extent = new Vector3(referenceextent.y, referenceextent.z, referenceextent.x);
                    basis.x_ = referencetransform.rotation_.y_;
                    basis.y_ = referencetransform.rotation_.z_;
                    basis.z_ = referencetransform.rotation_.x_;
                }
                else
                {
                    output[0] = 11;
                    output[1] = 3;
                    output[2] = 10;
                    output[3] = 5;

                    extent = new Vector3(referenceextent.z, referenceextent.y, referenceextent.x);
                    basis.x_ =  referencetransform.rotation_.z_;
                    basis.y_ =  referencetransform.rotation_.y_;
                    basis.z_ = -referencetransform.rotation_.x_;
                }
                break;
            case 1:
                if (rotatedN.y >= 0.0f)
                {
                    output[0] = 0;
                    output[1] = 1;
                    output[2] = 2;
                    output[3] = 3;

                    extent = new Vector3(referenceextent.z, referenceextent.x, referenceextent.y);
                    basis.x_ = referencetransform.rotation_.z_;
                    basis.y_ = referencetransform.rotation_.x_;
                    basis.z_ = referencetransform.rotation_.y_;
                }
                else
                {
                    output[0] = 4;
                    output[1] = 5;
                    output[2] = 6;
                    output[3] = 7;

                    extent = new Vector3(referenceextent.z, referenceextent.x, referenceextent.y);
                    basis.x_ =  referencetransform.rotation_.z_;
                    basis.y_ = -referencetransform.rotation_.x_;
                    basis.z_ = -referencetransform.rotation_.y_;
                }
                break;
            case 2:
                if (rotatedN.z >= 0.0f)
                {
                    output[0] = 11;
                    output[1] = 4;
                    output[2] = 8;
                    output[3] = 0;

                    extent = new Vector3(referenceextent.y, referenceextent.x, referenceextent.z);
                    basis.x_ = -referencetransform.rotation_.y_;
                    basis.y_ =  referencetransform.rotation_.x_;
                    basis.z_ =  referencetransform.rotation_.z_;
                }
                else
                {
                    output[0] = 6;
                    output[1] = 10;
                    output[2] = 2;
                    output[3] = 9;

                    extent = new Vector3(referenceextent.y, referenceextent.x, referenceextent.z);
                    basis.x_ = -referencetransform.rotation_.y_;
                    basis.y_ = -referencetransform.rotation_.x_;
                    basis.z_ = -referencetransform.rotation_.z_;
                }
                break;
        }
    }

    public static bool InFront(float a)
    {
        return (a < 0.0f);
    }
    public static bool Behind(float a)
    {
        return (a >= 0.0f);
    }
    public static bool On(float a)
    {
        return (a < 0.005f && a > -0.005f);
    }

    public static int Orthographic(float sign, float extent, int axis, int clipedge, ref Vector3[] vertices, int incount, ref Vector3[] verticesout)
    {
        int outCount = 0;
        Vector3 a = vertices[incount - 1];
        for (int i = 0; i < incount; i++)
        {
            Vector3 b = vertices[i];

            float da = 0.0f;
            float db = 0.0f;
            switch (axis)
            {
                case 0:
                    da = sign * a.x - extent;
                    db = sign * b.x - extent;
                    break;
                case 1:
                    da = sign * a.y - extent;
                    db = sign * b.y - extent;
                    break;
                case 2:
                    da = sign * a.z - extent;
                    db = sign * b.z - extent;
                    break;
            }

            Vector3 cv;

            if ((InFront(da) && InFront(db)) || On(da) || On(db))
            {
                if (!(outCount < 8))
                {
                    Debug.Log("OUTPUT COUNT EXCEED!");
                }
                verticesout[outCount] = b;
                outCount++;
            }
            else if (InFront(da) && Behind(db))
            {
                cv = a + (b - a) * (da / (da - db));
                verticesout[outCount] = cv;
                outCount++;
            }
            else if (Behind(da) && InFront(db))
            {
                cv = a + (b - a) * (da / (da - db));
                verticesout[outCount] = cv;
                outCount++;
                verticesout[outCount] = b;
                outCount++;
            }
        }
        return outCount;
    }

    public static int Clip(Vector3 reference_position, Vector3 extent, ref int[] clipedges, Mat3 basis, ref Vector3[] vertices, ref Vector3[] outputvertices, ref float[] outputdepths)
    {
        int inCount = 4;
        int outCount;
        Vector3[] vertices_in = new Vector3[16];
        Vector3[] vertices_out = new Vector3[16];

        for (int i = 0; i < 4; i++)
        {
            vertices_in[i] = basis.Transpose().VMult(vertices[i] - reference_position);
        }

        outCount = Orthographic(1.0f, extent.x, 0, clipedges[0], ref vertices_in, inCount, ref vertices_out);

        if (outCount == 0)
        {
            return 0;
        }

        inCount = Orthographic(1.0f, extent.y, 1, clipedges[1], ref vertices_out, outCount, ref vertices_in);

        if (inCount == 0)
        {
            return 0;
        }

        outCount = Orthographic(-1.0f, extent.x, 0, clipedges[2], ref vertices_in, inCount, ref vertices_out);

        if (outCount == 0)
        {
            return 0;
        }

        inCount = Orthographic(-1.0f, extent.y, 1, clipedges[3], ref vertices_out, outCount, ref vertices_in);

        outCount = 0;

        for (int i = 0; i < inCount; i++)
        {
            float d = vertices_in[i].z - extent.z;

            if (d <= 0.0f)
            {
                outputvertices[outCount] = basis.VMult(vertices_in[i]) + reference_position;
                outputdepths[outCount] = d;
                outCount++;
            }
        }
        if (outCount > 8)
        {
            Debug.Log("OUTCOUNT > 8");
        }
        return outCount;
    }
}
