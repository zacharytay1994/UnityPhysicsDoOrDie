using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct ContactState
{
    public Vector3 ra_;
    public Vector3 rb_;
    public float penetration_;
    public float bias_;
    public float normalMass_;
}

public struct ContactConstraintState
{
    public Box A;
    public Box B;
    public ContactState[] contacts_;
    public int contact_count_;
    public Vector3 normal_;
    public Vector3 center_a_;
    public Vector3 center_b_;
    public Mat3 IA;
    public Mat3 IB;
    public float mass_a_;
    public float mass_b_;
    public float restitution_;
}

public class ColSolve
{
    public static ContactConstraintState GetContactConstraintState(Manifold m)
    {
        ContactConstraintState ccs = new ContactConstraintState();
        // initialize contact constraint state
        ccs.A = m.A;
        ccs.B = m.B;
        ccs.contact_count_ = m.contact_count_;
        ccs.normal_ = m.normal_;
        ccs.contacts_ = new ContactState[16];

        ccs.IA = m.A.inv_inertia_;
        ccs.IB = m.B.inv_inertia_;
        ccs.mass_a_ = m.A.inv_mass_;
        ccs.mass_b_ = m.B.inv_mass_;

        ccs.center_a_ = m.A.transform_.position_;
        ccs.center_b_ = m.B.transform_.position_;

        for (int i = 0; i < m.contact_count_; i++)
        {
            ContactState cs = new ContactState();
            cs.ra_ = m.contacts_[i].position_ - ccs.A.transform_.position_;
            cs.rb_ = m.contacts_[i].position_ - ccs.B.transform_.position_;
            cs.penetration_ = m.contacts_[i].penetration_;
            ccs.contacts_[i] = cs;
        }

        return ccs;
    }

    public static void Presolve(float dt, ref ContactConstraintState ccs)
    {
        for (int i = 0; i < ccs.contact_count_; i++)
        {
            ContactState c = ccs.contacts_[i];

            // precalculate JM^-1JT for contact and friction constraints
            Vector3 raCn = Vector3.Cross(c.ra_, ccs.normal_);
            Vector3 rbCn = Vector3.Cross(c.rb_, ccs.normal_);

            float normal_mass = ccs.mass_a_ + ccs.mass_b_;
            normal_mass += Vector3.Dot(raCn, ccs.IA.VMult(raCn)) + Vector3.Dot(rbCn, ccs.IB.VMult(rbCn));
            c.normalMass_ = MathStuff.Invert(normal_mass);

            // precalculate bias factor
            c.bias_ = -0.2f * (1.0f / dt) * Mathf.Min(0.0f, c.penetration_ + 0.05f);
            ccs.contacts_[i] = c;
        }
    }

    public static void Solve(ref ContactConstraintState ccs)
    {
        for (int i = 0; i < ccs.contact_count_; i++)
        {
            Vector3 va = ccs.A.vs_.velocity_;
            Vector3 wa = ccs.A.vs_.angular_velocity_;
            Vector3 vb = ccs.B.vs_.velocity_;
            Vector3 wb = ccs.B.vs_.angular_velocity_;

            ContactState c = ccs.contacts_[i];

            // relative velocity at contact
            Vector3 dv = vb + Vector3.Cross(wb, c.rb_) - va - Vector3.Cross(wa, c.ra_);

            float vn = Vector3.Dot(dv, ccs.normal_);

            float lambda = Mathf.Max(c.normalMass_ * (-vn + c.bias_), 0.0f);

            Vector3 impulse = ccs.normal_ * lambda;
            va -= impulse * ccs.mass_a_;
            wa -= ccs.IA.VMult(Vector3.Cross(c.ra_, impulse));

            vb += impulse * ccs.mass_b_;
            wb += ccs.IB.VMult(Vector3.Cross(c.rb_, impulse));

            ccs.A.vs_.velocity_ = va;
            ccs.A.vs_.angular_velocity_ = wa;
            ccs.B.vs_.velocity_ = vb;
            ccs.B.vs_.angular_velocity_ = wb;
        }
    }
}
