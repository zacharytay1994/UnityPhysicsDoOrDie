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
    public float tangent_mass_a_;
    public float tangent_mass_b_;
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
    public Vector3 tangent_a_;
    public Vector3 tangent_b_;
    public float friction_;
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

        // calculate tangent vectors
        if (Mathf.Abs(m.normal_.x) >= 0.57735027f)
        {
            ccs.tangent_a_ = new Vector3(m.normal_.y, -m.normal_.x, 0.0f);
        }
        else
        {
            ccs.tangent_a_ = new Vector3(0.0f, m.normal_.z, -m.normal_.y);
        }
        ccs.tangent_a_ = ccs.tangent_a_.normalized;
        ccs.tangent_b_ = Vector3.Cross(m.normal_, ccs.tangent_a_);

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

            // calculate friction/tangent stuff
            Vector3 raCt1 = Vector3.Cross(ccs.tangent_a_, c.ra_);
            Vector3 rbCt1 = Vector3.Cross(ccs.tangent_a_, c.rb_);
            Vector3 testvec = ccs.IA.VMult(raCt1);
            float test1float = Vector3.Dot(raCt1, testvec);
            float tm1 = ccs.mass_a_ + ccs.mass_b_ + Vector3.Dot(raCt1, ccs.IA.VMult(raCt1)) + Vector3.Dot(rbCt1, ccs.IB.VMult(rbCt1));
            c.tangent_mass_a_ = MathStuff.Invert(tm1);

            Vector3 raCt2 = Vector3.Cross(ccs.tangent_b_, c.ra_);
            Vector3 rbCt2 = Vector3.Cross(ccs.tangent_b_, c.rb_);
            float tm2 = ccs.mass_a_ + ccs.mass_b_ + Vector3.Dot(raCt2, ccs.IA.VMult(raCt2)) + Vector3.Dot(rbCt2, ccs.IB.VMult(rbCt2));
            c.tangent_mass_b_ = MathStuff.Invert(tm2);

            // precalculate bias factor
            c.bias_ = -0.2f * (1.0f / dt) * Mathf.Min(0.0f, c.penetration_ + 0.05f);
            ccs.contacts_[i] = c;
        }
    }

    public static void Solve(ref ContactConstraintState ccs)
    {
        Vector3 va = ccs.A.vs_.velocity_;
        Vector3 wa = ccs.A.vs_.angular_velocity_;
        Vector3 vb = ccs.B.vs_.velocity_;
        Vector3 wb = ccs.B.vs_.angular_velocity_;
        for (int i = 0; i < ccs.contact_count_; i++)
        {
            ContactState c = ccs.contacts_[i];

            // relative velocity at contact
            Vector3 dv = vb + Vector3.Cross(wb, c.rb_) - va - Vector3.Cross(wa, c.ra_);

            // friction
            float friction_coefficient = 0.1f;
            float lambda1 = -Vector3.Dot(dv, ccs.tangent_a_) * c.tangent_mass_a_;
            Vector3 impulse1 = ccs.tangent_a_ * lambda1;
            va -= impulse1 * ccs.mass_a_ * friction_coefficient;
            wa -= ccs.IA.VMult(Vector3.Cross(c.ra_, impulse1)) * friction_coefficient;
            vb += impulse1 * ccs.mass_b_ * friction_coefficient;
            wb += ccs.IB.VMult(Vector3.Cross(c.rb_, impulse1)) * friction_coefficient;

            float lambda2 = -Vector3.Dot(dv, ccs.tangent_b_) * c.tangent_mass_b_;
            Vector3 impulse2 = ccs.tangent_b_ * lambda2;
            va -= impulse2 * ccs.mass_a_ * friction_coefficient;
            wa -= ccs.IA.VMult(Vector3.Cross(c.ra_, impulse2)) * friction_coefficient;
            vb += impulse2 * ccs.mass_b_ * friction_coefficient;
            wb += ccs.IB.VMult(Vector3.Cross(c.rb_, impulse2)) * friction_coefficient;

            // recalculate dv with updated friction velocities
            dv = vb + Vector3.Cross(wb, c.rb_) - va - Vector3.Cross(wa, c.ra_);

            // not friction stuff
            float vn = Vector3.Dot(dv, ccs.normal_);

            float lambda = Mathf.Max(c.normalMass_ * (-vn + c.bias_), 0.0f);

            Vector3 impulse = ccs.normal_ * lambda;
            va -= impulse * ccs.mass_a_;
            wa -= ccs.IA.VMult(Vector3.Cross(c.ra_, impulse));

            vb += impulse * ccs.mass_b_;
            wb += ccs.IB.VMult(Vector3.Cross(c.rb_, impulse));
        }

        // if velocities are small, set to 0
        float threshold = 0.1f;
        float ang_thres = 0.1f;
        
        ccs.A.vs_.velocity_ = va.magnitude < threshold ? Vector3.zero : va;
        ccs.A.vs_.angular_velocity_ = (Mathf.Abs(wa.x) < ang_thres && Mathf.Abs(wa.y) < ang_thres && Mathf.Abs(wa.z) < ang_thres) ? Vector3.zero : wa;
        ccs.B.vs_.velocity_ = vb.magnitude < threshold ? Vector3.zero : vb;
        ccs.B.vs_.angular_velocity_ = (Mathf.Abs(wb.x) < ang_thres && Mathf.Abs(wb.y) < ang_thres && Mathf.Abs(wb.z) < ang_thres) ? Vector3.zero : wb;


        //ccs.A.vs_.velocity_ = va;
        //ccs.A.vs_.angular_velocity_ = wa;
        //ccs.B.vs_.velocity_ = vb;
        //ccs.B.vs_.angular_velocity_ = wb;
    }
}
