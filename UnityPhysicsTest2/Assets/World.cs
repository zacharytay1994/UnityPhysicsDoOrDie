using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class World : MonoBehaviour
{
    [SerializeField]
    GameObject cube1;
    [SerializeField]
    GameObject cube2;

    [SerializeField]
    GameObject cube_prefab_;
    [SerializeField]
    GameObject parent_cube_;

    Vector3 pos = new Vector3(3.0f, 0.5f, -3.0f);
    Vector3 pos1 = new Vector3(-3.0f, 1.5f, -3.0f);
    Vector3 pos2 = new Vector3(3.0f, 2.0f, 3.0f);
    Vector3 pos3 = new Vector3(-3.0f, 2.5f, 3.0f);

    float timer_ = 0.2f;
    float counter_ = 0.0f;
    float index_ = 0;
    float index_counter_ = 0;

    List<Cube> cube_list_ = new List<Cube>();
    List<Manifold> manifold_list_ = new List<Manifold>();
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start");
        cube_list_.Add(cube2.GetComponent<Cube>());
        cube_list_.Add(cube1.GetComponent<Cube>());
    }

    // Update is called once per frame
    void Update()
    {
        if (counter_ > timer_)
        {
            if (index_ < 10)
            {
                switch (index_counter_ % 4)
                {
                    case 0:
                        AddCube(pos);
                        pos.y += 1.0f;
                        index_++;
                        break;
                    case 1:
                        AddCube(pos1);
                        pos1.y += 1.0f;
                        break;
                    case 2:
                        AddCube(pos2);
                        pos2.y += 1.0f;
                        break;
                    case 3:
                        AddCube(pos3);
                        pos3.y += 1.0f;
                        break;
                }
                index_counter_++;
            }
            else
            {
                timer_ = 0.1f;
                AddCube(new Vector3(Random.Range(-5.0f, 5.0f), 5.0f, Random.Range(-5.0f, 5.0f)));
                index_++;
            }
            counter_ = 0.0f;
        }
        else
        {
            counter_ += Time.deltaTime;
        }
        //if (Input.GetKeyDown(KeyCode.B))
        //{
        //    AddCube(new Vector3(Random.Range(-5.0f, 5.0f), 5.0f, Random.Range(-5.0f, 5.0f)));
        //    //AddCube(pos);
        //    //pos.y += 1.0f;
        //}
        //if (cube1 != null || cube2 != null)
        //{
        //    Box b1 = cube1.GetComponent<Cube>().box_;

        //    if (b1 != null)
        //    {
        //        // apply external force
        //        if (Input.GetKeyDown(KeyCode.LeftArrow))
        //        {
        //            b1.ApplyLinearForce(new Vector3(-1.0f, 0.0f, 0.0f) * 50.0f);
        //        }
        //        b1.ApplyLinearForce(new Vector3(0.0f, -9.81f, 0.0f) * 1.0f);

        //        // integrate velocity
        //        b1.vs_.velocity_ += (b1.force_ * b1.inv_mass_) * Time.deltaTime;
        //        b1.vs_.angular_velocity_ += (b1.inv_inertia_.VMult(b1.torque_)) * Time.deltaTime;

        //        Manifold m = new Manifold();
        //        if (TestCubes(ref m))
        //        {
        //            ContactConstraintState cs = ColSolve.GetContactConstraintState(m);
        //            ColSolve.Presolve(Time.deltaTime, ref cs);
        //            ColSolve.Solve(ref cs);
        //        }

        //        // integrate position
        //        cube1.GetComponent<Cube>().IntegratePosition();
        //        cube2.GetComponent<Cube>().IntegratePosition();

        //        // set forces back to 0
        //        cube1.GetComponent<Cube>().ResetForce();
        //        cube2.GetComponent<Cube>().ResetForce();
        //    }
        //}
        for (int i = 1; i < cube_list_.Count; i++)
        {
            if (cube_list_[i].box_ != null)
            {
                cube_list_[i].box_.ApplyLinearForce(new Vector3(0.0f, -9.81f, 0.0f) * 1.0f);
            }
        }
        IntegrateCubeVelocities();
        TestAllCubes();
        for (int iter = 0; iter < 10; iter++)
        {
            PreSolveSolve();
        }
        //TestAllCubes();
        //PreSolveSolve();
        IntegrateCubeForces();
    }

    void TestAllCubes()
    {
        for (int i = 0; i < cube_list_.Count; i++)
        {
            Cube c = cube_list_[i];
            if (c.box_ == null)
            {
                continue;
            }
            for (int x = i+1; x < cube_list_.Count; x++)
            {
                Manifold m = new Manifold();
                Cube c2 = cube_list_[x];
                if (c2.box_ == null)
                {
                    continue;
                }
                if (SAT.OBoxToOBox(ref m, ref c.box_, ref cube_list_[x].box_))
                {
                    manifold_list_.Add(m);
                }
            }
        }
    }

    void PreSolveSolve()
    {
        foreach (Manifold m in manifold_list_)
        {
            ContactConstraintState cs = ColSolve.GetContactConstraintState(m);
            ColSolve.Presolve(Time.deltaTime, ref cs);
            ColSolve.Solve(ref cs);
        }
    }

    void IntegrateCubeVelocities()
    {
        foreach (Cube c in cube_list_)
        {
            if (c.box_ == null)
            {
                continue;
            }
            c.box_.vs_.velocity_ += (c.box_.force_ * c.box_.inv_mass_) * Time.deltaTime;
            c.box_.vs_.angular_velocity_ += (c.box_.inv_inertia_.VMult(c.box_.torque_)) * Time.deltaTime;
        }
    }

    void IntegrateCubeForces()
    {
        foreach (Cube c in cube_list_)
        {
            if (c.box_ == null)
            {
                continue;
            }
            c.GetComponent<Cube>().IntegratePosition();
            c.GetComponent<Cube>().ResetForce();
        }
        manifold_list_.Clear();
    }

    bool TestCubes(ref Manifold m)
    {
        if (cube1 == null || cube2 == null)
        {
            return false;
        }
        else
        {
            if (SAT.OBoxToOBox(ref m, ref cube1.GetComponent<Cube>().box_, ref cube2.GetComponent<Cube>().box_))
            {
                Debug.Log("Intersecting");
                return true;
            }
            return false;
        }
    }

    void AddCube(Vector3 position)
    {
        if (cube_prefab_ != null)
        {
            GameObject temp = GameObject.Instantiate(cube_prefab_, parent_cube_.transform);
            temp.transform.position = position;
            cube_list_.Add(temp.GetComponent<Cube>());
        }
    }
}
