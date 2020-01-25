using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class World : MonoBehaviour
{
    [SerializeField]
    GameObject cube1;
    [SerializeField]
    GameObject cube2;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        TestCubes();
    }

    void TestCubes()
    {
        if (cube1 == null || cube2 == null)
        {
            return;
        }
        else
        {
            Manifold m = new Manifold();
            if (SAT.OBoxToOBox(ref m, ref cube1.GetComponent<Cube>().box_, ref cube2.GetComponent<Cube>().box_))
            {
                Debug.Log("Intersecting");
            }
        }
    }
}
