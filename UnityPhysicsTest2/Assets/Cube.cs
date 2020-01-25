using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cube : MonoBehaviour
{
    [SerializeField]
    float size_ = 1.0f;

    public Box box_ = new Box(new Vector3(0.5f, 0.5f, 0.5f));

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        SyncBoxPosition();

        if (Input.GetKeyDown(KeyCode.P))
        {
            Debug.Log("Rotation X: " + box_.transform_.rotation_.x_);
            Debug.Log("Rotation Y: " + box_.transform_.rotation_.y_);
            Debug.Log("Rotation Z: " + box_.transform_.rotation_.z_);
        }
    }

    void SyncBoxPosition()
    {
        if (box_ != null)
        {
            box_.transform_.position_ = gameObject.transform.position;
            //box_.transform_.rotation_.x_ = MathStuff.RotateVector(gameObject.transform.eulerAngles, new Vector3(1.0f, 0.0f, 0.0f));
            //box_.transform_.rotation_.y_ = MathStuff.RotateVector(gameObject.transform.eulerAngles, new Vector3(0.0f, 1.0f, 0.0f));
            //box_.transform_.rotation_.z_ = MathStuff.RotateVector(gameObject.transform.eulerAngles, new Vector3(0.0f, 0.0f, 1.0f));
            box_.transform_.rotation_ = MathStuff.QToMat(gameObject.transform.rotation);
        }
    }
}
