using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cube : MonoBehaviour
{
    [SerializeField]
    float size_ = 1.0f;
    [SerializeField]
    float mass_ = 5.0f;

    public Box box_;

    // Start is called before the first frame update
    void Start()
    {
        box_ = new Box(new Vector3(size_ / 2.0f, size_ / 2.0f, size_ / 2.0f), mass_);
        box_.transform_.position_ = gameObject.transform.position;
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
            gameObject.transform.position = box_.transform_.position_;
            //box_.transform_.rotation_.x_ = MathStuff.RotateVector(gameObject.transform.eulerAngles, new Vector3(1.0f, 0.0f, 0.0f));
            //box_.transform_.rotation_.y_ = MathStuff.RotateVector(gameObject.transform.eulerAngles, new Vector3(0.0f, 1.0f, 0.0f));
            //box_.transform_.rotation_.z_ = MathStuff.RotateVector(gameObject.transform.eulerAngles, new Vector3(0.0f, 0.0f, 1.0f));
            box_.transform_.rotation_ = MathStuff.QToMat(gameObject.transform.rotation);
        }
    }

    public void IntegratePosition()
    {
        //Debug.Log(box_.vs_.velocity_);
        box_.transform_.position_ += box_.vs_.velocity_ * Time.deltaTime;
        gameObject.transform.rotation = MathStuff.AngularVelocityToQuarternion(box_.vs_.angular_velocity_, gameObject.transform.rotation);
    }

    public void ResetForce()
    {
        box_.force_ = Vector3.zero;
        box_.torque_ = Vector3.zero;
    }
}
