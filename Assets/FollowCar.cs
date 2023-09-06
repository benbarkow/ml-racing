using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FollowCar : MonoBehaviour
{
    public GameObject car;
    public Vector3 offset;
    // Start is called before the first frame update
    void Start()
    {
        transform.position = car.transform.position + offset;
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = car.transform.position + offset;
    }
}
