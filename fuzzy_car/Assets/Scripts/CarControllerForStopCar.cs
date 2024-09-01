using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class CarControllerForStopCar : MonoBehaviour
{
    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float horizontalInput;
    private float verticalInput;
    private float currentSteerAngle;
    private float currentbreakForce;
    private bool isBreaking;
    private bool respawned;
    Rigidbody rb;

    [SerializeField] private float motorForce;
    [SerializeField] private float breakForce;
    [SerializeField] private float maxSteerAngle;

    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheeTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    //[SerializeField] private TextMeshProUGUI speedometerText; // Text UI element to display speed
    //[SerializeField] private TextMeshProUGUI accelerationText; // Text UI element to display speed
    [SerializeField] private float accelerationRate = 200f; // Rate at which the car speeds up
    [SerializeField] private float maxSpeed = 15f; // Maximum speed in km/h
    //private float currentSpeed;
    //private float previousSpeed;
    //private float currentAcceleration;
    private bool isAccelerating = false;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        if (respawned)
        {
            rb.isKinematic = true;
            respawned = false;
            rb.velocity = Vector3.zero;
        }
        else
        {
            rb.isKinematic = false;
        }

        GetInput();
        HandleMotor();
        HandleSteering();
        UpdateWheels();
        //CalculateAcceleration();
        //DisplaySpeedAndAcceleration();
    }

    private void GetInput()
    {
        // Automatically increase speed when a button is pressed (e.g., Left Shift)
        if (Input.GetKeyDown(KeyCode.F))
        {
            //print("Oto Hız Sistemi açıldı Stop Car");
            isAccelerating = true;
        }
        if (Input.GetKeyDown(KeyCode.G))
        {
            //print("Oto Hız Sistemi kapatıldı");
            isAccelerating = false;
        }
    }

    public void StopCar()
    {
        frontRightWheelCollider.brakeTorque = float.MaxValue;
        frontLeftWheelCollider.brakeTorque = float.MaxValue;
        rearLeftWheelCollider.brakeTorque = float.MaxValue;
        rearRightWheelCollider.brakeTorque = float.MaxValue;
        frontLeftWheelCollider.motorTorque = 0f;
        frontRightWheelCollider.motorTorque = 0f;
        frontLeftWheelCollider.steerAngle = 0f;
        frontRightWheelCollider.steerAngle = 0f;
        respawned = true;
        rb.isKinematic = true;
        rb.velocity = Vector3.zero;
    }


    public void SetInput(float throttle, int braking, float steering)
    {
        horizontalInput = steering;
        verticalInput = throttle;
        isBreaking = braking == 1;
    }

    private void HandleMotor()
    {
        if (isAccelerating)
        {
            //print("Increase speed e girdi");
            isBreaking = false;
            IncreaseSpeed();
        }
        else
        {
            //frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
            //frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            ////print("Vertical input : " + verticalInput + " -- Motor Torque : " + verticalInput * motorForce);
            isBreaking = true;
        }

        currentbreakForce = isBreaking ? breakForce : 0f;
        ApplyBreaking();
    }

    private void ApplyBreaking()
    {
        frontRightWheelCollider.brakeTorque = currentbreakForce;
        frontLeftWheelCollider.brakeTorque = currentbreakForce;
        rearLeftWheelCollider.brakeTorque = currentbreakForce;
        rearRightWheelCollider.brakeTorque = currentbreakForce;
    }

    private void HandleSteering()
    {
        currentSteerAngle = maxSteerAngle * horizontalInput;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheeTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.rotation = rot;
        wheelTransform.position = pos;
    }

    private void IncreaseSpeed()
    {
        if (isAccelerating)
        {
            // Calculate the target speed in m/s
            float targetSpeedInMetersPerSecond = maxSpeed / 3.6f;

            // If the car's current speed is less than the target speed
            if (rb.velocity.magnitude < targetSpeedInMetersPerSecond)
            {
                //print("Oto Gaz açık Stop Car");
                // Gradually increase the motor torque to accelerate
                frontLeftWheelCollider.motorTorque += accelerationRate * Time.deltaTime;
                frontRightWheelCollider.motorTorque += accelerationRate * Time.deltaTime;
                //print("Time.deltaTime : " + Time.deltaTime + " -- Motor Torque : " + accelerationRate * Time.deltaTime);
            }
            else
            {
                // Stop further acceleration if the target speed is reached
                isAccelerating = false;
                //print("Oto Gaz kapalı Stop Car");
                verticalInput = 0f;
                frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
                frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            }
        }
    }

    //private void CalculateAcceleration()
    //{
    //    // Acceleration is the change in speed (velocity) over time
    //    float currentVelocity = rb.velocity.magnitude; // in meters per second (m/s)
    //    currentAcceleration = (currentVelocity - previousSpeed) / Time.fixedDeltaTime; // m/s²
    //    previousSpeed = currentVelocity;
    //}

    //private void DisplaySpeedAndAcceleration()
    //{
    //    float speed = rb.velocity.magnitude * 3.6f; // Convert m/s to km/h
    //    speedometerText.text = $"Speed: {speed:0} km/h";
    //    accelerationText.text = $"Acceleration: {currentAcceleration:0.00} m/s²";
    //}
}