﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class CarControllerLeftLane : MonoBehaviour
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

    //public GameObject redCar;
    //private float redCarPosition;
    //private float redCarSpeed;

    // For lane change
    private bool isChangingLane = false;
    private bool isLaneChangeDoneBefore = false;
    private float targetLanePosition = -2.43f; // The target X position for the lane change
    private float targetSteeringAngle = 0f; // The target X position for the lane change
    private float steeringSpeed = 4f; // Speed at which the steering angle changes
    private bool isAccelerating = false;
    //public Transform frontPosition;

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
        //HandleSteering();
        UpdateWheels();
        //ChangeLane();
        //CalculateSpeedRedCarCar();
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
            frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
            frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            //print("Vertical input : " + verticalInput + " -- Motor Torque : " + verticalInput * motorForce);
            //isBreaking = true;
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
            if (rb.velocity.z < targetSpeedInMetersPerSecond && !isChangingLane)
            {
                //print("Oto Gaz açık Stop Car");
                // Gradually increase the motor torque to accelerate
                frontLeftWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime;
                frontRightWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime;
                //print("Time.deltaTime : " + Time.deltaTime + " -- Motor Torque : " + accelerationRate * Time.deltaTime);
            }
            else if (rb.velocity.z < targetSpeedInMetersPerSecond && isChangingLane)
            {
                //print("Oto Gaz açık Stop Car");
                // Gradually increase the motor torque to accelerate
                frontLeftWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime * 2;
                frontRightWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime * 2;
                //print("Time.deltaTime : " + Time.deltaTime + " -- Motor Torque : " + accelerationRate * Time.deltaTime);
            }
            else
            {
                // Stop further acceleration if the target speed is reached
                //isAccelerating = false;
                //print("Oto Gaz kapalı Stop Car");
                verticalInput = 0f;
                frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
                frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            }
        }
    }
    //private void StartLaneChange(int direction)
    //{
    //    if (!isLaneChangeDoneBefore)
    //    {
    //        isLaneChangeDoneBefore = true;
    //        isChangingLane = true;
    //        targetLanePosition = frontPosition.position.x + (direction * 2.43f); // Adjust target based on front position
    //        targetSteeringAngle = maxSteerAngle * direction; // Set the target steering angle for lane change
    //    }

    //}
    //private void ChangeLane()
    //{
    //    float frontXPosition = frontPosition.position.x;

    //    if (isChangingLane)
    //    {
    //        // Apply the target steering angle to the front wheels to start lane change
    //        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetSteeringAngle, Time.fixedDeltaTime * steeringSpeed);
    //        frontLeftWheelCollider.steerAngle = currentSteerAngle;
    //        frontRightWheelCollider.steerAngle = currentSteerAngle;

    //        // Check if the car's front has reached the target lane position
    //        if (Mathf.Abs(frontXPosition - targetLanePosition) < 0.1f)
    //        {
    //            // Lane change complete
    //            isChangingLane = false;
    //        }
    //    }
    //    else
    //    {
    //        // Gradually reduce the steering angle to zero after lane change
    //        currentSteerAngle = Mathf.Lerp(currentSteerAngle, 0f, Time.fixedDeltaTime * steeringSpeed);
    //        frontLeftWheelCollider.steerAngle = currentSteerAngle;
    //        frontRightWheelCollider.steerAngle = currentSteerAngle;

    //        // Correct the car's yaw (rotation around the y-axis)
    //        float yaw = transform.eulerAngles.y;
    //        yaw = (yaw > 180) ? yaw - 360 : yaw; // Normalize yaw to range -180 to 180

    //        // Apply corrective steering based on the yaw angle
    //        if (Mathf.Abs(yaw) > 0.1f)
    //        {
    //            float correctionSteer;
    //            if (Mathf.Abs(yaw) > 10f)
    //            {
    //                // Apply a stronger correction for large yaw deviations
    //                correctionSteer = Mathf.Clamp(-yaw * 1.0f, -maxSteerAngle, maxSteerAngle);
    //            }
    //            else if (Mathf.Abs(yaw) > 1f)
    //            {
    //                // Apply a moderate correction for medium yaw deviations
    //                correctionSteer = Mathf.Clamp(-yaw * 0.7f, -maxSteerAngle, maxSteerAngle);
    //            }
    //            else
    //            {
    //                // Apply a gentle correction for small yaw deviations
    //                correctionSteer = Mathf.Clamp(-yaw * 0.3f, -maxSteerAngle, maxSteerAngle);
    //            }
    //            frontLeftWheelCollider.steerAngle = correctionSteer;
    //            frontRightWheelCollider.steerAngle = correctionSteer;
    //        }

    //        // Stop adjusting once the car is straight
    //        if (Mathf.Abs(currentSteerAngle) < 0.01f && Mathf.Abs(yaw) < 0.1f)
    //        {
    //            currentSteerAngle = 0f; // Ensure it's fully straightened
    //            frontLeftWheelCollider.steerAngle = 0f;
    //            frontRightWheelCollider.steerAngle = 0f;
    //        }
    //    }

    //}
    #region Red Car
    //private void CalculateSpeedRedCarCar()
    //{
    //    // Calculate the distance the other car has traveled since the last frame
    //    float distanceTraveled = System.Math.Abs(redCar.transform.position.z - redCarPosition);

    //    // Calculate the speed based on distance and time (speed = distance / time)
    //    redCarSpeed = distanceTraveled / Time.fixedDeltaTime * 3.6f;

    //    // Update the previous position of the other car
    //    redCarPosition = redCar.transform.position.z;
    //}

    #endregion
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