using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using Unity.VisualScripting;

public class CarController : MonoBehaviour
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

    [SerializeField] private TextMeshProUGUI speedometerText; // Text UI element to display speed
    [SerializeField] private TextMeshProUGUI accelerationText; // Text UI element to display speed
    [SerializeField] private TextMeshProUGUI distanceText; // Text UI element to display distance with fron car
    [SerializeField] private float accelerationRate = 200f; // Rate at which the car speeds up
    [SerializeField] private float maxSpeed = 100f; // Maximum speed in km/h

    [SerializeField] private TextMeshProUGUI blueCarSppedText; // Text UI element to display speed

    public GameObject blueCar;
    private float blueCarPosition;
    private float blueCarSpeed;
    private bool isTiming = false; // Flag to check if the timer is running
    private float timer = 0f; // Timer variable

    public Transform frontPosition;

    //For auto break -- Test Only 
    private bool isAutoBreaking = false;

    // For lane change
    private bool isChangingLane = false;
    private bool isLaneChangeDoneBefore = false;
    private float targetLanePosition = 2.43f; // The target X position for the lane change
    private float targetSteeringAngle = 0f; // The target X position for the lane change
    //private float laneChangeSteeringAngle = 30f; // Adjust the steering angle for lane change
    //private float laneWidth = 3.0f; // Adjust the lane width as needed
    //private float laneChangeSpeed = 5f; // The speed of lane change
    private float steeringSpeed = 5f; // Speed at which the steering angle changes

    //private float currentSpeed;
    private float previousSpeed;
    private float currentAcceleration;
    private bool isAccelerating = false;


    //Average acc
    private float totalAcc;
    private float accCounter;
    private float totalBlueCarSpeed;

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
        if (isTiming)
        {
            timer += Time.fixedDeltaTime; // For collision 
        }

        GetInput();
        HandleMotor();
        //HandleSteering();
        UpdateWheels();
        CalculateAcceleration();
        DisplaySpeedAndAcceleration();
        DisplaySpeedBlueCar();
        CalculateDistance();
        //ChangeLane(); //Need Later
        CheckBlueCar();
    }

    private void GetInput()
    {
        horizontalInput = Input.GetAxis(HORIZONTAL);
        verticalInput = Input.GetAxis(VERTICAL);
        isBreaking = Input.GetKey(KeyCode.Space);
        // Automatically increase speed when a button is pressed (e.g., Left Shift)
        if (Input.GetKeyDown(KeyCode.F))
        {
            //print("Oto Hız Sistemi açıldı");
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
            IncreaseSpeed();
        }
        else if (isAutoBreaking)
        {
            verticalInput = 0f;
            frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
            frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            totalAcc += currentAcceleration;
            accCounter += 1.0f;
            totalBlueCarSpeed += blueCarSpeed;
            currentbreakForce = CalculateBrakeForceForDeceleration(-6.5f); // Desired constant deceleration
            //print("Front : " + frontLeftWheelCollider.motorTorque); //0
            //print("Rear : " + rearLeftWheelCollider.motorTorque); // 0
            //print("Break Force: " + currentbreakForce);
            ApplyBreaking();
            //print($"Break Acceleration: {currentAcceleration:0.00} m/s²");
            //print($"Position: {rb.position.z} ");
        }
        else
        {
            frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
            frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            //print("Vertical input : " + verticalInput + " -- Motor Torque : " + verticalInput * motorForce);
        }
        // Handle regular braking
        if (isBreaking)
        {
            currentbreakForce = breakForce / 8;
        }
        else if (isAutoBreaking)
        {
            currentbreakForce = CalculateBrakeForceForDeceleration(-6.5f);
        }
        else
        {
            currentbreakForce = 0;
        }


        //currentbreakForce = isBreaking || isAutoBreaking ? breakForce : 0f;
        //print("Current break force : " + currentbreakForce);
        ApplyBreaking();
    }

    private void ApplyBreaking()
    {
        frontRightWheelCollider.brakeTorque = currentbreakForce;
        frontLeftWheelCollider.brakeTorque = currentbreakForce;
        rearLeftWheelCollider.brakeTorque = currentbreakForce;
        rearRightWheelCollider.brakeTorque = currentbreakForce;
    }
    private float CalculateBrakeForceForDeceleration(float desiredDeceleration)
    {
        float vehicleMass = rb.mass; // Get the mass of the vehicle
        float wheelRadius = frontLeftWheelCollider.radius; // Assuming all wheels have the same radius
        float decelerationForce = vehicleMass * Mathf.Abs(desiredDeceleration);
        float brakeTorque = decelerationForce * wheelRadius; // Convert force to torque

        return brakeTorque;
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
            float targetMinusTenSpeedInMetersPerSecond = (maxSpeed - 20) / 3.6f;

            if (rb.velocity.z < targetSpeedInMetersPerSecond && rb.velocity.z > targetMinusTenSpeedInMetersPerSecond)
            {
                frontLeftWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime / 2;
                frontRightWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime / 2;
                isBreaking = false;
            }

            // If the car's current speed is less than the target speed
            else if (rb.velocity.z < targetSpeedInMetersPerSecond)
            {
                //print("Oto Gaz açık");
                // Gradually increase the motor torque to accelerate
                frontLeftWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime;
                frontRightWheelCollider.motorTorque += accelerationRate * Time.fixedDeltaTime;
                isBreaking = false;
                //print("Time.deltaTime : " + Time.deltaTime + " -- Motor Torque : " + accelerationRate * Time.deltaTime);
            }

            else
            {
                // Stop further acceleration if the target speed is reached
                //isAccelerating = false;
                //isAutoBreaking = true;
                //print("Oto Gaz kapalı");
                //StartLaneChange(-1);
                isBreaking = true;
                verticalInput = 0f;
                frontLeftWheelCollider.motorTorque = 0.0f; //verticalInput * motorForce;
                frontRightWheelCollider.motorTorque = 0.0f;

            }
            if (blueCar.transform.position.x >= 0.0f)//(rb.position.z >= 370.0f)
            {
                verticalInput = 0.00f;
                frontLeftWheelCollider.motorTorque = 0.0f; //verticalInput * motorForce;
                frontRightWheelCollider.motorTorque = 0.0f; //verticalInput * motorForce;
                isAccelerating = false;
                isAutoBreaking = true;
                isBreaking = false;
            }
        }
        if (blueCar.transform.position.x >= 0.0f)//(rb.position.z >= 370.0f)
        {
            verticalInput = 0.00f;
            frontLeftWheelCollider.motorTorque = 0.0f; //verticalInput * motorForce;
            frontRightWheelCollider.motorTorque = 0.0f; //verticalInput * motorForce;
            isAccelerating = false;
            isAutoBreaking = true;
            isBreaking = false;
        }
    }

    private void CalculateAcceleration()
    {
        // Acceleration is the change in speed (velocity) over time
        float currentVelocity = rb.velocity.z; // in meters per second (m/s)
        currentAcceleration = (currentVelocity - previousSpeed) / Time.fixedDeltaTime; // m/s²
        previousSpeed = currentVelocity;
    }

    private void DisplaySpeedAndAcceleration()
    {
        float speed = rb.velocity.z * 3.6f; // Convert m/s to km/h
        speedometerText.text = $"Speed: {speed:0.00} km/h";
        accelerationText.text = $"Acceleration: {currentAcceleration:0.00} m/s²";
    }
    private void StartLaneChange(int direction)
    {
        if (!isLaneChangeDoneBefore)
        {
            isLaneChangeDoneBefore = true;
            isChangingLane = true;
            targetLanePosition = frontPosition.position.x + (direction * 2.43f); // Adjust target based on front position
            targetSteeringAngle = maxSteerAngle * direction; // Set the target steering angle for lane change
        }

    }
    private void ChangeLane()
    {
        float frontXPosition = frontPosition.position.x;

        if (isChangingLane)
        {
            // Apply the target steering angle to the front wheels to start lane change
            currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetSteeringAngle, Time.fixedDeltaTime * steeringSpeed);
            frontLeftWheelCollider.steerAngle = currentSteerAngle;
            frontRightWheelCollider.steerAngle = currentSteerAngle;

            // Check if the car's front has reached the target lane position
            if (Mathf.Abs(frontXPosition - targetLanePosition) < 0.1f)
            {
                // Lane change complete
                isChangingLane = false;
            }
        }
        else
        {
            // Gradually reduce the steering angle to zero after lane change
            currentSteerAngle = Mathf.Lerp(currentSteerAngle, 0f, Time.fixedDeltaTime * steeringSpeed);
            frontLeftWheelCollider.steerAngle = currentSteerAngle;
            frontRightWheelCollider.steerAngle = currentSteerAngle;

            // Correct the car's yaw (rotation around the y-axis)
            float yaw = transform.eulerAngles.y;
            yaw = (yaw > 180) ? yaw - 360 : yaw; // Normalize yaw to range -180 to 180

            // Apply corrective steering based on the yaw angle
            if (Mathf.Abs(yaw) > 0.1f)
            {
                float correctionSteer;
                if (Mathf.Abs(yaw) > 10f)
                {
                    // Apply a stronger correction for large yaw deviations
                    correctionSteer = Mathf.Clamp(-yaw * 1.0f, -maxSteerAngle, maxSteerAngle);
                }
                else if (Mathf.Abs(yaw) > 1f)
                {
                    // Apply a moderate correction for medium yaw deviations
                    correctionSteer = Mathf.Clamp(-yaw * 0.7f, -maxSteerAngle, maxSteerAngle);
                }
                else
                {
                    // Apply a gentle correction for small yaw deviations
                    correctionSteer = Mathf.Clamp(-yaw * 0.3f, -maxSteerAngle, maxSteerAngle);
                }
                frontLeftWheelCollider.steerAngle = correctionSteer;
                frontRightWheelCollider.steerAngle = correctionSteer;
            }

            // Stop adjusting once the car is straight
            if (Mathf.Abs(currentSteerAngle) < 0.01f && Mathf.Abs(yaw) < 0.1f)
            {
                currentSteerAngle = 0f; // Ensure it's fully straightened
                frontLeftWheelCollider.steerAngle = 0f;
                frontRightWheelCollider.steerAngle = 0f;
            }
        }

    }

    #region Blue Car Related
    private void CheckBlueCar()
    {
        if (blueCar.transform.position.x >= 0.0f)
        {
            if (!isTiming)
            {
                isTiming = true;
                //float t = (System.Math.Abs((float)transform.position.z - (float)blueCar.transform.position.z) - 4.34f) / (rb.velocity.magnitude - (30.0f / 3.6f));
                //print("Potential crash time : " + t);

                //float a_red = -2.0f; // Deceleration of the red car in m/s²
                //float v_red_initial = rb.velocity.magnitude; // Initial velocity of the red car in m/s
                //float v_blue_initial = 30.0f / 3.6f; // Initial velocity of the blue car in m/s (30 km/h)
                //float d_initial = Mathf.Abs(transform.position.z - blueCar.transform.position.z) - 4.34f; // Initial distance minus safety buffer
                //print("Distance : " + d_initial);

                float a_red = -5.75f; // Deceleration of the red car in m/s²
                float v_red_initial = rb.velocity.z; // Initial velocity of the red car in m/s
                //print("Red speed : " + v_red_initial + " m/s --- " + v_red_initial * 3.6f + " km/h"); // Red speed : 27.79093 m/s --- 100.0473 km/h
                float v_blue_initial = blueCarSpeed; //8.56f; //30.0f / 3.6f; //30.0f / 3.6f; //blueCarSpeed + 0.08f; //30.0f / 3.6f; //blueCarSpeed; //30.0f / 3.6f; // Initial velocity of the blue car in m/s (30 km/h)
                //print("Blue car speed : " + blueCarSpeed * 3.6f);
                //print("Angle : " + blueCar.transform.rotation.y);
                float d_initial = Mathf.Abs(blueCar.transform.position.z - transform.position.z) - 4.8f; //- (2.17f +1.98f); // Initial distance car center position difference
                //print("Red car pos : " + transform.position.z);
                //print("Blue car pos : " + blueCar.transform.position.z);
                //print("Distance : " + d_initial);

                //float collisionTime = FindCollisionTime(a_red,v_red_initial,v_blue_initial,d_initial);
                //print("Collision time : " + collisionTime);

                // Coefficients for the quadratic equation
                float A = 0.5f * a_red;
                float B = v_red_initial - v_blue_initial;
                float C = -d_initial;

                // Discriminant
                float discriminant = B * B - (4 * A * C);

                if (discriminant >= 0)
                {
                    // Two possible solutions
                    float t1 = (-B + Mathf.Sqrt(discriminant)) / (2 * A);
                    float t2 = (-B - Mathf.Sqrt(discriminant)) / (2 * A);

                    // Choose the smallest positive time
                    float t = Mathf.Min(t1, t2);

                    // Calculate crash velocity
                    float crashVelocity = v_red_initial + a_red * t;
                    //float crashSpeed = Mathf.Abs(crashVelocity); // Ensure it's positive

                    print("Calcucalted Time until collision: " + t + " seconds");
                    print("Calculated Crash speed of red car: " + crashVelocity + " m/s ---- " + crashVelocity * 3.6f + " km/h");
                }
                else
                {
                    print("No collision will occur.");
                }

            }
        }

    }
    float FindCollisionTime(float aRed, float vRed, float vBlue, float distance, float maxT = 10f, float tolerance = 0.1f, float step = 0.01f)
    {
        for (float t = 0; t <= maxT; t += step)
        {
            // Evaluate the equation: (1/2) * aRed * t^2 + (vRed - vBlue) * t - distance
            float equationResult = 0.5f * aRed * t * t + (vRed - vBlue) * t - distance;

            // Check if the result is close to zero within the tolerance
            if (Mathf.Abs(equationResult) <= tolerance)
            {
                // If close enough, return this value of t
                return t;
            }
        }

        // If no solution is found within maxT, return a negative value or an indicator that no solution was found
        return -1f; // or return maxT + step;
    }
    private void CalculateDistance()
    {
        if (blueCar != null)
        {
            double zDifference = System.Math.Abs((double)transform.position.z - (double)blueCar.transform.position.z); // car is 4.34
            zDifference = zDifference - 4.34;
            zDifference = System.Math.Round(zDifference, 2);
            //Debug.Log("Distance to other object: " + distance + " meters");
            distanceText.text = zDifference + " meters";
        }
    }
    private void DisplaySpeedBlueCar()
    {
        // Calculate the distance the other car has traveled since the last frame
        float distanceTraveled = System.Math.Abs(blueCar.transform.position.z - blueCarPosition);

        // Calculate the speed based on distance and time (speed = distance / time)
        blueCarSpeed = distanceTraveled / Time.fixedDeltaTime;

        // Update the previous position of the other car
        blueCarPosition = blueCar.transform.position.z;

        // Optionally, print or use the speed
        //Debug.Log("Other Car Speed: " + blueCar + " units/second");
        float speed = blueCarSpeed * 3.6f; // Convert m/s to km/h
        blueCarSppedText.text = $"Blue Car Speed: {speed:0.00} km/h";
    }
    private void OnCollisionEnter(Collision collision)
    {
        // Check if the object we collided with has the correct tag
        if (collision.gameObject.CompareTag("BlueCar"))
        {
            StopTimer(); // Stop the timer when collision occurs
        }
    }
    public void StopTimer()
    {
        isTiming = false; // Stop the timer
        print("Timer stopped at: " + timer + " seconds.");
        print("Crash velocity of red car: " + previousSpeed + " m/s---- " + previousSpeed * 3.6f + " km/h");
        //print("Crash velocity of red car: " + rb.velocity.magnitude + " m/s---- " + rb.velocity.magnitude * 3.6f + " km/h");
        print("Average Acc : " + totalAcc / accCounter); // -5.07
        print("Average Blue Car Speed : " + totalBlueCarSpeed / accCounter); // 7.92
        print("Time ellapsed : " + Time.fixedDeltaTime * accCounter); 
        float distance = Mathf.Abs(blueCar.transform.position.z - transform.position.z) - 4.34f;
        print("Distance : " + distance); // 0.399
        //print("Fixed Delta Time : " + Time.fixedDeltaTime); // 0.02
    }
    #endregion
}