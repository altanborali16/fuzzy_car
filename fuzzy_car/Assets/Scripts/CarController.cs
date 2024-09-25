using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using Unity.VisualScripting;
using Newtonsoft.Json;
using UnityEngine.Windows;

#region Notes

#region 100 vs 30
//For 100 Km/h vs 30 km/h Stop Car start position -2.43 , 0 , 280
//redCarPosition >= 370.0f
#endregion
#region 80 vs 30
//For 80 Km/h vs 30 km/h Stop Car start position -2.43 , 0 , 170
//redCarPosition >= 250.0f
#endregion

#region 50 vs 30
//For 50 Km/h vs 30 km/h Stop Car start position -2.43 , 0 , 170
//redCarPosition >= 250.0f
#endregion

#endregion
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
        horizontalInput = UnityEngine.Input.GetAxis(HORIZONTAL);
        verticalInput = UnityEngine.Input.GetAxis(VERTICAL);
        isBreaking = UnityEngine.Input.GetKey(KeyCode.Space);
        // Automatically increase speed when a button is pressed (e.g., Left Shift)
        if (UnityEngine.Input.GetKeyDown(KeyCode.F))
        {
            //print("Oto Hız Sistemi açıldı");
            isAccelerating = true;
        }
        if (UnityEngine.Input.GetKeyDown(KeyCode.G))
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

    #region Fuzzy
    private float GetFuzzyResult(float frontCarSpeed, float rearCarSpeed , string jsonPoints) // speeds are m/s
    {
        string json = System.IO.File.ReadAllText("Assets//Scripts//FuzzyInference_Fuzzy1_SpeedBased.json");
        var jsonFriendlyDict = JsonConvert.DeserializeObject<Dictionary<int, List<List<float[]>>>>(jsonPoints);

        // Step 2: Convert the intermediate structure back into the original Dictionary<int, List<Vector3[]>>
        Dictionary<int, List<Vector3[]>> pointLists = new Dictionary<int, List<Vector3[]>>();

        foreach (var kvp in jsonFriendlyDict)
        {
            List<Vector3[]> vector3ArraysList = new List<Vector3[]>();
            foreach (var vectorsAsArrays in kvp.Value)
            {
                Vector3[] vector3Array = new Vector3[vectorsAsArrays.Count];
                for (int i = 0; i < vectorsAsArrays.Count; i++)
                {
                    float[] vectorAsArray = vectorsAsArrays[i];
                    vector3Array[i] = new Vector3(vectorAsArray[0], vectorAsArray[1], vectorAsArray[2]);
                }
                vector3ArraysList.Add(vector3Array);
            }
            pointLists[kvp.Key] = vector3ArraysList;
        }

        // Output the deserialized data to check correctness
        foreach (var kvp in pointLists)
        {
            Console.WriteLine($"Key: {kvp.Key}");
            foreach (var vectorArray in kvp.Value)
            {
                foreach (var vector in vectorArray)
                {
                    Console.WriteLine($"Vector3: {vector}");
                }
            }
        }
        FuzzyData speedBasedFuzzyData = JsonConvert.DeserializeObject<FuzzyData>(json);
        if (speedBasedFuzzyData == null)
        {
            Debug.LogError("Failed to deserialize JSON data.");
            return -1.0f;
        }
        // Example: Accessing output values
        List<float> outputValues = speedBasedFuzzyData.Output.Values;
        List<string> outputLabels = speedBasedFuzzyData.Output.Labels;
        List<List<float>> ruleTable = speedBasedFuzzyData.Output.RuleTable;

        //// Process output values

        Debug.Log("Fuzzy inference data imported successfully.");

        Dictionary<int, List<float>> outs = new Dictionary<int, List<float>>() { { 1, new List<float>() }, { 2, new List<float>() } };
        Dictionary<int, float> inputTestValues = new Dictionary<int, float>() { { 1, 0 }, { 2, 0 } };
        inputTestValues[2] = Math.Abs(frontCarSpeed * 3.6f - rearCarSpeed * 3.6f); // km/hr or m/s
        inputTestValues[1] = (frontCarSpeed * 3.6f + rearCarSpeed * 3.6f) / 2; // km/hr or m/s
        print("Input 1 : " + inputTestValues[1] + "---  Input 2 : " + inputTestValues[2]);
        for (int k = 0; k < 2; k++)
        {
            var inputID = k + 1;
            //foreach (var inputEntry in speedBasedFuzzyData.Inputs)
            //{
            //    string inputName = inputEntry.Key;
            //    InputFuzzy input = inputEntry.Value;

            //    foreach (var membershipEntry in input.Memberships)
            //    {
            //        string membershipName = membershipEntry.Key;
            //        Membership membership = membershipEntry.Value;

            //        outs[inputID].Add(CalculateFiring(membership.xPoints.ToArray(), membership.yPoints.ToArray(), inputTestValues[inputID]));
            //    }
            //}
            for (int l = 0; l < 3; l++)
            {
                List<float> xPoints = new List<float>();
                List<float> yPoints = new List<float>();
                for (int m = 0; m < 3; m++)
                {
                    xPoints.Add(pointLists[inputID][l][m].x);
                    yPoints.Add(pointLists[inputID][l][m].y);
                }
                outs[inputID].Add(CalculateFiring(xPoints.ToArray(), yPoints.ToArray(), inputTestValues[inputID]));
            }
        }
        float[,] array = ConvertTo2DArray(ruleTable);
        float outputTest = CalculateFuzzy(array, outs);
        Debug.Log("Fuzzy output : " + outputTest);
        return outputTest;

    }
    private float CalculateFiring(float[] points_x, float[] points_y, float input)
    {
        float output = 0f;
        if (input <= points_x[0])
            output = points_y[0];
        else if (input >= points_x[2])
            output = points_y[2];
        else if (input > points_x[0] && input <= points_x[1])
        {
            if (points_y[0] == points_y[1])
                output = points_y[1];
            else
                output = (input - points_x[0]) / Mathf.Abs(points_x[1] - points_x[0]);
        }
        else if (input > points_x[1] && input < points_x[2])
        {
            if (points_y[1] == points_y[2])
                output = points_y[2];
            else
                output = (points_x[2] - input) / Mathf.Abs(points_x[2] - points_x[1]);
        }

        return output;
    }
    private float CalculateFuzzy(float[,] ruleValues, Dictionary<int, List<float>> firings)
    {
        float outputSum = 0f;
        float weightSum = 0f;
        for (int i = 0; i < ruleValues.GetLength(1); i++)
        {
            for (int j = 0; j < ruleValues.GetLength(0); j++)
            {
                var w = Mathf.Min(firings[1][j], firings[2][i]);
                outputSum += w * ruleValues[i, j];
                weightSum += w;
            }
        }

        if (weightSum == 0)
            return 0f;
        else
            return outputSum / weightSum;
    }
    public float[,] ConvertTo2DArray(List<List<float>> list)
    {
        // Determine the dimensions
        int rows = list.Count;
        int cols = list[0].Count;

        // Create the 2D array with the same dimensions
        float[,] array = new float[rows, cols];

        // Fill the array with values from the List<List<float>>
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                array[i, j] = list[i][j];
            }
        }

        return array;
    }
    #endregion

    #region Blue Car Related
    private void CheckBlueCar()
    {
        if (blueCar.transform.position.x >= 0.0f)
        {
            if (!isTiming)
            {
                float calculatedCrashVelocityOnBlueCar = CalculateCollisionSpeedOnBlueCar(rb.velocity.z, blueCarSpeed, Mathf.Abs(blueCar.transform.position.z - transform.position.z) - 4.8f);
                if (calculatedCrashVelocityOnBlueCar > 0.00f)
                {
                    string jsonPoints = System.IO.File.ReadAllText("Assets//Scripts//30-80PointList.json");
                    float stayInLaneFuzzy = GetFuzzyResult(blueCarSpeed, calculatedCrashVelocityOnBlueCar, jsonPoints);


                    string jsonPointsChangeLane = System.IO.File.ReadAllText("Assets//Scripts//120-80PointList.json"); //33.33f, rb.velocity.z, 10f
                    float calculatedCrashVelocityOnEgoCar = CalculateCollisionSpeedOnBlueCar(33.33f, rb.velocity.z, 10f);
                    if(calculatedCrashVelocityOnEgoCar < 0.0f)
                    {
                        print("No crash on Lane Change");
                        return;
                    }

                    float changeLaneFuzzy = GetFuzzyResult(rb.velocity.z, calculatedCrashVelocityOnEgoCar, jsonPointsChangeLane);

                    if(changeLaneFuzzy > stayInLaneFuzzy)
                    {
                        print("STAY LANE !!");
                    }
                    else
                    {
                        print("CHANGE LANE !!");
                    }

                }
                    

            }
        }

    }

    private float CalculateCollisionSpeedOnBlueCar(float rearCarSpeed, float frontCarSpeed, float distance)
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
        float v_red_initial = rearCarSpeed; // Initial velocity of the red car in m/s
                                             //print("Red speed : " + v_red_initial + " m/s --- " + v_red_initial * 3.6f + " km/h"); // Red speed : 27.79093 m/s --- 100.0473 km/h
        float v_blue_initial = frontCarSpeed; //8.56f; //30.0f / 3.6f; //30.0f / 3.6f; //blueCarSpeed + 0.08f; //30.0f / 3.6f; //blueCarSpeed; //30.0f / 3.6f; // Initial velocity of the blue car in m/s (30 km/h)
                                             //print("Blue car speed : " + blueCarSpeed * 3.6f);
                                             //print("Angle : " + blueCar.transform.rotation.y);
        float d_initial = distance; //- (2.17f +1.98f); // Initial distance car center position difference
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
            return crashVelocity;
        }
        else
        {
            print("No collision will occur.");
            return -1.0f; ;
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
        //print("Timer stopped at: " + timer + " seconds.");
        //print("Crash velocity of red car: " + previousSpeed + " m/s---- " + previousSpeed * 3.6f + " km/h");
        ////print("Crash velocity of red car: " + rb.velocity.magnitude + " m/s---- " + rb.velocity.magnitude * 3.6f + " km/h");
        //print("Average Acc : " + totalAcc / accCounter); // -5.07
        //print("Average Blue Car Speed : " + totalBlueCarSpeed / accCounter); // 7.92
        //print("Time ellapsed : " + Time.fixedDeltaTime * accCounter);
        //float distance = Mathf.Abs(blueCar.transform.position.z - transform.position.z) - 4.34f;
        //print("Distance : " + distance); // 0.399
        //print("Fixed Delta Time : " + Time.fixedDeltaTime); // 0.02
    }
    #endregion
}