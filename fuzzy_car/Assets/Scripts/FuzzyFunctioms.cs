using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Windows;
using System.IO;
using Newtonsoft.Json;

public class FuzzyFunctioms : MonoBehaviour
{

    int NrInpMem = 1;
    int NrOutputFunction = 3;
    int NrOutputFunctionOld = 0;
    float minOutputFunction = 0;
    float maxOutputFunction = 1;
    float outputTest = 0;
    int[,] rulesTable;
    float[,] rulesValues;

    // To Do: These dictionaries must be programmatically filled out
    // Key: Input ID, value: # of membership function for that ID from GUI
    Dictionary<int, int> NrInpMemFcn = new Dictionary<int, int>() { { 1, 3 }, { 2, 3 }, { 3, 3 } };
    Dictionary<int, int> NrInpMemFcnOld = new Dictionary<int, int>() { { 1, 0 }, { 2, 0 }, { 3, 0 } };
    Dictionary<int, float> minMemFcn = new Dictionary<int, float>() { { 1, 0 }, { 2, 0 }, { 3, 0 } };
    Dictionary<int, float> maxMemFcn = new Dictionary<int, float>() { { 1, 100 }, { 2, 100 }, { 3, 100 } };
    Dictionary<int, Rect> inputRects = new Dictionary<int, Rect>() { { 1, new Rect() }, { 2, new Rect() }, { 3, new Rect() } };
    Dictionary<int, float> inputTestValues = new Dictionary<int, float>() { { 1, 0 }, { 2, 0 }, { 3, 0 } };

    // Key1: Input ID, Key2: ID of membership function in input, Value: X,Y points (3 for triangle)
    Dictionary<int, Dictionary<int, float[,]>> membershipPoints = new Dictionary<int, Dictionary<int, float[,]>>()
    {
        {1, new Dictionary<int, float[,]>(){{1, new float[2, 3]}, {2, new float[2, 3]}, {3, new float[3, 3]}}},
        {2, new Dictionary<int, float[,]>(){{1, new float[2, 3]}, {2, new float[2, 3]}, {3, new float[3, 3]}}},
        {3, new Dictionary<int, float[,]>(){{1, new float[2, 3]}, {2, new float[2, 3]}, {3, new float[3, 3]}}},
    };
    // Key1: Input ID, Key2: ID of membership function in input, Value: Membership label
    Dictionary<int, Dictionary<int, string>> membershipLabels = new Dictionary<int, Dictionary<int, string>>()
    {
        {1, new Dictionary<int, string>(){{1, string.Empty}, {2, string.Empty }, {3, string.Empty } }},
        {2, new Dictionary<int, string>(){{1, string.Empty}, {2, string.Empty }, {3, string.Empty } }},
        {3, new Dictionary<int, string>(){{1, string.Empty}, {2, string.Empty }, {3, string.Empty } }},
    };
    // Key1: Input ID
    Dictionary<int, List<Vector3[]>> defaultPointLists = new Dictionary<int, List<Vector3[]>>()
    {
        {1, new List<Vector3[]>()},
        {2, new List<Vector3[]>()},
        {3, new List<Vector3[]>()},
    };
    // Key1: Input ID
    Dictionary<int, List<Vector3[]>> pointLists = new Dictionary<int, List<Vector3[]>>()
    {
        {1, new List<Vector3[]>()},
        {2, new List<Vector3[]>()},
        {3, new List<Vector3[]>()},
    };
    //
    List<Vector3[]> defaultOutPointLists = new List<Vector3[]>() { };
    //
    List<Vector3[]> pointOutLists = new List<Vector3[]>() { };
    //
    Dictionary<int, float[,]> outputFunctionPoints = new Dictionary<int, float[,]>()
    {
        {1, new float[2, 2]},
        {2, new float[2, 2]},
        {3, new float[2, 2]},
    };
    //
    Dictionary<int, string> outputLabels = new Dictionary<int, string>()
    {
        {1, string.Empty}, {2, string.Empty }, {3, string.Empty }
    };

    private Vector2 scrollPosition;

    private void OnGUI()
    {
        scrollPosition = GUILayout.BeginScrollView(scrollPosition);
        GUILayout.Label("Set Structure", EditorStyles.boldLabel);
        NrInpMem = Mathf.Min(Mathf.Max(EditorGUILayout.IntField("Input Number:", NrInpMem, GUILayout.Width(200)), 1), 3);
        GUILayout.Space(10);
        GUILayout.Label("", EditorStyles.boldLabel);
        DrawSeparator();

        // 
        GUILayout.Label("Membership Functions", EditorStyles.boldLabel);

        for (int i = 0; i < NrInpMem; i++)
        {
            var inputID = i + 1;
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label($"Input {inputID} Membership Functions", EditorStyles.linkLabel);
            GUILayout.Label("-  Type: ", EditorStyles.linkLabel, GUILayout.Width(50));
            EditorGUILayout.Popup("", 0, new string[] { "Triangle" }, GUILayout.Width(200)); // To Do: Implement other types. E.g.: trapezoid
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginHorizontal();
            NrInpMemFcn[inputID] = Mathf.Min(Mathf.Max(EditorGUILayout.IntField("Membership Number:", NrInpMemFcn[inputID], GUILayout.Width(200)), 1), 3); // Limited to 3 for now
            GUILayout.Label("Min:", GUILayout.Width(50));
            minMemFcn[inputID] = EditorGUILayout.FloatField(minMemFcn[inputID], GUILayout.Width(50));
            GUILayout.Label("Max:", GUILayout.Width(50));
            maxMemFcn[inputID] = EditorGUILayout.FloatField(maxMemFcn[inputID], GUILayout.Width(50));
            EditorGUILayout.EndHorizontal();
            GUILayout.Space(15);
            Rect rect = DrawRectangle();
            inputRects[inputID] = rect;

            defaultPointLists[inputID] = CalculateMemPoints(NrInpMemFcn[inputID], minMemFcn[inputID], maxMemFcn[inputID]);
            if (NrInpMemFcn[inputID] != NrInpMemFcnOld[inputID])
            {
                pointLists[inputID].Clear();
                pointLists[inputID] = defaultPointLists[inputID];
            }
            DrawMembershipFunctions(rect, pointLists[inputID], minMemFcn[inputID], maxMemFcn[inputID], membershipLabels[inputID].Values.ToArray());
            GUILayout.Space(rect.height + 50);

            for (int j = 0; j < NrInpMemFcn[inputID]; j++)
            {
                var membershipID = j + 1;
                membershipLabels[inputID][membershipID] = EditorGUILayout.TextField($"Membership {membershipID} Label: ", membershipLabels[inputID][membershipID], GUILayout.Width(200));

                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField("X Points: ", GUILayout.Width(100));
                for (int m = 0; m < 3; m++) // Limited to 3 for now
                {
                    membershipPoints[inputID][membershipID][0, m] = pointLists[inputID][j][m].x;
                    membershipPoints[inputID][membershipID][0, m] = EditorGUILayout.FloatField(membershipPoints[inputID][membershipID][0, m], GUILayout.Width(100));
                    pointLists[inputID][j][m].x = membershipPoints[inputID][membershipID][0, m];
                }
                EditorGUILayout.EndHorizontal();
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField("Y Points: ", GUILayout.Width(100));
                for (int m = 0; m < 3; m++) // Limited to 3 for now
                {
                    membershipPoints[inputID][membershipID][1, m] = pointLists[inputID][j][m].y;
                    membershipPoints[inputID][membershipID][1, m] = EditorGUILayout.FloatField(membershipPoints[inputID][membershipID][1, m], GUILayout.Width(100));
                    pointLists[inputID][j][m].y = membershipPoints[inputID][membershipID][1, m];
                }
                EditorGUILayout.EndHorizontal();
            }

            NrInpMemFcnOld[inputID] = NrInpMemFcn[inputID];
            GUILayout.Label("", EditorStyles.boldLabel);
        }

        GUILayout.Label("", EditorStyles.boldLabel);
        DrawSeparator();

        //
        GUILayout.Label("Output Functions", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();
        GUILayout.Label("Inference  Type: ", GUILayout.Width(200));
        EditorGUILayout.Popup("", 0, new string[] { "Sugeno" }, GUILayout.Width(200)); // To Do: Implement other types. E.g.: Mamdani
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
        NrOutputFunction = Mathf.Min(Mathf.Max(EditorGUILayout.IntField("Output Number:", NrOutputFunction, GUILayout.Width(200)), 1), 3); // Limited to 3 for now
        GUILayout.Label("Min:", GUILayout.Width(50));
        minOutputFunction = EditorGUILayout.FloatField(minOutputFunction, GUILayout.Width(50));
        GUILayout.Label("Max:", GUILayout.Width(50));
        maxOutputFunction = EditorGUILayout.FloatField(maxOutputFunction, GUILayout.Width(50));
        EditorGUILayout.EndHorizontal();
        GUILayout.Space(15);
        Rect rectOut = DrawRectangle();

        defaultOutPointLists = CalculateOutPoints(NrOutputFunction, minOutputFunction, maxOutputFunction);
        if (NrOutputFunction != NrOutputFunctionOld)
        {
            pointOutLists.Clear();
            pointOutLists = defaultOutPointLists;
        }
        DrawOutputFunctions(rectOut, pointOutLists, minOutputFunction, maxOutputFunction, outputLabels.Values.ToArray());
        GUILayout.Space(rectOut.height + 50);

        for (int j = 0; j < NrOutputFunction; j++)
        {
            var outputFunctionID = j + 1;

            EditorGUILayout.BeginHorizontal();
            outputLabels[outputFunctionID] = EditorGUILayout.TextField($"Output {outputFunctionID} Label: ", outputLabels[outputFunctionID], GUILayout.Width(200));

            EditorGUILayout.LabelField("X Point: ", GUILayout.Width(100));
            if (pointOutLists.Count == 0) // Notice: This shouldn't be necessary. It's not necessary for input functions. Why?
                pointOutLists = defaultOutPointLists;
            outputFunctionPoints[outputFunctionID][0, 0] = pointOutLists[j][0].x;
            outputFunctionPoints[outputFunctionID][0, 0] = EditorGUILayout.FloatField(outputFunctionPoints[outputFunctionID][0, 0], GUILayout.Width(100));
            outputFunctionPoints[outputFunctionID][0, 1] = outputFunctionPoints[outputFunctionID][0, 1];
            pointOutLists[j][0].x = outputFunctionPoints[outputFunctionID][0, 0];
            pointOutLists[j][1].x = outputFunctionPoints[outputFunctionID][0, 1];
            EditorGUILayout.EndHorizontal();
        }



        GUILayout.Label("", EditorStyles.boldLabel);
        DrawSeparator();

        //
        GUILayout.Label("Rule Table", EditorStyles.boldLabel);
        if (NrInpMem == 2)
        {
            var ruleOptions = outputLabels.Values.ToArray(); // To Do: Convert to string only for NrOutputFunction elements
            if (rulesTable == null || NrOutputFunction != NrOutputFunctionOld) // To Do: Reset also when input funcions are updated
                rulesTable = new int[NrInpMemFcn[2], NrInpMemFcn[1]];
            if (rulesValues == null || NrOutputFunction != NrOutputFunctionOld) // To Do: Reset also when input funcions are updated
                rulesValues = new float[NrInpMemFcn[2], NrInpMemFcn[1]];
            //
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("", GUILayout.Width(100));
            for (int i = 0; i < NrInpMemFcn[1]; i++)
            {
                EditorGUILayout.LabelField("Input 1: " + membershipLabels[1][i + 1], GUILayout.Width(100));
            }
            EditorGUILayout.EndHorizontal();
            for (int i = 0; i < NrInpMemFcn[2]; i++)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField("Input 2: " + membershipLabels[2][i + 1], GUILayout.Width(100));
                for (int j = 0; j < NrInpMemFcn[1]; j++)
                {
                    //EditorGUILayout.TextField("", GUILayout.Width(100));
                    rulesTable[i, j] = EditorGUILayout.Popup("", rulesTable[i, j], ruleOptions, GUILayout.Width(100));
                    rulesValues[i, j] = outputFunctionPoints[rulesTable[i, j] + 1][0, 0];
                }
                EditorGUILayout.EndHorizontal();
            }

            NrOutputFunctionOld = NrOutputFunction;
        }

        GUILayout.Label("", EditorStyles.boldLabel);
        DrawSeparator();

        GUILayout.Label("Test", EditorStyles.boldLabel);
        if (NrInpMem == 2)
        {
            Dictionary<int, List<float>> outs = new Dictionary<int, List<float>>() {
                {1, new List<float>()}, {2, new List<float>()} };
            for (int k = 0; k < 2; k++)
            {
                var inputID = k + 1;

                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField($"Input {inputID}: ", GUILayout.Width(100));
                inputTestValues[inputID] = EditorGUILayout.FloatField(inputTestValues[inputID], GUILayout.Width(100));
                EditorGUILayout.EndHorizontal();

                float width = Mathf.Abs(maxMemFcn[inputID] - minMemFcn[inputID]);
                var normalizedTestPointAX = inputTestValues[inputID] / width;
                Vector3 testPointA = new Vector3(inputRects[inputID].xMin + normalizedTestPointAX * inputRects[inputID].width, inputRects[inputID].yMin);
                Vector3 testPointB = new Vector3(inputRects[inputID].xMin + normalizedTestPointAX * inputRects[inputID].width, inputRects[inputID].yMin + inputRects[inputID].height);
                Handles.color = Color.blue;
                Handles.DrawDottedLine(testPointA, testPointB, 5);

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

            outputTest = CalculateFuzzy(rulesValues, outs);
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Output: ", GUILayout.Width(100));
            EditorGUILayout.LabelField($"{outputTest}", GUILayout.Width(100));
            EditorGUILayout.EndHorizontal();
        }

        GUILayout.Label("", EditorStyles.boldLabel);
        DrawSeparator();
        if (GUILayout.Button("Export"))
        {
            ExportFuzzy();
        }
        GUILayout.EndScrollView();
    }

    private List<Vector3[]> CalculateOutPoints(int NrMemFcn, float minVal, float maxVal)
    {
        float width = Mathf.Abs(maxVal - minVal);
        List<Vector3[]> pointList = new List<Vector3[]> { };
        for (int i = 0; i < NrMemFcn; i++)
        {
            Vector3 pointA = new Vector3(minVal + (i + 1) * (width / (NrMemFcn + 1)), 0);
            Vector3 pointB = new Vector3(minVal + (i + 1) * (width / (NrMemFcn + 1)), 1);

            pointList.Add(new Vector3[] { pointA, pointB });
        }
        return pointList;
    }
    private void DrawOutputFunctions(Rect rect, List<Vector3[]> points, float minVal, float maxVal, string[] labels)
    {
        float width = Mathf.Abs(maxVal - minVal);
        for (int i = 0; i < points.Count; i++)
        {
            var normalizedPointX = points[i][0].x / width;
            // Calculate the triangle vertices
            Vector3 pointA = new Vector3(rect.xMin + normalizedPointX * rect.width, rect.yMin + (1 - points[i][0].y) * rect.height);
            Vector3 pointB = new Vector3(rect.xMin + normalizedPointX * rect.width, rect.yMin + (1 - points[i][1].y) * rect.height);

            Handles.color = Color.red;
            Handles.DrawLine(pointA, pointB, 5);

            Handles.color = Color.white;
            Handles.Label(new Vector3(rect.xMin + normalizedPointX * rect.width, rect.yMax + 15), $"{points[i][0].x}");

            Handles.Label(new Vector3(rect.xMin - 20, rect.yMax), "0");
            Handles.Label(new Vector3(rect.xMin - 20, rect.yMin), "1");

            Handles.Label(new Vector3(rect.xMin + normalizedPointX * rect.width, rect.yMin - 15), $"{labels[i]}");
        }
    }

    private List<Vector3[]> CalculateMemPoints(int NrMemFcn, float minVal, float maxVal)
    {
        float width = Mathf.Abs(maxVal - minVal);
        List<Vector3[]> pointList = new List<Vector3[]> { };
        for (int i = 0; i < NrMemFcn; i++)
        {
            // Calculate the triangle vertices
            Vector3 pointA = new Vector3(minVal + i * (width / (NrMemFcn + 1)), 0);
            Vector3 pointC = new Vector3(minVal + (i + 1) * (width / (NrMemFcn + 1)), 1);
            Vector3 pointB = new Vector3(maxVal - (NrMemFcn - i - 1) * (width / (NrMemFcn + 1)), 0);

            pointList.Add(new Vector3[] { pointA, pointC, pointB });
        }
        return pointList;
    }

    private void DrawMembershipFunctions(Rect rect, List<Vector3[]> points, float minVal, float maxVal, string[] labels)
    {
        float width = Mathf.Abs(maxVal - minVal);
        for (int i = 0; i < points.Count; i++)
        {
            var normalizedPoint1X = points[i][0].x / width;
            var normalizedPoint2X = points[i][2].x / width;
            var normalizedPoint3X = points[i][1].x / width;
            // Calculate the triangle vertices
            Vector3 pointA = new Vector3(rect.xMin + normalizedPoint1X * rect.width, rect.yMin + (1 - points[i][0].y) * rect.height);
            Vector3 pointB = new Vector3(rect.xMin + normalizedPoint2X * rect.width, rect.yMin + (1 - points[i][2].y) * rect.height);
            Vector3 pointC = new Vector3(rect.xMin + normalizedPoint3X * rect.width, rect.yMin + (1 - points[i][1].y) * rect.height);

            Handles.color = Color.red;
            //Handles.DrawLine(pointA, pointB);
            Handles.DrawLine(pointB, pointC, 5);
            Handles.DrawLine(pointC, pointA, 5);

            Handles.color = Color.white;
            Handles.Label(new Vector3(rect.xMin + normalizedPoint1X * rect.width, rect.yMax + 15), $"{points[i][0].x}");
            Handles.Label(new Vector3(rect.xMin + normalizedPoint2X * rect.width, rect.yMax + 15), $"{points[i][2].x}");
            Handles.Label(new Vector3(rect.xMin + normalizedPoint3X * rect.width, rect.yMax + 15), $"{points[i][1].x}");

            Handles.Label(new Vector3(rect.xMin - 20, rect.yMax), "0");
            Handles.Label(new Vector3(rect.xMin - 20, rect.yMin), "1");

            Handles.Label(new Vector3(rect.xMin + normalizedPoint3X * rect.width, rect.yMin - 15), $"{labels[i]}");
        }
    }

    private Rect DrawRectangle()
    {
        // Get the rect of the last drawn control (the label field)
        Rect lastRect = GUILayoutUtility.GetLastRect();
        // Define the rectangle area for the triangle to start right after the label field
        Rect rect = new Rect(lastRect.xMin + 20, lastRect.yMax + 10, 400, 100);

        // Draw the rectangle outline
        EditorGUI.DrawRect(rect, new Color(0, 0, 0, 0)); // transparent fill
        Handles.color = Color.black;
        Handles.DrawSolidRectangleWithOutline(rect, new Color(0, 0, 0, 0), Color.black);

        //GUILayout.Space(rect.height + 50);

        return rect;
    }
    private void DrawSeparator()
    {
        Rect lastRect = GUILayoutUtility.GetLastRect();
        Handles.color = Color.gray;
        Handles.DrawLine(new Vector3(lastRect.xMin, lastRect.yMax + 5), new Vector3(lastRect.xMax, lastRect.yMax + 5));
        GUILayout.Space(10); // Add some space after the separator
    }

    // JSON
    #region Export Json
    private void ExportFuzzy()
    {
        //// Create the data object

        List<float> outputValues = new List<float>();
        foreach (var key in outputFunctionPoints.Keys)
        {
            outputValues.Add(outputFunctionPoints[key][0, 0]);
        }
        var rulesValueTable = new List<List<float>> { };
        for (int i = 0; i < rulesValues.GetLength(0); i++)
        {
            var tmpValues = new List<float>();
            for (int j = 0; j < rulesValues.GetLength(0); j++)
            {
                tmpValues.Add(rulesValues[i, j]);
            }
            rulesValueTable.Add(tmpValues);
        }
        // Create the data object
        FuzzyData data = new FuzzyData
        {
            Output = new FuzzyOutput
            {
                Values = outputValues,
                Labels = outputLabels.Values.ToList(),
                RuleTable = rulesValueTable
            }
        };

        // Add dynamic inputs
        for (int i = 0; i < NrInpMem; i++)
        {
            var inputID = i + 1;
            AddInput(data, $"Input{inputID}");
            for (int j = 0; j < NrInpMemFcn[inputID]; j++)
            {
                var membershipID = j + 1;
                var xPoints = new List<float> { };
                var yPoints = new List<float> { };
                for (int m = 0; m < membershipPoints[inputID][membershipID].GetLength(1); m++)
                {
                    xPoints.Add(membershipPoints[inputID][membershipID][0, m]);
                    yPoints.Add(membershipPoints[inputID][membershipID][1, m]);
                }
                AddMembership(data.Inputs[$"Input{inputID}"], $"Membership{membershipID}", xPoints, yPoints, membershipLabels[inputID][membershipID]);
            }
        }

        //// Populate Input1
        //AddMembership(data.Inputs["Input1"], "Membership1", new List<float> { 0, 10, 20 }, new List<float> { 1, 1, 0 }, "Yavas");
        //AddMembership(data.Inputs["Input1"], "Membership2", new List<float> { 10, 20, 30 }, new List<float> { 0, 1, 0 }, "Orta");
        //AddMembership(data.Inputs["Input1"], "Membership3", new List<float> { 20, 30, 40 }, new List<float> { 0, 1, 1 }, "Hizli");

        //// Populate Input2
        //AddMembership(data.Inputs["Input2"], "Membership1", new List<float> { 0, 1, 2 }, new List<float> { 1, 1, 0 }, "Kucuk");
        //AddMembership(data.Inputs["Input2"], "Membership2", new List<float> { 1, 2, 3 }, new List<float> { 0, 1, 0 }, "Orta");
        //AddMembership(data.Inputs["Input2"], "Membership3", new List<float> { 2, 3, 4 }, new List<float> { 0, 1, 1 }, "Buyuk");

        // Serialize the data object to JSON
        string json = JsonConvert.SerializeObject(data, Formatting.Indented);

        // Write JSON to a file
        System.IO.File.WriteAllText("Assets//FuzzyInference.json", json);

        Debug.Log("JSON file written to " + "Assets//FuzzyInference.json");
    }
    // Method to add a new Input
    void AddInput(FuzzyData data, string inputName)
    {
        data.Inputs[inputName] = new InputFuzzy();
    }

    // Method to add a membership to an Input object
    void AddMembership(InputFuzzy input, string key, List<float> xPoints, List<float> yPoints, string label)
    {
        input.Memberships[key] = new Membership
        {
            xPoints = xPoints,
            yPoints = yPoints,
            Label = label
        };
    }
    #endregion

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
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
