using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Membership
{
    public List<float> xPoints;
    public List<float> yPoints;
    public string Label;
}

[System.Serializable]
public class InputFuzzy
{
    public Dictionary<string, Membership> Memberships = new Dictionary<string, Membership>();
}

[System.Serializable]
public class FuzzyOutput
{
    public List<float> Values;
    public List<string> Labels;
    public List<List<float>> RuleTable;
}

[System.Serializable]
public class FuzzyData
{
    public Dictionary<string, InputFuzzy> Inputs = new Dictionary<string, InputFuzzy>();
    public FuzzyOutput Output;
}
