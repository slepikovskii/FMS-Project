using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;
using Uduino;
public class SMTHG : MonoBehaviour
{
    public string imuName = "r";
    public UduinoManager arduino;
    public text stext;
    SerialPort sp = new SerialPort("COM4", 9600);
    // Start is called before the first frame update
    void Start()
    {
        UduinoManager.Instance.pinMode(AnalogPin.A0, PinMode.Output);
    }

    // Update is called once per frame
    void Update()
    {
        int Value = UduinoManager.Instance.analogRead(AnalogPin.A0);
        Debug.Log(Value);
        UduinoManager.Instance.sendCommand("go");

    }
}
