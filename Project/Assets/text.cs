using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using Uduino;
using UnityEngine.UI;
public class text : MonoBehaviour
{
    public Text trytext;
    SerialPort sp = new SerialPort("COM4", 9600);
    string DebugLoge;
    Queue myLoge = new Queue();
    // Start is called before the first frame update
    void Start()
    {
        sp.Open();

        sp.ReadTimeout = 1;
        Debug.Log(sp.NewLine);
     
    }
    private void OnEnable()
    {
        // Application.logMessageReceived += ILogHandler;

    }
}

   