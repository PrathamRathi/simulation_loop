using System;
using System.Xml;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetBoundingBox : MonoBehaviour
{
	public string label;
	public Transform topLeft, topRight, bottomLeft, bottomRight;
	public Camera cam;
	public Texture aTexture;
	private float[] boundingBox;
	public float cutoffFraction;
    public int screenShotCount;

	public float[] getBounds()
	{
		return boundingBox;
	}
    
	// Start is called before the first frame update
	void Start()
    {
		boundingBox = new float[4];
		ScreenCapture.CaptureScreenshot("test");
		cutoffFraction = 3f;
		screenShotCount = 0;

	}

    public Boolean isInFrame()
	{
		Boolean above = false;
		Boolean right = false;
		Boolean below = false;
		Boolean left = false;
		if (boundingBox[0] == 0 && boundingBox[2] == 0) left = true;
		if (boundingBox[0] == Screen.width && boundingBox[2] == Screen.width) right = true;
		if (boundingBox[1] == 0 && boundingBox[3] == 0) above = true;
		if (boundingBox[1] == Screen.height && boundingBox[3] == Screen.height) below = true;

		if (boundingBox[0] == 0 && boundingBox[2] == Screen.width) return false;
		if (boundingBox[0] == Screen.width && boundingBox[2] == 0) return false;
		if (boundingBox[1] == 0 && boundingBox[3] == Screen.height) return false;
		if (boundingBox[1] == Screen.height && boundingBox[2] == 0) return false;

		if ((above && right) || (below && right) || (below && left) || (above && left))
		{
			return false;
		}
		else return true;
	}
	// Update is called once per frame
	void FixedUpdate()
	{
		// get initial positions of points
		Vector3 SPTopLeft = cam.WorldToScreenPoint(topLeft.position);
		Vector3 SPTopRight = cam.WorldToScreenPoint(topRight.position);
		Vector3 SPBottomLeft = cam.WorldToScreenPoint(bottomLeft.position);
		Vector3 SPBottomRight = cam.WorldToScreenPoint(bottomRight.position);

		// find min/max x/y values
		if (SPTopLeft.x < SPBottomLeft.x) {
			boundingBox[0] = SPTopLeft.x;
		} else {
			boundingBox[0] = SPBottomLeft.x;
		}
		if (SPBottomLeft.y < SPBottomRight.y) {
			boundingBox[1] = SPBottomLeft.y;
		} else {
			boundingBox[1] = SPBottomRight.y;
		}
		if (SPTopRight.x > SPBottomRight.x) {
			boundingBox[2] = SPTopRight.x;
		} else {
			boundingBox[2] = SPBottomRight.x;
		}
		if (SPTopLeft.y > SPTopRight.y) {
			boundingBox[3] = SPTopLeft.y;
		} else {
			boundingBox[3] = SPTopRight.y;
		}

		// move origin to top left
		float temp = boundingBox[1];
		boundingBox[1] = Screen.height - boundingBox[3];
		boundingBox[3] = Screen.height - temp;

		// cut off values that are beyond screen dimensions
		if (boundingBox[0] < 0f) boundingBox[0] = 0f;
		if (boundingBox[0] > Screen.width) boundingBox[0] = Screen.width;
		if (boundingBox[1] < 0f) boundingBox[1] = 0f;
		if (boundingBox[1] > Screen.height) boundingBox[1] = Screen.height;
		if (boundingBox[2] > Screen.width) boundingBox[2] = Screen.width;
		if (boundingBox[2] < 0f) boundingBox[2] = 0f;
		if (boundingBox[3] > Screen.height) boundingBox[3] = Screen.height;
		if (boundingBox[3] < 0f) boundingBox[3] = 0f;
        /*
        Debug.Log("1: "+((boundingBox[0] > Screen.width / cutoffFraction) && boundingBox[0] < Screen.width - Screen.width/cutoffFraction)
            + " 2: "+ ((boundingBox[2] < (Screen.width - Screen.width / cutoffFraction)) && boundingBox[2] > Screen.width / cutoffFraction)
			+ " 3: "+ (boundingBox[2] - boundingBox[0] > Screen.width / cutoffFraction)
            + " 4: "+ ((boundingBox[1] > Screen.height / cutoffFraction) && boundingBox[1] < Screen.height - Screen.height/cutoffFraction)
            + " 5: "+ ((boundingBox[3] < (Screen.height - Screen.height / cutoffFraction)) && boundingBox[3] > Screen.height / cutoffFraction)
			+ " 6: "+ (boundingBox[3] - boundingBox[1] > Screen.height / cutoffFraction));
            
        // checks if object is within 1/cutoffFraction of the screen dimensions ie; at least 1/3 of the screen's width and height is covered.
		if (
            (
               (boundingBox[0] > Screen.width / cutoffFraction && boundingBox[0] < Screen.width - Screen.width / cutoffFraction)
            || (boundingBox[2] < Screen.width - Screen.width/cutoffFraction && boundingBox[2] > Screen.width/cutoffFraction)
            || (boundingBox[2]-boundingBox[0] > Screen.width/cutoffFraction)
            )
            &&
            (
               (boundingBox[1] > Screen.height / cutoffFraction && boundingBox[1] < Screen.height - Screen.height / cutoffFraction)
            || (boundingBox[3] < Screen.height - Screen.height / cutoffFraction && boundingBox[3] > Screen.height/cutoffFraction)
			|| (boundingBox[3] - boundingBox[1] > Screen.height / cutoffFraction)
            )
           )
		{
			Debug.Log("Would include data!");
			if (screenShotCount < 1)
			{
				string[] lines = { "xmin: " + boundingBox[0] / Screen.width, "ymin: " + boundingBox[1] / Screen.height, "xmax: " + boundingBox[2] / Screen.width, "ymax:" + boundingBox[3] / Screen.height };
                
				// Set a variable to the Documents path.
				//string docPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
				String docPath = Application.dataPath;
				if (!File.Exists(Path.Combine(docPath, "Images"))) Directory.CreateDirectory(Path.Combine(docPath, "Images"));
				// Write the string array to a new file named "WriteLines.txt".
				long ms = DateTimeOffset.Now.ToUnixTimeMilliseconds();
				using (StreamWriter outputFile = new StreamWriter(Path.Combine(docPath, ms + ".xml")))
				{
					foreach (string line in lines)
						outputFile.WriteLine(line);
				}
                
				XmlTextWriter textWriter = new XmlTextWriter(Path.Combine(Path.Combine(docPath,"Images"), ms + ".xml"), null);
				textWriter.WriteStartDocument();
				// Write first element  
				textWriter.WriteStartElement("annotation");
				textWriter.WriteStartElement("folder");
				textWriter.WriteString("Images");
				textWriter.WriteEndElement();
				textWriter.WriteStartElement("size");
				textWriter.WriteStartElement("width");
				textWriter.WriteString(""+Screen.width);
				textWriter.WriteEndElement();
				textWriter.WriteStartElement("height");
				textWriter.WriteString(""+Screen.height);
				textWriter.WriteEndElement();
				textWriter.WriteEndElement();
				// Write next element  
				textWriter.WriteStartElement("Name", "");
				textWriter.WriteString("Student");
				textWriter.WriteEndElement();
				// Write one more element  
				textWriter.WriteStartElement("Address", "");
				textWriter.WriteString("Colony");
                
				textWriter.WriteEndElement();
				textWriter.WriteEndDocument();
				// close writer  
				textWriter.Close();
				screenShotCount++;
                
			}
			
            
		}
		Debug.Log("xmin: " + boundingBox[0] + " ymin: " + boundingBox[1] +
			" xmax: " + boundingBox[2] + " ymax: " + boundingBox[3] + " Screen width: " + Screen.width
            + " Screen Height: " + Screen.height + " Box height: "+(boundingBox[3]-boundingBox[1])+" Box Width: "
            + (boundingBox[2]-boundingBox[0]));
        
		Debug.Log("BL: "+SPBottomLeft.x+", "+SPBottomLeft.y
            + "BR: " + SPBottomRight.x + ", " + SPBottomRight.y
            + "TL: " + SPTopLeft.x + ", " + SPTopLeft.y
            + "TR: " + SPTopRight.x + ", " + SPTopRight.y);
        */    
	}
	void OnGUI()
	{
		if (Event.current.type.Equals(EventType.Repaint))
		{
            // testing texture for evaluating points
			//Graphics.DrawTexture(new Rect(boundingBox[0], boundingBox[1], (boundingBox[2] - boundingBox[0]), (boundingBox[3] - boundingBox[1])), aTexture);
			//Graphics.DrawTexture(new Rect(30, 10, 100, 50), aTexture);

		}
	}
}