using System.IO;
using UnityEngine;
 
public class customSnapshotSaver : MonoBehaviour {
    void Start(){
        Debug.Log("testing");
        ScreenCapture.CaptureScreenshot("/home/pratham-rathi/Unity-Robotics-Hub/ROSTesting/Assets/Images");
        onMouseDown();
    }
    void onMouseDown(){
        Debug.Log("mouse testing");
        ScreenCapture.CaptureScreenshot("/home/pratham-rathi/Unity-Robotics-Hub/ROSTesting/Assets/Images");
    }
 
    // public int FileCounter = 0;
 
    // private void Start()
    // {
    //     Debug.Log("testing");
    //     Update();
    // }
 
    // void Update () {
    //     if (Input.GetKeyDown(KeyCode.F9))
    //     {
    //         CamCapture();  
    //     }
    // }
    // void CamCapture()
    // {
    //     Camera Cam = GetComponent<Camera>();
 
    //     RenderTexture currentRT = RenderTexture.active;
    //     RenderTexture.active = Cam.targetTexture;
 
    //     Cam.Render();
 
    //     Texture2D Image = new Texture2D(Cam.targetTexture.width, Cam.targetTexture.height);
    //     Image.ReadPixels(new Rect(0, 0, Cam.targetTexture.width, Cam.targetTexture.height), 0, 0);
    //     Image.Apply();
    //     RenderTexture.active = currentRT;
 
    //     var Bytes = Image.EncodeToPNG();
    //     Destroy(Image);
 
    //     File.WriteAllBytes("/home/pratham-rathi/Unity-Robotics-Hub/ROSTesting/Assets/Images" + FileCounter + ".png", Bytes);
    //     FileCounter++;
    // }
   
}
