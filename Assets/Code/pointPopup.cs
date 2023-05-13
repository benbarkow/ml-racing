using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pointPopup : MonoBehaviour
{

    public float DecayTime = 5f;
    public float heightOffset = 1f;
    private float durationLeft = 20f;
    private float initialDurationLeft = 20f;

    private TextMesh textMesh;

    public void display(float reward){
        reward *= 100f;
        
        this.transform.position += Vector3.up * heightOffset;

        textMesh = GetComponent<TextMesh>();
        textMesh.text = Mathf.RoundToInt(reward) + "";
        textMesh.color = Color.Lerp(Color.yellow, Color.green, reward / 100f);
    }

    // Update is called once per frame
    void Update()
    {
        this.transform.rotation = Camera.main.transform.rotation;
        this.transform.position += Vector3.up * Time.deltaTime * 0.5f;
        this.textMesh.color = new Color(this.textMesh.color.r, this.textMesh.color.g, this.textMesh.color.b, durationLeft / initialDurationLeft);

        if(durationLeft >= DecayTime) durationLeft -= DecayTime * Time.deltaTime;
        else Destroy(this.gameObject);
    }
}
