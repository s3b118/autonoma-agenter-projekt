using UnityEngine;

public class LocationRandomizer : MonoBehaviour
{
    private Vector3 originalPosition;
    private Quaternion originalRotation;

    private void Awake()
    {
        originalPosition = transform.position;
        originalRotation = transform.rotation;
        RandomizePosition();
        RandomizeRotation();
    }

    public void RandomizePosition()
    {
        transform.position = new Vector3(
            originalPosition.x = Random.Range(5f, 495f),
            originalPosition.y,
            originalPosition.z = Random.Range(5f, 495f)
        );
    }

    public void RandomizeRotation()
    {
        transform.rotation = originalRotation * Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
    }
}
