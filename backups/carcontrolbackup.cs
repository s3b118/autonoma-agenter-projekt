using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CarControl : Agent
{
    public float enginePower = 2000.0f; // Engine power
    public float turnSpeed = 25.0f; // Maximum turn speed
    public float turnSmoothness = 5.0f; // Turn smoothness
    public int maxEpisodeSteps = 3000;
    public Transform[] wheels; // Array of Wheel Collider components
    public Transform[] wheelMeshes; // Array of wheel meshes
    public Transform centerOfMass; // Center of Mass point
    public GameObject steeringWheel;

    private Rigidbody body; // Car's Rigidbody component
    private float currentTurnAngle = 0.0f;
    private float horizontalInput;
    private float throttle;
    private GameObject target;
    private ProceduralTerrain terrainGenerator;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private bool episodeEnded;
    private bool isSpawning;
    private float previousTargetDistance;

    private const float TargetDistanceScale = 750.0f;
    private const float ProgressRewardScale = 0.002f;

    public override void Initialize()
    {
        Debug.Log("Test initialize");
        body = GetComponent<Rigidbody>();
        MaxStep = maxEpisodeSteps;
        if (centerOfMass != null)
        {
         body.centerOfMass = centerOfMass.localPosition; // Set center of mass
        }

        target = GameObject.FindGameObjectWithTag("Target");
        terrainGenerator = FindAnyObjectByType<ProceduralTerrain>();
        initialPosition = transform.position;
        initialRotation = transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        Reset();
    }

    public void Reset()
    {
        isSpawning = true;
        episodeEnded = false;
        horizontalInput = 0.0f;
        throttle = 0.0f;
        currentTurnAngle = 0.0f;

        Collider[] truckColliders = GetComponentsInChildren<Collider>();
        Collider[] targetColliders = target != null ? target.GetComponentsInChildren<Collider>() : new Collider[0];

        do
        {
            Vector3 truckSpawn = GetRandomSpawnPosition(initialPosition.y);
            Quaternion randomRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f) * initialRotation;
            transform.SetPositionAndRotation(truckSpawn, randomRotation);

            if (target != null)
            {
                target.transform.position = GetRandomSpawnPosition(target.transform.position.y);
            }

            Physics.SyncTransforms();
        }
        while (target != null && CollidersOverlap(truckColliders, targetColliders));

        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
        previousTargetDistance = GetTargetDistance();
        isSpawning = false;
        Debug.Log("Episode started. Car reset to initial position.");
    }

    private Vector3 GetRandomSpawnPosition(float fallbackY)
    {
        return terrainGenerator != null
            ? terrainGenerator.GetRandomPosition()
            : new Vector3(Random.Range(5f, 495f), fallbackY, Random.Range(5f, 495f));
    }

    private bool CollidersOverlap(Collider[] first, Collider[] second)
    {
        foreach (Collider firstCollider in first)
        {
            if (!firstCollider.enabled)
            {
                continue;
            }

            foreach (Collider secondCollider in second)
            {
                if (secondCollider.enabled && firstCollider.bounds.Intersects(secondCollider.bounds))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private float GetTargetDistance()
    {
        return target != null ? Vector3.Distance(transform.position, target.transform.position) : 0.0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (target != null)
        {
            Vector3 relativeTargetPosition = transform.InverseTransformPoint(target.transform.position);
            sensor.AddObservation(Vector3.ClampMagnitude(relativeTargetPosition, TargetDistanceScale) / TargetDistanceScale);
            sensor.AddObservation(Mathf.Clamp01(relativeTargetPosition.magnitude / TargetDistanceScale));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0.0f);
        }

        sensor.AddObservation(Vector3.ClampMagnitude(transform.InverseTransformDirection(body.linearVelocity), 20.0f) / 20.0f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        horizontalInput = Mathf.Clamp(actions.ContinuousActions[0], -1.0f, 1.0f);
        throttle = Mathf.Clamp(actions.ContinuousActions[1], -1.0f, 1.0f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        float steering = 0.0f;
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            steering = -1.0f;
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            steering = 1.0f;
        }

        float acceleration = 0.0f;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            acceleration = 1.0f;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            acceleration = -1.0f;
        }

        continuousActionsOut[0] = steering;
        continuousActionsOut[1] = acceleration;
    }

    private void FixedUpdate()
    {
        if (episodeEnded)
        {
            return;
        }

        float centerOfMassHeight = centerOfMass != null ? centerOfMass.position.y : transform.position.y;
        if (centerOfMassHeight < -1.0f)
        {
            Debug.Log("Car off map. -1");
            EndEpisodeWithReward(-1.0f);
            return;
        }

        if (target != null)
        {
            float targetDistance = GetTargetDistance();
            AddReward((previousTargetDistance - targetDistance) * ProgressRewardScale);
            previousTargetDistance = targetDistance;
        }

        float targetTurnAngle = horizontalInput * turnSpeed;
        currentTurnAngle = Mathf.Lerp(currentTurnAngle, targetTurnAngle, Time.fixedDeltaTime * turnSmoothness);

        if (steeringWheel != null)
        {
            steeringWheel.transform.localEulerAngles = new Vector3(-64, 0, currentTurnAngle * 3);
        }

        for (int i = 0; i < wheels.Length; i++)
        {
            WheelCollider wheelCollider = wheels[i].GetComponent<WheelCollider>();
            if (i < 2) // First two wheels steer
            {
                wheelCollider.steerAngle = currentTurnAngle;
            }
            else
            {
                wheelCollider.steerAngle = 0f; // Other two wheels remain straight
            }

            wheelCollider.motorTorque = throttle * enginePower;

            Quaternion wheelRotation;
            Vector3 wheelPosition;
            wheelCollider.GetWorldPose(out wheelPosition, out wheelRotation);
            wheelMeshes[i].position = wheelPosition;
            wheelMeshes[i].rotation = wheelRotation;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (isSpawning)
        {
            return;
        }

        if (other.CompareTag("Target"))
        {
            Debug.Log("Target reached. +1");
            EndEpisodeWithReward(1.0f);
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (isSpawning)
        {
            return;
        }

        if (collision.gameObject.CompareTag("Building"))
        {   
            Debug.Log("Collision with building. -1");
            EndEpisodeWithReward(-1.0f);
        }

        else if (collision.gameObject.CompareTag("Rock"))
        {
            Debug.Log("Collision with rock. -1");
            EndEpisodeWithReward(-1.0f);
        }
    }

    private void EndEpisodeWithReward(float reward)
    {
        if (episodeEnded || isSpawning)
        {
            return;
        }

        episodeEnded = true;

        AddReward(reward);
        EndEpisode();
    }
}