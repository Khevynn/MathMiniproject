using System;
using UnityEngine;

public enum Rotation3D
{
    Euler, Quaternions
}
public enum RotationAxis
{
    None,
    X,
    Y,
    Z
}

public class Object : MonoBehaviour
{
    private Mesh mesh;
    private Vector3[] vertices;
    private Vector3[] normals;

    public Rotation3D rotation3d;
    public RotationAxis rotationAxis;

    public float xRotation;
    public float yRotation;
    public float zRotation;

    // Increment to work with only 1 angle at a time
    public float angleIncrement;

    // Increment in the angle rotation for X, Y and Z
    private float incrementX;
    private float incrementY;
    private float incrementZ;

    // Start is called before the first frame update
    void Start()
    {
        // Create mesh
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        mesh.name = "MyMesh";

        // Define vertices manually
        vertices = new Vector3[]
        {
            // Bottom vertices
            new Vector3(0, 0, 5),       // 0
            new Vector3(5f, 0, 2.5f), // 1
            new Vector3(5f, 0, -2.5f),// 2
            new Vector3(0, 0, -5),      // 3
            new Vector3(-5f, 0, -2.5f),// 4
            new Vector3(-5f, 0, 2.5f),// 5
            // Top vertices
            new Vector3(0, 10, 5),      // 6
            new Vector3(5f, 10, 2.5f),// 7
            new Vector3(5f, 10, -2.5f),// 8
            new Vector3(0, 10, -5),     // 9
            new Vector3(-5f, 10, -2.5f),// 10
            new Vector3(-5f, 10, 2.5f), // 11
            // Top center vertex
            new Vector3(0, 10, 0),      // 12
            new Vector3(0, 0, 0)       // 13
        };

        // Define triangles manually
        int[] triangles = new int[]
        {
            // Bottom face
            0, 5, 1,
            0, 5, 2,
            1, 5, 4,
            1, 4, 2,
            2, 5, 3,
            2, 5, 4,
            3, 4, 5,
            3, 5, 2,
            4, 3, 5,
            4, 1, 5,
            5, 1, 3,
            5, 1, 4,

            // Side faces
            0, 1, 6,
            1, 7, 6,
            1, 2, 7,
            2, 8, 7,
            2, 3, 8,
            3, 9, 8,
            3, 4, 9,
            4, 10, 9,
            4, 5, 10,
            5, 11, 10,
            5, 0, 11,
            0, 6, 11,

            // Top face
            6, 7, 12,
            7, 8, 12,
            8, 9, 12,
            9, 10, 12,
            10, 11, 12,
            11, 6, 12
        };

        // Update mesh
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;

        mesh.RecalculateNormals();

        MeshRenderer meshRenderer = gameObject.AddComponent<MeshRenderer>();

        Material material = new Material(Shader.Find("Standard"));
        material.color = Color.green;

        meshRenderer.material = material;
    }

    // Update is called once per frame
    void Update()
    {
        switch (rotation3d)
        {
            case Rotation3D.Euler:
                {
                    RotateAnglesEuler();
                    break;
                }
            case Rotation3D.Quaternions:
                {
                    RotateAnglesQuaternions();
                    break;
                }
            default: break;
        }
        
    }

    void RotateAnglesEuler()
    {
        switch (rotationAxis)
        {
            case RotationAxis.X:
                {
                    incrementX = angleIncrement;
                    break;
                }
            case RotationAxis.Y:
                {
                    incrementY = angleIncrement;
                    break;
                }
            case RotationAxis.Z:
                {
                    incrementZ = angleIncrement;
                    break;
                }
            default: break;
        }
        // Increase angles continuously
        xRotation += incrementX;
        yRotation += incrementY;
        zRotation += incrementZ;
        Reset();
        Rotate3DEuler(xRotation * Mathf.Deg2Rad, yRotation * Mathf.Deg2Rad, zRotation * Mathf.Deg2Rad);
    }

    void RotateAnglesQuaternions()
    {
        Reset();
        Rotate3DQuaternion(xRotation * Mathf.Deg2Rad, yRotation * Mathf.Deg2Rad, zRotation * Mathf.Deg2Rad);
    }
    void Rotate3DEuler(float x, float y, float z)
    {
        // Keep angle between 0 and 360
        xRotation = xRotation % 360;
        yRotation = yRotation % 360;
        zRotation = zRotation % 360;

        // Create individual rotation matrices for X, Y, and Z axes
        Matrix4x4 rxmat = RotXMat(x);
        Matrix4x4 rymat = RotYMat(y);
        Matrix4x4 rzmat = RotZMat(z);

        Matrix4x4 combinedRotation = rxmat * rymat * rzmat;

        // Apply the combined rotation to each vertex and normal
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = combinedRotation.MultiplyPoint(vertices[i]);
        }
        mesh.RecalculateNormals();

        // Update mesh with modified vertices and normals
        mesh.vertices = vertices;
        mesh.normals = normals;
    }


    void Rotate3DQuaternion(float x, float y, float z)
    {
        float[] qX = QuaternionFromAxisAngle(new float[] { 1, 0, 0 }, x);
        float[] qY = QuaternionFromAxisAngle(new float[] { 0, 1, 0 }, y);
        float[] qZ = QuaternionFromAxisAngle(new float[] { 0, 0, 1 }, z);

        // Calcula o quaternion combinado
        float[] combinedQuaternion = MultiplyQuaternion(MultiplyQuaternion(qX, qY), qZ);

        // Aplica a rotação a cada vértice
        for (int i = 0; i < vertices.Length; i++)
        {
            float[] qVertice = new float[] { 0, vertices[i].x, vertices[i].y, vertices[i].z };
            float[] rotatedVertex = MultiplyQuaternion(MultiplyQuaternion(combinedQuaternion, qVertice), InverseQuaternion(combinedQuaternion));

            vertices[i].x = rotatedVertex[1];
            vertices[i].y = rotatedVertex[2];
            vertices[i].z = rotatedVertex[3];
        }

        mesh.vertices = vertices;
        mesh.normals = normals;
    } 


    float[] QuaternionFromAxisAngle(float[] axis, float angle)
    {
        float[] quaternion = new float[4];
        float halfAngle = angle / 2;
        float sinHalfAngle = Mathf.Sin(halfAngle);
        quaternion[0] = Mathf.Cos(halfAngle);
        quaternion[1] = axis[0] * sinHalfAngle;
        quaternion[2] = axis[1] * sinHalfAngle;
        quaternion[3] = axis[2] * sinHalfAngle;
        return quaternion;
    }
    float[] MultiplyQuaternion(float[] quaternion1, float[] quaternion2)
    {
        float[] multiplication = new float[4];
        multiplication[0] = quaternion1[0] * quaternion2[0] - quaternion1[1] * quaternion2[1] - quaternion1[2] * quaternion2[2] - quaternion1[3] * quaternion2[3];
        multiplication[1] = quaternion1[0] * quaternion2[1] + quaternion1[1] * quaternion2[0] + quaternion1[2] * quaternion2[3] + quaternion1[3] * quaternion2[2];
        multiplication[2] = quaternion1[0] * quaternion2[2] - quaternion1[1] * quaternion2[3] + quaternion1[2] * quaternion2[0] + quaternion1[3] * quaternion2[1];
        multiplication[3] = quaternion1[0] * quaternion2[3] + quaternion1[1] * quaternion2[2] - quaternion1[2] * quaternion2[1] + quaternion1[3] * quaternion2[0];
        return multiplication;
    }
    float[] InverseQuaternion(float[] quaternion)
    {
        return new float[] { quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3] };
    }



    void Reset()
    {
        vertices = new Vector3[]
        {
            // Bottom vertices
            new Vector3(0, 0, 5),       // 0
            new Vector3(5f, 0, 2.5f), // 1
            new Vector3(5f, 0, -2.5f),// 2
            new Vector3(0, 0, -5),      // 3
            new Vector3(-5f, 0, -2.5f),// 4
            new Vector3(-5f, 0, 2.5f),// 5
            // Top vertices
            new Vector3(0, 10, 5),      // 6
            new Vector3(5f, 10, 2.5f),// 7
            new Vector3(5f, 10, -2.5f),// 8
            new Vector3(0, 10, -5),     // 9
            new Vector3(-5f, 10, -2.5f),// 10
            new Vector3(-5f, 10, 2.5f), // 11
            // Top center vertex
            new Vector3(0, 10, 0),      // 12
            new Vector3(0, 0, 0)       // 13
        };
    }

    Matrix4x4 RotXMat(float angle)
    {
        Matrix4x4 rxmat = new Matrix4x4();
        rxmat.SetRow(0, new Vector4(1f, 0f, 0f, 0f));
        rxmat.SetRow(1, new Vector4(0f, Mathf.Cos(angle), -Mathf.Sin(angle), 0f));
        rxmat.SetRow(2, new Vector4(0f, Mathf.Sin(angle), Mathf.Cos(angle), 0f));
        rxmat.SetRow(3, new Vector4(0f, 0f, 0f, 1f));

        return rxmat;
    }

    Matrix4x4 RotYMat(float angle)
    {
        Matrix4x4 rymat = new Matrix4x4();
        rymat.SetRow(0, new Vector4(Mathf.Cos(angle), 0f, Mathf.Sin(angle), 0f));
        rymat.SetRow(1, new Vector4(0f, 1f, 0f, 0f));
        rymat.SetRow(2, new Vector4(-Mathf.Sin(angle), 0f, Mathf.Cos(angle), 0f));
        rymat.SetRow(3, new Vector4(0f, 0f, 0f, 1f));

        return rymat;
    }

    Matrix4x4 RotZMat(float angle)
    {
        Matrix4x4 rzmat = new Matrix4x4();
        rzmat.SetRow(0, new Vector4(Mathf.Cos(angle), -Mathf.Sin(angle), 0f, 0f));
        rzmat.SetRow(1, new Vector4(Mathf.Sin(angle), Mathf.Cos(angle), 0f, 0f));
        rzmat.SetRow(2, new Vector4(0f, 0f, 1f, 0f));
        rzmat.SetRow(3, new Vector4(0f, 0f, 0f, 1f));

        return rzmat;
    }

    private void OnDrawGizmos()
    {
        if (vertices == null || normals == null) return;
        for (int i = 0; i < vertices.Length; i++)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(vertices[i], normals[i]);
        }
    }

}
