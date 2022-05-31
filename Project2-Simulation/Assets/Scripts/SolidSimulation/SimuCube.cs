// !!!!!!!!!!!!!!!!!!!
// 姓名：张启哲
// 学号：1900011638
// !!!!!!!!!!!!!!!!!!!

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Matrix4x4Extension
{
    public static Matrix4x4 Add(this Matrix4x4 lhs, Matrix4x4 rhs)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        for (int i = 0; i < 4; ++i)
            ret.SetColumn(i, lhs.GetColumn(i) + rhs.GetColumn(i));

        return ret;
    }

    public static Matrix4x4 Substract(this Matrix4x4 lhs, Matrix4x4 rhs)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        for (int i = 0; i < 4; ++i)
            ret.SetColumn(i, lhs.GetColumn(i) - rhs.GetColumn(i));

        return ret;
    }

    public static Matrix4x4 Negative(this Matrix4x4 lhs)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        for (int i = 0; i < 4; ++i)
            ret.SetColumn(i, -lhs.GetColumn(i));

        return ret;
    }

    public static Matrix4x4 Multiply(this Matrix4x4 lhs, float s)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        for (int i = 0; i < 4; ++i)
            ret.SetColumn(i, lhs.GetColumn(i) * s);

        return ret;
    }
    public static Matrix4x4 Multiply(this float s, Matrix4x4 lhs)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        for (int i = 0; i < 4; ++i)
            ret.SetColumn(i, s * lhs.GetColumn(i));

        return ret;
    }

    public static Matrix4x4 Multiply(this Matrix4x4 lhs, Matrix4x4 rhs)
    {
        return lhs * rhs;
    }

    public static float Trace(this Matrix4x4 lhs)
    {
        return lhs.m00 + lhs.m11 + lhs.m22;
    }


    public static Vector3 Add3(this Vector3 lhs, Vector4 rhs)
    {
        return new Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }
    public static Vector3 Add3(this Vector4 lhs, Vector3 rhs)
    {
        return new Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }
    public static Vector3 Add3(this Vector4 lhs, Vector4 rhs)
    {
        return new Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }
    public static Vector3 Add3(this Vector3 lhs, Vector3 rhs)
    {
        return lhs + rhs;
    }

    public static Vector3 AddAssign3(ref this Vector3 lhs, Vector4 rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        return lhs;
    }
    public static Vector3 AddAssign3(ref this Vector4 lhs, Vector3 rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        return lhs;
    }
    public static Vector3 AddAssign3(ref this Vector4 lhs, Vector4 rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        return lhs;
    }
    public static Vector3 AddAssign3(ref this Vector3 lhs, Vector3 rhs)
    {
        return lhs += rhs;
    }
}


public class SimuCube : MonoBehaviour
{
    // 杨氏模量
    public float youngModulus = 1e6f;

    // 泊松比
    public float possionRatio = 0.47f;

    // 密度
    public float density = 1000f;

    // 重力加速度
    public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);

    // 时间步
    public float simuTimeStep = 0.001f;

    // 最大仿真次数, 防止过于卡顿
    public int maxSimuSteps = -1;


    // size
    readonly float cubeSize = 1.0f;
    readonly int subdivide = 2;

    float[] masses;
    Vector3[] positions;
    Vector3[] originalPositions;
    Vector3[] velocity;
    Vector3[] forces;

    // ground
    GameObject groundObject;

    // mesh
    MeshFilter mesh;
    int[] meshVertexMap;
    int numVertices = 0;

    Matrix4x4 identityMatrix = Matrix4x4.identity;

    private class Tetrahedron
    {
        public int[] vi = new int[4];
        public float volume = 1.0f / 6.0f;
        public Matrix4x4 Dm = Matrix4x4.identity;
        public Matrix4x4 Bm = Matrix4x4.identity;
    }
    Tetrahedron[] tets;
    int numTets = 0;
    int[,] division = new int[6, 4] { {0, 1, 2, 5}, {0, 2, 3, 4}, {0, 2, 4, 5}, {2, 3, 4, 6}, {2, 4, 5, 6}, {3, 4, 6, 7} };

    // 拉梅参数
    float mu = 0;
    float lmbda = 0;

    // 弹性系数
    private float muN = 0.5f;
    // 摩擦系数
    private float muT = 0.5f;

    // 判断方块是否处于稳定状态
    int stableTime = 0;
    // 播放模式
    int playMode = 0;
    // FEM仿真模式
    int FEMMode = 0;
    // 键盘控制锁
    int keyControlTime = 0;


    // 用于计算形变的函数
    Matrix4x4 D(int i){
        int[] vi = tets[i].vi;
        Matrix4x4 ret = Matrix4x4.zero;
        ret[0, 0] = positions[vi[1]].x - positions[vi[0]].x;
        ret[1, 0] = positions[vi[1]].y - positions[vi[0]].y;
        ret[2, 0] = positions[vi[1]].z - positions[vi[0]].z;
        ret[0, 1] = positions[vi[2]].x - positions[vi[0]].x;
        ret[1, 1] = positions[vi[2]].y - positions[vi[0]].y;
        ret[2, 1] = positions[vi[2]].z - positions[vi[0]].z;
        ret[0, 2] = positions[vi[3]].x - positions[vi[0]].x;
        ret[1, 2] = positions[vi[3]].y - positions[vi[0]].y;
        ret[2, 2] = positions[vi[3]].z - positions[vi[0]].z;
        ret[3, 3] = 1;
        return ret;
    }

    // 在以上部分填写你的代码
    void Start()
    {
        int numVerticesPerDim = subdivide + 2;
        numVertices = numVerticesPerDim * numVerticesPerDim * numVerticesPerDim;

        masses = new float[numVertices];
        positions = new Vector3[numVertices];
        originalPositions = new Vector3[numVertices];
        velocity = new Vector3[numVertices];
        forces = new Vector3[numVertices];
        
        // 初始化仿真格点
        int posIdx = 0;
        for (int i = 0; i < numVerticesPerDim; ++i)
            for (int j = 0; j < numVerticesPerDim; ++j)
                for(int k = 0; k < numVerticesPerDim; ++k)
                {
                    var offset = cubeSize * new Vector3((float)i / (subdivide + 1), (float)j / (subdivide + 1), (float)k / (subdivide + 1));
                    positions[posIdx] = transform.TransformPoint(offset) + 3.7f * Vector3.up;
                    originalPositions[posIdx] = transform.TransformPoint(offset) + 3.7f * Vector3.up;
                    ++posIdx;
                }

        groundObject = GameObject.Find("Ground");

        // 获取mesh
        mesh = GetComponentInChildren<MeshFilter>();
        var vertices = mesh.mesh.vertices;
        meshVertexMap = new int[vertices.Length];

        Vector3 vMin = vertices[0];
        Vector3 vMax = vertices[0];
        foreach (var v in vertices)
        {
            vMin = Vector3.Min(v, vMin);
            vMax = Vector3.Max(v, vMax);
        }

        var meshOffset = vMin;
        var meshScale = (vMax - vMin) / cubeSize;
        var invScale = new Vector3(1.0f / meshScale.x, 1.0f / meshScale.y, 1.0f / meshScale.z);

        for (int i = 0; i < vertices.Length; i++)
        {
            var pos = (vertices[i] - meshOffset);
            pos.Scale(invScale);
            var idx = pos * (subdivide + 1);
            int xi = Mathf.Clamp(Mathf.RoundToInt(idx.x), 0, numVerticesPerDim - 1);
            int yi = Mathf.Clamp(Mathf.RoundToInt(idx.y), 0, numVerticesPerDim - 1);
            int zi = Mathf.Clamp(Mathf.RoundToInt(idx.z), 0, numVerticesPerDim - 1);
            meshVertexMap[i] = (xi * numVerticesPerDim + yi) * numVerticesPerDim + zi;
        }

        // 计算拉梅参数
        mu = youngModulus / (2.0f * (1.0f + possionRatio));
        lmbda = youngModulus * possionRatio / ((1.0f + possionRatio) * (1.0f - 2.0f * possionRatio));

        // 其他初始化代码

        // 仿真格点四面体化
        int numCubesPerDim = subdivide + 1;
        int numCubes = numCubesPerDim * numCubesPerDim * numCubesPerDim;
        numTets = 6 * numCubes;

        tets = new Tetrahedron[numTets];
        float mass0 = density * 1.0f / numCubes / 6.0f / 4.0f;

        for (int i = 0; i < numCubesPerDim; ++i)
            for (int j = 0; j < numCubesPerDim; ++j)
                for(int k = 0; k < numCubesPerDim; ++k)
                {
                    int cubeIdx = (i * numCubesPerDim + j) * numCubesPerDim + k;
                    int[] vertexIdx = new int[8] 
                    {
                        (i * numVerticesPerDim + j) * numVerticesPerDim + k,
                        (i * numVerticesPerDim + (j + 1)) * numVerticesPerDim + k,
                        (i * numVerticesPerDim + (j + 1)) * numVerticesPerDim + (k + 1),
                        (i * numVerticesPerDim + j) * numVerticesPerDim + (k + 1),
                        ((i + 1) * numVerticesPerDim + j) * numVerticesPerDim + k,
                        ((i + 1) * numVerticesPerDim + (j + 1)) * numVerticesPerDim + k,
                        ((i + 1) * numVerticesPerDim + (j + 1)) * numVerticesPerDim + (k + 1),
                        ((i + 1) * numVerticesPerDim + j) * numVerticesPerDim + (k + 1),
                    };
                    for (int l = 0; l < 6; ++l) 
                    {
                        tets[6 * cubeIdx + l] = new Tetrahedron();
                        for (int m = 0; m < 4; ++m)
                        {
                            tets[6 * cubeIdx + l].vi[m] = vertexIdx[division[l, m]];
                            masses[vertexIdx[division[l, m]]] += mass0;
                        }
                        tets[6 * cubeIdx + l].Bm = D(6 * cubeIdx + l).inverse;
                    }
                }
        Debug.Log("Start with mode 0");
        // 在以上部分填写你的代码
    }


    void FixedUpdate()
    {
        int simuSteps = (int)(Mathf.Round(Time.deltaTime / simuTimeStep));
        if (maxSimuSteps > 0)
            simuSteps = Mathf.Min(simuSteps, maxSimuSteps);

        for (int simuCnt = 0; simuCnt < simuSteps; ++simuCnt)
        {
            UpdateFunc();
        }

        // 判断方块是否处于稳定状态
        bool isStable = true;
        for (int i = 0; i < numVertices; ++i)
        {
            if (positions[i].y > 1.01f) // 若最高点高度不超过1, 则认为已经达到稳定状态
            {
                isStable = false;
                break;
            }
        }
        if (isStable)
        {
            stableTime += 1;
            if (stableTime > 10)    // 稳定状态持续10以上
            {
                if (playMode == 0)  // 普通仿真模式下回到初始状态循环播放
                {
                    for (int i = 0; i < numVertices; ++i)
                    {
                        positions[i] = originalPositions[i];
                        velocity[i] = Vector3.zero;
                    }
                    stableTime = 0;
                }
                else if (playMode == 2) // 可交互模式默认状态下原地弹起
                {
                    for (int i = 0; i < numVertices; ++i)
                    {
                        velocity[i] = -gravity;
                    }
                    stableTime = 0;
                }
            }
            if (stableTime > 100)   // 防止stableTime溢出
            {
                stableTime = 11;
            }
        }
        else    // 若稳定状态不连续则需要清零
        {
            stableTime = 0;
        }

        if (keyControlTime > 0) // 键盘控制锁冷却
        {
            keyControlTime -= 1;
        }
        if (Input.GetKey(KeyCode.W) && keyControlTime == 0)
        {
            if (playMode == 3 && stableTime > 10)
            {
                Debug.Log("Forward");
                for (int i = 0; i < numVertices; ++i)
                {
                    velocity[i] = -gravity;
                    velocity[i] += 2.0f * Vector3.forward;
                }
                stableTime = 0;
            }
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.A) && keyControlTime == 0)
        {
            if (playMode == 3 && stableTime > 10)
            {
                Debug.Log("Left");
                for (int i = 0; i < numVertices; ++i)
                {
                    velocity[i] = -gravity;
                    velocity[i] += 2.0f * Vector3.left;
                }
                stableTime = 0;
            }
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.S) && keyControlTime == 0)
        {
            if (playMode == 3 && stableTime > 10)
            {
                Debug.Log("Back");
                for (int i = 0; i < numVertices; ++i)
                {
                    velocity[i] = -gravity;
                    velocity[i] += 2.0f * Vector3.back;
                }
                stableTime = 0;
            }
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.D) && keyControlTime == 0)
        {
            if (playMode == 3 && stableTime > 10)
            {
                Debug.Log("Right");
                for (int i = 0; i < numVertices; ++i)
                {
                    velocity[i] = -gravity;
                    velocity[i] += 2.0f * Vector3.right;
                }
                stableTime = 0;
            }
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.J) && keyControlTime == 0)
        {
            playMode = 0;
            Debug.Log("Switch to mode 0");
            for (int i = 0; i < numVertices; ++i)
            {
                positions[i] = originalPositions[i];
                velocity[i] = Vector3.zero;
            }
            stableTime = 0;
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.K) && keyControlTime == 0)
        {
            playMode = 1;
            Debug.Log("Switch to mode 1");
            for (int i = 0; i < numVertices; ++i)
            {
                positions[i] = originalPositions[i];
                velocity[i] = Vector3.zero;
            }
            stableTime = 0;
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.L) && keyControlTime == 0)
        {
            playMode = 2;
            Debug.Log("Switch to mode 2");
            for (int i = 0; i < numVertices; ++i)
            {
                positions[i] = originalPositions[i];
                velocity[i] = Vector3.zero;
            }
            stableTime = 0;
            keyControlTime = 10;
        }
        if (Input.GetKey(KeyCode.Space) && keyControlTime == 0)
        {
            if (playMode == 2)
            {
                playMode = 3;
                Debug.Log("Switch to mode 3");
            } 
            else if (playMode == 3)
            {
                playMode = 2;
                Debug.Log("Switch to mode 2");
            } 
            keyControlTime = 10;
        }
        // if (Input.GetKey(KeyCode.X) && keyControlTime == 0)
        // {
        //     FEMMode = 1 - FEMMode;
        //     if (FEMMode == 0)
        //     {
        //         Debug.Log("Switch to St.Venant Kirchhoff Model");
        //     }
        //     else
        //     {
        //         Debug.Log("Switch to Hookean Model");
        //     }
        //     keyControlTime = 10;
        // }

        // 更新顶点位置
        var vertices = mesh.mesh.vertices;
        for (int i = 0; i < vertices.Length; i++)
        {
            var pos = positions[meshVertexMap[i]];
            pos = mesh.transform.InverseTransformPoint(pos);
            vertices[i] = pos;
        }
        mesh.mesh.vertices = vertices;
        mesh.mesh.RecalculateNormals();
    }
    void UpdateFunc()
    {
        float groundHeight = 0;
        if (groundObject != null)
            groundHeight = groundObject.transform.position.y + groundObject.transform.localScale.y / 2;

        // 进行仿真，计算每个顶点的位置
        // ! 在以下部分实现FEM仿真
        for (int i = 0; i < numVertices; ++i)
        {
            forces[i] = masses[i] * gravity;
        }
        // 逐顶点计算FEM模型
        for (int i = 0; i < numTets; ++i)
        {
            Tetrahedron tet = tets[i];

            Matrix4x4 F = D(i) * tet.Bm;
            Matrix4x4 E = identityMatrix;
            Matrix4x4 P = identityMatrix;
            if (FEMMode == 0)   // St.Venant Kirchhoff Model
            {
                E = ((F.transpose * F).Substract(identityMatrix)).Multiply(0.5f);
                P = F * ((E.Multiply(2 * mu)).Add(identityMatrix.Multiply(lmbda * E.Trace())));
            }
            else if (FEMMode == 1)  // Hookean Model(不稳定)
            {
                E = ((F.Add(F.transpose)).Multiply(0.5f)).Substract(identityMatrix);
                P = (E.Multiply(2 * mu)).Add(identityMatrix.Multiply(lmbda * E.Trace()));
            }
            Matrix4x4 H = (P * tet.Bm.transpose).Multiply(-1.0f / (6.0f * tet.Bm.determinant));

            int[] vi = tet.vi;
            forces[vi[0]] += -1.0f * new Vector3((H[0,0] + H[0,1] + H[0,2]), (H[1,0] + H[1,1] + H[1,2]), (H[2,0] + H[2,1] + H[2,2]));
            forces[vi[1]] += new Vector3(H[0,0], H[1,0], H[2,0]);
            forces[vi[2]] += new Vector3(H[0,1], H[1,1], H[2,1]);
            forces[vi[3]] += new Vector3(H[0,2], H[1,2], H[2,2]);
        }
        // 更新速度与位置
        for(int i = 0; i < numVertices; ++i)
        {
            // velocity[i] *= 0.999f;  // 平滑防止剧烈抖动
            velocity[i] += forces[i] / masses[i] * simuTimeStep;
            Vector3 position_old = positions[i];
            positions[i] += velocity[i] * simuTimeStep;
            float Dis = positions[i].y - groundHeight;
            if (Dis < 0 && velocity[i].y < 0)   // 碰撞检测
            {
                if (playMode == 1)  // 能量无损模式, 速度反向弹起
                {
                    positions[i] = position_old;
                    velocity[i] = -velocity[i];
                }
                else    // 否则, 根据弹性系数与摩擦系数计算碰撞后速度
                {
                    positions[i] -= Dis * Vector3.up;
                    Vector3 vN = velocity[i].y * Vector3.up;
                    Vector3 vT = velocity[i] - vN;
                    velocity[i] = -muN * vN + (1 - muT * (1 + muN) * vN.magnitude / vT.magnitude) * vT;
                }
            }
        }
        // 在以上部分填写你的代码
    }
}
