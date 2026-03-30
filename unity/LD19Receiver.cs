/**
 * LD19Receiver.cs
 * Unity C# — LD19 LiDAR UDP 수신 + 점구름/클러스터 시각화
 * 2026
 *
 * 사용법:
 *   1. 빈 GameObject에 이 스크립트를 부착
 *   2. Inspector에서 listenPort = 9090 설정
 *   3. pointPrefab에 작은 Sphere 프리팹 할당 (또는 null이면 자동 생성)
 *   4. 실행 후 C++ 앱이 UDP 전송 시작하면 점구름이 표시됨
 */

using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;

public class LD19Receiver : MonoBehaviour
{
    [Header("UDP Settings")]
    public int listenPort = 9090;

    [Header("Visualization")]
    public float scale = 0.001f;          // mm → Unity meters
    public float pointSize = 0.02f;
    public Color pointColor = Color.green;
    public Color clusterColor = Color.red;
    public Color departedColor = Color.yellow;

    // ── 바이너리 프로토콜 (C++ packed 구조체와 동일) ────────────────

    const ushort UDP_MAGIC         = 0x4C44;
    const byte   PKT_TYPE_POINTS   = 0x01;
    const byte   PKT_TYPE_CLUSTERS = 0x02;

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct PacketHeader
    {
        public ushort magic;
        public byte   type;
        public byte   flags;
        public uint   frameId;
        public ulong  timestampMs;
        public byte   fragmentIndex;
        public byte   fragmentCount;
        public ushort itemCount;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct PointItem
    {
        public float x_mm;
        public float y_mm;
        public byte  intensity;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct ClusterItem
    {
        public ushort clusterId;
        public float  centroidX;
        public float  centroidY;
        public ushort pointCount;
        public uint   trackId;
        public byte   trackState;     // 0=moving,1=stop,2=departed,3=lost,0xFF=untracked
        public float  velocityX;
        public float  velocityY;
    }

    // ── 메인 스레드로 전달할 데이터 ─────────────────────────────────

    struct FrameData
    {
        public uint frameId;
        public List<PointItem> points;
        public List<ClusterItem> clusters;
    }

    // ── 필드 ────────────────────────────────────────────────────────

    UdpClient       _udp;
    Thread          _recvThread;
    volatile bool   _running;

    readonly object _lock = new object();
    FrameData       _latestFrame;
    bool            _newData;

    // 프레임 조립용 버퍼 (fragment 수집)
    uint                  _assembleFrameId;
    List<PointItem>       _assemblePoints   = new List<PointItem>();
    List<ClusterItem>     _assembleClusters = new List<ClusterItem>();
    int                   _pointFragsRecv;
    int                   _pointFragsTotal;
    int                   _clusterFragsRecv;
    int                   _clusterFragsTotal;

    // 시각화 오브젝트 풀
    List<GameObject>      _pointPool  = new List<GameObject>();
    List<GameObject>      _clusterPool = new List<GameObject>();
    Material              _pointMat;
    Material              _clusterMat;
    Material              _departedMat;

    // ── Unity 라이프사이클 ──────────────────────────────────────────

    void Start()
    {
        _pointMat    = CreateUnlitMaterial(pointColor);
        _clusterMat  = CreateUnlitMaterial(clusterColor);
        _departedMat = CreateUnlitMaterial(departedColor);

        _running = true;
        _udp = new UdpClient(listenPort);
        _recvThread = new Thread(ReceiveLoop)
        {
            IsBackground = true,
            Name = "LD19-UDP"
        };
        _recvThread.Start();

        Debug.Log($"[LD19] Listening on UDP port {listenPort}");
    }

    void OnDestroy()
    {
        _running = false;
        _udp?.Close();
        _recvThread?.Join(500);
    }

    void Update()
    {
        FrameData frame;
        bool hasNew;

        lock (_lock)
        {
            hasNew = _newData;
            frame  = _latestFrame;
            _newData = false;
        }

        if (!hasNew) return;

        RenderPoints(frame.points);
        RenderClusters(frame.clusters);
    }

    // ── UDP 수신 스레드 ─────────────────────────────────────────────

    void ReceiveLoop()
    {
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);

        while (_running)
        {
            try
            {
                byte[] data = _udp.Receive(ref remote);
                if (data.Length < Marshal.SizeOf<PacketHeader>()) continue;

                PacketHeader hdr = FromBytes<PacketHeader>(data, 0);
                if (hdr.magic != UDP_MAGIC) continue;

                // 새 프레임이면 버퍼 리셋
                if (hdr.frameId != _assembleFrameId)
                {
                    FlushAssembled();
                    _assembleFrameId    = hdr.frameId;
                    _assemblePoints.Clear();
                    _assembleClusters.Clear();
                    _pointFragsRecv     = 0;
                    _pointFragsTotal    = 0;
                    _clusterFragsRecv   = 0;
                    _clusterFragsTotal  = 0;
                }

                int offset = Marshal.SizeOf<PacketHeader>();

                if (hdr.type == PKT_TYPE_POINTS)
                {
                    _pointFragsTotal = hdr.fragmentCount;
                    _pointFragsRecv++;
                    for (int i = 0; i < hdr.itemCount; i++)
                    {
                        _assemblePoints.Add(FromBytes<PointItem>(data, offset));
                        offset += Marshal.SizeOf<PointItem>();
                    }
                }
                else if (hdr.type == PKT_TYPE_CLUSTERS)
                {
                    _clusterFragsTotal = hdr.fragmentCount;
                    _clusterFragsRecv++;
                    for (int i = 0; i < hdr.itemCount; i++)
                    {
                        _assembleClusters.Add(FromBytes<ClusterItem>(data, offset));
                        offset += Marshal.SizeOf<ClusterItem>();
                    }
                }

                // 모든 조각 수신 완료 시 flush
                bool pointsDone   = (_pointFragsTotal > 0 &&
                                     _pointFragsRecv >= _pointFragsTotal);
                bool clustersDone = (_clusterFragsTotal > 0 &&
                                     _clusterFragsRecv >= _clusterFragsTotal);

                if (pointsDone && clustersDone)
                {
                    FlushAssembled();
                }
            }
            catch (SocketException)
            {
                // 소켓 닫힘 (OnDestroy)
                break;
            }
            catch (ObjectDisposedException)
            {
                break;
            }
        }
    }

    void FlushAssembled()
    {
        if (_assemblePoints.Count == 0 && _assembleClusters.Count == 0)
            return;

        lock (_lock)
        {
            _latestFrame = new FrameData
            {
                frameId  = _assembleFrameId,
                points   = new List<PointItem>(_assemblePoints),
                clusters = new List<ClusterItem>(_assembleClusters)
            };
            _newData = true;
        }
    }

    // ── 시각화 ──────────────────────────────────────────────────────

    void RenderPoints(List<PointItem> points)
    {
        if (points == null) return;

        // 풀 확장
        while (_pointPool.Count < points.Count)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Quad);
            go.name = "LidarPt";
            go.transform.localScale = Vector3.one * pointSize;
            go.GetComponent<MeshRenderer>().sharedMaterial = _pointMat;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform);
            _pointPool.Add(go);
        }

        for (int i = 0; i < _pointPool.Count; i++)
        {
            if (i < points.Count)
            {
                var pt = points[i];
                // LiDAR 2D → Unity XZ 평면 (Y=0)
                _pointPool[i].transform.localPosition = new Vector3(
                    pt.x_mm * scale,
                    0f,
                    pt.y_mm * scale
                );
                _pointPool[i].SetActive(true);
            }
            else
            {
                _pointPool[i].SetActive(false);
            }
        }
    }

    void RenderClusters(List<ClusterItem> clusters)
    {
        if (clusters == null) return;

        while (_clusterPool.Count < clusters.Count)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = "LidarCluster";
            go.transform.localScale = Vector3.one * 0.1f;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform);
            _clusterPool.Add(go);
        }

        for (int i = 0; i < _clusterPool.Count; i++)
        {
            if (i < clusters.Count)
            {
                var cl = clusters[i];
                _clusterPool[i].transform.localPosition = new Vector3(
                    cl.centroidX * scale,
                    0.05f,  // 점구름 위에 표시
                    cl.centroidY * scale
                );

                // 상태별 색상
                var mr = _clusterPool[i].GetComponent<MeshRenderer>();
                mr.sharedMaterial = (cl.trackState == 2)
                    ? _departedMat   // DEPARTED → 노란색
                    : _clusterMat;   // 기타 → 빨간색

                // 크기: 포인트 수에 비례
                float s = Mathf.Clamp(cl.pointCount * 0.003f, 0.05f, 0.3f);
                _clusterPool[i].transform.localScale = Vector3.one * s;
                _clusterPool[i].SetActive(true);
            }
            else
            {
                _clusterPool[i].SetActive(false);
            }
        }
    }

    // ── 유틸리티 ────────────────────────────────────────────────────

    static T FromBytes<T>(byte[] data, int offset) where T : struct
    {
        int size = Marshal.SizeOf<T>();
        IntPtr ptr = Marshal.AllocHGlobal(size);
        try
        {
            Marshal.Copy(data, offset, ptr, size);
            return Marshal.PtrToStructure<T>(ptr);
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }
    }

    static Material CreateUnlitMaterial(Color color)
    {
        var mat = new Material(Shader.Find("Unlit/Color"));
        mat.color = color;
        return mat;
    }
}
