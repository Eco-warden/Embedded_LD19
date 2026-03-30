/**
 * LD19JsonReceiver.cs
 * Unity C# — LD19 LiDAR JSON 메타데이터 UDP 수신
 * 2026
 *
 * 사용법:
 *   1. 빈 GameObject에 이 스크립트를 부착
 *   2. listenPort = 9090 (C++ 앱의 UDP 목적지 포트와 동일)
 *   3. OnClusterUpdate / OnAbandonedEvent 콜백을 구독하여 UI 연동
 *
 * 참고: LD19Receiver.cs(바이너리) 또는 이 스크립트(JSON) 중 택 1 사용.
 *       동시 사용 시 포트를 분리해야 합니다.
 */

using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class LD19JsonReceiver : MonoBehaviour
{
    [Header("UDP Settings")]
    public int listenPort = 9090;

    // ── JSON 데이터 모델 (C++ JsonPacketSender와 동일 구조) ─────────

    [Serializable]
    public class ClusterData
    {
        public int    id;
        public float  x;
        public float  y;
        public int    count;
        public string type;   // "normal" or "abandoned"
    }

    [Serializable]
    public class EventData
    {
        public string type;          // "abandoned"
        public float  x;
        public float  y;
        public int    cluster_id;
        public string timestamp;     // ISO 8601
    }

    [Serializable]
    public class FramePacket
    {
        public int              frame_id;
        public string           timestamp;
        public List<ClusterData> clusters;
        public EventData        @event;    // "event" is C# keyword
    }

    // ── 이벤트 콜백 ────────────────────────────────────────────────

    public event Action<List<ClusterData>> OnClusterUpdate;
    public event Action<EventData>         OnAbandonedEvent;

    // ── 필드 ────────────────────────────────────────────────────────

    UdpClient     _udp;
    Thread        _recvThread;
    volatile bool _running;

    readonly object _lock = new object();
    FramePacket     _latest;
    bool            _newData;
    int             _packetCount;

    // ── Unity 라이프사이클 ──────────────────────────────────────────

    void Start()
    {
        _running = true;
        _udp = new UdpClient(listenPort);
        _recvThread = new Thread(ReceiveLoop)
        {
            IsBackground = true,
            Name = "LD19-JSON"
        };
        _recvThread.Start();

        Debug.Log($"[LD19-JSON] Listening on UDP port {listenPort}");
    }

    void OnDestroy()
    {
        _running = false;
        _udp?.Close();
        _recvThread?.Join(500);
    }

    void Update()
    {
        FramePacket pkt;
        bool hasNew;

        lock (_lock)
        {
            hasNew   = _newData;
            pkt      = _latest;
            _newData = false;
        }

        if (!hasNew || pkt == null) return;

        // 클러스터 업데이트 콜백
        if (pkt.clusters != null && pkt.clusters.Count > 0)
        {
            OnClusterUpdate?.Invoke(pkt.clusters);
        }

        // 이탈 이벤트 콜백
        if (pkt.@event != null && pkt.@event.type == "abandoned")
        {
            OnAbandonedEvent?.Invoke(pkt.@event);

            Debug.LogWarning(
                $"[LD19] ABANDONED at ({pkt.@event.x:F0}, {pkt.@event.y:F0}) mm " +
                $"cluster={pkt.@event.cluster_id} ts={pkt.@event.timestamp}");
        }
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

                // 바이너리 패킷 필터링: magic 0x4C44("LD")로 시작하면 스킵
                if (data.Length >= 2 && data[0] == 0x44 && data[1] == 0x4C)
                    continue;

                // JSON 문자열 파싱
                string json = Encoding.UTF8.GetString(data);

                // '{'로 시작하지 않으면 JSON이 아님
                if (string.IsNullOrEmpty(json) || json[0] != '{')
                    continue;

                FramePacket pkt = JsonUtility.FromJson<FramePacket>(json);
                if (pkt == null) continue;

                _packetCount++;

                lock (_lock)
                {
                    _latest  = pkt;
                    _newData = true;
                }
            }
            catch (SocketException)
            {
                break;
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception e)
            {
                Debug.LogError($"[LD19-JSON] Parse error: {e.Message}");
            }
        }
    }

    // ── 디버그 정보 ─────────────────────────────────────────────────

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 100));
        GUILayout.Label($"[LD19 JSON] Port: {listenPort}");
        GUILayout.Label($"Packets received: {_packetCount}");

        FramePacket pkt;
        lock (_lock) { pkt = _latest; }
        if (pkt != null)
        {
            GUILayout.Label($"Frame: {pkt.frame_id} | " +
                            $"Clusters: {pkt.clusters?.Count ?? 0}");
        }
        GUILayout.EndArea();
    }
}
