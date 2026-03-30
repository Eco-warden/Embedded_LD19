"""
LD19 LiDAR 이탈 이벤트 수신 FastAPI 서버
2026

실행:
    pip install fastapi uvicorn
    cd server
    uvicorn main:app --host 0.0.0.0 --port 8000 --reload
"""

from datetime import datetime
from typing import Optional

from fastapi import FastAPI, status
from pydantic import BaseModel

app = FastAPI(
    title="LD19 LiDAR Event API",
    version="1.0.0",
)

# ── 이벤트 모델 ──────────────────────────────────────────────────────

class DepartureEventIn(BaseModel):
    timestamp: str          # ISO 8601  "2026-03-27T12:34:56.789Z"
    x: float                # mm
    y: float                # mm
    cluster_id: int
    type: str               # "abandoned"

class DepartureEventOut(BaseModel):
    id: int
    received_at: str
    event: DepartureEventIn

# ── 인메모리 저장소 (프로토타입용) ────────────────────────────────────

events_db: list[DepartureEventOut] = []
next_id: int = 1

# ── 엔드포인트 ───────────────────────────────────────────────────────

@app.post(
    "/api/v1/events",
    response_model=DepartureEventOut,
    status_code=status.HTTP_201_CREATED,
)
async def create_event(event: DepartureEventIn):
    """이탈 이벤트 수신 — C++ EventNotifier가 호출하는 엔드포인트"""
    global next_id

    record = DepartureEventOut(
        id=next_id,
        received_at=datetime.utcnow().isoformat() + "Z",
        event=event,
    )
    events_db.append(record)
    next_id += 1

    print(f"[EVENT #{record.id}] {event.type} | "
          f"cluster={event.cluster_id} | "
          f"({event.x:.0f}, {event.y:.0f}) mm | "
          f"ts={event.timestamp}")

    return record

@app.get(
    "/api/v1/events",
    response_model=list[DepartureEventOut],
)
async def list_events(
    limit: int = 50,
    cluster_id: Optional[int] = None,
):
    """수신된 이벤트 목록 조회"""
    result = events_db
    if cluster_id is not None:
        result = [e for e in result if e.event.cluster_id == cluster_id]
    return result[-limit:]

@app.get("/api/v1/events/count")
async def event_count():
    """총 이벤트 수"""
    return {"count": len(events_db)}

@app.get("/health")
async def health():
    return {"status": "ok"}
