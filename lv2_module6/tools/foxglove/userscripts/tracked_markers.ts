// Foxglove User Script — /tracked_objects (perception/msg/TrackedObjectArray) → 3D 패널용 MarkerArray
//   3D 패널은 우리 msg를 못 그림 → 물체마다 상자(퍽 cyl이면 원기둥) + 글자(class · #tag · 바코드 내용)
//   bee-mission-layout.json 안에 같은 코드가 들어 있음 (이 파일은 리뷰·수정용 원본 — 고치면 레이아웃에 다시 붙여 넣기)
//   size.x·y는 영상 박스 폭을 m로 바꾼 값(map 축 아님) — 크기·방향은 대략으로만 볼 것
import { Input, Message } from "./types.ts";

export const inputs = ["/tracked_objects"];
export const output = "/foxglove_script/tracked_markers";

// ROS 2 소스에서 스키마 이름이 "visualization_msgs/msg/MarkerArray" — 타입 오류가 나면 "visualization_msgs/MarkerArray"로
type Markers = Message<"visualization_msgs/msg/MarkerArray">;

const RGB: Record<string, [number, number, number]> = {
  red: [1, 0.15, 0.15], orange: [1, 0.55, 0], yellow: [1, 0.9, 0], green: [0.1, 0.8, 0.2],
  blue: [0.2, 0.4, 1], purple: [0.6, 0.2, 0.9], white: [0.95, 0.95, 0.95], black: [0.2, 0.2, 0.2],
};

export default function script(event: Input<"/tracked_objects">): Markers {
  const { header, objects } = event.message;
  const markers: unknown[] = [{ header, ns: "", id: 0, action: 3 }]; // DELETEALL — 사라진 물체 지우기
  for (const o of objects) {
    const kv: Record<string, string> = {};
    for (const d of o.details) kv[d.key] = d.value;
    const [r, g, b] = RGB[o.color] ?? [0.7, 0.7, 0.7];
    const { position, orientation } = o.pose.pose;
    const z = Math.max(o.size.z, 0.02);
    markers.push({
      header, ns: "body", id: o.id, action: 0, type: kv.shape === "cyl" ? 3 : 1, // CYLINDER | CUBE
      pose: o.pose.pose, color: { r, g, b, a: 0.8 },
      scale: { x: Math.max(o.size.x, 0.02), y: Math.max(o.size.y, 0.02), z },
    });
    const text = [o.class_name, kv.tag !== undefined ? `#${kv.tag}` : "", kv.content ?? ""].filter((s) => s).join(" ");
    markers.push({
      header, ns: "label", id: o.id, action: 0, type: 9, // TEXT_VIEW_FACING
      pose: { position: { ...position, z: position.z + z + 0.04 }, orientation },
      scale: { x: 0, y: 0, z: 0.04 }, color: { r: 1, g: 1, b: 1, a: 1 }, text,
    });
  }
  return { markers } as unknown as Markers; // 빈 필드(lifetime·points …)는 3D 패널이 기본값으로 채움 — 실기에서 확인
}
