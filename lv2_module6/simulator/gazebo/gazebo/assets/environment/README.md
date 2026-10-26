# Gazebo 환경 텍스처

`../../../../vazebo/images`의 Poly Haven CC0 자산을 Gazebo 선택용으로 복사한 것입니다.

| 종류 | 파일 | 사용 |
| --- | --- | --- |
| 파노라마 | `background/pano_*.jpg` | `GZ_BACKGROUND=<stem>` |
| 바닥 | `floor/floor_*.jpg` | `GZ_FLOOR_TEXTURE=<stem>` |
| 벽 | `wall/wall_*.jpg` | `GZ_WALL_TEXTURE=<stem>` |

- `none`이면 기존 Gazebo 기본 재질을 유지합니다.
- Gazebo `수동 액터 → 환경` 탭에서도 같은 항목을 선택해 즉시 월드 모델을 교체할 수 있습니다.
- `GZ_BACKGROUND`는 equirectangular 이미지를 안쪽 면 sphere에 매핑합니다.
- 바닥/벽은 Gazebo에서 생성한 OBJ/MTL mesh로 적용됩니다.
- 각 폴더의 `*_CREDITS.txt`에 원본 출처(Poly Haven, CC0)가 있습니다.

실행 예:

```bash
GZ_FLOOR_TEXTURE=floor_wood \
GZ_WALL_TEXTURE=wall_painted \
GZ_BACKGROUND=pano_wooden_lounge \
lv2_module6/develop/gazebo/start.sh
```

자세한 설명은 상위 [`../README.md`](../README.md)를 보세요.
