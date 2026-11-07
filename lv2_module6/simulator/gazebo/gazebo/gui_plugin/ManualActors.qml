import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

ColumnLayout {
  id: manualActors
  width: 400
  Layout.minimumWidth: 380
  Layout.minimumHeight: 620
  spacing: 6

  TabBar {
    id: tabs
    Layout.fillWidth: true
    Layout.preferredWidth: 380
    TabButton { text: "액터" }
    TabButton { text: "환경" }
  }

  StackLayout {
    id: pages
    currentIndex: tabs.currentIndex
    Layout.fillWidth: true
    Layout.fillHeight: true

    ColumnLayout {
      spacing: 6

      Label {
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        text: "소환 버튼을 누르면 무작위 빈 위치에 생성됩니다."
        opacity: 0.8
      }

      Button {
        Layout.fillWidth: true
        text: "사람 소환"
        Material.background: Material.Orange
        onClicked: _ManualActors.SpawnHuman()
      }

      Button {
        Layout.fillWidth: true
        text: "박스 장애물 소환"
        Material.background: Material.Brown
        onClicked: _ManualActors.SpawnBox()
      }

      Label {
        Layout.fillWidth: true
        text: "퍽 (6면 AprilTag 포함)"
        opacity: 0.8
      }

      Button {
        Layout.fillWidth: true
        text: "빨강 퍽"
        Material.background: Material.Red
        onClicked: _ManualActors.SpawnPuckRed()
      }

      Button {
        Layout.fillWidth: true
        text: "주황 퍽"
        Material.background: Material.Orange
        onClicked: _ManualActors.SpawnPuckOrange()
      }

      Button {
        Layout.fillWidth: true
        text: "초록 퍽"
        Material.background: Material.Green
        onClicked: _ManualActors.SpawnPuckGreen()
      }

      Button {
        Layout.fillWidth: true
        text: "가로형 신호등 소환"
        Material.background: Material.Indigo
        onClicked: _ManualActors.SpawnTrafficLight()
      }

      Button {
        Layout.fillWidth: true
        text: "수동 모델 전체 삭제"
        Material.background: Material.Grey
        onClicked: _ManualActors.ClearManualModels()
      }

      Label {
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        text: "임의 위치는 Gazebo 기본 Spawn/드래그를 사용하세요."
        opacity: 0.7
        font.pixelSize: 11
      }
    }

    ColumnLayout {
      spacing: 8

      Label {
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        text: "환경 모델 (즉시 교체)"
        font.bold: true
      }

      Label {
        Layout.fillWidth: true
        text: "바닥"
        opacity: 0.8
      }
      ComboBox {
        id: floorBox
        Layout.fillWidth: true
        model: ["none", "floor_concrete", "floor_tiles", "floor_wood", "floor_garage"]
      }

      Label {
        Layout.fillWidth: true
        text: "벽"
        opacity: 0.8
      }
      ComboBox {
        id: wallBox
        Layout.fillWidth: true
        model: ["none", "wall_concrete", "wall_painted"]
      }

      Label {
        Layout.fillWidth: true
        text: "파노라마 배경"
        opacity: 0.8
      }
      ComboBox {
        id: backgroundBox
        Layout.fillWidth: true
        model: ["none", "pano_hospital_room", "pano_industrial_wooden_attic",
                "pano_lebombo", "pano_photo_studio_loft_hall",
                "pano_unfinished_office", "pano_vintage_measuring_lab",
                "pano_wooden_lounge"]
      }

      Button {
        Layout.fillWidth: true
        text: "환경 적용"
        Material.background: Material.Teal
        onClicked: _ManualActors.ApplyEnvironment(
          floorBox.currentText, wallBox.currentText, backgroundBox.currentText)
      }

      Label {
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        text: "적용하면 현재 월드의 바닥/벽/배경 모델을 교체합니다. 수동 소환 퍽·신호등은 영향받지 않습니다."
        opacity: 0.7
        font.pixelSize: 11
      }
    }
  }
}
